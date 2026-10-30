"""箱の配置リセットと成功判定を行うノード。

- /scene/reset (std_srvs/Trigger)：アームを初期姿勢へ戻し、箱を左右2か所＋小さなずれで再配置する。
  赤青の左右は毎回ランダムに入れ替える。
- /scene/status (std_msgs/String, JSON)：箱の位置・配置・持ち上がり判定・掴んでいる箱（10Hz）。
- /vla/instruction (std_msgs/String)：指示文から対象の色（red / blue）を読み取り、成功判定に使う。
- 疑似固定（B1。docs/設計.md 7章）：開いていたグリッパが閉じたとき、箱の中心が指の間の範囲内（gripper_link 座標）
  なら掴んだとみなし、グリッパが開くまで箱を手先に追従させる（/set_entity_state。掴んだ瞬間の相対位置を保つ）。

座標：ロボットは原点で +x を向く。「左」は +y。机の上面が z=0。
"""
import json
import math
import random
import re
import time

import rclpy
from gazebo_msgs.msg import LinkStates, ModelStates
from gazebo_msgs.srv import SetEntityState
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# 初期姿勢（shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper）[rad]
HOME_POSE = [0.0, -0.454, 0.138, 1.658, 0.0, 0.3]
COLORS = ('red', 'blue')
GRIPPER_LINK = 'so101::gripper_link'


# 四元数は (x, y, z, w)
def q_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def q_conj(q):
    return (-q[0], -q[1], -q[2], q[3])


def q_rotate(q, v):
    return q_mul(q_mul(q, (v[0], v[1], v[2], 0.0)), q_conj(q))[:3]


class SceneManager(Node):
    def __init__(self):
        super().__init__('scene_manager')
        self.declare_parameter('box_size', 0.03)
        self.declare_parameter('slot_x', 0.25)
        self.declare_parameter('slot_y', 0.08)       # 左右の箱の y（±）
        self.declare_parameter('jitter', 0.03)       # 位置のランダムずれ（±）[m]
        self.declare_parameter('lift_threshold', 0.05)  # 初期 z からこれ以上上がったら「持ち上げた」
        self.declare_parameter('seed', -1)
        self.declare_parameter('home_pose', HOME_POSE)
        self.declare_parameter('home_wait', 1.0)     # 初期姿勢へ戻るのを待つ時間 [s]
        # 疑似固定。グリッパの角度は小さいほど閉じる。grip_close を下回ったら判定し、grip_open を上回ったら離す
        # （3cm の箱に可動指が触れ始めるのは約 0.15〜0.3。初期姿勢の 0.3 では掴まない）
        self.declare_parameter('grip_close', 0.25)
        self.declare_parameter('grip_open', 0.4)
        # 掴める範囲（箱の中心、gripper_link 座標）[m]。固定指のパッド面は x=-0.0079 で、可動指は +x 側にある。
        # x：箱が固定指に触れる位置（0.007）から 1.5cm 以内。y：固定指の幅の中。z：指先（約 -0.105）から上で、指と 1cm 以上重なる
        self.declare_parameter('grasp_x', [0.004, 0.022])
        self.declare_parameter('grasp_y', [-0.012, 0.012])
        self.declare_parameter('grasp_z', [-0.100, -0.055])
        seed = self.get_parameter('seed').value
        self.rng = random.Random(None if seed < 0 else seed)

        self.box_size = self.get_parameter('box_size').value
        self.rest_z = self.box_size / 2
        self.target = None
        self.layout = {}          # {'red': 'left', 'blue': 'right'}
        self.z = {}
        self.pos = {}
        self.quat = {}
        self.grip = None
        self.armed = False        # グリッパが一度開いたら True。閉じた瞬間に1回だけ判定する
        self.grasped = None       # 掴んでいる箱の色
        self.rel = None           # 掴んだ瞬間の箱の位置・向き（gripper_link 座標）

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 1)
        self.status_pub = self.create_publisher(String, '/scene/status', 1)
        self.create_subscription(ModelStates, '/model_states', self._on_states, 1)
        self.create_subscription(String, '/vla/instruction', self._on_instruction, 1)
        self.create_subscription(JointState, '/joint_states', self._on_joints, 1)
        self.create_subscription(LinkStates, '/link_states', self._on_links, 1)
        # リセット中にサービス応答を待つので、コールバックを並列に動かせるようにする
        group = ReentrantCallbackGroup()
        self.set_state = self.create_client(SetEntityState, '/set_entity_state', callback_group=group)
        self.create_service(Trigger, '/scene/reset', self._on_reset, callback_group=group)
        self.create_timer(0.1, self._publish_status)
        self.get_logger().info('scene_manager ready')

    def _on_instruction(self, msg):
        m = re.search(r'\b(red|blue)\b', msg.data.lower())
        self.target = m.group(1) if m else None

    def _on_states(self, msg):
        for c in COLORS:
            name = f'{c}_box'
            if name in msg.name:
                p = msg.pose[msg.name.index(name)].position
                o = msg.pose[msg.name.index(name)].orientation
                self.pos[c] = (p.x, p.y, p.z)
                self.quat[c] = (o.x, o.y, o.z, o.w)
                self.z[c] = p.z

    def _on_joints(self, msg):
        if 'gripper' in msg.name:
            self.grip = msg.position[msg.name.index('gripper')]

    def _on_links(self, msg):
        if GRIPPER_LINK not in msg.name or self.grip is None or len(self.pos) < 2:
            return
        pose = msg.pose[msg.name.index(GRIPPER_LINK)]
        g_p = (pose.position.x, pose.position.y, pose.position.z)
        g_q = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        if self.grasped:
            if self.grip > self.get_parameter('grip_open').value:
                self.get_logger().info(f'release: {self.grasped}')
                self.grasped = None
                self.armed = True
            else:
                self._follow(self.grasped, g_p, g_q)
            return
        if self.grip > self.get_parameter('grip_open').value:
            self.armed = True
        elif self.armed and self.grip < self.get_parameter('grip_close').value:
            self.armed = False
            self._try_grasp(g_p, g_q)

    def _try_grasp(self, g_p, g_q):
        """閉じた瞬間に、指の間に中心がある箱を探す。"""
        lim = [self.get_parameter(k).value for k in ('grasp_x', 'grasp_y', 'grasp_z')]
        inv = q_conj(g_q)
        for c in COLORS:
            rel_p = q_rotate(inv, [b - g for b, g in zip(self.pos[c], g_p)])
            ok = all(lo <= v <= hi for v, (lo, hi) in zip(rel_p, lim))
            self.get_logger().info(f'grasp check {c}: rel=({rel_p[0]:+.4f}, {rel_p[1]:+.4f}, {rel_p[2]:+.4f}) '
                                   f'{"OK" if ok else "NG"}')
            if ok:
                self.grasped = c
                self.rel = (rel_p, q_mul(inv, self.quat[c]))
                return

    def _follow(self, color, g_p, g_q):
        """掴んだ箱を手先に置き直す（速度は 0）。応答は待たない。"""
        rel_p, rel_q = self.rel
        p = [g + d for g, d in zip(g_p, q_rotate(g_q, rel_p))]
        q = q_mul(g_q, rel_q)
        n = math.sqrt(sum(v * v for v in q))
        req = SetEntityState.Request()
        req.state.name = f'{color}_box'
        req.state.reference_frame = 'world'
        req.state.pose.position.x, req.state.pose.position.y, req.state.pose.position.z = p
        (req.state.pose.orientation.x, req.state.pose.orientation.y,
         req.state.pose.orientation.z, req.state.pose.orientation.w) = [v / n for v in q]
        self.set_state.call_async(req)

    def _on_reset(self, request, response):
        if not self.set_state.wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = '/set_entity_state が使えない（gazebo_ros_state が未起動）'
            return response
        self.grasped = None
        # 先にアームを初期姿勢へ戻す。箱を先に置くと、下りたままの指と重なって弾き飛ばされる
        self.cmd_pub.publish(Float64MultiArray(data=list(self.get_parameter('home_pose').value)))
        time.sleep(self.get_parameter('home_wait').value)
        x0 = self.get_parameter('slot_x').value
        y0 = self.get_parameter('slot_y').value
        j = self.get_parameter('jitter').value
        sides = ['left', 'right']
        self.rng.shuffle(sides)
        self.layout = dict(zip(COLORS, sides))
        futures = []
        for color, side in self.layout.items():
            req = SetEntityState.Request()
            req.state.name = f'{color}_box'
            req.state.reference_frame = 'world'
            req.state.pose.position.x = x0 + self.rng.uniform(-j, j)
            req.state.pose.position.y = (y0 if side == 'left' else -y0) + self.rng.uniform(-j, j)
            req.state.pose.position.z = self.rest_z + 0.002
            req.state.pose.orientation.w = 1.0  # 向きは固定（箱は立方体なので yaw のずれは入れない）
            futures.append(self.set_state.call_async(req))
        deadline = time.time() + 2.0
        while not all(f.done() for f in futures) and time.time() < deadline:
            time.sleep(0.01)
        response.success = all(f.done() and f.result().success for f in futures)
        response.message = json.dumps(self.layout)
        self.get_logger().info(f'reset: {response.message}')
        return response

    def _publish_status(self):
        if len(self.pos) < 2:
            return
        thr = self.get_parameter('lift_threshold').value
        lifted = {c: self.z[c] >= self.rest_z + thr for c in COLORS}
        other = None if self.target is None else [c for c in COLORS if c != self.target][0]
        status = {
            'layout': self.layout,
            'target': self.target,
            'box_pos': {c: [round(v, 4) for v in self.pos[c]] for c in COLORS},
            'lifted': lifted,
            'grasped': self.grasped,
            'success': bool(self.target and lifted[self.target] and not lifted[other]),
            'wrong': bool(self.target and lifted[other]),
        }
        self.status_pub.publish(String(data=json.dumps(status)))


def main():
    rclpy.init()
    node = SceneManager()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()
