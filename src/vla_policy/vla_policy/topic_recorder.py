"""ROS トピックを購読して rerun に記録する（挙動の目視確認用）。
誰がアームを動かしていても（grasp_test.py・デモ収集・vla_policy）同じ形で残る。
エンティティ名は docs/設計.md 6.3。scripts/rrd_record.py と vla_policy_node が使う。
"""
import json
import os
import warnings

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from rcl_interfaces.msg import Log
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String

warnings.filterwarnings('ignore')  # ikpy が固定ジョイントについて出す警告

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
# 生画像（約 900KB/枚）は rclpy のデシリアライズが遅く、その間に他のトピックを取りこぼすので、
# image_transport の JPEG（/compressed）をデコードせずにそのまま記録する
CAMERAS = {'/cam_front/image_raw/compressed': 'obs/cam_front', '/cam_wrist/image_raw/compressed': 'obs/cam_wrist'}
BOX_COLORS = {'red': [220, 40, 40], 'blue': [40, 80, 220]}
LOG_NODES = ('scene_manager', 'vla_policy')  # /rosout のうち記録するノード（grasp check・推論の状況など）


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def blueprint():
    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Vertical(
                rrb.Spatial3DView(origin='world', name='3D'),
                rrb.TextLogView(origin='log', name='log'),
                row_shares=[3, 1]),
            rrb.Vertical(
                rrb.Horizontal(rrb.Spatial2DView(origin='obs/cam_front', name='front'),
                               rrb.Spatial2DView(origin='obs/cam_wrist', name='wrist')),
                rrb.TimeSeriesView(origin='obs/state', name='state [rad]'),
                rrb.TimeSeriesView(origin='action', name='action [rad]（cmd：送った指令、pred：予測）'),
                rrb.TimeSeriesView(origin='scene/box_z', name='box z [m]'),
                row_shares=[2, 1, 1, 1]),
            column_shares=[1, 2]),
        collapse_panels=True)


def open_recording(path, live=False):
    """path に .rrd を書き出す記録を始め、机などの固定のものを描く。"""
    os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
    rr.init('so101_vla', recording_id=os.path.basename(path))
    sinks = [rr.FileSink(path)]
    if live:
        sinks.append(rr.GrpcSink())  # Viewer（ポート 9876）にも送る
    rr.set_sinks(*sinks)
    # 画面レイアウトもファイルに入れる。rr.init(default_blueprint=...) だと set_sinks の前に送られてファイルに残らない
    rr.send_blueprint(blueprint(), make_active=False, make_default=True)
    rr.log('world', rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
    # 机の上面（z=0）の目安
    rr.log('world/table', rr.Boxes3D(centers=[[0.25, 0.0, -0.005]], half_sizes=[[0.25, 0.3, 0.005]],
                                     colors=[[120, 100, 80, 80]]), static=True)


class Recorder:
    def __init__(self, node, urdf='outputs/so101.urdf', image_hz=10.0):
        self.node = node
        self.image_dt = 1.0 / image_hz
        self.last_image = {}
        self.last_stamp = 0.0
        self.chain = None
        if os.path.exists(urdf):
            from ikpy.chain import Chain
            self.chain = Chain.from_urdf_file(urdf, base_elements=['base_link'])
        else:
            node.get_logger().warn(f'{urdf} が無いので 3D のアームは描かない')
        for topic, entity in CAMERAS.items():
            node.create_subscription(CompressedImage, topic, lambda m, e=entity: self.on_image(m, e), 1)
        node.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        node.create_subscription(Float64MultiArray, '/arm_controller/commands', self.on_cmd, 10)
        node.create_subscription(String, '/scene/status', self.on_status, 10)
        node.create_subscription(String, '/vla/instruction', self.on_instruction, 10)
        node.create_subscription(Log, '/rosout', self.on_rosout, 50)

    def set_time(self, sec=None):
        """sec が無いメッセージ（指令・status・/rosout）は直近の stamp で刻む。
        /clock は 10Hz しか出ないので、ノードの時計を使うと 0.1 秒単位で重なってしまう"""
        if sec is not None:
            self.last_stamp = max(self.last_stamp, sec)
        rr.set_time('sim_time', duration=sec if sec is not None else self.last_stamp)

    def on_image(self, msg, entity):
        t = stamp_sec(msg.header.stamp)
        if t - self.last_image.get(entity, -1e9) < self.image_dt:
            return
        self.last_image[entity] = t
        self.set_time(t)
        rr.log(entity, rr.EncodedImage(contents=bytes(msg.data), media_type='image/jpeg'))

    def on_joints(self, msg):
        self.set_time(stamp_sec(msg.header.stamp))
        q = dict(zip(msg.name, msg.position))
        for j in JOINTS:
            if j in q:
                rr.log(f'obs/state/{j}', rr.Scalars(q[j]))
        if self.chain is not None:
            angles = [q.get(link.name, 0.0) for link in self.chain.links]
            frames = self.chain.forward_kinematics(angles, full_kinematics=True)
            pts = np.array([f[:3, 3] for f in frames])
            rr.log('world/arm', rr.LineStrips3D([pts], colors=[[200, 200, 200]], radii=0.004))
            rr.log('world/gripper_tip', rr.Points3D([pts[-1]], colors=[[255, 200, 0]], radii=0.008))

    def on_cmd(self, msg):
        self.set_time()
        for j, v in zip(JOINTS, msg.data):
            rr.log(f'action/cmd/{j}', rr.Scalars(v))

    def on_status(self, msg):
        self.set_time()
        s = json.loads(msg.data)
        for c, p in s.get('box_pos', {}).items():
            rr.log(f'world/box/{c}', rr.Boxes3D(centers=[p], half_sizes=[[0.015] * 3],
                                                 colors=[BOX_COLORS.get(c, [128, 128, 128])],
                                                 labels=['grasped' if s.get('grasped') == c else c]))
            rr.log(f'scene/box_z/{c}', rr.Scalars(p[2]))
        for k in ('success', 'wrong'):
            rr.log(f'scene/{k}', rr.Scalars(float(bool(s.get(k)))))
        rr.log('scene/status', rr.TextDocument(json.dumps(s, indent=1)))

    def on_instruction(self, msg):
        self.set_time()
        rr.log('instruction', rr.TextDocument(msg.data))
        rr.log('log', rr.TextLog(f'instruction: {msg.data}', level='INFO'))

    def on_rosout(self, msg):
        if msg.name not in LOG_NODES:
            return
        self.set_time()  # /rosout の stamp は壁時計なので使わない
        level = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'CRITICAL'}.get(msg.level, 'INFO')
        rr.log('log', rr.TextLog(f'[{msg.name}] {msg.msg}', level=level))
