#!/usr/bin/env python3
"""疑似固定（B1）で箱を掴めるかの確認。ikpy で「箱の上 → 降下 → 閉じる → 持ち上げ」の関節角を計算して動かし、箱の z を表示する。
policy コンテナで実行する（ikpy があるため。箱の位置は /scene/status から読むので gazebo_msgs は不要）。
事前に sim コンテナで URDF を書き出しておく：
  xacro src/so101_description/urdf/so101.urdf.xacro > outputs/so101.urdf
使い方:
  python scripts/grasp_test.py red [--close 0.0] [--dx 0] [--dz 0]
"""
import argparse
import json
import time
import warnings

import numpy as np
import rclpy
from ikpy.chain import Chain
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

warnings.filterwarnings('ignore')  # ikpy が固定ジョイントについて出す警告

ARM_HOME = [0.0, -0.454, 0.138, 1.658, 0.0]
# 掴むときの箱の中心（gripper_link 座標）。x：固定指のパッド面（-0.0079）から箱の半分＋余裕 8mm
# （余裕が 2mm だと、関節の追従誤差で固定指が箱の縁に乗って押してしまった）。閉じると可動指が箱を固定指へ寄せる。
# z：指先（約 -0.105）が机に当たらない高さ。指は gripper_link の -z 方向に伸び、可動指は +x 側に開く
GRASP_CENTER = np.array([0.015, 0.0, -0.093])
# gripper_frame_joint（gripper_link → gripper_frame_link）の固定変換。ikpy のチェーンの先端は gripper_frame_link になる
FRAME_T = np.array([-0.0079, -0.000218121, -0.0981274])
FRAME_R = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])  # Ry(π)
# 掴むときの gripper_link の向き（world 座標）：指先を真下（gripper -z = world -z）、
# 可動指の開く向き（gripper +x）を world +y にする。箱の ±y の面を挟む
GRIPPER_R = np.array([[0, -1.0, 0], [1.0, 0, 0], [0, 0, 1.0]])


def make_chain(urdf):
    chain = Chain.from_urdf_file(urdf, base_elements=['base_link'])
    chain.active_links_mask = np.array([l.name in (
        'shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll') for l in chain.links])
    return chain


def solve(chain, center, init):
    """箱の中心 center（world）を GRASP_CENTER に合わせる腕の5関節角を返す。"""
    # gripper_link の原点 = center - R·GRASP_CENTER。そこから gripper_frame_link の姿勢に直す
    g_pos = center - GRIPPER_R @ GRASP_CENTER
    f_pos = g_pos + GRIPPER_R @ FRAME_T
    f_rot = GRIPPER_R @ FRAME_R
    q0 = np.zeros(len(chain.links))
    q0[1:6] = init
    q = chain.inverse_kinematics(f_pos, f_rot, orientation_mode='all', initial_position=q0)
    fk = chain.forward_kinematics(q)
    err_p = np.linalg.norm(fk[:3, 3] - f_pos)
    err_r = np.degrees(np.arccos(np.clip((np.trace(fk[:3, :3].T @ f_rot) - 1) / 2, -1, 1)))
    return list(q[1:6]), err_p, err_r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('color', nargs='?', default='red')
    ap.add_argument('--urdf', default='outputs/so101.urdf')
    ap.add_argument('--open', type=float, default=1.0, help='降下時の開き量 [rad]')
    ap.add_argument('--close', type=float, default=0.0, help='閉じるときの目標角 [rad]（scene_manager の grip_close より小さくする）')
    ap.add_argument('--dx', type=float, default=0.0, help='目標位置のずれ（world）[m]')
    ap.add_argument('--dy', type=float, default=0.0)
    ap.add_argument('--dz', type=float, default=0.0)
    ap.add_argument('--lift', type=float, default=0.08, help='持ち上げ量 [m]')
    ap.add_argument('--no-reset', action='store_true')
    ap.add_argument('--stop', choices=['descend', 'close'], help='この段階で止める（画像で確認する用）')
    args = ap.parse_args()

    rclpy.init()
    n = rclpy.create_node('grasp_test')
    pub = n.create_publisher(Float64MultiArray, '/arm_controller/commands', 1)
    g = {}
    n.create_subscription(String, '/scene/status', lambda m: g.__setitem__('status', json.loads(m.data)), 1)
    n.create_subscription(JointState, '/joint_states',
                          lambda m: g.__setitem__('grip', m.position[m.name.index('gripper')]), 1)

    def spin(sec):
        end = time.time() + sec
        while time.time() < end:
            rclpy.spin_once(n, timeout_sec=0.02)

    def box():
        spin(0.3)
        return np.array(g['status']['box_pos'][args.color])

    if not args.no_reset:
        cli = n.create_client(Trigger, '/scene/reset')
        cli.wait_for_service(timeout_sec=5.0)
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
        spin(1.5)
    while 'status' not in g:
        spin(0.1)

    chain = make_chain(args.urdf)
    b0 = box()
    target = b0 + np.array([args.dx, args.dy, args.dz])
    q_grasp, ep, er = solve(chain, target, ARM_HOME)
    q_hover, ep2, er2 = solve(chain, target + [0, 0, 0.06], q_grasp)
    q_lift, ep3, er3 = solve(chain, target + [0, 0, args.lift], q_grasp)
    # 上下の移動は 1cm ごとの経由点にする
    down = [solve(chain, target + [0, 0, h], q_grasp)[0] for h in np.linspace(0.06, 0, 7)[1:]]
    up = [solve(chain, target + [0, 0, h], q_grasp)[0] for h in np.linspace(0, args.lift, 9)[1:]]
    print(f'box0={b0.round(4)}  IK 誤差: grasp {ep*1000:.1f}mm/{er:.1f}deg, hover {ep2*1000:.1f}mm/{er2:.1f}deg, '
          f'lift {ep3*1000:.1f}mm/{er3:.1f}deg')
    print('q_grasp', np.round(q_grasp, 3))

    cur = np.array(ARM_HOME + [args.open], float)  # 腕5関節 + グリッパ

    def move_line(qs, sec, grip):
        """IK で刻んだ経由点を順にたどる（関節角の直線補間だと手先が横に振れて箱に当たるため）"""
        for q in qs:
            move(q, sec / len(qs), grip)

    def move(target, sec, grip):
        """腕とグリッパを一緒に補間する（グリッパに目標値をいきなり送ると、指が勢いよく当たって箱が倒れる）"""
        nonlocal cur
        start, target = cur.copy(), np.array(list(target) + [grip], float)
        steps = int(sec * 30)
        for i in range(1, steps + 1):
            cur = start + (target - start) * i / steps
            pub.publish(Float64MultiArray(data=cur.tolist()))
            spin(1 / 30)

    move(ARM_HOME, 1.0, args.open)
    move(q_hover, 2.0, args.open)
    spin(0.5)
    print(f'箱の上:   box={box().round(4)}')
    move_line(down, 2.0, args.open)
    spin(0.5)
    print(f'降下後:   box={box().round(4)}  gripper={g.get("grip", float("nan")):.3f}')
    if args.stop == 'descend':
        return
    move(q_grasp, 1.5, args.close)
    spin(0.5)
    print(f'閉じた後: box={box().round(4)}  gripper={g.get("grip", float("nan")):.3f}  grasped={g["status"].get("grasped")}')
    if args.stop == 'close':
        return
    move_line(up, 2.5, args.close)
    spin(1.5)
    b1 = box()
    print(f'持ち上げ: box={b1.round(4)}  gripper={g.get("grip", float("nan")):.3f}  '
          f'dz={b1[2] - b0[2]:+.4f}  lifted={g["status"]["lifted"][args.color]}  grasped={g["status"].get("grasped")}')
    move_line(up[::-1][1:] + [q_grasp], 2.0, args.close)
    move(q_hover, 1.0, args.open)
    move(ARM_HOME, 1.5, args.open)


if __name__ == '__main__':
    main()
