"""ikpy で「初期姿勢 → 箱の上 → 降下 → 閉じる → 持ち上げ」の経由点（関節角）を計算する。
掴む姿勢の考え方は scripts/grasp_test.pyと同じ。座標・単位は rad・m。
"""
import warnings

import numpy as np
from ikpy.chain import Chain

warnings.filterwarnings('ignore')  # ikpy が固定ジョイントについて出す警告

ARM_HOME = [0.0, -0.454, 0.138, 1.658, 0.0]   # scene_manager の HOME_POSE の腕5関節
GRIP_HOME = 0.3
# 箱の中心（gripper_link 座標）。値の根拠は scripts/grasp_test.py
GRASP_CENTER = np.array([0.015, 0.0, -0.093])
FRAME_T = np.array([-0.0079, -0.000218121, -0.0981274])
FRAME_R = np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])
GRIPPER_R = np.array([[0, -1.0, 0], [1.0, 0, 0], [0, 0, 1.0]])
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll')
BOX_Z = 0.015  # 箱の中心の高さ（机の上）


def make_chain(urdf):
    chain = Chain.from_urdf_file(urdf, base_elements=['base_link'])
    chain.active_links_mask = np.array([l.name in ARM_JOINTS for l in chain.links])
    return chain


def solve(chain, center, init, keep_orientation=True):
    """箱の中心 center（world）を GRASP_CENTER に合わせる腕5関節の角度、位置誤差 [m] を返す。"""
    g_pos = center - GRIPPER_R @ GRASP_CENTER
    f_pos = g_pos + GRIPPER_R @ FRAME_T
    q0 = np.zeros(len(chain.links))
    q0[1:6] = init
    if keep_orientation:
        q = chain.inverse_kinematics(f_pos, GRIPPER_R @ FRAME_R, orientation_mode='all', initial_position=q0)
    else:
        q = chain.inverse_kinematics(f_pos, initial_position=q0)
    err = np.linalg.norm(chain.forward_kinematics(q)[:3, 3] - f_pos)
    return np.array(q[1:6]), err


def plan(chain, box, hover=0.06, lift=0.08, step=0.01, tol=0.005):
    """box（world の中心）に対する経由点を返す。各要素は腕5関節の配列。

    - grasp：箱を掴む姿勢、hover：箱の真上 hover [m]
    - down：hover → grasp を step ごとに、up：grasp → 持ち上げを step ごとに（手先をまっすぐ上下させる）
    遠い位置では姿勢を保ったまま持ち上げると届かないので、誤差が tol を超えたら位置だけ合わせる。
    """
    box = np.asarray(box, float)
    grasp, err = solve(chain, box, ARM_HOME)
    if err > tol:
        raise ValueError(f'届かない: box={box.round(3)} 誤差 {err * 1000:.1f}mm')
    hover_q, _ = solve(chain, box + [0, 0, hover], grasp)
    down, prev = [], grasp
    for h in np.linspace(hover, 0, int(round(hover / step)) + 1)[:-1]:
        q, _ = solve(chain, box + [0, 0, h], prev)
        down.append(q)
        prev = q
    up, prev = [], grasp
    for h in np.linspace(0, lift, int(round(lift / step)) + 1)[1:]:
        q, e = solve(chain, box + [0, 0, h], prev)
        if e > tol:
            q, e = solve(chain, box + [0, 0, h], prev, keep_orientation=False)
        up.append(q)
        prev = q
    return {'home': np.array(ARM_HOME), 'hover': hover_q, 'down': down, 'grasp': grasp, 'up': up}
