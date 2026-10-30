"""自動デモ収集（docs/設計.md 8.1）。sim を動かしておき、policy コンテナの venv で実行する（scripts/collect_demo.sh）。

1 エピソード：/scene/reset → 指示文を送る → ikpy の経由点を sim 時間 10Hz で補間して /arm_controller/commands へ送りながら、
フレーム（画像2枚・関節角・指令）を LeRobotDataset へ書く → 成功（success かつ not wrong）なら保存、失敗なら捨てる。

データセットの形式（fine-tune と smolvla_base の入力に合わせる。docs/設計.md 6.4）：
  observation.state / action：6 次元、単位は度（rad から変換）。action は、そのフレームで送った関節指令
  observation.images.camera1 = 正面カメラ、camera2 = 手首カメラ（mp4 で保存）
  task = "pick up the red box" / "pick up the blue box"
  fps = 10（sim 時間。フレームは正面カメラの画像が届くたびに 1 つ、t = フレーム番号 / fps で指令を決める）
"""
import argparse
import io
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

from demo_collector.ik_plan import BOX_Z, GRIP_HOME, make_chain, plan

JOINTS = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper']
COLORS = ('red', 'blue')
GRIP_OPEN, GRIP_CLOSED = 1.0, 0.0   # [rad]。閉じる目標は scene_manager の grip_close（0.25）より小さくする
FRONT, WRIST = '/cam_front/image_raw/compressed', '/cam_wrist/image_raw/compressed'


def stamp_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def decode(msg):
    return np.asarray(Image.open(io.BytesIO(bytes(msg.data))).convert('RGB'))


def trajectory(p, rng):
    """経由点 p（ik_plan.plan）から、時刻 t [s] → 6 関節の指令 [rad] の関数を作る。各区間の長さに ±15% のずれを入れる。"""
    def dur(sec):
        return sec * rng.uniform(0.85, 1.15)

    key = lambda q, g: np.append(q, g)
    home = key(p['home'], GRIP_HOME)
    # (区間の長さ, 区間の終わりの姿勢のリスト)。区間内は姿勢のリストを等時間で順にたどる
    segs = [(dur(0.5), [home]),
            (dur(2.0), [key(p['hover'], GRIP_OPEN)]),
            (dur(2.0), [key(q, GRIP_OPEN) for q in p['down']] + [key(p['grasp'], GRIP_OPEN)]),
            (dur(0.5), [key(p['grasp'], GRIP_OPEN)]),
            (dur(1.5), [key(p['grasp'], GRIP_CLOSED)]),
            (dur(0.3), [key(p['grasp'], GRIP_CLOSED)]),
            (dur(2.5), [key(q, GRIP_CLOSED) for q in p['up']]),
            (dur(1.0), [key(p['up'][-1], GRIP_CLOSED)])]
    knots, t = [(0.0, home)], 0.0
    for sec, qs in segs:
        for q in qs:
            t += sec / len(qs)
            knots.append((t, q))
    ts = np.array([k[0] for k in knots])
    qs = np.array([k[1] for k in knots])

    def at(t):
        return np.array([np.interp(t, ts, qs[:, j]) for j in range(6)])
    return at, ts[-1]


class Collector(Node):
    def __init__(self):
        super().__init__('demo_collector')
        self.state = None
        self.status = None
        self.front = None   # (stamp, msg)
        self.wrist = None
        self.n_front = 0
        self.create_subscription(JointState, '/joint_states', self._on_joints, 1)
        self.create_subscription(String, '/scene/status', lambda m: setattr(self, 'status', json.loads(m.data)), 1)
        self.create_subscription(CompressedImage, FRONT, self._on_front, 1)
        self.create_subscription(CompressedImage, WRIST, lambda m: setattr(self, 'wrist', m), 1)
        self.cmd_pub = self.create_publisher(Float64MultiArray, '/arm_controller/commands', 1)
        self.instr_pub = self.create_publisher(String, '/vla/instruction', 1)
        self.reset_cli = self.create_client(Trigger, '/scene/reset')

    def _on_joints(self, msg):
        self.state = np.array([msg.position[msg.name.index(j)] for j in JOINTS])

    def _on_front(self, msg):
        self.front = msg
        self.n_front += 1

    def spin_until(self, cond, timeout):
        end = time.time() + timeout
        while not cond():
            if time.time() > end:
                raise TimeoutError('待ち時間切れ')
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset(self, color):
        """リセットして、指示文を送り、リセット後の画像が来るまで待つ。"""
        self.reset_cli.wait_for_service(timeout_sec=5.0)
        fut = self.reset_cli.call_async(Trigger.Request())
        self.spin_until(fut.done, 15.0)
        if not fut.result().success:
            raise RuntimeError(fut.result().message)
        self.instr_pub.publish(String(data=f'pick up the {color} box'))
        n = self.n_front
        self.spin_until(lambda: self.n_front >= n + 3, 30.0)   # リセット前の画像を捨てる
        self.status = None
        self.spin_until(lambda: self.status is not None and self.status['target'] == color, 5.0)
        return self.status

    def run_episode(self, fps, color, plans, rng, max_sec=20.0):
        """1 エピソードを動かして記録する。（フレームのリスト, task, 成功したか, メモ）を返す。書き込みは呼び出し側。

        max_sec は sim 時間での上限 [s]。画像レートは CPU / GPU で違う（約 10Hz / 30Hz）ので、枚数ではなく時間で打ち切る。
        軌道（trajectory）は最長でも約 11.9 秒なので、通常は軌道の終わりで止まる。
        """
        status = self.reset(color)
        box = np.array(status['box_pos'][color])
        box[2] = BOX_Z
        p = plan(plans, box)
        at, t_end = trajectory(p, rng)
        task = f'pick up the {color} box'
        raw = []   # (stamp, state, action, front, wrist)。届いた画像ごとに 1 つ
        t0 = None
        last = self.front
        while True:
            self.spin_until(lambda: self.front is not last, 30.0)
            front = last = self.front
            stamp = stamp_sec(front.header.stamp)
            self.spin_until(lambda: self.wrist is not None and stamp_sec(self.wrist.header.stamp) >= stamp - 1e-6, 30.0)
            t0 = stamp if t0 is None else t0
            if stamp - t0 > min(t_end, max_sec):
                break
            # 画像が届いた sim 時刻に合わせて指令を決める（画像の間隔は一定ではないので、フレーム番号ではなく stamp で決める）
            action = at(stamp - t0)
            self.cmd_pub.publish(Float64MultiArray(data=action.tolist()))
            # デコードと書き込みは遅く（1 フレーム数百 ms）、その間に画像を取りこぼすので、エピソードの後でまとめて行う
            raw.append((stamp - t0, self.state.copy(), action, front, self.wrist))
        # 画像の間隔が 0.1 秒にそろわない（0.08〜0.25 秒）ので、sim 時刻が k/fps に最も近いフレームを選んで fps に合わせる
        ts = np.array([r[0] for r in raw])
        grid = np.arange(0.0, ts[-1], 1.0 / fps)
        pick = [int(np.abs(ts - g).argmin()) for g in grid]
        stamps = ts[pick]
        frames = [raw[i][1:] for i in pick]
        idx = len(frames)
        rclpy.spin_once(self, timeout_sec=0.5)
        st = self.status
        ok = bool(st['success'] and not st['wrong'])
        info = {'color': color, 'layout': st['layout'], 'box': box.round(3).tolist(), 'frames': idx,
                't_end': round(t_end, 2), 'truncated': bool(t_end > max_sec),
                'raw_frames': len(raw), 'duplicated': idx - len(set(pick)), 'raw_dt_mean': float(np.diff(ts).mean()),
                'raw_dt_max': float(np.diff(ts).max()), 'pick_err_max': float(np.abs(stamps - grid).max()),
                'success': st['success'], 'wrong': st['wrong'], 'box_z': st['box_pos'][color][2]}
        return frames, task, ok, info


def write_episode(ds, frames, task):
    """フレームをデコードして書き込み、エピソードを保存する。"""
    for state, action, front, wrist in frames:
        ds.add_frame({
            'observation.state': np.degrees(state).astype(np.float32),
            'action': np.degrees(action).astype(np.float32),
            'observation.images.camera1': decode(front),
            'observation.images.camera2': decode(wrist),
            'task': task,
        })
    ds.save_episode()


def open_dataset(root, repo_id, fps, size):
    h, w = size
    root = Path(root)
    if root.exists():
        return LeRobotDataset(repo_id, root=root)
    names = ['height', 'width', 'channels']
    return LeRobotDataset.create(
        repo_id, fps, root=root, robot_type='so101', use_videos=True,
        features={
            'observation.state': {'dtype': 'float32', 'shape': (6,), 'names': JOINTS},
            'action': {'dtype': 'float32', 'shape': (6,), 'names': JOINTS},
            'observation.images.camera1': {'dtype': 'video', 'shape': (h, w, 3), 'names': names},
            'observation.images.camera2': {'dtype': 'video', 'shape': (h, w, 3), 'names': names},
        })


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=1, help='成功させたいエピソード数')
    ap.add_argument('--max-attempts', type=int, default=None, help='試行回数の上限（既定は episodes の 2 倍）')
    ap.add_argument('--root', default='outputs/datasets/so101_pick_box', help='データセットの置き場所（無ければ作る。あれば追記する）')
    ap.add_argument('--max-duplicates', type=int, default=10,
                    help='10Hz にそろえるときの重複フレームがこれを超えたエピソードは採用しない（画像が途切れて止まって見えるため）')
    ap.add_argument('--repo-id', default='local/so101_pick_box')
    ap.add_argument('--fps', type=int, default=10)
    ap.add_argument('--urdf', default='outputs/so101.urdf')
    ap.add_argument('--seed', type=int, default=0)
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    rng = np.random.default_rng(args.seed)
    chain = make_chain(args.urdf)
    node.spin_until(lambda: node.front is not None and node.wrist is not None and node.state is not None, 60.0)
    size = decode(node.front).shape[:2]
    ds = open_dataset(args.root, args.repo_id, args.fps, size)
    rejected = None   # 採用しなかったエピソードは <root>_rejected に同じ形式で残す（初めて出たときに作る）
    n_rej = 0
    saved, tries = 0, 0
    max_tries = args.max_attempts or args.episodes * 2
    while saved < args.episodes and tries < max_tries:
        # 赤・青を交互にする（左右の入れ替えは scene_manager がランダムに行う）
        color = COLORS[ds.num_episodes % 2]
        tries += 1
        t0 = time.time()
        try:
            frames, task, ok, info = node.run_episode(args.fps, color, chain, rng)
        except (ValueError, TimeoutError) as e:
            node.get_logger().error(f'試行 {tries} 失敗: {e}')
            continue
        info['wall_sec'] = round(time.time() - t0, 1)
        reason = None if ok and info['duplicated'] <= args.max_duplicates else ('not_success' if not ok else 'duplicates')
        if reason is None:
            write_episode(ds, frames, task)
            saved += 1
        else:
            if rejected is None:
                rejected = open_dataset(args.root.rstrip('/') + '_rejected', args.repo_id + '_rejected', args.fps, size)
            write_episode(rejected, frames, task)
            n_rej += 1
        info['saved'] = reason is None
        info['reject_reason'] = reason
        print(json.dumps(info, ensure_ascii=False), flush=True)
    ds.finalize()
    if rejected is not None:
        rejected.finalize()
    print(f'保存 {saved} / 試行 {tries}（不採用 {n_rej}）、データセット：{args.root}（計 {ds.num_episodes} エピソード）')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
