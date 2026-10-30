"""SmolVLA で SO-101 を動かす推論ノード（docs/設計.md 6章）。Phase A / B 共通で、設定ファイルでモデルを切り替える。

制御ループ（6.2 の同期モード）：
  行動キューが空なら sim を一時停止 → 最新の観測で推論 → 行動チャンクをキューへ → 再開
  キューから1ステップずつ /arm_controller/commands へ送る（control_hz）
指示文は /vla/instruction で実行中に切り替えられる（切り替えたらキューを捨てて推論し直す）。
1エピソード（max_chunks 回の推論）で終わり、outputs/rrd/<日時>_<指示>.rrd を残す。

policy コンテナで実行する：bash scripts/run_policy.sh [config/policy_base.yaml] [--instruction "..."]
"""
import argparse
import datetime
import json
import os
import re
import time

import numpy as np
import rclpy
import rerun as rr
import yaml
from sensor_msgs.msg import CompressedImage, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Empty, Trigger

from .adapters import JOINTS, ActionAdapter, ObsAdapter
from .rerun_logger import RerunLogger
from .topic_recorder import Recorder, open_recording


class VlaPolicyNode:
    def __init__(self, cfg):
        self.cfg = cfg
        self.node = rclpy.create_node('vla_policy')
        self.log = self.node.get_logger()
        self.joints = None
        self.images = {}
        self.status = None
        self.instruction = cfg['instruction']
        self.instruction_changed = False
        for topic in cfg['camera_map']:
            self.node.create_subscription(CompressedImage, topic, lambda m, t=topic: self.images.__setitem__(t, m), 1)
        self.node.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'joints', m), 1)
        self.node.create_subscription(String, '/scene/status', lambda m: setattr(self, 'status', json.loads(m.data)), 1)
        self.node.create_subscription(String, '/vla/instruction', self._on_instruction, 10)
        self.cmd_pub = self.node.create_publisher(Float64MultiArray, '/arm_controller/commands', 10)
        self.inst_pub = self.node.create_publisher(String, '/vla/instruction', 10)
        self.pause = self.node.create_client(Empty, '/pause_physics')
        self.unpause = self.node.create_client(Empty, '/unpause_physics')
        self.reset = self.node.create_client(Trigger, '/scene/reset')

        self.obs_adapter = ObsAdapter(cfg['camera_map'], cfg['state_unit'], cfg.get('image_size'))
        self.action_adapter = ActionAdapter(cfg['state_unit'], cfg.get('max_step'))
        self.recorder = Recorder(self.node, cfg['rerun'].get('urdf', 'outputs/so101.urdf'))
        self.rerun = RerunLogger(self.recorder)

        from .policy_runner import PolicyRunner  # lerobot の import に時間がかかるので、ROS の準備の後にする
        t = time.time()
        self.runner = PolicyRunner(cfg['model_path'], cfg['device'], cfg.get('norm_stats'), cfg.get('num_threads'))
        self.log.info(f'model loaded: {cfg["model_path"]} ({time.time() - t:.1f}s)')

    def _on_instruction(self, msg):
        if msg.data != self.instruction:
            self.log.info(f'instruction: "{self.instruction}" -> "{msg.data}"')
            self.instruction = msg.data
            self.instruction_changed = True

    def spin(self, sec):
        end = time.monotonic() + sec
        while True:
            rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.01, end - time.monotonic())))
            if time.monotonic() >= end:
                return

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'{client.srv_name} が使えない')
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result()

    def wait_obs(self, timeout=10.0):
        """リセット後の観測（関節・全カメラ）が揃うまで待つ。古いメッセージを使わないよう、一度捨てる。"""
        self.joints, self.images = None, {}
        end = time.monotonic() + timeout
        while self.joints is None or len(self.images) < len(self.cfg['camera_map']):
            if time.monotonic() > end:
                raise RuntimeError('観測が揃わない（sim は起動しているか、カメラは出ているか）')
            self.spin(0.1)

    def infer(self):
        sync = self.cfg['sync_mode']
        if sync:
            self.call(self.pause, Empty.Request())
        try:
            state, images = self.obs_adapter(self.joints, self.images)
            t = time.time()
            chunk = self.runner.infer(state, images, self.instruction)
            ms = (time.time() - t) * 1000
            chunk_rad = self.action_adapter.to_rad(chunk[:self.cfg['n_action_steps']])
            self.rerun.log_inference(state, images, chunk_rad, self.cfg['control_hz'], ms)
            self.log.info(f'inference {ms:.0f} ms, chunk[0]={np.round(chunk_rad[0], 3).tolist()} rad')
            return chunk_rad, ms
        finally:
            if sync:
                self.call(self.unpause, Empty.Request())

    def run(self):
        cfg = self.cfg
        if cfg.get('reset', True):
            res = self.call(self.reset, Trigger.Request())
            self.log.info(f'reset: {res.message}')
            self.spin(1.5)
        self.wait_obs()
        self.inst_pub.publish(String(data=self.instruction))  # scene_manager の成功判定に使う
        self.spin(0.3)
        self.instruction_changed = False
        cur = np.array([dict(zip(self.joints.name, self.joints.position))[j] for j in JOINTS])
        dt = 1.0 / cfg['control_hz']
        times = []
        result = 'timeout'
        for i in range(cfg['max_chunks']):
            chunk, ms = self.infer()
            times.append(ms)
            self.instruction_changed = False
            next_t = time.monotonic()
            for target in chunk:
                cur = self.action_adapter(target, cur)
                self.cmd_pub.publish(Float64MultiArray(data=cur.tolist()))
                next_t += dt
                self.spin(max(0.0, next_t - time.monotonic()))
                if self.instruction_changed:
                    break  # 指示が変わったら残りのチャンクを捨てて推論し直す
            s = self.status or {}
            self.log.info(f'chunk {i + 1}/{cfg["max_chunks"]}: target={s.get("target")} lifted={s.get("lifted")} '
                          f'grasped={s.get("grasped")} success={s.get("success")} wrong={s.get("wrong")}')
            if cfg.get('stop_on_done', True) and (s.get('success') or s.get('wrong')):
                result = 'success' if s.get('success') else 'wrong'
                break
        self.log.info(f'done: {result}, inference ms mean={np.mean(times):.0f} min={np.min(times):.0f} '
                      f'max={np.max(times):.0f} (n={len(times)})')
        self.spin(0.5)  # 最後のログを /rosout 経由で記録させる
        return result


def slug(text):
    return re.sub(r'[^a-z0-9]+', '_', text.lower()).strip('_')[:40]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('config', nargs='?', default='config/policy_base.yaml')
    ap.add_argument('--instruction', help='設定ファイルの instruction を上書きする')
    ap.add_argument('--max-chunks', type=int, help='推論の回数（1エピソードの長さ）')
    ap.add_argument('--model-path', help='設定ファイルの model_path を上書きする（チェックポイントの比較用）')
    ap.add_argument('--n-action-steps', type=int, help='1 回の推論のうち実行するステップ数（設定ファイルを上書き）')
    ap.add_argument('--live', action='store_true', help='Viewer（ポート 9876）にも送る')
    args = ap.parse_args()
    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    if args.instruction:
        cfg['instruction'] = args.instruction
    if args.model_path:
        cfg['model_path'] = args.model_path
    if args.n_action_steps:
        cfg['n_action_steps'] = args.n_action_steps
    if args.max_chunks:
        cfg['max_chunks'] = args.max_chunks

    path = os.path.join(cfg['rerun']['save_dir'],
                        f'{datetime.datetime.now():%Y%m%d_%H%M%S}_{slug(cfg["instruction"])}.rrd')
    open_recording(path, args.live or cfg['rerun'].get('live', False))
    rclpy.init()
    node = VlaPolicyNode(cfg)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if cfg['sync_mode'] and rclpy.ok():
            node.call(node.unpause, Empty.Request())  # 一時停止したまま終わらないようにする
        rr.disconnect()
        node.node.destroy_node()
        rclpy.try_shutdown()
        print(f'saved {path} ({os.path.getsize(path) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
