#!/usr/bin/env python3
"""ROS トピックを購読して rerun の .rrd に記録する（挙動の目視確認用）。
記録の中身は src/vla_policy/vla_policy/topic_recorder.py（vla_policy_node と共通）。エンティティ名は docs/設計.md 6.3。
policy コンテナで実行する（rerun-sdk・ikpy があるため）。3D 表示には outputs/so101.urdf を使う（grasp_test.py と同じ）。
使い方:
  python scripts/rrd_record.py grasp_red [--duration 20] [--image-hz 10] [--live]
  Ctrl-C で止める。出力は outputs/rrd/<日時>_<名前>.rrd（日時はコンテナの時刻＝UTC）
"""
import argparse
import datetime
import os
import signal
import sys
import time

import rclpy
import rerun as rr

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'vla_policy'))
from vla_policy.topic_recorder import Recorder, open_recording  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('name', nargs='?', default='record', help='ファイル名に付ける名前')
    ap.add_argument('--out-dir', default='outputs/rrd')
    ap.add_argument('--duration', type=float, default=0.0, help='記録する秒数（実時間）。0 なら Ctrl-C まで')
    ap.add_argument('--image-hz', type=float, default=10.0, help='画像を記録する頻度（sim 時間）')
    ap.add_argument('--urdf', default='outputs/so101.urdf')
    ap.add_argument('--live', action='store_true', help='Windows 側の Viewer（ポート 9876）にも送る')
    args = ap.parse_args()

    path = os.path.join(args.out_dir, f'{datetime.datetime.now():%Y%m%d_%H%M%S}_{args.name}.rrd')
    open_recording(path, args.live)

    rclpy.init()
    node = rclpy.create_node('rrd_record')
    Recorder(node, args.urdf, args.image_hz)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt))
    print(f'recording -> {path}（Ctrl-C で終了）', flush=True)
    end = time.time() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (end is None or time.time() < end):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        rr.disconnect()
        node.destroy_node()
        rclpy.try_shutdown()
        print(f'saved {path} ({os.path.getsize(path) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
