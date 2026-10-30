#!/usr/bin/env python3
"""LeRobotDataset のエピソードを rerun の .rrd に変換する（収集したデータの目視確認用）。
学習で実際に使われる中身（動画をデコードした画像・state・action）を、設計.md 6.3 と同じエンティティ名で書く。
3D のアームは state（度→rad）から ikpy の FK で再計算する。箱の位置・grasp check のログはデータセットに無いので出ない。
policy コンテナで実行する。
使い方:
  python scripts/dataset_to_rrd.py [--root outputs/datasets/so101_pick_box] [--episodes 0 1 | --all] [--out-dir outputs/rrd/dataset]
出力は <out-dir>/episode_000_<指示>.rrd。Windows 側の Viewer 0.26.2 で開く。時間軸は sim_time（フレーム番号 / fps）。
"""
import argparse
import os
import sys

import numpy as np
import rerun as rr
import torch
from lerobot.datasets.lerobot_dataset import LeRobotDataset

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src', 'vla_policy'))
from vla_policy.topic_recorder import JOINTS, open_recording  # noqa: E402


def to_image(t):
    """(3,H,W) float [0,1] → (H,W,3) uint8"""
    return (t.permute(1, 2, 0).clamp(0, 1) * 255).to(torch.uint8).numpy()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--root', default='outputs/datasets/so101_pick_box')
    ap.add_argument('--repo-id', default='local/so101_pick_box')
    ap.add_argument('--episodes', type=int, nargs='*', default=[0])
    ap.add_argument('--all', action='store_true', help='全エピソードを変換する')
    ap.add_argument('--out-dir', default='outputs/rrd/dataset')
    ap.add_argument('--urdf', default='outputs/so101.urdf')
    args = ap.parse_args()

    chain = None
    if os.path.exists(args.urdf):
        from ikpy.chain import Chain
        chain = Chain.from_urdf_file(args.urdf, base_elements=['base_link'])
    ds = LeRobotDataset(args.repo_id, root=args.root, video_backend='pyav')
    episodes = list(range(ds.num_episodes)) if args.all else args.episodes

    for ep in episodes:
        # episodes= で絞ると 0.4.0 では全フレームが残ったので、エピソードの範囲を自分で切り出す
        meta = ds.meta.episodes[ep]
        start, end = int(meta['dataset_from_index']), int(meta['dataset_to_index'])
        task = ds[start]['task']
        path = os.path.join(args.out_dir, f'episode_{ep:03d}_{task.replace(" ", "_")}.rrd')
        open_recording(path)
        rr.log('instruction', rr.TextDocument(task), static=True)
        for i in range(start, end):
            item = ds[i]
            rr.set_time('sim_time', duration=float(item['timestamp']))
            rr.log('obs/cam_front', rr.Image(to_image(item['observation.images.camera1'])).compress(jpeg_quality=90))
            rr.log('obs/cam_wrist', rr.Image(to_image(item['observation.images.camera2'])).compress(jpeg_quality=90))
            state = np.radians(item['observation.state'].numpy())   # データセットは度。panel は rad
            action = np.radians(item['action'].numpy())
            for j, name in enumerate(JOINTS):
                rr.log(f'obs/state/{name}', rr.Scalars(state[j]))
                rr.log(f'action/cmd/{name}', rr.Scalars(action[j]))
            if chain is not None:
                q = dict(zip(JOINTS, state))
                frames = chain.forward_kinematics([q.get(l.name, 0.0) for l in chain.links], full_kinematics=True)
                pts = np.array([f[:3, 3] for f in frames])
                rr.log('world/arm', rr.LineStrips3D([pts], colors=[[200, 200, 200]], radii=0.004))
                rr.log('world/gripper_tip', rr.Points3D([pts[-1]], colors=[[255, 200, 0]], radii=0.008))
        print(f'episode {ep}: {end - start} フレーム → {path} ({os.path.getsize(path) / 1e6:.1f}MB)', flush=True)


if __name__ == '__main__':
    main()
