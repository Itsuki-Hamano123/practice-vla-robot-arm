#!/usr/bin/env bash
# fine-tune のスモークテスト：policy コンテナで CPU 学習を数ステップ回し、データセットの形式の不整合がないか確認する。
# 使い方: bash scripts/smoke_train.sh [データセットのパス] [ステップ数]
cd "$(dirname "$0")/.."
ROOT="${1:-outputs/datasets/smoke}"
STEPS="${2:-3}"
docker compose exec -T policy bash -lc "umask 000; cd /ws && rm -rf outputs/train_smoke && lerobot-train \
  --policy.path=lerobot/smolvla_base --policy.device=cpu --policy.push_to_hub=false \
  --dataset.repo_id=local/so101_pick_box --dataset.root=$ROOT \
  --rename_map='{\"observation.images.camera1\": \"observation.images.camera1\"}' \
  --batch_size=2 --steps=$STEPS --num_workers=0 --save_freq=$STEPS --log_freq=1 --eval_freq=0 \
  --wandb.enable=false --output_dir=outputs/train_smoke"
