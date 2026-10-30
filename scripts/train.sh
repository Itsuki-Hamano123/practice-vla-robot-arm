#!/usr/bin/env bash
# fine-tune：policy コンテナで smolvla_base を収集したデータセットで学習する。GPU マシン用。
# 使い方: bash scripts/train.sh [データセットのパス] [ステップ数] [バッチサイズ] [出力名]
# 引数の意味と値の理由は docs/学習設定.md。出力は outputs/train/<出力名>（チェックポイントは checkpoints/<step>/pretrained_model）。長いので nohup / run_in_background で動かす
cd "$(dirname "$0")/.."
ROOT="${1:-outputs/datasets/so101_pick_box}"
STEPS="${2:-20000}"
BATCH="${3:-32}"
NAME="${4:-so101_smolvla}"
docker compose exec -T policy bash -lc "umask 000; cd /ws && lerobot-train \
  --policy.path=lerobot/smolvla_base --policy.device=cuda --policy.push_to_hub=false \
  --dataset.repo_id=local/so101_pick_box --dataset.root=$ROOT --dataset.video_backend=pyav \
  --rename_map='{\"observation.images.camera1\": \"observation.images.camera1\"}' \
  --batch_size=$BATCH --steps=$STEPS --policy.scheduler_decay_steps=$STEPS --num_workers=4 --save_freq=2000 --log_freq=${LOG_FREQ:-100} --eval_freq=0 \
  --wandb.enable=false --job_name=$NAME --output_dir=outputs/train/$NAME"
