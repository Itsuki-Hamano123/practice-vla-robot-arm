#!/usr/bin/env bash
# policy コンテナでデモを収集する。sim を先に起動しておく（scripts/sim_restart.sh）。
# 使い方: bash scripts/collect_demo.sh [--episodes 2] [--root outputs/datasets/so101_pick_box] [--seed 0]
# 出力は LeRobotDataset（outputs/datasets/ 以下。既にあれば追記）。事前に sim コンテナで outputs/so101.urdf を書き出しておく
cd "$(dirname "$0")/.."
docker compose exec -T policy bash -lc "umask 000; cd /ws && source /opt/ros/humble/setup.bash && PYTHONPATH=/ws/src/demo_collector:\$PYTHONPATH python -m demo_collector.collect $(printf '%q ' "$@")"
