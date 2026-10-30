#!/usr/bin/env bash
# policy コンテナで vla_policy ノードを1エピソード動かす。sim を先に起動しておく（scripts/sim_restart.sh）。
# 使い方: bash scripts/run_policy.sh [config/policy_base.yaml] [--instruction "pick up the blue box"] [--max-chunks 3]
# 出力は outputs/rrd/<日時>_<指示>.rrd。venv の Python で動かすため ros2 run は使わない（src/vla_policy/setup.py）
cd "$(dirname "$0")/.."
docker compose exec -T policy bash -lc "umask 000; cd /ws && source /opt/ros/humble/setup.bash && PYTHONPATH=/ws/src/vla_policy:\$PYTHONPATH python -m vla_policy.vla_policy_node $(printf '%q ' "$@")"
