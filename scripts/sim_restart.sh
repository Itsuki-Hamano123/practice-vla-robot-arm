#!/usr/bin/env bash
# sim コンテナ内の Gazebo 関連プロセスを止めて、ヘッドレスで起動し直す（開発用）。
# 使い方: bash scripts/sim_restart.sh [gui:=true]   ログは sim コンテナの /tmp/sim.log
cd "$(dirname "$0")/.."
ARGS="${*:-gui:=false}"
docker compose exec -T sim bash -lc 'for p in "[g]zserver" "[g]zclient" "[r]os2 launch" "[r]obot_state_publisher" "[s]cene_manager_node" "[s]pawn_entity" "[r]os2 control load_controller"; do pkill -9 -f "$p"; done; sleep 2; true'
docker compose exec -d sim bash -lc "cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch so101_gazebo sim.launch.py $ARGS > /tmp/sim.log 2>&1"
