#!/usr/bin/env bash
# 6関節の目標角 [rad] を1回送る（動作確認用）。例: bash scripts/cmd.sh 0 0 0 0 0 0
cd "$(dirname "$0")/.."
docker compose exec -T sim bash -lc "source /opt/ros/humble/setup.bash; ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [$(echo "$@" | sed 's/ /, /g')]}'" | tail -1
