#!/usr/bin/env bash
# 環境の確認用。WSL 上のリポジトリルートで `bash scripts/check_env.sh` を実行する。
# 事前に `docker compose up -d --build` を済ませておく。
set -u
cd "$(dirname "$0")/.."

echo "== 1. sim：Gazebo の版 =="
docker compose exec -T sim bash -lc 'which gazebo gz 2>&1; gazebo --version 2>&1 | head -3; gz sim --version 2>&1 | head -3; printenv ROS_DISTRO'

echo "== 2. policy：Python / torch / lerobot / rerun =="
docker compose exec -T policy bash -lc '
source /opt/ros/humble/setup.bash
python --version
python -c "import rclpy; print(\"rclpy OK\")"
python -c "import torch, lerobot, rerun; print(\"torch\", torch.__version__, \"lerobot\", lerobot.__version__, \"rerun\", rerun.__version__)"
python -c "from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy; print(\"SmolVLAPolicy import OK\")"'

echo "== 3. WSLg：GUI（sim から xeyes 相当で確認）=="
docker compose exec -T sim bash -lc 'echo DISPLAY=$DISPLAY; ls /mnt/wslg 2>&1 | head -3; (command -v xdpyinfo >/dev/null && xdpyinfo | head -3) || echo "xdpyinfo なし。gazebo を起動して目視で確認"'

echo "== 4. コンテナ間の ROS 通信（sim で publish → policy で受信）=="
docker compose exec -d sim bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic pub -r 5 /env_check std_msgs/msg/String "{data: hello}"'
sleep 3
docker compose exec -T policy bash -lc 'source /opt/ros/humble/setup.bash; timeout 10 ros2 topic echo --once /env_check'
docker compose exec -T sim bash -lc 'pkill -f "[t]opic pub" || true'
