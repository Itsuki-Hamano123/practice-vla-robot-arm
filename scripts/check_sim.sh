#!/usr/bin/env bash
# sim の確認用。WSL 上のリポジトリルートで `bash scripts/check_sim.sh` を実行する。
# 事前に `docker compose up -d --build` と `bash scripts/sim_restart.sh`（起動に約1分）を済ませておく。
set -u
cd "$(dirname "$0")/.."
SRC='source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; cd /ws'

echo "== 1. コントローラ =="
docker compose exec -T sim bash -lc "$SRC; ros2 control list_controllers"

echo "== 2. 関節が指令どおり動くか（pan=0.5, gripper=0.8 を指令）=="
docker compose exec -T sim bash -lc "$SRC; ros2 topic pub --once /arm_controller/commands std_msgs/msg/Float64MultiArray '{data: [0.5, 0, 0, 0, 0, 0.8]}' >/dev/null; sleep 2; ros2 topic echo --once /joint_states --field position"

echo "== 3. カメラ画像（PNG は outputs/ に保存）=="
mkdir -p outputs
docker compose exec -T sim bash -lc "$SRC; python3 scripts/save_image.py /cam_front/image_raw outputs/cam_front.png; python3 scripts/save_image.py /cam_wrist/image_raw outputs/cam_wrist.png"

echo "== 4. リセットで赤青の左右が入れ替わるか（5回）=="
docker compose exec -T sim bash -lc "$SRC; for i in 1 2 3 4 5; do ros2 service call /scene/reset std_srvs/srv/Trigger | grep -o 'message=.*'; done; ros2 topic echo --once /scene/status --field data"
