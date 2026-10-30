#!/usr/bin/env bash
# チェックポイントの比較（過学習・学習不足の確認用）。sim を先に起動しておく（scripts/sim_restart.sh）。
# 各チェックポイントの前に scene_manager を同じ seed で起動し直し、箱の配置の並びをそろえる。
# 使い方: bash scripts/eval_ckpt.sh [試行数(赤青の合計。既定10)] [seed(既定0)] [チェックポイントのステップ...(既定 002000 004000 005000)]
# 環境変数 NSTEPS（実行ステップ数。既定 50）・MAXCHUNKS（推論回数。既定 4）・TAG（出力名の接尾辞）で実行のしかたを変えられる
# 出力: outputs/eval/ckpt_<ステップ>.log（1 行 1 試行：color / layout / result）。rrd は outputs/rrd/
cd "$(dirname "$0")/.."
N="${1:-10}"; SEED="${2:-0}"; shift 2 2>/dev/null
STEPS="${*:-002000 004000 005000}"
CFG=config/policy_finetuned.yaml
NSTEPS="${NSTEPS:-50}"; MAXCHUNKS="${MAXCHUNKS:-4}"; TAG="${TAG:-}"
mkdir -p outputs/eval
for st in $STEPS; do
  OUT=outputs/eval/ckpt_$st$TAG.log; : > "$OUT"
  docker compose exec -T sim bash -lc 'pkill -9 -f "[s]cene_manager_node"; sleep 1; true'
  docker compose exec -d sim bash -lc "cd /ws && source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run scene_manager scene_manager_node --ros-args -p use_sim_time:=true -p seed:=$SEED > /tmp/scene_eval.log 2>&1"
  sleep 8
  for i in $(seq 0 $((N - 1))); do
    if [ $((i % 2)) -eq 0 ]; then color=red; else color=blue; fi
    res=$(bash scripts/run_policy.sh $CFG --instruction "pick up the $color box" --max-chunks $MAXCHUNKS --n-action-steps $NSTEPS \
          --model-path outputs/train/so101_smolvla/checkpoints/$st/pretrained_model 2>&1)
    layout=$(echo "$res" | grep -a "reset:" | sed 's/.*reset: //')
    result=$(echo "$res" | grep -a "done:" | sed 's/.*done: \([a-z]*\),.*/\1/')
    rrd=$(echo "$res" | grep -a "^saved" | awk '{print $2}')
    echo "$i $color $layout $result $rrd" | tee -a "$OUT"
  done
done
