#!/bin/bash
# 회전교차로 진입 장시간 실험: 탐색(EXPLORE_H 시간, BLOCK_H 마다 재학습) → 평가(EVAL_H 시간, 학습 모델 vs 기본 규칙 번갈아)
# 사용: run_long.sh [탐색 시간=8] [평가 시간=2]   (먼저 source ~/26aisw_control_ws/devel/setup.bash)
# MORAI: 시나리오 rb_entry_mix.json Load, Object Info = UDP 7505, Multi Ego Setting = UDP 7604
EXPLORE_H="${1:-8}"; EVAL_H="${2:-2}"; BLOCK_H="${BLOCK_H:-2}"
HERE="$(cd "$(dirname "$0")" && pwd)"; PKG="$(cd "$HERE/../../.." && pwd)"
OUT="${AISW_LOG_DIR:-$HOME/aisw_logs}/roundabout_$(date +%m%d_%H%M)"; mkdir -p "$OUT"
rospack find aisw_2026 >/dev/null 2>&1 || { echo "source ~/26aisw_control_ws/devel/setup.bash 먼저"; exit 1; }
exec > >(tee -a "$OUT/run.log") 2>&1

stack_up() {
  roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1 obstacles:=morai >> "$OUT/launch.log" 2>&1 &
  sleep 25
}
stack_down() {
  for p in $(pgrep -f "bin/roslaunch aisw_2026 aisw_midterm"); do kill -INT $p; done
  for i in $(seq 1 40); do pgrep -f "bin/roslaunch aisw_2026 aisw_midterm" >/dev/null || break; sleep 0.5; done
}
run_block() {   # $1 mode, $2 시간
  local end=$(( $(date +%s) + $(python3 -c "print(int($2 * 3600))") )) seed=$RANDOM
  while [ "$(date +%s)" -lt "$end" ]; do
    local left=$(python3 -c "print(max(0.01, ($end - $(date +%s)) / 3600))")
    python3 "$HERE/run_episodes.py" --mode "$1" --hours "$left" --seed "$seed" --out "$OUT/$1.jsonl"
    [ $? -eq 0 ] && return 0
    echo "=== $(date '+%F %T') 스택 재시작"; stack_down; stack_up; seed=$((seed + 1))
  done
}

systemd-inhibit --what=idle:sleep --who=rb_entry --why="roundabout entry experiment" \
  sleep $(python3 -c "print(int(($EXPLORE_H + $EVAL_H + 1) * 3600))") &
INHIBIT=$!
trap 'stack_down; rosparam delete /aisw/roundabout/trial 2>/dev/null; kill $INHIBIT 2>/dev/null' EXIT

stack_down; stack_up
left=$EXPLORE_H; k=1
while python3 -c "import sys; sys.exit(0 if $left > 0.01 else 1)"; do
  blk=$(python3 -c "print(min($BLOCK_H, $left))")
  echo "=== $(date '+%F %T') 탐색 블록 $k (${blk} h)"
  run_block explore "$blk"
  python3 "$HERE/train_entry.py" "$OUT/explore.jsonl" --out "$OUT/roundabout_entry_v$k.npz" | tee "$OUT/train_v$k.log"
  python3 "$HERE/report.py" "$OUT/explore.jsonl" | tee "$OUT/report_v$k.txt"
  left=$(python3 -c "print($left - $blk)"); k=$((k + 1))
done

echo "=== $(date '+%F %T') 평가 (${EVAL_H} h): 학습 모델 vs 기본 규칙"
last=$(ls -t "$OUT"/roundabout_entry_v*.npz 2>/dev/null | head -1)
if [ -n "$last" ]; then
  mkdir -p "$PKG/models" && cp "$last" "$PKG/models/roundabout_entry.npz"
  rosnode kill /master; sleep 8      # respawn → 새 모델 로드
fi
run_block eval "$EVAL_H"
python3 "$HERE/report.py" "$OUT/explore.jsonl" "$OUT/eval.jsonl" | tee "$OUT/report_final.txt"
echo "=== $(date '+%F %T') 완료: $OUT"
