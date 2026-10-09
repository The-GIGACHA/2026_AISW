#!/bin/bash
# 연습용 스택 실행 (roscore 포함). 사용: run_stack.sh <태그> [추가 roslaunch 인자...]
# 먼저 ROS·morai_msgs·aisw_2026 워크스페이스를 source 해 둘 것 (PC 마다 경로가 달라 여기서 source 하지 않는다).
#   예) source /opt/ros/noetic/setup.bash; source <morai_msgs 워크스페이스>/devel/setup.bash; source <aisw 워크스페이스>/devel/setup.bash
# 환경변수: SIM_IP (기본 127.0.0.1), AISW_LOG_DIR (기본 ~/aisw_logs)
set -e
rospack find aisw_2026 >/dev/null 2>&1 || { echo "aisw_2026 패키지를 찾을 수 없음 — 워크스페이스를 먼저 source 하세요"; exit 1; }
LOG="${AISW_LOG_DIR:-$HOME/aisw_logs}/practice"; mkdir -p "$LOG"
TAG="${1:?태그 필요 (예: r01)}"; shift
pgrep -f "rosmaster --core" >/dev/null || { roscore > "$LOG/roscore.log" 2>&1 & sleep 4; }
exec roslaunch aisw_2026 aisw_midterm.launch sim_ip:="${SIM_IP:-127.0.0.1}" record:=true "$@" > "$LOG/launch_$TAG.log" 2>&1
