#!/bin/bash
# K-City 자율주행 실행 스크립트: 플래너 + 마스터를 함께 실행
# 사용법: ./run.sh   (종료: Ctrl+C)
set -e
cd "$(dirname "$0")"
# ROS·morai_msgs 워크스페이스는 실행 전에 직접 source (PC 마다 경로가 달라 여기서 고정하지 않음)
[ -n "$ROS_DISTRO" ] || source /opt/ros/noetic/setup.bash
python3 -c "import morai_msgs" 2>/dev/null || { echo "morai_msgs 없음 — 워크스페이스를 먼저 source 하세요"; exit 1; }
export PYTHONUNBUFFERED=1

python3 obstacle_bridge.py &
BRIDGE_PID=$!
python3 lattice_planner_v2.py &
PLANNER_PID=$!
trap "kill $PLANNER_PID $BRIDGE_PID 2>/dev/null" EXIT
python3 master_v2.py
