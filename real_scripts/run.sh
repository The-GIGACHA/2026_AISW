#!/bin/bash
# K-City 자율주행 실행 스크립트: 플래너 + 마스터를 함께 실행
# 사용법: ./run.sh   (종료: Ctrl+C)
set -e
cd "$(dirname "$0")"
source /opt/ros/noetic/setup.bash
source ~/taeho_ws/devel/setup.bash   # morai_msgs
export PYTHONUNBUFFERED=1

python3 obstacle_bridge.py &
BRIDGE_PID=$!
python3 lattice_planner_v2.py &
PLANNER_PID=$!
trap "kill $PLANNER_PID $BRIDGE_PID 2>/dev/null" EXIT
python3 master_v2.py
