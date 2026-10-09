#!/bin/bash
# 연습용 스택 실행 (roscore 포함). 사용: run_stack.sh <태그>
source /opt/ros/noetic/setup.bash
source ~/taeho_ws/devel/setup.bash
source ~/aisw_ws/devel/setup.bash
pgrep -f "rosmaster --core" >/dev/null || { roscore > ~/aisw_logs/practice/roscore.log 2>&1 & sleep 4; }
exec roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1 record:=true ${2:-} > ~/aisw_logs/practice/launch_$1.log 2>&1
