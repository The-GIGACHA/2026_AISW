#!/bin/bash
# 실행 중인 aisw 스택 정지 (roscore 유지)
for p in $(ps -eo pid,args | awk '/bin\/roslaunch aisw_2026/ && !/awk/ {print $1}'); do kill -INT $p; done
for i in $(seq 1 30); do ps -eo args | grep -q "[r]oslaunch aisw_2026" || break; sleep 0.5; done
for p in $(ps -eo pid,args | awk '/aisw_2026\/(scripts|tools)\/.*\.py/ && !/awk/ {print $1}'); do kill $p; done
echo stopped
