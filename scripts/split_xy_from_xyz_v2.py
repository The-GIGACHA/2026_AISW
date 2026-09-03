#!/usr/bin/env python3
# -*- coding: utf-8 -*-
input_file = "/home/yhj/catkin_ws/src/hlfma_morai/map/25hl_global_path_ver3.txt"
output_file = "/home/yhj/catkin_ws/src/hlfma_morai/map/25hl_global_path_ver3_xy_split.txt"

with open(input_file, "r") as f:
    lines = f.readlines()

with open(output_file, "w") as f:
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 2:  # 최소 x, y 값은 있어야 함
            x, y = parts[0], parts[1]
            f.write(f"({x}, {y}),\n")

print(f"좌표 (x, y)만 추출해서 {output_file}에 저장 완료")
