#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
_MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map')   # 패키지 map/ (PC 경로 하드코딩 제거)

input_file = os.path.join(_MAP_DIR, "25hl_global_path_ver3.txt")
output_file = os.path.join(_MAP_DIR, "25hl_global_path_ver3_xy_split.txt")

with open(input_file, "r") as f:
    lines = f.readlines()

with open(output_file, "w") as f:
    for line in lines:
        parts = line.strip().split()
        if len(parts) >= 2:  # 최소 x, y 값은 있어야 함
            x, y = parts[0], parts[1]
            f.write(f"({x}, {y}),\n")

print(f"좌표 (x, y)만 추출해서 {output_file}에 저장 완료")
