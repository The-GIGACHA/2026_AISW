#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

# 입력/출력 경로 설정
IN_PATH = "/home/yhj/catkin_ws/src/hlfma_morai/map/25hl_global_path_ver2.txt"
# IN_PATH = "/home/yhj/catkin_ws/src/hlfma_morai/map/Sangam_1_center_pts_1_2_route_xy_split.txt"
OUT_PATH = os.path.join(os.path.dirname(IN_PATH), "25hl_global_path_ver2_xy_split.txt")

def main():
    xs, ys = [], []
    with open(IN_PATH, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            # 쉼표/공백 혼합 대비
            line = line.replace(",", " ")
            parts = line.split()
            if len(parts) < 2:
                continue  # x, y 둘 다 없으면 스킵
            xs.append(parts[0])  # 문자열 그대로 보존
            ys.append(parts[1])

    with open(OUT_PATH, "w", encoding="utf-8") as f:
        f.write("x: " + ",".join(xs) + "\n\n")
        f.write("y: " + ",".join(ys) + "\n")

    print(f"저장 완료: {OUT_PATH} (포인트 수: {len(xs)})")

if __name__ == "__main__":
    main()
