#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
_MAP_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map')   # 패키지 map/ (PC 경로 하드코딩 제거)

import os

# 입력/출력 경로 설정
IN_PATH = os.path.join(_MAP_DIR, "25hl_global_path_ver2.txt")
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
