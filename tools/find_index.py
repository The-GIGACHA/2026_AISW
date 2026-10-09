#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 전역경로 인덱스 찾기 — config/kcity_sections.yaml 구간을 채울 때 쓴다.

ROS 없이 동작한다.

    python3 tools/find_index.py 312.5 -104.2          # 지역좌표 한 점 → 인덱스, 누적거리 s
    python3 tools/find_index.py --idx 1650             # 인덱스 → 좌표
    python3 tools/find_index.py --csv <로그폴더>/run_xxx.csv --event n_collision
        # data_recorder 로그에서 해당 열 값이 0 → 양수로 바뀐 지점의 인덱스 목록
    python3 tools/find_index.py --csv <로그폴더>/run_xxx.csv --event jamming
        # 재밍(음영) 구간 진입/이탈 인덱스 확인
"""
import argparse
import csv
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from aisw_common import DEFAULT_MAP, load_map_fields, nearest_index_global  # noqa: E402


def main():
    p = argparse.ArgumentParser(description='전역경로 인덱스 조회')
    p.add_argument('xy', nargs='*', type=float, help='지역좌표 x y')
    p.add_argument('--idx', type=int, help='인덱스 → 좌표')
    p.add_argument('--csv', help='data_recorder 로그')
    p.add_argument('--event', default='n_collision', help='--csv 에서 변화를 볼 열 이름')
    p.add_argument('--map', default=DEFAULT_MAP)
    a = p.parse_args()

    rx, ry = load_map_fields(a.map)[:2]
    s = [0.0]
    for i in range(1, len(rx)):
        s.append(s[-1] + math.hypot(rx[i] - rx[i - 1], ry[i] - ry[i - 1]))

    if a.idx is not None:
        i = a.idx
        print('idx=%d  x=%.2f  y=%.2f  s=%.1f m' % (i, rx[i], ry[i], s[i]))
    elif len(a.xy) == 2:
        i = nearest_index_global(rx, ry, *a.xy)
        dist = math.hypot(rx[i] - a.xy[0], ry[i] - a.xy[1])
        print('idx=%d  s=%.1f m  (경로까지 %.2f m)' % (i, s[i], dist))
    elif a.csv:
        prev = None
        with open(os.path.expanduser(a.csv)) as f:
            for row in csv.DictReader(f):
                try:
                    val = float(row[a.event])
                except (KeyError, ValueError):
                    continue
                if prev is not None and (prev > 0) != (val > 0):
                    edge = '시작' if val > 0 else '끝'
                    print('t=%s  %s %s  global_idx=%s  x=%s y=%s'
                          % (row['t'], a.event, edge, row['global_idx'], row['x'], row['y']))
                prev = val
    else:
        p.print_help()


if __name__ == '__main__':
    main()
