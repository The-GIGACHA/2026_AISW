#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 주행 로그로 MGeo link_id ↔ 전역경로 인덱스 표를 만든다.

규정의 속도 예외 구간(A2256W000411 시작점 ~ A2256W000153 끝점)처럼 링크 ID 로만
주어진 위치를 인덱스로 바꾸는 용도. MGeo 링크 좌표가 없어서 실제 주행에서 측정한다.

  python3 tools/link_index_table.py [로그.csv ...]      (기본 $AISW_LOG_DIR/run_*.csv)
"""
import csv
import glob
import os
import sys

SPEED_EXEMPT = ('A2256W000411', 'A2256W000153')   # 규정집 v1.1 3-2 속도 제한 예외


def main():
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
    from aisw_common import LOG_DIR
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(LOG_DIR, 'run_*.csv')))
    if not files:
        sys.exit('로그 없음 — record:=true 로 한 바퀴 주행 후 실행')
    table = {}   # link → [첫 등장 인덱스, 최소, 최대]
    for p in files:
        with open(p) as f:
            for r in csv.DictReader(f):
                link, idx = r.get('link_id', ''), r.get('global_idx', '')
                if not link or idx in ('', 'nan'):
                    continue
                i = int(float(idx))
                e = table.setdefault(link, [i, i, i])
                e[1], e[2] = min(e[1], i), max(e[2], i)
    if not table:
        sys.exit('link_id 열이 비어 있음 — aisw_udp_bridge 가 Competition Status(909) 를 받는지 확인')
    for link, (first, lo, hi) in sorted(table.items(), key=lambda kv: kv[1][0]):
        mark = '  ← 속도예외 ' + ('시작' if link == SPEED_EXEMPT[0] else '끝') if link in SPEED_EXEMPT else ''
        print('%-14s %5d ~ %5d%s' % (link, lo, hi, mark))
    a, b = table.get(SPEED_EXEMPT[0]), table.get(SPEED_EXEMPT[1])
    if a and b:
        print('\nkcity_sections.yaml missions → speed_exempt: start: %d, end: %d, source: rule' % (a[1], b[2]))
    else:
        print('\n속도예외 링크가 로그에 없음: %s' % [l for l, e in zip(SPEED_EXEMPT, (a, b)) if not e])


if __name__ == '__main__':
    main()
