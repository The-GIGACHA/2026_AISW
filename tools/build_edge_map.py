#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] edge_logger CSV 들 → 전역경로 인덱스별 좌/우 도로 경계 지도 (map/road_edges.npz).

경계 = LiDAR 로 잡은 경계석/가드레일 선의 경로 기준 횡거리 (좌 +, 우 -).
  자차 기준 경계 거리 + 자차의 경로 횡편차 = 경로 기준 경계 거리
인덱스 BIN 개 단위 중앙값, 관측 없는 칸은 앞뒤 보간, 끝까지 없으면 nan (플래너는 기본 폭 사용).

  python3 tools/build_edge_map.py [edges_*.csv ...]
"""
import csv, glob, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from aisw_common import DEFAULT_MAP, LOG_DIR, PKG_DIR, load_map_fields

BIN = 10          # 인덱스 묶음 (0.5 m 간격 → 5 m)
MIN_OBS = 3       # 칸당 최소 관측 수
MAX_GAP_BINS = 6  # 이보다 긴 빈 구간(30 m)은 보간하지 않음


def main():
    files = sys.argv[1:] or sorted(glob.glob(os.path.join(LOG_DIR, 'edges_*.csv')))
    n = len(load_map_fields(DEFAULT_MAP)[0])
    nb = (n + BIN - 1) // BIN
    L = [[] for _ in range(nb)]; R = [[] for _ in range(nb)]
    for p in files:
        for r in csv.DictReader(open(p)):
            i, off = int(r['idx']), float(r['path_off'])
            for key, acc in (('left', L), ('right', R)):
                v = float(r[key])
                if np.isfinite(v):
                    acc[i // BIN].append(v + off)
    def reduce(acc):
        a = np.array([np.median(x) if len(x) >= MIN_OBS else np.nan for x in acc])
        good = np.where(np.isfinite(a))[0]
        for k in range(len(good) - 1):           # 짧은 빈 구간만 선형 보간
            g0, g1 = good[k], good[k + 1]
            if 1 < g1 - g0 <= MAX_GAP_BINS:
                a[g0 + 1:g1] = np.interp(np.arange(g0 + 1, g1), [g0, g1], [a[g0], a[g1]])
        return a
    left, right = reduce(L), reduce(R)
    out = os.path.join(PKG_DIR, 'map', 'road_edges.npz')
    np.savez(out, bin=BIN, left=left, right=right)
    cov = lambda a: 100.0 * np.isfinite(a).mean()
    print('파일 %d개 → %s  (좌 %.0f%%, 우 %.0f%% 구간 관측)' % (len(files), out, cov(left), cov(right)))
    for b in range(0, nb, 40):
        print('  idx %4d  좌 %5.2f  우 %5.2f' % (b * BIN, left[b], right[b]))


if __name__ == '__main__':
    main()
