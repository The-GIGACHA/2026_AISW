#!/usr/bin/env python3
"""연습 주행 로그 요약: 진행, 속도, 횡오차, 조향 매끄러움, 충돌, 모드별 구간."""
import csv, glob, math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import load_map_fields

rx, ry = [np.array(a) for a in load_map_fields()[:2]]
ryaw = np.arctan2(np.gradient(ry), np.gradient(rx))


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


def lat_err(x, y, idx):
    i = int(idx)
    dx, dy = x - rx[i], y - ry[i]
    return -math.sin(ryaw[i]) * dx + math.cos(ryaw[i]) * dy   # 좌+


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.expanduser('~/aisw_logs/run_*.csv')), key=os.path.getmtime)
    rows = list(csv.DictReader(open(path)))
    t = np.array([f(r['t']) for r in rows]); t -= t[0]
    idx = np.array([f(r['global_idx']) for r in rows])
    x = np.array([f(r['x']) for r in rows]); y = np.array([f(r['y']) for r in rows])
    v = np.array([f(r['vel']) for r in rows]) * 3.6
    st = np.array([f(r['cmd_steer']) for r in rows])
    mode = [r.get('drive_mode', '') for r in rows]
    coll = np.array([f(r['n_collision']) for r in rows])
    ok = np.isfinite(idx) & np.isfinite(x)
    le = np.full(len(rows), np.nan)
    le[ok] = [lat_err(a, b, i) for a, b, i in zip(x[ok], y[ok], idx[ok])]
    print('%s  %.0fs, %d행' % (os.path.basename(path), t[-1], len(rows)))
    if not ok.any():
        print('위치 없음'); return
    print('인덱스 %d → %d, 속도 평균 %.1f / 최대 %.1f kph' % (np.nanmin(idx), np.nanmax(idx), np.nanmean(v), np.nanmax(v)))
    dst = np.diff(st) * 15
    print('조향 변화율 |dδ/dt| 평균 %.2f°/s, 95%% %.1f°/s, 부호반전 %d회' % (
        math.degrees(np.nanmean(np.abs(dst))), math.degrees(np.nanpercentile(np.abs(dst), 95)),
        int(np.sum(np.diff(np.sign(st[np.abs(st) > 0.01])) != 0))))
    newc = int(np.sum(np.diff((coll > 0).astype(int)) > 0))
    print('충돌 상승에지 %d회' % newc, ('@idx ' + str([int(idx[i + 1]) for i in np.where(np.diff((coll > 0).astype(int)) > 0)[0]][:10])) if newc else '')
    # 구간별 횡오차 (200 인덱스 단위)
    print('구간별 |횡오차| 평균/최대 [m], 조향률95 [°/s], 모드:')
    for a in range(0, 4400, 200):
        m = ok & (idx >= a) & (idx < a + 200)
        if m.sum() < 5:
            continue
        mi = np.where(m)[0]
        sr = np.abs(np.diff(st[mi])) * 15
        modes = sorted(set(mode[i].split(':')[0] for i in mi))
        print('  %4d-%4d  %.2f / %.2f   %5.1f   %s' % (a, a + 199, np.nanmean(np.abs(le[m])), np.nanmax(np.abs(le[m])),
                                                  math.degrees(np.nanpercentile(sr, 95)) if len(sr) else 0, ','.join(modes)))


if __name__ == '__main__':
    main()
