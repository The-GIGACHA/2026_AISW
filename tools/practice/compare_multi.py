#!/usr/bin/env python3
"""두 로그의 완주 바퀴들을 구간별로 평균 비교 (첫 바퀴는 출발 지점 idx<100 부터).
사용: compare_multi.py A.csv B.csv [이름A 이름B]"""
import csv, math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import load_map_fields
rx, ry = [np.array(a) for a in load_map_fields()[:2]]
ryaw = np.arctan2(np.gradient(ry), np.gradient(rx))
f = lambda v: float(v) if v not in ('', None, 'nan') else float('nan')
SEGS = [(0, 600, '출발~S자'), (600, 760, '정적장애물'), (760, 1000, '보행자'), (1000, 1300, '도심회전'),
        (1300, 1755, '도심~회전교차로'), (1755, 1865, '회전교차로(AI)'), (1865, 2260, '합류'), (2260, 3600, '고주로'),
        (3600, 3760, '음영 전'), (3760, 4020, '음영(AI)'), (4020, 4392, '마무리')]


def laps(path):
    R = [r for r in csv.DictReader(open(path)) if r['global_idx'] not in ('', 'nan')]
    idx = [f(r['global_idx']) for r in R]
    w = [i for i in range(1, len(idx)) if idx[i - 1] > 4200 and idx[i] < 200]
    out = []
    s0 = next((i for i, v in enumerate(idx) if v < 100), None)
    if s0 is not None and w and s0 < w[0]:
        out.append(R[s0:w[0]])
    out += [R[w[k]:w[k + 1]] for k in range(len(w) - 1)]
    return out


def stats(rows):
    lat = [abs(-math.sin(ryaw[int(f(r['global_idx']))]) * (f(r['x']) - rx[int(f(r['global_idx']))])
               + math.cos(ryaw[int(f(r['global_idx']))]) * (f(r['y']) - ry[int(f(r['global_idx']))])) for r in rows]
    st = np.array([f(r['cmd_steer']) for r in rows])
    v = np.array([f(r['vel']) for r in rows]) * 3.6
    return (np.mean(lat), np.max(lat), math.degrees(np.percentile(np.abs(np.diff(st)) * 15, 95)),
            f(rows[-1]['t']) - f(rows[0]['t']), sum(1 for r in rows if f(r['n_collision']) > 0), np.min(v),
            int(np.sum((v[1:] < 5) & (v[:-1] >= 5))))


A, B = sys.argv[1], sys.argv[2]
na, nb = (sys.argv[3], sys.argv[4]) if len(sys.argv) > 4 else ('A', 'B')
LA, LB = laps(A), laps(B)
lt = lambda L: [round(f(l[-1]['t']) - f(l[0]['t'])) for l in L]
print('%s %d바퀴 %s (평균 %.0fs) | %s %d바퀴 %s (평균 %.0fs)' % (na, len(LA), lt(LA), np.mean(lt(LA)), nb, len(LB), lt(LB), np.mean(lt(LB))))
print('%-14s| %-48s| %-48s' % ('구간', na + ': 횡오차평균/최대 조향률95 시간 최저속도 서행횟수', nb))
for a, b, n in SEGS:
    out = []
    for L in (LA, LB):
        S = np.array([stats([r for r in lap if a <= f(r['global_idx']) < b]) for lap in L
                      if sum(1 for r in lap if a <= f(r['global_idx']) < b) > 5])
        out.append('%.2f/%.2f %5.1f°/s %5.1f±%.1fs %4.1fkph 서행%d%s' % (
            S[:, 0].mean(), S[:, 1].max(), S[:, 2].mean(), S[:, 3].mean(), S[:, 3].std(), S[:, 5].min(),
            int(S[:, 6].sum()), ' 충돌!' if S[:, 4].sum() else ''))
    print('%-14s| %-48s| %-48s' % (n, out[0], out[1]))
