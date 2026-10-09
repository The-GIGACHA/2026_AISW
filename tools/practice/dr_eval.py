#!/usr/bin/env python3
"""음영 구간 추측항법 평가: 로그의 추정 위치(pose_dr=1) vs 충돌패킷 기준 위치(truth_x/y)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import LOG_DIR
import csv, glob, math, os, sys
p = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.join(LOG_DIR, 'run_*.csv')), key=os.path.getmtime)
R = list(csv.DictReader(open(p)))
f = lambda v: float(v) if v not in ('', None, 'nan') else float('nan')
errs = []
for r in R:
    if r.get('pose_dr') == '1' and r.get('truth_x'):
        e = math.hypot(f(r['x']) - f(r['truth_x']), f(r['y']) - f(r['truth_y']))
        if e == e:
            errs.append((f(r['global_idx']), e, f(r['vel']) * 3.6, r['drive_mode']))
if not errs:
    print('추측항법 구간 없음'); sys.exit()
print('추측항법 %d행, 인덱스 %d~%d' % (len(errs), errs[0][0], errs[-1][0]))
for k in range(0, len(errs), max(1, len(errs) // 12)):
    i, e, v, m = errs[k]
    print('  idx %4d  오차 %.2f m  v %.1f  %s' % (i, e, v, m))
print('최대 오차 %.2f m, 끝 오차 %.2f m' % (max(e for _, e, _, _ in errs), errs[-1][1]))
