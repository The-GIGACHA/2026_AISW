#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""회전교차로 진입 실험 결과 요약: 모드별 충돌·근접·대기·통과 시간, 탐색 조건별 결과.
사용: report.py <결과.jsonl> [...]"""
import collections
import json
import sys

import numpy as np


def stats(rows):
    ok = [r for r in rows if not r.get('teleport_fail')]
    done = [r for r in ok if not r.get('timeout')]
    coll = [r for r in ok if r.get('collision')]
    near = [r for r in done if not r.get('collision') and min(r['min_gap_wait'], r['min_gap_go']) < 1.0]
    med = lambda k: np.median([r[k] for r in done if k in r]) if any(k in r for r in done) else float('nan')
    gaps = [min(r['min_gap_wait'], r['min_gap_go']) for r in done]
    return ('%4d 회 | 충돌 %d (%.1f%%) | 1 m 미만 근접 %d | 시간초과 %d | 대기 중앙 %.1f s | 통과 중앙 %.1f s | 최소간격 5%% %.2f m'
            % (len(ok), len(coll), 100.0 * len(coll) / max(len(ok), 1), len(near), len(ok) - len(done),
               med('wait_s'), med('pass_s'), np.percentile(gaps, 5) if gaps else float('nan')))


def main():
    rows = [json.loads(l) for p in sys.argv[1:] for l in open(p) if l.strip()]
    by = collections.defaultdict(list)
    for r in rows:
        by[r['mode']].append(r)
    for mode, rs in sorted(by.items()):
        print('[%s] %s' % (mode, stats(rs)))
    ex = [r for r in by.get('explore', []) if r.get('trial') and not r.get('teleport_fail')]
    if ex:
        print('\n탐색: 진입 기준(accept_gap) x 진입 속도별 충돌률')
        for g0, g1 in ((0, 2), (2, 4), (4, 6.1)):
            cells = []
            for v0, v1 in ((8, 15), (15, 22), (22, 30.1)):
                sub = [r for r in ex if g0 <= r['trial']['accept_gap'] < g1 and v0 <= r['trial']['v_entry_kph'] < v1]
                if sub:
                    c = sum(1 for r in sub if r.get('collision'))
                    cells.append('%2.0f~%2.0fkph %3d회 충돌 %4.1f%%' % (v0, v1, len(sub), 100.0 * c / len(sub)))
            print('  기준 %.0f~%.0f m | %s' % (g0, g1, ' | '.join(cells)))


if __name__ == '__main__':
    main()
