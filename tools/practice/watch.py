#!/usr/bin/env python3
"""주행 감시: 60초마다 진행 1줄, 충돌/정지/노드 오류/랩 완료 즉시 1줄."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import LOG_DIR
import csv, glob, os, subprocess, sys, time
LOG = sys.argv[1] if len(sys.argv) > 1 else None
start = time.time()
last_report = 0
stuck_since = None
seen_err = set()
last_idx = None
max_idx = 0


def f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float('nan')


while True:
    time.sleep(2)
    files = glob.glob(os.path.join(LOG_DIR, 'run_*.csv'))
    if not files:
        continue
    path = max(files, key=os.path.getmtime)
    try:
        with open(path) as fh:
            lines = fh.readlines()
        hdr = lines[0].strip().split(',')
        tail = [dict(zip(hdr, l.strip().split(','))) for l in lines[-40:] if l.count(',') == len(hdr) - 1]
    except Exception:
        continue
    if not tail:
        continue
    r = tail[-1]
    idx, vel = f(r.get('global_idx')), f(r.get('vel')) * 3.6
    coll = [f(x.get('n_collision')) for x in tail]
    if any(c > 0 for c in coll[-8:]) and not any(c > 0 for c in coll[:-8]):
        print('COLLISION idx=%s mode=%s' % (r.get('global_idx'), r.get('drive_mode')), flush=True)
    if idx == idx:
        if last_idx is not None and last_idx > 4200 and idx < 200:
            print('LAP_DONE (%.0fs)' % (time.time() - start), flush=True)
        last_idx = idx
        max_idx = max(max_idx, idx)
    if vel < 1.0:
        stuck_since = stuck_since or time.time()
        if time.time() - stuck_since > 20:
            print('STUCK idx=%s %.0fs mode=%s' % (r.get('global_idx'), time.time() - stuck_since, r.get('drive_mode')), flush=True)
            stuck_since = time.time() + 40   # 1분에 한 번만
    else:
        stuck_since = None
    if LOG and os.path.exists(LOG):
        out = subprocess.run(['grep', '-nE', 'Traceback|process has died|Error', LOG], capture_output=True, text=True).stdout
        for line in out.splitlines():
            if 'Status 포트' in line or line in seen_err:
                continue
            seen_err.add(line)
            print('ERR ' + line[:160], flush=True)
    if time.time() - last_report > 90:
        last_report = time.time()
        print('PROGRESS idx=%s vel=%.1fkph mode=%s mission=%s' % (r.get('global_idx'), vel, r.get('drive_mode'), r.get('mission')), flush=True)
