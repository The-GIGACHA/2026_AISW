#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] AI 구간 정책 학습 (행동 복제, numpy 전용 — torch 불필요).

data_recorder 로그(~/aisw_logs/run_*.csv)에서 해당 AI 구간(ai_zones 의 mode) 주변 행만 골라
  입력 = ai.features.build_features (경로 미리보기 + 속도/yaw rate + LiDAR 스캔)
  정답 = [/ctrl_cmd 조향(rad), label_shift 초 뒤 실제 속도(m/s)]
로 MLP 를 학습해 models/<mode>.npz 로 저장한다. master_v2 가 다음 실행 때 자동으로 읽는다.

사용:
  python3 tools/train_policy.py --mode shaded
  python3 tools/train_policy.py --mode roundabout --logs ~/aisw_logs/run_2026*.csv --epochs 400

정답의 질 = 시연 주행의 질이다. 회전교차로에서 '양보'를 배우게 하려면, NPC 가 있을 때
멈췄다 들어가는 주행(룰 서행 + 수동 개입 또는 튜닝된 룰)이 로그에 충분히 있어야 한다.
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
from aisw_common import DEFAULT_MAP, DEFAULT_SECTIONS, PKG_DIR, load_map_fields, load_sections, get_section  # noqa: E402
from ai.features import FEATURE_DIM, FEATURE_VERSION, SCAN_BINS, PathPreview, build_features  # noqa: E402
from ai.policy import MLPPolicy, forward_train  # noqa: E402


def _f(row, key):
    try:
        v = float(row.get(key, 'nan'))
    except ValueError:
        return float('nan')
    return v


def load_run(path, preview, lo, hi, shift_s):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([_f(r, 't') for r in rows])
    vel = np.array([_f(r, 'vel') for r in rows])
    X, Y = [], []
    for i, r in enumerate(rows):
        idx = _f(r, 'global_idx')
        if not (math.isfinite(idx) and lo <= idx <= hi):
            continue
        x, y, yaw, v, yr, steer = (_f(r, k) for k in ('x', 'y', 'yaw_deg', 'vel', 'yaw_rate', 'cmd_steer'))
        if not all(math.isfinite(a) for a in (x, y, yaw, v, steer)):
            continue
        j = int(np.searchsorted(t, t[i] + shift_s))
        if j >= len(rows) or not math.isfinite(vel[j]):
            continue
        scan = [_f(r, 'scan_%d' % k) for k in range(SCAN_BINS)]
        scan = scan if all(math.isfinite(a) for a in scan) else None
        X.append(build_features(preview, int(idx), x, y, math.radians(yaw), v,
                                yr if math.isfinite(yr) else 0.0, scan))
        Y.append([steer, max(0.0, vel[j])])
    return np.array(X).reshape(-1, FEATURE_DIM), np.array(Y).reshape(-1, 2)


def init_params(sizes, rng):
    p = {}
    for k in range(len(sizes) - 1):
        p['W%d' % k] = rng.normal(0, 1.0 / math.sqrt(sizes[k]), (sizes[k], sizes[k + 1]))
        p['b%d' % k] = np.zeros(sizes[k + 1])
    return p


def train(Xn, Yn, Xv, Yv, hidden, epochs, lr, batch, seed=0):
    rng = np.random.default_rng(seed)
    params = init_params([Xn.shape[1]] + hidden + [Yn.shape[1]], rng)
    m = {k: np.zeros_like(v) for k, v in params.items()}
    s = {k: np.zeros_like(v) for k, v in params.items()}
    nl = len(hidden) + 1
    best, best_p, step = float('inf'), None, 0
    for ep in range(epochs):
        perm = rng.permutation(len(Xn))
        for b0 in range(0, len(Xn), batch):
            bi = perm[b0:b0 + batch]
            acts = forward_train(params, Xn[bi])
            g = 2.0 * (acts[-1] - Yn[bi]) / len(bi)
            grads = {}
            for k in reversed(range(nl)):
                grads['W%d' % k] = acts[k].T @ g
                grads['b%d' % k] = g.sum(0)
                if k > 0:
                    g = (g @ params['W%d' % k].T) * (1.0 - acts[k] ** 2)
            step += 1
            for k in params:   # Adam
                m[k] = 0.9 * m[k] + 0.1 * grads[k]
                s[k] = 0.999 * s[k] + 0.001 * grads[k] ** 2
                params[k] -= lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(s[k] / (1 - 0.999 ** step)) + 1e-8)
        val = float(np.mean((forward_train(params, Xv)[-1] - Yv) ** 2))
        if val < best:
            best, best_p = val, {k: v.copy() for k, v in params.items()}
        if ep % max(1, epochs // 10) == 0 or ep == epochs - 1:
            print('  epoch %4d  val mse %.4f' % (ep, val))
    return best_p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--mode', required=True, choices=['shaded', 'roundabout'])
    ap.add_argument('--logs', nargs='*', default=None, help='기본 ~/aisw_logs/run_*.csv')
    ap.add_argument('--sections', default=DEFAULT_SECTIONS)
    ap.add_argument('--margin', type=int, default=20, help='구간 앞뒤로 더 쓰는 인덱스 수')
    ap.add_argument('--label_shift', type=float, default=0.5, help='목표속도 정답 = 이 시간 뒤 실제 속도 [s]')
    ap.add_argument('--hidden', default='64,64')
    ap.add_argument('--epochs', type=int, default=300)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--out', default=None, help='기본 models/<mode>.npz')
    args = ap.parse_args()

    zones = [z for z in get_section(load_sections(args.sections), 'ai_zones', []) if z.get('mode') == args.mode]
    if not zones:
        sys.exit('ai_zones 에 mode=%s 구간이 없음' % args.mode)
    lo = min(int(z['enter']) for z in zones) - args.margin
    hi = max(int(z['end']) for z in zones) + args.margin
    files = sorted(args.logs if args.logs else glob.glob(os.path.expanduser('~/aisw_logs/run_*.csv')))
    if not files:
        sys.exit('로그가 없음 — roslaunch aisw_2026 aisw_midterm.launch record:=true 로 먼저 주행 기록')

    rx, ry = load_map_fields(DEFAULT_MAP)[:2]
    preview = PathPreview(rx, ry)
    runs = []
    for p in files:
        X, Y = load_run(p, preview, lo, hi, args.label_shift)
        print('%s: %d행 (인덱스 %d~%d)' % (os.path.basename(p), len(X), lo, hi))
        if len(X):
            runs.append((X, Y))
    if not runs:
        sys.exit('구간 안 행이 없음 — 로그가 그 구간을 지나갔는지 확인')
    if len(runs) >= 2:   # 마지막 주행을 검증용으로 (같은 주행 안 인접 행 누수 방지)
        Xt, Yt = np.concatenate([r[0] for r in runs[:-1]]), np.concatenate([r[1] for r in runs[:-1]])
        Xv, Yv = runs[-1]
    else:
        X, Y = runs[0]
        cut = int(len(X) * 0.8)
        Xt, Yt, Xv, Yv = X[:cut], Y[:cut], X[cut:], Y[cut:]
    if len(Xt) < 500:
        print('⚠️ 학습 행 %d개 — 적다. 같은 구간을 여러 번(다양한 속도/NPC 상황) 기록할 것' % len(Xt))

    x_mean, x_std = Xt.mean(0), Xt.std(0) + 1e-6
    y_mean, y_std = Yt.mean(0), Yt.std(0) + 1e-6
    hidden = [int(h) for h in args.hidden.split(',') if h]
    print('학습 %d행 / 검증 %d행, 입력 %d차원, 은닉 %s' % (len(Xt), len(Xv), FEATURE_DIM, hidden))
    best = train((Xt - x_mean) / x_std, (Yt - y_mean) / y_std, (Xv - x_mean) / x_std, (Yv - y_mean) / y_std,
                 hidden, args.epochs, args.lr, args.batch)

    save = dict(best, x_mean=x_mean, x_std=x_std, y_mean=y_mean, y_std=y_std,
                feature_version=np.array(FEATURE_VERSION))
    pol = MLPPolicy(save)
    pred = np.array([pol.predict(x) for x in Xv])
    rmse = np.sqrt(np.mean((pred - Yv) ** 2, axis=0))
    base = np.sqrt(np.mean((Yt.mean(0) - Yv) ** 2, axis=0))
    print('검증 RMSE: 조향 %.2f° (평균값 예측 %.2f°), 목표속도 %.2f kph (평균값 예측 %.2f kph)'
          % (math.degrees(rmse[0]), math.degrees(base[0]), rmse[1] * 3.6, base[1] * 3.6))

    out = args.out or os.path.join(PKG_DIR, 'models', args.mode + '.npz')
    os.makedirs(os.path.dirname(out), exist_ok=True)
    np.savez(out, **save)
    print('저장: %s  (master_v2 재시작 시 자동 로드, 끄려면 ai_enable:=false)' % out)


if __name__ == '__main__':
    main()
