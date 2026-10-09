#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""회전교차로 AI 속도 보조 학습 (DAgger, numpy 전용 — torch 불필요).

data_recorder 로그($AISW_LOG_DIR/run_*.csv, 기본 ~/aisw_logs)에서 회전교차로(ai_zones mode=roundabout) 주변 행만 골라
  입력 = control.ai_input.build_features (경로 미리보기 + 속도/yaw rate + 가까운 NPC 6대)
  정답 = 전문가 속도 label_v (룰 + 안전 감독, master 가 매 주기 기록) [m/s]
로 MLP 를 학습해 models/roundabout.npz 로 저장한다. master 가 다음 실행 때 자동으로 읽는다.

DAgger: AI 가 속도를 낮춘 상태(drive_mode '+ai_slow')에도 전문가 정답이 기록되므로, 주행 → 기록 → 재학습을
반복하면 AI 가 스스로 만든 상황에서의 정답까지 배운다. label_v 가 없는 로그는 label_shift 초 뒤 실제 속도를
정답으로 쓰되 룰이 운전한 행만 쓴다.

사용:
  python3 tools/train/train_policy.py
  python3 tools/train/train_policy.py --logs <로그폴더>/run_2026*.csv --epochs 400
"""
import argparse
import csv
import glob
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from control.path_utils import DEFAULT_MAP, DEFAULT_SECTIONS, LOG_DIR, PKG_DIR, load_map_fields, load_sections, get_section  # noqa: E402
from control.ai_input import FEATURE_DIM, FEATURE_VERSION, N_NPC, PathPreview, build_features  # noqa: E402
from control.ai_model import MLPPolicy, forward_train  # noqa: E402


MAX_POSE_ERR = 5.0     # [m] 추정 위치 vs 검증 기준위치 차이가 이보다 큰 행 제외 (추측항법이 크게 틀어진 주행만 거른다)


def _f(row, key):
    try:
        v = float(row.get(key, 'nan'))
    except ValueError:
        return float('nan')
    return v


def row_objects(r):
    """로그의 npc 열 → (N,6) [x, y, vx, vy, 길이, 폭] (있는 것만)."""
    objs = []
    for k in range(N_NPC):
        if _f(r, 'npc%d_on' % k) == 1.0:
            objs.append([_f(r, 'npc%d_%s' % (k, f)) for f in ('x', 'y', 'vx', 'vy', 'l', 'w')])
    return np.array(objs, np.float64).reshape(-1, 6)


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
        x, y, yaw, v, yr = (_f(r, k) for k in ('x', 'y', 'yaw_deg', 'vel', 'yaw_rate'))
        if not all(math.isfinite(a) for a in (x, y, yaw, v)):
            continue
        tx, ty = _f(r, 'truth_x'), _f(r, 'truth_y')
        if math.isfinite(tx) and math.hypot(x - tx, y - ty) > MAX_POSE_ERR:
            continue               # 위치 추정이 틀린 상태는 제외 (검증용 기준위치가 기록된 로그만)
        label = _f(r, 'label_v')   # 전문가 속도 (DAgger)
        if not math.isfinite(label):
            if '+ai' in r.get('drive_mode', ''):
                continue           # 정답 없이 AI 가 낮춘 속도는 배우지 않는다
            j = int(np.searchsorted(t, t[i] + shift_s))
            if j >= len(rows) or not math.isfinite(vel[j]):
                continue
            label = vel[j]
        X.append(build_features(preview, int(idx), x, y, math.radians(yaw), v,
                                yr if math.isfinite(yr) else 0.0, row_objects(r)))
        Y.append([max(0.0, label)])
    return np.array(X).reshape(-1, FEATURE_DIM), np.array(Y).reshape(-1, 1)


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
    ap.add_argument('--logs', nargs='*', default=None, help='기본 $AISW_LOG_DIR/run_*.csv')
    ap.add_argument('--sections', default=DEFAULT_SECTIONS)
    ap.add_argument('--margin', type=int, default=20, help='구간 앞뒤로 더 쓰는 인덱스 수')
    ap.add_argument('--label_shift', type=float, default=0.5, help='label_v 없는 로그: 정답 = 이 시간 뒤 실제 속도 [s]')
    ap.add_argument('--hidden', default='64,64')
    ap.add_argument('--epochs', type=int, default=300)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--batch', type=int, default=256)
    ap.add_argument('--out', default=None, help='기본 models/roundabout.npz')
    args = ap.parse_args()

    zones = [z for z in get_section(load_sections(args.sections), 'ai_zones', []) if z.get('mode') == 'roundabout']
    if not zones:
        sys.exit('ai_zones 에 mode=roundabout 구간이 없음')
    lo = min(int(z['enter']) for z in zones) - args.margin
    hi = max(int(z['end']) for z in zones) + args.margin
    files = sorted(args.logs if args.logs else glob.glob(os.path.join(LOG_DIR, 'run_*.csv')))
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
    print('검증 RMSE: 목표속도 %.2f kph (평균값 예측 %.2f kph)' % (rmse[0] * 3.6, base[0] * 3.6))

    out = args.out or os.path.join(PKG_DIR, 'models', 'roundabout.npz')
    os.makedirs(os.path.dirname(out) or '.', exist_ok=True)
    np.savez(out, **save)
    print('저장: %s  (master 재시작 시 자동 로드, 끄려면 ai_enable:=false)' % out)


if __name__ == '__main__':
    main()
