#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""회전교차로 진입 판단 모델 학습 (numpy MLP).

입력: run_episodes.py 결과 jsonl — 회차마다 출발 순간의 특징(control/roundabout_entry.features)과
      실제 결과(출발 후 앞뒤 차량과의 실제 최소 간격 min_gap_go)
학습: 특징 → 실제 최소 간격 [m]. 손실은 10% 분위수(pinball) — 평균이 아니라 '나쁜 쪽' 값을 예측해
      모델이 안전하다고 할 때 실제로도 안전하도록 보수적으로 맞춘다.
출력: models/roundabout_entry.npz — master 가 다음 실행 때 읽는다 (회전교차로 진입: 후보 속도 중
      예측 간격 >= safe_gap 인 가장 빠른 속도로 출발, 없으면 대기)

사용: train_entry.py <결과.jsonl> [...] [--out models/roundabout_entry.npz]
"""
import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts'))
from control.path_utils import PKG_DIR, get_section, load_sections  # noqa: E402
from control.roundabout_entry import FEATURE_DIM, FEATURE_VERSION  # noqa: E402
from control.mlp import MLP, forward_train  # noqa: E402

TAU = 0.1            # 분위수 (낮을수록 보수적)
GAP_CLIP = (-1.0, 8.0)


def load(paths):
    X, Y = [], []
    for p in paths:
        for line in open(p):
            e = json.loads(line)
            d = e.get('decision')
            if not d or e.get('timeout') or e.get('teleport_fail') or len(d.get('features', [])) != FEATURE_DIM:
                continue
            gap = -1.0 if e.get('collision') else e['min_gap_go']
            X.append(d['features'])
            Y.append([float(np.clip(gap, *GAP_CLIP))])
    return np.array(X, np.float64).reshape(-1, FEATURE_DIM), np.array(Y, np.float64).reshape(-1, 1)


def train(Xt, Yt, Xv, Yv, hidden, epochs, lr, batch, seed=0):
    rng = np.random.default_rng(seed)
    sizes = [Xt.shape[1]] + hidden + [1]
    p = {}
    for k in range(len(sizes) - 1):
        p['W%d' % k] = rng.normal(0, 1.0 / math.sqrt(sizes[k]), (sizes[k], sizes[k + 1]))
        p['b%d' % k] = np.zeros(sizes[k + 1])
    m = {k: np.zeros_like(v) for k, v in p.items()}
    v2 = {k: np.zeros_like(v) for k, v in p.items()}
    best, best_loss, step = None, float('inf'), 0
    n_layers = len(sizes) - 1

    def pinball(pred, y):
        e = y - pred
        return float(np.mean(np.maximum(TAU * e, (TAU - 1) * e)))

    for ep in range(epochs):
        order = rng.permutation(len(Xt))
        for i in range(0, len(Xt), batch):
            idx = order[i:i + batch]
            acts = forward_train(p, Xt[idx])
            e = Yt[idx] - acts[-1]
            g = np.where(e > 0, -TAU, 1.0 - TAU) / len(idx)          # d pinball / d pred
            grads = {}
            for k in range(n_layers - 1, -1, -1):
                grads['W%d' % k] = acts[k].T @ g
                grads['b%d' % k] = g.sum(0)
                if k:
                    g = (g @ p['W%d' % k].T) * (1.0 - acts[k] ** 2)
            step += 1
            for k in p:
                m[k] = 0.9 * m[k] + 0.1 * grads[k]
                v2[k] = 0.999 * v2[k] + 0.001 * grads[k] ** 2
                p[k] -= lr * (m[k] / (1 - 0.9 ** step)) / (np.sqrt(v2[k] / (1 - 0.999 ** step)) + 1e-8)
        loss = pinball(forward_train(p, Xv)[-1], Yv)
        if loss < best_loss:
            best_loss, best = loss, {k: v.copy() for k, v in p.items()}
        if ep % max(1, epochs // 10) == 0 or ep == epochs - 1:
            print('  epoch %4d  val pinball %.4f' % (ep, loss))
    return best


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('results', nargs='+')
    ap.add_argument('--out', default=os.path.join(PKG_DIR, 'models', 'roundabout_entry.npz'))
    ap.add_argument('--hidden', default='32,32')
    ap.add_argument('--epochs', type=int, default=400)
    ap.add_argument('--lr', type=float, default=2e-3)
    ap.add_argument('--batch', type=int, default=64)
    a = ap.parse_args()

    X, Y = load(a.results)
    print('출발 기록 %d 회 (충돌 %d)' % (len(X), int((Y[:, 0] <= -1.0).sum())))
    if len(X) < 30:
        sys.exit('기록이 너무 적다 (30 회 미만)')
    rng = np.random.default_rng(0)
    perm = rng.permutation(len(X))
    cut = int(len(X) * 0.8)
    tr, va = perm[:cut], perm[cut:]
    x_mean, x_std = X[tr].mean(0), X[tr].std(0) + 1e-6
    y_mean, y_std = Y[tr].mean(0), Y[tr].std(0) + 1e-6
    hidden = [int(h) for h in a.hidden.split(',') if h]
    best = train((X[tr] - x_mean) / x_std, (Y[tr] - y_mean) / y_std, (X[va] - x_mean) / x_std, (Y[va] - y_mean) / y_std,
                 hidden, a.epochs, a.lr, a.batch)
    save = dict(best, x_mean=x_mean, x_std=x_std, y_mean=y_mean, y_std=y_std,
                feature_version=np.array(FEATURE_VERSION))

    # 검증: 모델이 '안전'이라 한 경우 실제로 안전했는지
    model = MLP(save)
    pred = np.array([model.predict(x)[0] for x in X[va]])
    safe_gap = float(get_section(load_sections(), 'roundabout', {}).get('safe_gap', 2.0))
    said_safe = pred >= safe_gap
    actual = Y[va, 0]
    print('검증 %d 회: 모델이 안전(예측 >= %.1f m)이라 한 %d 회 중 실제 간격 < 0.5 m %d 회, 충돌 %d 회'
          % (len(va), safe_gap, int(said_safe.sum()), int((actual[said_safe] < 0.5).sum()),
             int((actual[said_safe] <= -1.0).sum())))
    os.makedirs(os.path.dirname(a.out) or '.', exist_ok=True)
    np.savez(a.out, **save)
    print('저장: %s  (master 재시작 시 자동 로드)' % a.out)


if __name__ == '__main__':
    main()
