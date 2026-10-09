# -*- coding: utf-8 -*-
"""경량 MLP 정책 (numpy 추론). 대회 PC 에 torch 없이 돌리기 위해 가중치를 .npz 로 저장한다.

출력: [목표속도 v (m/s)] — 회전교차로에서 룰 속도를 낮추는 보조로만 쓴다
학습: tools/train/train_policy.py  →  models/<mode>.npz
"""
import os

import numpy as np

from .ai_input import FEATURE_DIM, FEATURE_VERSION


class MLPPolicy:
    def __init__(self, params):
        self.layers = []
        i = 0
        while 'W%d' % i in params:
            self.layers.append((params['W%d' % i], params['b%d' % i]))
            i += 1
        self.x_mean, self.x_std = params['x_mean'], params['x_std']
        self.y_mean, self.y_std = params['y_mean'], params['y_std']

    @classmethod
    def load(cls, path):
        """모델이 없거나 특징 버전이 다르면 None (→ 룰 폴백)."""
        if not path or not os.path.exists(path):
            return None, 'no model: %s' % path
        p = dict(np.load(path))
        ver = int(p.get('feature_version', -1))
        if ver != FEATURE_VERSION:
            return None, 'feature_version %d != %d (재학습 필요)' % (ver, FEATURE_VERSION)
        if p['W0'].shape[0] != FEATURE_DIM:
            return None, 'input dim %d != %d' % (p['W0'].shape[0], FEATURE_DIM)
        return cls(p), 'loaded %s' % path

    def predict(self, x):
        h = (np.asarray(x, np.float64) - self.x_mean) / self.x_std
        for k, (W, b) in enumerate(self.layers):
            h = h @ W + b
            if k < len(self.layers) - 1:
                h = np.tanh(h)
        return h * self.y_std + self.y_mean


def forward_train(params, X):
    """학습용 순전파: 각 층 활성값을 함께 돌려준다 (역전파에 사용)."""
    acts = [X]
    h = X
    n = len([k for k in params if k.startswith('W')])
    for k in range(n):
        h = h @ params['W%d' % k] + params['b%d' % k]
        if k < n - 1:
            h = np.tanh(h)
        acts.append(h)
    return acts
