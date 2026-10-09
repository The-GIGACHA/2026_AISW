# -*- coding: utf-8 -*-
"""경량 MLP (numpy). 대회 PC 에 torch 없이 돌리기 위해 가중치를 .npz 로 저장한다.
학습: tools/test/roundabout/train_entry.py"""
import os

import numpy as np


class MLP:
    def __init__(self, params):
        self.layers = []
        i = 0
        while 'W%d' % i in params:
            self.layers.append((params['W%d' % i], params['b%d' % i]))
            i += 1
        self.x_mean, self.x_std = params['x_mean'], params['x_std']
        self.y_mean, self.y_std = params['y_mean'], params['y_std']

    @classmethod
    def load(cls, path, input_dim, version):
        """모델이 없거나 입력 형식(version, 차원)이 다르면 (None, 이유)."""
        if not path or not os.path.exists(path):
            return None, 'no model: %s' % path
        p = dict(np.load(path))
        if int(p.get('feature_version', -1)) != version:
            return None, 'feature_version %d != %d (재학습 필요)' % (int(p.get('feature_version', -1)), version)
        if p['W0'].shape[0] != input_dim:
            return None, 'input dim %d != %d' % (p['W0'].shape[0], input_dim)
        return cls(p), 'loaded %s' % path

    def predict(self, x):
        h = (np.asarray(x, np.float64) - self.x_mean) / self.x_std
        for k, (W, b) in enumerate(self.layers):
            h = h @ W + b
            if k < len(self.layers) - 1:
                h = np.tanh(h)
        return h * self.y_std + self.y_mean


def forward_train(params, X):
    """학습용 순전파: 각 층 활성값 (역전파에 사용)."""
    acts = [X]
    h = X
    n = len([k for k in params if k.startswith('W')])
    for k in range(n):
        h = h @ params['W%d' % k] + params['b%d' % k]
        if k < n - 1:
            h = np.tanh(h)
        acts.append(h)
    return acts
