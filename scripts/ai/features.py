# -*- coding: utf-8 -*-
"""AI 정책 입력 특징. 주행(ai_zone_controller)과 학습(tools/train_policy.py)이 공유한다.

입력 벡터 (FEATURE_DIM = 2*len(PREVIEW_DISTS) + 2 + SCAN_BINS):
  - 경로 미리보기: 전역경로 위 PREVIEW_DISTS[m] 앞 점들의 자차 좌표 (x, y) / PREVIEW_SCALE
  - 속도 / SPEED_SCALE, yaw rate [rad/s]
  - LiDAR 근접도: 방위 SCAN_BINS 칸, (SCAN_RMAX - r) / SCAN_RMAX  (0=비었음, 1=바로 앞)

GPS 음영에서는 자차 위치가 추측항법 값이라 미리보기도 그 위치 기준으로 만든다.
특징 정의를 바꾸면 FEATURE_VERSION 을 올려야 이전 모델이 잘못 로드되지 않는다.
"""
import math

import numpy as np

FEATURE_VERSION = 1
PREVIEW_DISTS = (2.0, 4.0, 6.0, 8.0, 10.0, 13.0, 16.0, 20.0)
PREVIEW_SCALE = 20.0
SPEED_SCALE = 15.0
SCAN_BINS = 72
SCAN_RMAX = 30.0
FEATURE_DIM = 2 * len(PREVIEW_DISTS) + 2 + SCAN_BINS


class PathPreview:
    """전역경로 누적거리 테이블. 인덱스에서 앞으로 d[m] 떨어진 점을 빠르게 꺼낸다."""

    def __init__(self, xs, ys):
        self.x = np.asarray(xs, np.float64)
        self.y = np.asarray(ys, np.float64)
        seg = np.hypot(np.diff(self.x), np.diff(self.y))
        self.s = np.concatenate([[0.0], np.cumsum(seg)])

    def points_ahead(self, idx, dists):
        """idx 기준 전방 거리 dists 의 경로점 (N,2). 경로 끝을 넘으면 끝점."""
        idx = int(min(max(idx, 0), len(self.s) - 1))
        target = self.s[idx] + np.asarray(dists, np.float64)
        j = np.clip(np.searchsorted(self.s, target), 0, len(self.s) - 1)
        return np.column_stack([self.x[j], self.y[j]])

    def points_ahead_offset(self, idx, dists, offset):
        """points_ahead 를 경로 좌측(+)으로 offset[m] 평행이동한 점들."""
        pts = self.points_ahead(idx, dists)
        if offset == 0.0 or len(pts) < 2:
            return pts
        t = np.gradient(pts, axis=0)
        n = np.hypot(t[:, 0], t[:, 1])
        n[n < 1e-9] = 1.0
        return pts + offset * np.column_stack([-t[:, 1] / n, t[:, 0] / n])

    def to_ego(self, pts, x, y, yaw_rad):
        dx, dy = pts[:, 0] - x, pts[:, 1] - y
        c, s = math.cos(yaw_rad), math.sin(yaw_rad)
        return np.column_stack([c * dx + s * dy, -s * dx + c * dy])


def scan_to_proximity(ranges):
    """LaserScan.ranges(길이 SCAN_BINS) → 근접도. None/길이 불일치면 0(비었음)."""
    if ranges is None or len(ranges) != SCAN_BINS:
        return np.zeros(SCAN_BINS, np.float64)
    r = np.clip(np.asarray(ranges, np.float64), 0.0, SCAN_RMAX)
    r[~np.isfinite(r)] = SCAN_RMAX
    return (SCAN_RMAX - r) / SCAN_RMAX


def build_features(preview, idx, x, y, yaw_rad, v, yaw_rate, scan_ranges):
    ego_pts = preview.to_ego(preview.points_ahead(idx, PREVIEW_DISTS), x, y, yaw_rad)
    return np.concatenate([
        ego_pts.reshape(-1) / PREVIEW_SCALE,
        [v / SPEED_SCALE, yaw_rate],
        scan_to_proximity(scan_ranges),
    ])
