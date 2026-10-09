# -*- coding: utf-8 -*-
"""LiDAR 클러스터 다중 객체 추적 (월드 좌표, 등속 칼만 필터 + 최근접 연관).

입력: 매 회전(sweep)마다 자차 좌표계(후륜축 기준) 클러스터 중심 + 자차 월드 자세.
출력: 트랙 id, 월드 위치/속도, 방향(속도 방향), 확정 여부.

끼어들기·회전교차로 판단과 플래너의 동적 장애물 예측에 쓰인다.
LiDAR 가 부하 시 3 Hz 까지 떨어지는 것을 전제로 연관 게이트를 넉넉히 잡는다.
"""
import math
import itertools

import numpy as np

GATE_M = 2.5            # 예측 위치와의 연관 허용 거리 [m]
MAX_MISSES = 3          # 연속 미검출 허용 횟수 (3 Hz 기준 ≈ 1 s)
CONFIRM_HITS = 3        # 속도를 신뢰하기 위한 최소 연속 관측 수
ACC_NOISE = 2.0         # 프로세스 노이즈(가속도 표준편차) [m/s^2]
MEAS_NOISE = 0.5        # 관측 노이즈(클러스터 중심) [m]. 0.35 는 3 Hz 에서 정지 물체 속도 잡음 ~1 m/s (2026-10-09)
MAX_SPEED = 30.0        # 이보다 빠른 추정은 연관 오류로 보고 속도 리셋 [m/s]


class Track:
    _ids = itertools.count(1)

    def __init__(self, x, y, size, t):
        self.id = next(Track._ids)
        self.x = np.array([x, y, 0.0, 0.0])
        self.P = np.diag([MEAS_NOISE ** 2, MEAS_NOISE ** 2, 25.0, 25.0])
        self.size = size
        self.t = t
        self.hits = 1
        self.misses = 0

    @property
    def confirmed(self):
        return self.hits >= CONFIRM_HITS

    def predict(self, t):
        dt = max(0.0, t - self.t)
        if dt == 0.0:
            return
        F = np.eye(4); F[0, 2] = F[1, 3] = dt
        q = ACC_NOISE ** 2
        G = np.array([[0.5 * dt * dt, 0], [0, 0.5 * dt * dt], [dt, 0], [0, dt]])
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + G @ G.T * q
        self.t = t

    def update(self, z, size):
        H = np.zeros((2, 4)); H[0, 0] = H[1, 1] = 1.0
        R = np.eye(2) * MEAS_NOISE ** 2
        y = np.asarray(z) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        if math.hypot(self.x[2], self.x[3]) > MAX_SPEED:
            self.x[2:] = 0.0
            self.P[2, 2] = self.P[3, 3] = 25.0
        self.size = size
        self.hits += 1
        self.misses = 0

    @property
    def pos(self):
        return float(self.x[0]), float(self.x[1])

    @property
    def vel(self):
        """확정 전에는 (0, 0) — 초기 속도 추정은 믿을 수 없다."""
        return (float(self.x[2]), float(self.x[3])) if self.confirmed else (0.0, 0.0)


class Tracker:
    def __init__(self):
        self.tracks = []

    def step(self, t, ego_pose, dets):
        """t: 시각 [s], ego_pose: (x, y, yaw_rad) 월드, dets: [(x_ego, y_ego, (sx, sy, sz)), ...].
        → 이번 관측 각각에 대응하는 Track 리스트(같은 순서)."""
        ex, ey, eyaw = ego_pose
        c, s = math.cos(eyaw), math.sin(eyaw)
        zs = [(ex + c * dx - s * dy, ey + s * dx + c * dy) for dx, dy, _ in dets]
        for tr in self.tracks:
            tr.predict(t)
        # 최근접 연관 (거리 오름차순 탐욕)
        pairs = sorted(((math.hypot(z[0] - tr.x[0], z[1] - tr.x[1]), i, j)
                        for i, z in enumerate(zs) for j, tr in enumerate(self.tracks)))
        used_z, used_t, out = set(), set(), [None] * len(zs)
        for d, i, j in pairs:
            if d > GATE_M:
                break
            if i in used_z or j in used_t:
                continue
            self.tracks[j].update(zs[i], dets[i][2])
            out[i] = self.tracks[j]
            used_z.add(i); used_t.add(j)
        for j, tr in enumerate(self.tracks):
            if j not in used_t:
                tr.misses += 1
        for i, z in enumerate(zs):
            if out[i] is None:
                tr = Track(z[0], z[1], dets[i][2], t)
                self.tracks.append(tr)
                out[i] = tr
        self.tracks = [tr for tr in self.tracks if tr.misses <= MAX_MISSES]
        return out
