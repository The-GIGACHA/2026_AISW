# -*- coding: utf-8 -*-
"""AI 구간 제어기: 학습 정책(MLP) + 룰 폴백 + 안전 감독.

구성 (우선순위 높은 것부터):
  1. 안전 감독 — LiDAR 스캔이 경로 통로(차폭+여유)를 막으면 제동거리 기반 속도 상한/정지.
     정책이 무엇을 내든 항상 적용한다.
  2. 학습 정책 — models/<mode>.npz 가 있으면 조향·목표속도를 낸다.
     룰 조향과 MAX_STEER_DEVIATION 이상 다르면 신뢰하지 않고 룰을 쓴다(분포 밖 입력 방어).
  3. 룰 폴백 — 모델이 없거나 거부되면 Pure Pursuit(추측항법/GPS 위치) + 구간 제한속도.

mode:
  shaded     GPS 음영. 위치 = 추측항법. 랜덤 장애물 미션 가능 → 통로 감시가 핵심.
  roundabout 회전교차로. 위치 = GPS. 룰은 저속 + 통로 감시만 하고, 진입 양보 판단은
             학습 정책(사람/전문가 주행 로그)에서 배우는 것을 목표로 한다.
"""
import math
import os

import numpy as np

from ai.features import (PathPreview, build_features, SCAN_BINS, SCAN_RMAX)
from ai.policy import MLPPolicy

WHEELBASE = 3.0
MAX_STEER = math.radians(40.0)          # 규정 차량 최대 조향각
HALF_WIDTH = 1.892 / 2.0 + 0.6          # 차폭 절반 + 여유 [m]
LIDAR_X = 0.58                          # 후륜축 → 라이다 [m] (2026_molit_comp_full_set_2.json)
MAX_STEER_DEVIATION = math.radians(20.0)

ZONE_SPEED = {'shaded': 20.0 / 3.6, 'roundabout': 15.0 / 3.6}   # 룰 목표속도 [m/s]
SAFE_DECEL = 3.0                        # 통로 장애물 앞 제동 감속도 [m/s^2]
FRONT_BUMPER = 3.0 + 0.845              # 후륜축 → 앞범퍼 [m] (휠베이스 + 앞 오버행, 규정 2-6-1)
STOP_MARGIN = FRONT_BUMPER + 3.0        # 장애물 앞 정지 여유 [m] (후륜축 기준 = 범퍼 앞 3 m)
CORRIDOR_LEN = 25.0
# 음영구간 회피: 통로가 막히면 경로를 옆으로 평행이동한 후보 중 가장 가까운 빈 통로를 탄다.
# 규정: 음영구역 안에서는 차로 준수 패널티 미적용.
AVOID_OFFSETS = (1.8, -1.8, 3.6, -3.6)
AVOID_TRIGGER = 18.0                    # 이 거리 안에서 막히면 회피 시작 [m]
OFFSET_RATE = 1.5                       # 횡방향 이동 속도 [m/s] (급조향 방지)
RETURN_CLEAR_S = 1.5                    # 원래 경로가 이 시간 연속 비어야 복귀 [s]


class AIZoneController:
    def __init__(self, ref_x, ref_y, model_dir, speed_limit_mps, use_ai=True, log=print):
        self.preview = PathPreview(ref_x, ref_y)
        self.speed_limit = speed_limit_mps
        self.log = log
        self.policies = {}
        for mode in ZONE_SPEED:
            pol, why = (MLPPolicy.load(os.path.join(model_dir, mode + '.npz')) if use_ai
                        else (None, 'ai 비활성(~ai_enable=false)'))
            self.policies[mode] = pol
            log('[ai_zone] %s 정책: %s' % (mode, why if pol is None else 'OK (%s)' % why))
        corr = np.arange(1.0, CORRIDOR_LEN + 0.01, 1.0)
        self._corr_dists = corr
        self.offset = 0.0          # 현재 적용 중인 횡 오프셋 [m]
        self.offset_goal = 0.0
        self._clear_since = None
        self.last = {}

    def reset(self):
        self.offset = self.offset_goal = 0.0
        self._clear_since = None

    # ---------------------------------------------------------------- 룰
    def _pure_pursuit(self, idx, x, y, yaw, v, offset=0.0):
        ld = min(max(4.0 + 0.5 * v, 4.0), 10.0)
        p = self.preview.to_ego(self.preview.points_ahead_offset(idx, [ld, ld + 1.0], offset), x, y, yaw)[0]
        alpha = math.atan2(p[1], p[0])
        return max(-MAX_STEER, min(MAX_STEER, math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)))

    def _corridor_obstacle(self, idx, x, y, yaw, scan, offset=0.0):
        """경로 통로 안 가장 가까운 LiDAR 점까지의 경로 방향 거리 [m]. 없으면 inf."""
        if scan is None or len(scan) != SCAN_BINS:
            return float('inf')
        r = np.asarray(scan, np.float64)
        a = -math.pi + (np.arange(SCAN_BINS) + 0.5) * (2 * math.pi / SCAN_BINS)
        hit = np.isfinite(r) & (r < SCAN_RMAX - 0.01)
        if not hit.any():
            return float('inf')
        px = r[hit] * np.cos(a[hit]) + LIDAR_X
        py = r[hit] * np.sin(a[hit])
        front = px > 0.5
        if not front.any():
            return float('inf')
        px, py = px[front], py[front]
        cp = self.preview.to_ego(self.preview.points_ahead_offset(idx, self._corr_dists, offset), x, y, yaw)
        d2 = (px[:, None] - cp[None, :, 0]) ** 2 + (py[:, None] - cp[None, :, 1]) ** 2
        j = np.argmin(d2, axis=1)
        inside = d2[np.arange(len(px)), j] <= HALF_WIDTH ** 2
        if not inside.any():
            return float('inf')
        return float(self._corr_dists[j[inside]].min())

    def _update_offset(self, mode, idx, x, y, yaw, scan, dt):
        """음영 모드 전용 회피 오프셋 결정 (회전교차로는 NPC 가 움직이므로 회피 대신 감속/정지)."""
        if mode != 'shaded':
            self.reset()
            return
        d_center = self._corridor_obstacle(idx, x, y, yaw, scan, 0.0)
        if d_center < AVOID_TRIGGER:
            self._clear_since = None
            if self._corridor_obstacle(idx, x, y, yaw, scan, self.offset_goal) < AVOID_TRIGGER:
                best = None
                for off in AVOID_OFFSETS:
                    if self._corridor_obstacle(idx, x, y, yaw, scan, off) >= CORRIDOR_LEN:
                        best = off
                        break
                if best is not None and best != self.offset_goal:
                    self.log('[ai_zone] 회피: 오프셋 %.1f m (전방 %.1f m 막힘)' % (best, d_center))
                    self.offset_goal = best
        elif self.offset_goal != 0.0:
            self._clear_since = self._clear_since or 0.0
            self._clear_since += dt
            if self._clear_since >= RETURN_CLEAR_S:
                self.log('[ai_zone] 원래 경로 복귀')
                self.offset_goal = 0.0
                self._clear_since = None
        step = OFFSET_RATE * dt
        self.offset += max(-step, min(step, self.offset_goal - self.offset))

    # ---------------------------------------------------------------- 메인
    def step(self, mode, idx, x, y, yaw, v, yaw_rate, scan, dt=1.0 / 15):
        """→ (목표속도 m/s, 조향 rad, 출처 문자열)"""
        self._update_offset(mode, idx, x, y, yaw, scan, dt)
        steer_rule = self._pure_pursuit(idx, x, y, yaw, v, self.offset)
        v_rule = min(ZONE_SPEED.get(mode, 15.0 / 3.6), self.speed_limit)

        steer, v_cmd, source = steer_rule, v_rule, 'rule'
        pol = self.policies.get(mode)
        if pol is not None:
            out = pol.predict(build_features(self.preview, idx, x, y, yaw, v, yaw_rate, scan))
            s_ai, v_ai = float(out[0]), float(out[1])
            if not (math.isfinite(s_ai) and math.isfinite(v_ai)):
                source = 'rule(ai NaN)'
            elif self.offset != 0.0:
                source = 'rule(회피 중)'      # 학습 분포 밖 상황 → 룰 회피 우선
            elif abs(s_ai - steer_rule) > MAX_STEER_DEVIATION:
                source = 'rule(ai 거부 %.0f°)' % math.degrees(s_ai - steer_rule)
            else:
                steer, v_cmd, source = s_ai, max(0.0, v_ai), 'ai'

        # 안전 감독: 통로 장애물 → 제동거리 기반 상한 (정책 출력보다 우선)
        # 회피 중에는 목표 통로 기준(옮겨가는 도중 원래 장애물에 걸려 급제동하지 않게),
        # 단 지금 위치 통로 바로 앞(STOP_MARGIN+1 m)에 걸리면 그쪽을 따른다.
        d_obs = self._corridor_obstacle(idx, x, y, yaw, scan, self.offset_goal)
        if self.offset != self.offset_goal:
            d_now = self._corridor_obstacle(idx, x, y, yaw, scan, self.offset)
            if d_now < STOP_MARGIN + 1.0:
                d_obs = min(d_obs, d_now)
        v_safe = math.sqrt(2.0 * SAFE_DECEL * max(0.0, d_obs - STOP_MARGIN)) if math.isfinite(d_obs) else float('inf')
        if v_safe < v_cmd:
            v_cmd = v_safe
            source += '+guard(%.1fm)' % d_obs
        v_cmd = min(v_cmd, self.speed_limit)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        self.last = dict(steer_rule=steer_rule, d_obs=d_obs, offset=self.offset, source=source)
        return v_cmd, steer, source
