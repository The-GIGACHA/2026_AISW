# -*- coding: utf-8 -*-
"""AI 구간 공통 주행 (회전교차로·GPS 음영): 조향, 음영 구간 평행 회피, 장애물 안전 감독.

master.py 가 ai_zones(config/kcity_sections.yaml) 안에서 ZoneController.step() 을 매 주기 부른다.

  조향: 항상 룰 (Pure Pursuit, 음영 구간은 막히면 평행 회피)
  속도: shaded     → 구간 룰 속도
        roundabout → control/roundabout_entry: 정지선 앞 정지 → NPC 예측 → 진입 타이밍·속도 결정
        → 안전 감독: 장애물(/aisw/obstacles)이 경로 통로(차폭+여유)를 막으면 제동거리 기반 상한 (항상 우선)

mode:
  shaded     GPS 음영. 위치 = 추측항법.
  roundabout 회전교차로. 위치 = GPS.
"""
import math

import numpy as np

from .obstacles import objects_to_world
from .path_utils import PathPreview
from .roundabout_entry import RoundaboutEntry

WHEELBASE = 3.0
MAX_STEER = math.radians(40.0)          # 규정 차량 최대 조향각
HALF_WIDTH = 1.892 / 2.0 + 0.6          # 차폭 절반 + 여유 [m]
STEER_RATE = math.radians(75.0)         # 조향 변화율 한계 [rad/s]

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


class ZoneController:
    def __init__(self, ref_x, ref_y, model_dir, speed_limit_mps, roundabout_cfg, use_ai=True, log=print):
        self.preview = PathPreview(ref_x, ref_y)
        self.speed_limit = speed_limit_mps
        self.log = log
        self.entry = RoundaboutEntry(self.preview, roundabout_cfg, model_dir, use_model=use_ai, log=log)
        corr = np.arange(1.0, CORRIDOR_LEN + 0.01, 1.0)
        self._corr_dists = corr
        self._prev_steer = None
        self.offset = 0.0          # 현재 적용 중인 횡 오프셋 [m]
        self.offset_goal = 0.0
        self._clear_since = None
        self.last = {}

    def reset(self):
        self.offset = self.offset_goal = 0.0
        self._clear_since = None
        self._prev_steer = None

    # ---------------------------------------------------------------- 룰
    def _pure_pursuit(self, idx, x, y, yaw, v, offset=0.0):
        ld = min(max(4.0 + 0.5 * v, 4.0), 10.0)
        p = self.preview.to_ego(self.preview.points_ahead_offset(idx, [ld, ld + 1.0], offset), x, y, yaw)[0]
        alpha = math.atan2(p[1], p[0])
        return max(-MAX_STEER, min(MAX_STEER, math.atan2(2.0 * WHEELBASE * math.sin(alpha), ld)))

    @staticmethod
    def _obstacle_points(objects):
        """(N,7) 물체 → 박스 중심 + 네 모서리 (5N,2)."""
        if objects is None or len(objects) == 0:
            return None
        x, y, hx, hy = objects[:, 0], objects[:, 1], objects[:, 4] / 2.0, objects[:, 5] / 2.0
        return np.concatenate([np.column_stack(p) for p in
                               ((x, y), (x - hx, y - hy), (x - hx, y + hy), (x + hx, y - hy), (x + hx, y + hy))])

    def _corridor_obstacle(self, idx, x, y, yaw, obs, offset=0.0):
        """경로 통로 안 가장 가까운 장애물 점까지의 경로 방향 거리 [m]. 없으면 inf.
        obs: 자차(후륜축) 기준 장애물 점 (M,2)"""
        if obs is None or len(obs) == 0:
            return float('inf')
        px, py = obs[:, 0], obs[:, 1]
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

    def _update_offset(self, mode, idx, x, y, yaw, obs, dt):
        """음영 모드 전용 회피 오프셋 결정 (회전교차로는 NPC 가 움직이므로 회피 대신 감속/정지)."""
        if mode != 'shaded':
            self.reset()
            return
        d_center = self._corridor_obstacle(idx, x, y, yaw, obs, 0.0)
        if d_center < AVOID_TRIGGER:
            self._clear_since = None
            if self._corridor_obstacle(idx, x, y, yaw, obs, self.offset_goal) < AVOID_TRIGGER:
                best = None
                for off in AVOID_OFFSETS:
                    if self._corridor_obstacle(idx, x, y, yaw, obs, off) >= CORRIDOR_LEN:
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
    def step(self, mode, idx, x, y, yaw, v, yaw_rate, objects, now, dt=1.0 / 15):
        """objects: 자차 기준 장애물 (N,7) [x, y, vx, vy, 길이, 폭, 방향] 또는 None, now: 시각 [s]
        → (목표속도 m/s, 조향 rad, 출처 문자열)"""
        obs = self._obstacle_points(objects)
        self._update_offset(mode, idx, x, y, yaw, obs, dt)
        steer = self._pure_pursuit(idx, x, y, yaw, v, self.offset)
        v_rule = min(ZONE_SPEED.get(mode, 15.0 / 3.6), self.speed_limit)
        v_cmd, source = v_rule, 'rule'

        # 회전교차로 진입 판단 (정지 → 예측 → 출발)
        if mode == 'roundabout':
            v_rb = self.entry.step(idx, x, y, yaw, v, objects_to_world(objects, x, y, yaw), now)
            if v_rb is not None:
                v_cmd, source = v_rb, 'entry_' + self.entry.state

        # 안전 감독: 통로 장애물 → 제동거리 기반 상한 (진입 판단보다 우선)
        # 회피 중에는 목표 통로 기준(옮겨가는 도중 원래 장애물에 걸려 급제동하지 않게),
        # 단 지금 위치 통로 바로 앞(STOP_MARGIN+1 m)에 걸리면 그쪽을 따른다.
        d_obs = self._corridor_obstacle(idx, x, y, yaw, obs, self.offset_goal)
        if self.offset != self.offset_goal:
            d_now = self._corridor_obstacle(idx, x, y, yaw, obs, self.offset)
            if d_now < STOP_MARGIN + 1.0:
                d_obs = min(d_obs, d_now)
        v_safe = math.sqrt(2.0 * SAFE_DECEL * max(0.0, d_obs - STOP_MARGIN)) if math.isfinite(d_obs) else float('inf')
        if v_safe < v_cmd:
            v_cmd = v_safe
            source += '+guard(%.1fm)' % d_obs
        v_cmd = min(v_cmd, self.speed_limit)
        steer = max(-MAX_STEER, min(MAX_STEER, steer))
        # 조향 변화율 제한 (normal_drive 와 같은 5°/사이클 @15 Hz)
        if self._prev_steer is not None:
            lim = STEER_RATE * dt
            steer = self._prev_steer + max(-lim, min(lim, steer - self._prev_steer))
        self._prev_steer = steer
        self.last = dict(d_obs=d_obs, offset=self.offset, source=source)
        return v_cmd, steer, source
