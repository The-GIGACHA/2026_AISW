# -*- coding: utf-8 -*-
"""회전교차로 진입 판단: 정지선 앞 정지 → NPC 움직임 예측 → 진입 타이밍·진입 속도 결정.

ai_zone_drive.ZoneController 가 회전교차로 구간에서 매 주기 step() 을 부른다 (조향은 룰, 여기서는 속도만).

상태
  APPROACH  정지 목표(앞범퍼가 정지선 stop_dist[m] 앞)까지 감속
  WAIT      정지. NPC 를 예측해 지금 출발하면 안전한지 매 주기 판단
  GO        진입 속도 v_entry 로 출발 (안전 감독은 ZoneController 가 계속 적용)
  DONE      회전교차로 빠져나감 → 룰 속도

NPC 예측: 회전교차로 원(center, radius) 위의 NPC 는 원을 따라 각속도 일정, 나머지는 등속 직선.
자차 예측: 현재 속도에서 ACCEL 로 v_entry 까지 가속하며 전역경로를 따라감.
차체는 원 3개로 근사해 예측 시간 동안 자차–NPC 최소 간격(pred_gap)을 구한다.

진입 결정 (우선순위)
  1. 실험 trial (tools/test/roundabout/run_episodes.py 가 매 회 지정): pred_gap(v_entry) >= accept_gap 이면 출발
  2. 학습 모델 models/roundabout_entry.npz: 후보 속도마다 실제 최소 간격을 예측, safe_gap 이상인 가장 빠른 속도로 출발
  3. 기본 규칙: pred_gap(v_entry) >= accept_gap
"""
import math
import os

import numpy as np

from .mlp import MLP

FRONT_BUMPER = 3.845          # 후륜축 → 앞범퍼 [m]
ACCEL = 1.5                   # 출발 가속 가정 [m/s^2]
DECEL = 1.5                   # 정지 접근 감속 [m/s^2]
APPROACH_SPEED = 15.0 / 3.6   # 정지 목표까지 최대 속도 [m/s]
HORIZON = 8.0                 # 예측 시간 [s]
DT = 0.2
RING_BAND = 3.5               # 원 반지름 ± 이 안이면 회전교차로를 도는 NPC [m]
EGO_CIRCLES = (0.0, 1.53, 3.06)   # 후륜축 기준 자차 원 중심 [m] (차체 -0.79 ~ +3.845)
EGO_RADIUS = 1.22
V_CANDIDATES = np.arange(8.0, 30.1, 2.0) / 3.6   # 모델 사용 시 진입 속도 후보 [m/s]

FEATURE_VERSION = 1
N_UP = 3
FEATURE_DIM = 3 + 2 * N_UP + 2 + 2


class RoundaboutEntry:
    def __init__(self, preview, cfg, model_dir, use_model=True, log=print):
        self.preview = preview
        self.log = log
        self.s_line = preview.s[int(cfg['stop_line_idx'])]
        self.s_exit = preview.s[int(cfg['exit_idx'])]
        self.center = np.array(cfg['center'], np.float64)
        self.radius = float(cfg['radius'])
        mx, my, _ = preview.pose_at(preview.s[int(cfg['merge_idx'])])
        self.th_merge = math.atan2(my - self.center[1], mx - self.center[0])
        ex, ey, _ = preview.pose_at(self.s_exit)
        self.arc_exit = ((math.atan2(ey - self.center[1], ex - self.center[0]) - self.th_merge) % (2 * math.pi)) * self.radius
        self.default = dict(stop_dist=float(cfg.get('stop_dist', 4.0)),
                            accept_gap=float(cfg.get('accept_gap', 4.0)),
                            v_entry=float(cfg.get('v_entry_kph', 15.0)) / 3.6)
        self.safe_gap = float(cfg.get('safe_gap', 2.0))
        self.model, why = (MLP.load(os.path.join(model_dir, 'roundabout_entry.npz'), FEATURE_DIM, FEATURE_VERSION)
                           if use_model else (None, '모델 사용 안 함'))
        log('[roundabout_entry] 진입 판단 모델: %s' % (why if self.model is None else 'OK (%s)' % why))
        self.start()

    # ------------------------------------------------------------------ 상태
    def start(self, trial=None):
        """회전교차로 구간 진입 시 호출. trial = {stop_dist, accept_gap, v_entry_kph} (실험) 또는 None."""
        self.trial = None
        self.p = dict(self.default)
        if trial:
            self.trial = dict(trial)
            self.p.update(stop_dist=float(trial['stop_dist']), accept_gap=float(trial['accept_gap']),
                          v_entry=float(trial['v_entry_kph']) / 3.6)
        self.state = 'APPROACH'
        self.wait_t0 = None
        self.decision = None

    def info(self):
        return dict(state=self.state, trial=self.trial, decision=self.decision)

    def step(self, idx, x, y, yaw, v, objects_world, now):
        """→ 목표속도 [m/s] (DONE 이면 None = 룰 속도)"""
        s_ego = self.preview.s[int(idx)]
        s_stop = self.s_line - self.p['stop_dist'] - FRONT_BUMPER
        if s_ego >= self.s_exit:
            self.state = 'DONE'
        if self.state == 'DONE':
            return None
        if self.state == 'APPROACH':
            d = s_stop - s_ego
            if s_ego > self.s_line - FRONT_BUMPER:         # 이미 정지선을 넘음 → 그대로 진행
                self.state = 'GO'
            elif d <= 0.3 and v < 0.3:
                self.state, self.wait_t0 = 'WAIT', now
            else:
                return min(APPROACH_SPEED, math.sqrt(2.0 * DECEL * max(d, 0.0)))
        if self.state == 'WAIT':
            v_go = self._decide(s_ego, v, objects_world, now)
            if v_go is None:
                return 0.0
            self.state = 'GO'
            self.p['v_entry'] = v_go
        return self.p['v_entry']

    # ------------------------------------------------------------------ 결정
    def _decide(self, s_ego, v, objs, now):
        stop_gap = self.s_line - (s_ego + FRONT_BUMPER)
        if self.model is not None and self.trial is None:
            best = None
            for vc in V_CANDIDATES:
                pg = self.pred_gap(s_ego, v, vc, objs)
                f = self.features(stop_gap, vc, pg, objs)
                if float(self.model.predict(f)[0]) >= self.safe_gap:
                    best = (vc, pg, f)
            if best is None:
                return None
            vc, pg, f = best
        else:
            vc = self.p['v_entry']
            pg = self.pred_gap(s_ego, v, vc, objs)
            if pg < self.p['accept_gap']:
                return None
            f = self.features(stop_gap, vc, pg, objs)
        self.decision = dict(t=now, wait_s=now - self.wait_t0, stop_gap=stop_gap, v_entry=vc, pred_gap=pg,
                             features=[float(a) for a in f], source='trial' if self.trial else
                             ('model' if self.model is not None else 'rule'))
        return vc

    # ------------------------------------------------------------------ 예측
    def _ring_state(self, objs):
        """(N,) 원 위 여부, 각도, 각속도 [rad/s]"""
        rel = objs[:, :2] - self.center
        r = np.hypot(rel[:, 0], rel[:, 1])
        speed = np.hypot(objs[:, 2], objs[:, 3])
        on = (np.abs(r - self.radius) < RING_BAND) & (speed > 0.5)
        th = np.arctan2(rel[:, 1], rel[:, 0])
        w = (rel[:, 0] * objs[:, 3] - rel[:, 1] * objs[:, 2]) / np.maximum(r * r, 1e-6)
        return on, r, th, w

    def pred_gap(self, s_ego, v0, v_entry, objs):
        """지금 출발해 v_entry 로 진입할 때 예측 시간 동안 자차–NPC 최소 간격 [m] (차체 원 근사)."""
        if objs is None or len(objs) == 0:
            return 10.0
        t = np.arange(0.0, HORIZON + 1e-6, DT)
        t_acc = max(v_entry - v0, 0.0) / ACCEL
        s = np.where(t < t_acc, v0 * t + 0.5 * ACCEL * t * t, v0 * t_acc + 0.5 * ACCEL * t_acc ** 2 + v_entry * (t - t_acc))
        s = s_ego + s
        keep = s <= self.s_exit + 5.0
        t, s = t[keep], s[keep]
        ex, ey, eyaw = self.preview.pose_at(s)
        ego = np.stack([ex[:, None] + np.cos(eyaw)[:, None] * np.array(EGO_CIRCLES),
                        ey[:, None] + np.sin(eyaw)[:, None] * np.array(EGO_CIRCLES)], -1)        # (T,3,2)

        on, r, th, w = self._ring_state(objs)
        th_t = th[None, :] + w[None, :] * t[:, None]                                           # (T,N)
        ring_x = self.center[0] + r[None, :] * np.cos(th_t)
        ring_y = self.center[1] + r[None, :] * np.sin(th_t)
        ring_h = th_t + np.sign(w)[None, :] * math.pi / 2
        lin_x = objs[None, :, 0] + objs[None, :, 2] * t[:, None]
        lin_y = objs[None, :, 1] + objs[None, :, 3] * t[:, None]
        moving = np.hypot(objs[:, 2], objs[:, 3]) > 0.5
        lin_h = np.where(moving, np.arctan2(objs[:, 3], objs[:, 2]), objs[:, 6])[None, :].repeat(len(t), 0)
        ox = np.where(on[None, :], ring_x, lin_x)
        oy = np.where(on[None, :], ring_y, lin_y)
        oh = np.where(on[None, :], ring_h, lin_h)
        L, W = np.maximum(objs[:, 4], 0.5), np.maximum(objs[:, 5], 0.5)
        offs = np.stack([-L / 3.0, np.zeros_like(L), L / 3.0], -1)                              # (N,3)
        o_r = np.hypot(L / 6.0, W / 2.0)                                                       # (N,)
        npc = np.stack([ox[:, :, None] + np.cos(oh)[:, :, None] * offs[None],
                        oy[:, :, None] + np.sin(oh)[:, :, None] * offs[None]], -1)              # (T,N,3,2)
        d = np.linalg.norm(ego[:, None, :, None, :] - npc[:, :, None, :, :], axis=-1)          # (T,N,3,3)
        gap = d - EGO_RADIUS - o_r[None, :, None, None]
        return float(min(gap.min(), 10.0))

    def features(self, stop_gap, v_entry, pred_gap, objs):
        """모델 입력 (FEATURE_DIM): 정지 간격, 진입 속도, 예측 간격, 진입로 쪽으로 오는 NPC 3대 (호 거리, 속도),
        진입 후 앞쪽 NPC 1대 (호 거리, 속도), 원 위 NPC 수, 그 밖 근처 물체 수."""
        up = [(3.0, 0.0)] * N_UP
        down = (3.0, 0.0)
        n_ring = n_other = 0
        if objs is not None and len(objs):
            on, r, th, w = self._ring_state(objs)
            speed = np.hypot(objs[:, 2], objs[:, 3])
            cand_up, cand_down = [], []
            for k in range(len(objs)):
                if on[k] and w[k] > 0:                    # 반시계로 도는 NPC
                    n_ring += 1
                    arc_to = ((self.th_merge - th[k]) % (2 * math.pi)) * self.radius   # 합류점까지 남은 호
                    arc_past = ((th[k] - self.th_merge) % (2 * math.pi)) * self.radius
                    if arc_past < self.arc_exit:
                        cand_down.append((arc_past / 30.0, speed[k] / 10.0))
                    else:
                        cand_up.append((arc_to / 30.0, speed[k] / 10.0))
                elif np.hypot(objs[k, 0] - self.center[0], objs[k, 1] - self.center[1]) < self.radius + 30.0:
                    n_other += 1
            cand_up.sort()
            up = (cand_up + up)[:N_UP]
            if cand_down:
                down = min(cand_down)
        return np.array([stop_gap, v_entry * 3.6 / 10.0, min(pred_gap, 10.0)]
                        + [a for p in up for a in p] + list(down) + [n_ring / 5.0, n_other / 5.0], np.float64)
