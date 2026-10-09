# -*- coding: utf-8 -*-
"""[2026_AISW] Frenet 샘플링 플래너 (rospy 비의존 핵심).

전역경로 기준 (s, d) 에서 '현재 횡편차 → 목표 횡위치' 5차 다항식 후보를 만들고, 하나의 비용으로 고른다.
선택 경로 위에서 객체(정지·이동, 등속 예측)와 만나는 지점이 있으면 그 앞에 서도록 속도 상한을 낸다
→ 회피 / 앞차 추종 / 보행자 대기 / 합류·회전교차로 양보가 구간별 로직 없이 같은 규칙에서 나온다.

좌표: s = 전역경로 누적거리, d = 경로 기준 횡거리(좌 +). 순환 코스라 경로를 앞쪽으로 이어 붙여 다룬다.
"""
import math

import numpy as np

HALF_W = 1.892 / 2.0          # 차 반폭 [m]
FRONT = 3.845                 # 후륜축 → 앞범퍼 [m]
REAR = 0.79                   # 후륜축 → 뒤범퍼 [m]


class FrenetParams:
    horizon = 35.0                 # 경로 길이 [m]
    ds = 0.5                       # 경로 점 간격 [m]
    d_step = 0.25                  # 횡 목표 간격 [m]
    d_left_max = 3.5               # 좌측 최대 횡이동 (약 1개 차로, 도로 경계가 있으면 그쪽이 우선) [m]
    d_right_max = 1.0              # 우측 최대 (경계 정보가 없을 때) [m]
    d_side_max = 3.5               # 경계를 실측했을 때 좌우 최대 횡이동 [m]
    edge_margin = 0.35             # 도로 경계와 차체 사이 최소 여유 [m]
    trans_len = (1.6, 2.6)         # 횡 전이 거리 = max(최소, 계수 x 속도[m/s]) 후보들의 계수
    trans_min = (10.0, 16.0)       # 전이 거리 최소값 [m]
    free_len_min = 6.0             # 장애물 없을 때 중심 복귀 전이 거리 최소 [m]
    free_len_coef = 1.0            # 장애물 없을 때 전이 거리 = 계수 x 속도[m/s]
    clear_lat = 0.3                # 객체와 차체 사이 최소 횡여유 (넘으면 충돌로 간주) [m]
    prox_sigma = 1.2               # 근접 비용 폭 [m]
    w_prox = 30.0
    w_center = 4.0                 # 중심 이탈 (d_T^2)
    obj_d_gate = 2.0               # 정지 물체 중 경로 중심에서 이 거리 안(차로 안)만 경로 모양에 반영 [m]
    prox_lat = 1.0                 # 근접 비용을 매기는 차체-물체 횡여유 상한 [m]
    w_change = 0.8                 # 직전 선택과의 차이 (떨림 방지). w_center 보다 크면 0.25 m 에 갇혀 중심 복귀 못 함(2026-10-09)
    w_curv = 200.0                 # 최대 곡률^2
    stop_gap = 4.0                 # 정지 시 앞범퍼-객체 간격 [m]
    decel = 2.5                    # 속도 상한 계산용 감속도 [m/s^2]
    obj_speed_static = 1.2         # 이보다 느린 객체는 정지로 본다 [m/s]. 0.7 은 추적 잡음으로 길가 물체를 이동 객체로 오판(도심 감속)


class RefPath:
    """순환 전역경로 + 앞쪽으로 이어 붙인 확장 경로 (s 가 끝을 넘어가도 끊기지 않게)."""

    def __init__(self, xs, ys, wrap_m=80.0):
        x = np.asarray(xs, np.float64); y = np.asarray(ys, np.float64)
        seg = np.hypot(np.diff(x), np.diff(y))
        s = np.concatenate([[0.0], np.cumsum(seg)])
        self.length = s[-1]
        k = int(np.searchsorted(s, wrap_m))
        self.x = np.concatenate([x, x[1:k]]); self.y = np.concatenate([y, y[1:k]])
        self.s = np.concatenate([s, s[1:k] + self.length])
        t = np.gradient(np.column_stack([self.x, self.y]), axis=0)
        self.yaw = np.arctan2(t[:, 1], t[:, 0])
        self.n = len(x)
        self.last = None

    def project(self, px, py):
        """(px, py) → (s, d, 경로 방향). 직전 위치 근처 창 탐색, 멀면 전체 탐색."""
        if self.last is not None:
            lo, hi = max(0, self.last - 40), min(len(self.x), self.last + 160)
            d2 = (self.x[lo:hi] - px) ** 2 + (self.y[lo:hi] - py) ** 2
            j = int(np.argmin(d2))
            i = lo + j if d2[j] < 25.0 else None
        else:
            i = None
        if i is None:
            i = int(np.argmin((self.x[:self.n] - px) ** 2 + (self.y[:self.n] - py) ** 2))
        if i >= self.n - 1:          # 확장 구간에 들어가면 원래 경로 쪽 인덱스로 되돌림 (랩)
            i -= self.n - 1
        self.last = i
        c, sn = math.cos(self.yaw[i]), math.sin(self.yaw[i])
        dx, dy = px - self.x[i], py - self.y[i]
        return self.s[i] + c * dx + sn * dy, -sn * dx + c * dy, self.yaw[i]

    def to_xy(self, s, d):
        s = np.asarray(s, np.float64)
        x = np.interp(s, self.s, self.x); y = np.interp(s, self.s, self.y)
        yaw = np.interp(s, self.s, np.unwrap(self.yaw))
        return x - d * np.sin(yaw), y + d * np.cos(yaw)


def quintic_lateral(d0, dd0, dT, L, s):
    """s∈[0,L]: d(0)=d0, d'(0)=dd0, d''(0)=0 → d(L)=dT, d'(L)=d''(L)=0. L 이후 dT 유지."""
    a0, a1, a2 = d0, dd0, 0.0
    A = np.array([[L ** 3, L ** 4, L ** 5], [3 * L ** 2, 4 * L ** 3, 5 * L ** 4], [6 * L, 12 * L ** 2, 20 * L ** 3]])
    b = np.array([dT - a0 - a1 * L, -a1, 0.0])
    a3, a4, a5 = np.linalg.solve(A, b)
    u = np.minimum(s, L)
    d = a0 + a1 * u + a2 * u ** 2 + a3 * u ** 3 + a4 * u ** 4 + a5 * u ** 5
    d1 = a1 + 3 * a3 * u ** 2 + 4 * a4 * u ** 3 + 5 * a5 * u ** 4
    d2 = 6 * a3 * u + 12 * a4 * u ** 2 + 20 * a5 * u ** 3
    d1 = np.where(s > L, 0.0, d1); d2 = np.where(s > L, 0.0, d2)
    return d, d1, d2


class FrenetPlanner:
    def __init__(self, ref: RefPath, params=FrenetParams):
        self.ref = ref
        self.p = params
        self.prev_dT = 0.0
        self.last = {}

    def plan(self, ego_x, ego_y, ego_yaw, ego_v, objects, edges=(float('nan'), float('nan'))):
        """objects: [(x, y, vx, vy, half_len, half_wid), ...] 월드.
        edges: 자차 기준 도로 경계 (좌+, 우-), 모르면 nan.
        → dict(x, y [월드 경로], dT, speed_cap [m/s, inf=제한 없음], mode)"""
        p = self.p
        s0, d0, pyaw = self.ref.project(ego_x, ego_y)
        dyaw = (ego_yaw - pyaw + math.pi) % (2 * math.pi) - math.pi
        dd0 = math.tan(max(-1.0, min(1.0, dyaw)))
        ss = np.arange(0.0, p.horizon + 1e-6, p.ds)

        # 횡 허용 범위: 경계(자차 기준) → 경로 기준으로 환산
        # 경계를 실측했으면 경계까지(최대 d_side_max), 모르면 좌 d_left_max / 우 d_right_max
        d_hi = (min(p.d_side_max, d0 + edges[0] - HALF_W - p.edge_margin) if math.isfinite(edges[0])
                else p.d_left_max)
        d_lo = (max(-p.d_side_max, d0 + edges[1] + HALF_W + p.edge_margin) if math.isfinite(edges[1])
                else -p.d_right_max)
        d_lo = min(d_lo, 0.0); d_hi = max(d_hi, 0.0)       # 중심선은 항상 후보
        targets = np.unique(np.concatenate([np.arange(0.0, d_hi + 1e-6, p.d_step),
                                            -np.arange(p.d_step, -d_lo + 1e-6, p.d_step)]))

        # 객체를 (s, d) 로 투영 + 등속 예측용 속도 성분
        objs = []
        for (ox, oy, vx, vy, hl, hw) in objects:
            os_, od, oyaw = self._proj_free(ox, oy)
            if not (-5.0 < os_ - s0 < p.horizon + 10.0):
                continue
            c, sn = math.cos(oyaw), math.sin(oyaw)
            vs, vd = c * vx + sn * vy, -sn * vx + c * vy
            moving = math.hypot(vx, vy) > p.obj_speed_static
            objs.append((os_ - s0, od, vs if moving else 0.0, vd if moving else 0.0, hl, hw, moving))

        v_ref = max(ego_v, 2.0)
        # 길가 물체(가로등·표지판·경계석)가 경로를 밀어내지 않게: 차로 안(|d|<=gate) 정지 물체만 경로 모양에 반영
        statics = [o for o in objs if not o[6] and abs(o[1]) - o[5] <= p.obj_d_gate]
        best = None
        # 차로 안 정지 물체가 없으면 중심선으로 짧게 복귀(교정력 유지): 16 m 전이로 일반 구간 횡오차가
        # 래티스(0.12~0.17 m)보다 커지던 것(0.2~0.3 m) 개선 (2026-10-09 비교 주행)
        plans = [(max(mn, coef * v_ref), targets) for coef, mn in zip(p.trans_len, p.trans_min)]
        if not statics and abs(d0) < 1.5:
            plans = [(max(p.free_len_min, p.free_len_coef * v_ref), np.array([0.0]))]
        for L, cand in plans:
            for dT in cand:
                d, d1, d2 = quintic_lateral(d0, dd0, dT, L, ss)
                cost, _ = self._eval(ss, d, statics, v_ref)   # 경로 모양 = 차로 안 정지 물체만
                cost += p.w_center * dT ** 2 + p.w_change * (dT - self.prev_dT) ** 2
                cost += p.w_curv * float(np.max(np.abs(d2))) ** 2
                if best is None or cost < best[0]:
                    best = (cost, dT, L, d, None)
        cost, dT, L, d, _ = best
        self.prev_dT = dT
        # 속도 상한: (1) 선택 경로가 정지 물체에 막혔으면 그 앞 정지, (2) 이동 객체는 시간 창 충돌로 양보/추종.
        # 이동 객체는 피해 가지 않는다 (반대 차로 추월 = 차로 준수 위반).
        _, s_hit = self._eval(ss, d, statics, v_ref)
        x, y = self.ref.to_xy(s0 + ss, d)
        cap = float('inf')
        if s_hit is not None:
            room = max(0.0, s_hit - FRONT - p.stop_gap)
            cap = math.sqrt(2.0 * p.decel * room)
        mcap, why = self._moving_cap(ss, d, [o for o in objs if o[6]], max(ego_v, 0.5))
        if mcap < cap:
            cap = mcap
            s_hit = why
        self.last = dict(s0=s0, d0=d0, dT=dT, L=L, cost=cost, s_hit=s_hit, n_obj=len(objs))
        return dict(x=x, y=y, dT=dT, speed_cap=cap, mode=1 if abs(dT) > 0.3 or abs(d0) > 0.8 else 0)

    def _moving_cap(self, ss, d, movers, v, T=6.0, dt=0.2, buf_before=1.5, buf_after=1.0):
        """이동 객체: 앞으로 T 초 동안 등속 예측 위치가 내 경로 통로에 들어오는 시간 창 [t_in, t_out] 과
        내가 그 충돌 지점을 지나는 시간 창이 겹치면 충돌 지점 앞에서 멈출 수 있는 속도로 제한.
        같은 방향 앞차(통로 안, 경로 방향 속도 vo>0)는 '상대가 급정지해도 멈출 수 있는 속도' sqrt(vo²+2a·gap)."""
        p = self.p
        best, why = float('inf'), None
        taus = np.arange(0.0, T + 1e-6, dt)
        for (rs, rd, vs, vd, hl, hw, _) in movers:
            ps = rs + vs * taus; pd = rd + vd * taus
            ok = (ps > 0.0) & (ps < ss[-1])
            if not ok.any():
                continue
            dpath = np.interp(np.clip(ps, 0, ss[-1]), ss, d)
            inside = ok & (np.abs(pd - dpath) < HALF_W + hw + p.clear_lat + 0.3)
            if not inside.any():
                continue
            k = np.where(inside)[0]
            t_in, t_out = taus[k[0]], taus[k[-1]] + dt
            s_c = float(ps[k].min())                         # 내 경로 위 가장 가까운 충돌 지점
            gap = s_c - hl - FRONT
            vo = max(0.0, vs)                                # 경로 방향 속도 (앞차 추종용)
            if inside[0] and vo > p.obj_speed_static:        # 이미 내 통로 안에서 같은 방향 → 추종
                c = math.sqrt(max(0.0, vo * vo + 2.0 * p.decel * (gap - p.stop_gap)))
            else:
                t_arr = max(0.0, gap) / v
                t_clr = (s_c + hl + REAR) / v
                if t_arr - buf_before < t_out and t_in < t_clr + buf_after:   # 시간 창 겹침 → 양보
                    c = math.sqrt(2.0 * p.decel * max(0.0, gap - p.stop_gap))
                else:
                    continue
            if c < best:
                best, why = c, s_c
        return best, why

    def _proj_free(self, px, py):
        """객체 투영 (자차 인덱스 창을 건드리지 않게 전체 탐색)."""
        r = self.ref
        i = int(np.argmin((r.x - px) ** 2 + (r.y - py) ** 2))
        c, sn = math.cos(r.yaw[i]), math.sin(r.yaw[i])
        dx, dy = px - r.x[i], py - r.y[i]
        s = r.s[i] + c * dx + sn * dy
        if s > r.length:            # 확장 구간 → 원래 s
            s -= r.length
        return s, -sn * dx + c * dy, r.yaw[i]

    def _eval(self, ss, d, objs, v):
        """후보 경로 비용 + 경로상 첫 충돌 지점 s (없으면 None)."""
        p = self.p
        cost, s_hit = 0.0, None
        t = ss / v
        for (rs, rd, vs, vd, hl, hw, moving) in objs:
            # 객체의 시간별 예측 위치 (정지 객체는 고정)
            ps = rs + vs * t; pd = rd + vd * t
            ds_ = ps - ss                              # 경로점(후륜축)과 객체의 종방향 차
            along = (ds_ > -REAR - hl) & (ds_ < FRONT + hl)
            lat = np.abs(pd - d) - HALF_W - hw         # 차체-객체 횡여유
            hit = along & (lat < p.clear_lat)
            # 이미 차 옆/뒤에 있는 물체(뒤끝이 앞범퍼보다 뒤)는 서도 소용없음 → 충돌·정지 판단 제외, 근접 비용만.
            # (보행자 구간에서 옆 1.6~2.1 m 물체로 정지 상한 0 이 걸려 제자리 서행, 2026-10-09)
            if rs - hl < FRONT - 0.3 and not moving:
                hit = hit & False
            if hit.any():
                k = int(np.argmax(hit))
                sh = ss[k] + max(0.0, ds_[k])
                s_hit = sh if s_hit is None else min(s_hit, sh)
                cost += 1e4
            near = along & (lat < p.prox_lat)
            if near.any():
                cost += p.w_prox * float(np.sum(np.exp(-np.clip(lat[near], 0, None) ** 2 / p.prox_sigma ** 2))) * p.ds
        return cost, s_hit
