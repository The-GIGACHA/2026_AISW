# -*- coding: utf-8 -*-
"""추측항법(dead reckoning): GPS 가 끊긴 동안 속도 x IMU 절대 yaw 로 위치를 적분한다.

규정 v1.1: 올해 GPS/IMU 에 노이즈를 인가하지 않음 → IMU yaw 는 그대로 믿을 수 있고,
오차는 주로 속도 적분에서 나온다.

[2026_AISW 2026-10-09 실주행] Competition Status 속도를 벽시계로 적분하면 실제 이동보다
1.3~1.7배 크게 나왔다(부하가 클수록 커짐 — 시뮬 물리 시간이 벽시계보다 느리게 흐르는 것으로 보임).
그대로 쓰면 음영 구역 100 m 동안 위치가 30 m 앞서 나갔다. 그래서 GPS 가 있는 동안
'GPS 이동거리 / 보고속도 적분거리' 비율(scale)을 계속 추정해 두고, 음영에서는 v x scale 로 적분한다.
"""
import math


class DeadReckoning:
    def __init__(self, max_dt=0.2, scale_init=1.0, scale_alpha=0.2, scale_bounds=(0.3, 1.5)):
        self.x = None
        self.y = None
        self.t = None
        self.max_dt = max_dt          # 루프가 멈췄다 돌아올 때 한 번에 큰 점프 방지
        self.dist_since_fix = 0.0     # 마지막 GPS 이후 적분 거리 [m] (신뢰도 지표)
        # 속도 보정: 실제 이동 = scale x (보고속도 적분)
        self.scale = scale_init
        self.scale_alpha = scale_alpha
        self.scale_bounds = scale_bounds
        self._cal_anchor = None       # (x, y) 보정 구간 시작 GPS 위치
        self._cal_integral = 0.0      # 그 뒤 보고속도 적분 [m]
        self.scale_samples = 0

    @property
    def ready(self):
        return self.x is not None

    def fix(self, x, y, t=None):
        """새 GPS 측정: 위치를 그대로 덮어쓰고, 충분히 움직였으면 속도 보정값을 갱신.
        시간축(self.t)은 predict 가 매 주기 이어가므로 t 가 None 이면 건드리지 않는다."""
        if self._cal_anchor is not None and self._cal_integral > 0.0:
            gps_d = math.hypot(x - self._cal_anchor[0], y - self._cal_anchor[1])
            if self._cal_integral >= 8.0:          # 8 m 이상 모아서 비율 계산 (GPS 0.2 m 분해능 영향 축소)
                r = gps_d / self._cal_integral
                lo, hi = self.scale_bounds
                if lo <= r <= hi:
                    self.scale += self.scale_alpha * (r - self.scale)
                    self.scale_samples += 1
                self._cal_anchor, self._cal_integral = (x, y), 0.0
        elif self._cal_anchor is None:
            self._cal_anchor, self._cal_integral = (x, y), 0.0
        self.x, self.y = float(x), float(y)
        if t is not None and self.t is None:
            self.t = t
        self.dist_since_fix = 0.0

    def predict(self, v, yaw_rad, t, gps_available=False):
        """v [m/s] = 보고 속도, yaw_rad: ENU IMU yaw. GPS 샘플 사이와 음영에서 매 주기 호출.

        gps_available=True 면 보정 적분만 쌓는다(위치는 다음 fix 가 덮어씀).
        """
        if self.t is None:
            self.t = t
            return
        dt = min(max(t - self.t, 0.0), self.max_dt)
        self.t = t
        self._cal_integral += abs(v) * dt
        if self.x is None:
            return
        ds = v * self.scale * dt
        self.x += ds * math.cos(yaw_rad)
        self.y += ds * math.sin(yaw_rad)
        self.dist_since_fix += abs(ds)
