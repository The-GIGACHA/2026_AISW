# -*- coding: utf-8 -*-
"""추측항법(dead reckoning): GPS 가 끊긴 동안 속도 x IMU 절대 yaw 로 위치를 적분한다.

규정 v1.1: 올해 GPS/IMU 에 노이즈를 인가하지 않음 → IMU yaw 는 그대로 믿을 수 있고,
오차는 주로 속도(Competition Status vel_x) 적분에서 나온다. 음영 박스(약 100 m)를
지나는 동안 수십 cm 수준을 기대하지만, 실제 오차는 기록 로그로 확인할 것.
"""
import math


class DeadReckoning:
    def __init__(self, max_dt=0.2):
        self.x = None
        self.y = None
        self.t = None
        self.max_dt = max_dt          # 루프가 멈췄다 돌아올 때 한 번에 큰 점프 방지
        self.dist_since_fix = 0.0     # 마지막 GPS 이후 적분 거리 [m] (신뢰도 지표)

    @property
    def ready(self):
        return self.x is not None

    def fix(self, x, y, t):
        """새 GPS 측정: 위치를 그대로 덮어쓴다 (노이즈 없음 전제)."""
        self.x, self.y, self.t = float(x), float(y), t
        self.dist_since_fix = 0.0

    def predict(self, v, yaw_rad, t):
        """v [m/s], yaw_rad: ENU 기준 IMU yaw. GPS 가 없을 때 매 주기 호출."""
        if self.x is None or self.t is None:
            self.t = t
            return
        dt = min(max(t - self.t, 0.0), self.max_dt)
        self.t = t
        ds = v * dt
        self.x += ds * math.cos(yaw_rad)
        self.y += ds * math.sin(yaw_rad)
        self.dist_since_fix += abs(ds)
