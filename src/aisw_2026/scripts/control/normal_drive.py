# -*- coding: utf-8 -*-
"""룰 구간 제어: 플래너의 /local_path 를 Pure Pursuit 로 따라가고, 곡률로 목표속도를 정한다.

master.py 가 RuleController 를 만들어 별도 스레드에서 main() 을 돌린다.
AI 구간(/jamming_mode_active=True)에서는 대기하고 master 가 직접 /ctrl_cmd 를 낸다.
"""
import math
from collections import deque

import numpy as np
import rospy
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage
from nav_msgs.msg import Path
from pyproj import Proj
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, UInt8
from tf.transformations import euler_from_quaternion

from .path_utils import (PATH, CubicSpline2D, NearestIndexer, get_section, load_ref_path,
                     nearest_index_global, ros_sections)


class Parameter:
    vehicle_wheelbase = 3.000           # 아이오닉5 휠베이스 [m]
    max_velocity = 40.0 / 3.6           # 최대 속도 [m/s] (throttle 제어라 여유를 둔다. 46.5 kph 이상 금지)
    max_local_velocity = 25.0 / 3.6     # 로컬경로(회피) 주행 최대 속도 [m/s]

    curve_range_start = 8               # 곡률 판단 시작 (로컬경로 인덱스, 자차 기준)
    curve_range_end = 12                # 곡률 판단 끝
    curve_filter = 8                    # 반경 이동평균 개수
    mu = 0.55                           # 마찰계수


STEER_RATE_DEG = 5.0   # 조향 변화율 제한 [deg/사이클, 15 Hz]


class PurePursuit:
    def __init__(self):
        self.wb = Parameter.vehicle_wheelbase
        self.path = None

    @staticmethod
    def normalize_rad_angle(angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    @staticmethod
    def normalize_180(deg):
        return (deg + 180) % 360 - 180

    def update_path(self, path):
        self.path = path

    def run(self, ego_x, ego_y, ego_yaw, ego_ind, ego_vel, target_vel):
        """→ (목표속도 m/s, 조향 deg)"""
        if self.path is None:
            return None
        # 거리 기반 연속 룩어헤드: L=0.6v+3.0 (4~12m). 경로를 따라 실제 누적거리로 목표점 탐색
        # (로컬경로 실제 간격 0.25m 를 0.5m 로 가정하면 룩어헤드가 절반이 되어 위빙 발생)
        look = min(max(0.6 * abs(ego_vel) + 3.0, 4.0), 12.0)
        ti, acc = ego_ind, 0.0
        while ti < self.path.length - 1 and acc < look:
            acc += math.hypot(self.path.cx[ti + 1] - self.path.cx[ti], self.path.cy[ti + 1] - self.path.cy[ti])
            ti += 1
        tx, ty = self.path.cx[ti], self.path.cy[ti]

        alpha = self.normalize_rad_angle(math.atan2(ty - ego_y, tx - ego_x) - math.radians(ego_yaw))
        dist = math.hypot(tx - ego_x, ty - ego_y)
        steering_rad = 0.0 if dist <= 0.0 else math.atan2(2.0 * self.wb * math.sin(alpha), dist)

        # 횡편차(cross-track) 복귀항 — 순수추종이 평행이탈을 못 잡는 문제 보완
        try:
            px, py = self.path.cx[ego_ind], self.path.cy[ego_ind]
            pyaw = math.radians(self.path.cyaw[ego_ind])
            e_ct = -math.sin(pyaw) * (ego_x - px) + math.cos(pyaw) * (ego_y - py)  # 경로 왼쪽 +
            v_ct = max(abs(ego_vel), 2.0)
            steering_rad -= math.atan2(0.45 * e_ct, v_ct)
        except Exception:
            pass

        steering_deg = self.normalize_180(math.degrees(steering_rad))
        # 조향 EMA(α=0.45) + 변화율 제한(사이클당 5도@15Hz≈75도/s) — 위빙 억제하되 복귀조향 허용
        prev = getattr(self, '_steer_filt', 0.0)
        filt = 0.45 * steering_deg + 0.55 * prev
        filt = prev + np.clip(filt - prev, -STEER_RATE_DEG, STEER_RATE_DEG)
        self._steer_filt = filt
        return np.clip(abs(target_vel), 0.2, Parameter.max_velocity), np.clip(filt, -40.0, 40.0)


class AccelCmdConverter:
    """목표속도 → accel/brake (PID)."""

    def __init__(self, rate_hz):
        self.p_gain = 0.35
        self.i_gain = 0.05
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 1 / rate_hz
        self.output = 0.0

    def run(self, target_vel, current_vel):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        # 조건부 적분(|오차| 기준) — 큰 감속 오차에서 적분이 쌓여 재출발이 늦어지는 windup 방지
        if abs(error) <= 5 and abs(self.output) < 1.0:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error - self.prev_error) / self.controlTime
        self.output = p_control + self.i_control + d_control
        self.prev_error = error

        # 데드밴드: |출력|<0.12 는 타행 — accel/brake 채터링·브레이크등 점멸 방지
        if self.output > 0.12:
            return self.output, 0.0
        if self.output < -0.12:
            return 0.0, -self.output
        return 0.0, 0.0


class CurvatureVelocity:
    """로컬경로 앞쪽 몇 점에 원을 맞춰 반경 → 횡가속 한계 속도 (전역경로 구간 속도 이하)."""

    def __init__(self, global_path):
        self.curve_history = deque(maxlen=Parameter.curve_filter)
        self.global_path = global_path
        self.path = None

    def update_path(self, path):
        self.path = path

    def run(self, ego_global_ind, ego_local_ind):
        x_list, y_list = [], []
        for box in range(Parameter.curve_range_start, Parameter.curve_range_end):
            idx = ego_local_ind + box
            if idx < 0 or idx >= self.path.length:
                continue
            x, y = self.path.cx[idx], self.path.cy[idx]
            x_list.append([-2 * x, -2 * y, 1])
            y_list.append((-x * x) - (y * y))

        x_matrix, y_matrix = np.array(x_list), np.array(y_list)
        if x_matrix.shape[0] < 3:          # 점 3개 미만 → 원 적합 불가
            r = 1e6
        else:
            try:
                a_matrix, _, rank, _ = np.linalg.lstsq(x_matrix, y_matrix, rcond=None)
                if rank < 3:               # 거의 일직선
                    r = 1e6
                else:
                    a, b, c = a_matrix
                    r2 = a * a + b * b - c
                    r = math.sqrt(r2) if r2 > 0 else 1e6
            except np.linalg.LinAlgError:
                r = 1e6

        self.curve_history.append(r)
        smoothed_r = sum(self.curve_history) / len(self.curve_history) if len(self.curve_history) >= 2 else r
        v_max = math.sqrt(smoothed_r * 9.81 * Parameter.mu)
        return float(min(v_max, self.global_path.cv[ego_global_ind]))


class RuleController:
    def __init__(self):
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg = CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType = 1

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.rate_hz = 15

        self.ref_path = load_ref_path()
        # 구간 속도표 (config speed_profile) 로 맵 velocity(전부 20 kph) 대체
        self._apply_speed_profile(get_section(ros_sections(), 'speed_profile', None))
        # 전역경로 인덱스는 직전 인덱스 주변 창에서만 탐색 (루프 끝에서 0 으로 되감기 방지)
        self.global_indexer = NearestIndexer(self.ref_path.cx, self.ref_path.cy)

        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.adjusted_yaw = 0.0
        self.ego_vel = 0.0
        self.ego_index_global = 0
        self.ego_index_local = 0

        self.velocity = 0.0
        self.steering = 0.0
        self.curvedvelocity = 0.0

        self.odom_flag = False
        self.gps_flag = False
        self.imu_flag = False

        # 로컬경로: 새 경로는 pending 에 받아 두고 메인 루프에서 조건부로 교체
        self.local_path = None
        self.pending_local_path = None
        self.has_pending = False
        self.pending_stamp = 0.0
        self.SWAP_MIN_INTERVAL = 0.25   # 최소 교체 간격 [s]
        self.SWAP_TIMEOUT = 0.4         # 짧은 경로라도 이 시간 지나면 교체 허용 [s]
        self._last_swap_time = 0.0
        self.planner_mode = 0           # 0 전역경로 / 1 로컬경로(회피)

        self.traffic_light_brake = False   # master 가 설정
        self.external_speed_cap = None     # master 가 설정 [m/s] (신호등/플래너 상한)
        self.jamming_mode_active = False   # AI 구간 제어 중

        # 플래너 STOP 경로(첫 점 z=-100) 수신 시 제동 유지
        self._planner_stop_until = 0.0
        self._planner_stop_hold = 0.2

        self.cmd_converter = AccelCmdConverter(self.rate_hz)
        self.purepursuit = PurePursuit()
        self.curvature_vel = CurvatureVelocity(self.ref_path)

        # 모든 속성 초기화 후 구독 시작 (콜백 레이스 방지)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self.comp_callback)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/local_path', Path, self.local_path_callback)
        rospy.Subscriber('/planner_mode', UInt8, self.planner_mode_callback)
        rospy.Subscriber('/jamming_mode_active', Bool, self.jamming_mode_callback)

    # ------------------------------------------------------------------ 콜백
    def comp_callback(self, msg):
        self.ego_vel = msg.velocity.x   # 전진속도 [m/s] (후진 시 음수)
        self.odom_flag = True

    def gps_callback(self, msg):
        """차량 후륜중심 기준. UTM - offset = 맵 좌표."""
        self._last_gps_t = rospy.get_time()
        utm_x, utm_y = self.proj_UTM(msg.longitude, msg.latitude)
        self.ego_x = utm_x - msg.eastOffset
        self.ego_y = utm_y - msg.northOffset
        self._gps_x, self._gps_y = self.ego_x, self.ego_y   # 원시 GPS (사이 위치 예측 기준점)
        self.gps_flag = True

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.ego_yaw = math.degrees(yaw)
        self.imu_flag = True

    def local_path_callback(self, msg):
        # STOP 신호: 첫 pose 의 z == -100
        if msg.poses and msg.poses[0].pose.position.z == -100.0:
            self._planner_stop_until = max(self._planner_stop_until, rospy.get_time() + self._planner_stop_hold)
            return
        n = len(msg.poses)
        if n < 3:
            return
        xs = np.fromiter((p.pose.position.x for p in msg.poses), dtype=np.float64, count=n)
        ys = np.fromiter((p.pose.position.y for p in msg.poses), dtype=np.float64, count=n)

        sp = CubicSpline2D(xs, ys)
        ss = sp.s[:-1]
        cx, cy = sp.calc_position_vec(ss)
        cyaw = sp.calc_yaw_vec(ss)
        ck = sp.calc_curvature_vec(ss)
        m = ss.shape[0]
        new_path = PATH(cx, cy, cyaw, ck, [Parameter.max_velocity * 3.6] * m, ['NORMAL_DRIVING'] * m, [4] * m)

        # 바로 교체하지 않고 pending 에 둔다
        self.pending_local_path = new_path
        self.has_pending = True
        self.pending_stamp = rospy.get_time()

        # 첫 경로만 즉시 채택 (10점 미만 퇴화 경로는 제외)
        if self.local_path is None and new_path.length >= 10:
            self._swap_local_path()
            print('[local_path] first path applied immediately')

    def planner_mode_callback(self, msg):
        self.planner_mode = int(msg.data)

    def jamming_mode_callback(self, msg):
        self.jamming_mode_active = msg.data

    # ------------------------------------------------------------------ 유틸
    def _swap_local_path(self):
        self.local_path = self.pending_local_path
        self.has_pending = False
        self.purepursuit.update_path(self.local_path)
        self.curvature_vel.update_path(self.local_path)

    def _apply_speed_profile(self, prof):
        if not prof:
            rospy.loginfo('[normal_drive] speed_profile 없음 — 맵 velocity 사용')
            return
        n = self.ref_path.length
        cv = [float(prof.get('default_kph', 20.0))] * n
        for a, b, kph in prof.get('zones', []):
            for i in range(max(0, int(a)), min(n, int(b) + 1)):
                cv[i] = float(kph)
        cap = Parameter.max_velocity * 3.6
        self.ref_path.cv = [min(v, cap) / 3.6 for v in cv]
        rospy.loginfo('[normal_drive] speed_profile 적용: 기본 %.0f kph, 구간 %d개, 상한 %.0f kph',
                      prof.get('default_kph', 20.0), len(prof.get('zones', [])), cap)

    def _publish(self, accel, brake, steering_rad):
        self.ctrl_cmd_msg.accel = accel
        self.ctrl_cmd_msg.brake = brake
        self.ctrl_cmd_msg.steering = steering_rad
        self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

    # ------------------------------------------------------------------ 메인 루프
    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            # AI 구간이면 master 가 제어 — 대기
            if self.jamming_mode_active:
                rate.sleep()
                continue

            # 플래너 STOP 유지
            if rospy.get_time() < self._planner_stop_until:
                self._publish(0.0, 0.7, 0.0)
                rate.sleep()
                continue

            # GPS 두절: 1.5~3초 = 직진 크리프, 3초 초과 = 정지
            # (MORAI 부하 시 /gps 6~8 Hz → 0.5~0.7초 간격은 정상. 보통은 master 가 1.2초에 추측항법으로 먼저 가져간다)
            gps_age = rospy.get_time() - getattr(self, '_last_gps_t', 0.0)
            if getattr(self, '_last_gps_t', 0.0) > 0.0 and gps_age > 1.5:
                if gps_age > 3.0:
                    self._publish(0.0, 0.6, 0.0)
                    rospy.logwarn_throttle(2.0, '[GPS두절 %.1fs] 정지 유지', gps_age)
                else:
                    self._publish(0.12, 0.0, 0.0)
                    rospy.logwarn_throttle(1.0, '[GPS음영 %.1fs] 직진 크리프', gps_age)
                rate.sleep()
                continue

            if not (self.odom_flag and self.gps_flag and self.imu_flag):
                print('Waiting for GPS and IMU...')
                rate.sleep()
                self._publish(0.0, 1.0, 0.0)
                continue

            if self.local_path is None:
                print('Waiting for /local_path ...')
                self._publish(0.0, 1.0, 0.0)
                rate.sleep()
                continue

            # 신호등 제동 (master 판단)
            if self.traffic_light_brake:
                self._publish(0.0, 1.0, 0.0)
                rate.sleep()
                continue

            # pending → current 교체 (최소 간격, 10점 미만은 타임아웃 후에만)
            if self.has_pending:
                now = rospy.get_time()
                if (now - self._last_swap_time) >= self.SWAP_MIN_INTERVAL and \
                        (self.pending_local_path.length >= 10 or (now - self.pending_stamp) > self.SWAP_TIMEOUT):
                    self._swap_local_path()
                    self._last_swap_time = now

            # GPS 샘플 사이 위치 예측: 마지막 GPS + 속도 x 경과시간 (최대 0.5초)
            # (GPS 6~8 Hz 에서 위치가 0.4~0.8 m 계단식으로 튀어 조향이 떨리지 않게)
            if hasattr(self, '_gps_x'):
                dt = min(max(rospy.get_time() - self._last_gps_t, 0.0), 0.5)
                yaw = math.radians(self.ego_yaw)
                v = self.ego_vel * getattr(self, 'speed_scale', 1.0)   # master 가 GPS 로 추정한 속도 보정
                self.ego_x = self._gps_x + v * dt * math.cos(yaw)
                self.ego_y = self._gps_y + v * dt * math.sin(yaw)
            self.ego_index_global = self.global_indexer.find(self.ego_x, self.ego_y)
            self.ego_index_local = nearest_index_global(self.local_path.cx, self.local_path.cy, self.ego_x, self.ego_y)

            self.curvedvelocity = self.curvature_vel.run(self.ego_index_global, self.ego_index_local)
            if self.planner_mode == 1:
                self.curvedvelocity = min(self.curvedvelocity, Parameter.max_local_velocity)
            if self.external_speed_cap is not None:
                self.curvedvelocity = min(self.curvedvelocity, self.external_speed_cap)

            self.adjusted_yaw = self.ego_yaw
            self.velocity, self.steering = self.purepursuit.run(
                self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_index_local, self.ego_vel, self.curvedvelocity)

            accel_cmd, brake_cmd = self.cmd_converter.run(self.velocity, self.ego_vel)
            # longlCmdType 1: accel/brake 0~1
            self._publish(min(max(accel_cmd, 0.0), 1.0), min(max(brake_cmd, 0.0), 1.0),
                          math.radians(PurePursuit.normalize_180(self.steering)))
            rate.sleep()
