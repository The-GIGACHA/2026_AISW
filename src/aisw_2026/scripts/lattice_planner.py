#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""래티스 로컬 경로 플래너 (기본 플래너).

- 장애물 없음: 전역경로 앞 30 m 를 /local_path 로 (횡편차가 크면 코사인으로 부드럽게 복귀)
- 장애물 있음: 전역경로 기준 (s, d) 격자에서 DP 로 최소 비용 회피 경로
- 끼어들기 감시(/merge_stop_flag), 단일 차로 앞차 상대속도(/nearest_vrel) 도 낸다.
구간 설정: config/kcity_sections.yaml 의 lattice.*
"""
import math
import struct

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import GPSMessage
from nav_msgs.msg import Path
from pyproj import Proj
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray, UInt8
from tf.transformations import euler_from_quaternion
from vision_msgs.msg import Detection3DArray

from control.path_utils import DEFAULT_MAP, CubicSpline2D, get_section, load_map_fields, load_ref_path, ros_sections

OBS_HOLD_S = 1.5             # 장애물 기억 시간 [s]
OBS_S_RANGE = (-2.0, 30.0)   # 래티스 모드를 켜는 장애물의 경로 방향 범위 [m] (자차 기준)
OBS_D_GATE = 2.0             # 래티스 모드를 켜는 장애물 중심의 경로 횡거리 한계 [m] (차 반폭 0.95 + 여유 0.6 + 물체 반폭 ~0.5)
BLEND_D_MIN = 1.0            # 이 횡편차 이상일 때만 전역 복귀 블렌딩 [m]
BLEND_M = 20.0               # 전역 복귀 블렌딩 거리 [m]
RETURN_TIMEOUT_S = 3.0       # 장애물 없이 이 시간 지나면 횡편차 무관 전역 복귀 [s]
RETURN_D_MAX = 0.5           # 래티스→전역 전환 허용 횡편차 [m]
LATERAL_LANES = 1.0          # 래티스 횡방향 최대 이동 차로 수


class Parameter:
    road_width = 3.2            # 도로 폭 [m]
    dd_sampling_num = 9         # 횡방향 샘플 수
    lookahead_distance = 30.0   # 경로 종방향 길이 [m]
    ds_sampling_num = 16        # 종방향 샘플 수 (현재 위치 포함)
    ds_interval = lookahead_distance / (ds_sampling_num - 1)
    local_step_size = 0.5       # 로컬경로 점 간격 [m]

    safety_buf_s = 1.3          # 장애물 s방향 안전거리 [m]
    safety_buf_d = 1.2          # 장애물 d방향 안전거리 [m]

    pre_window_s = 18.0         # 장애물 앞쪽으로 미리 페널티를 뿌릴 s거리 [m]
    sigma_d = 1.7               # 가우시안 표준편차 [m]
    pre_cost_weight = 200.0     # 장애물 차선 선제적 가중치

    lateral_cost_weight = 3.0   # 전역경로에서 벗어남 비용
    obs_cost_weight = 100       # 장애물 근접 비용
    smooth_cost_weight = 2.5    # 횡방향 이동 비용 (1차)
    second_diff_weight = 9.0    # 2차 차분(지그재그 억제) 비용

    HYBRID_MODE_ENABLED = True          # 장애물 없을 때 전역경로 사용
    OBSTACLE_DETECTION_DISTANCE = lookahead_distance
    MODE_SWITCH_HYSTERESIS = 1.0        # 전역경로 모드 복귀 히스테리시스 [m]

    # 회피 후 래티스 모드 유지 (LiDAR 3 Hz 에서 장애물이 1 s 이상 안 잡혀도 회피 도중 전역 복귀 방지)
    AVOIDANCE_RECOVERY_TIME = 4.0       # [s]
    AVOIDANCE_RECOVERY_DISTANCE = 15.0  # [m]

    car_following_distance = 18.0       # 앞차 인식 최대 거리 [m]

    BIG = float('inf')
    MARGIN_BLOCK_COST = 1e8   # 마진영역 전용 큰 유한 비용

    block_time = 1.10         # 장애물과 겹친 전역경로 칸을 막아 두는 시간 [s]
    HOTSPOT_RADIUS = 1.3      # 막힌 전역경로 칸 원형 존 반지름 [m]

    # 끼어들기 구간별 파라미터 (i번째 = kcity_sections.yaml lattice.merge_zones[i])
    MERGE_ADJ_AHEAD = [car_following_distance - 2.3, car_following_distance, car_following_distance - 2.3,
                       car_following_distance - 2.3, car_following_distance - 2.3]   # 옆 차로 감시 전방거리 [m]
    MERGE_ADJ_D_MIN = [road_width * 0.6, road_width * 0.7, road_width * 0.6, road_width * 0.6, road_width * 0.7]
    MERGE_ADJ_D_MAX = [road_width * 3.5, road_width * 5.5, road_width * 4.5, road_width * 4.5, road_width * 5.5]
    MERGE_BAND = [road_width * 3.5, road_width * 5.5, road_width * 4.5, road_width * 4.5, road_width * 5.5]
    MERGE_DEC_THR = [local_step_size - 0.1, local_step_size, local_step_size - 0.1, local_step_size - 0.1, local_step_size]


class LatticePlanner:
    def __init__(self):
        rospy.init_node('lattice_planner', anonymous=True)
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.path_mode = rospy.Publisher('/planner_mode', UInt8, queue_size=1, latch=True)
        self.vrel_pub = rospy.Publisher('/nearest_vrel', Float32, queue_size=1)
        self.merge_stop_pub = rospy.Publisher('/merge_stop_flag', UInt8, queue_size=1)
        # 후보 경로 비용 [best_row, cost_row0..row8] (data_recorder 가 기록)
        self.candidates_pub = rospy.Publisher('/lattice_candidates', Float32MultiArray, queue_size=1)

        # 모든 후보가 inf(회피 불가)일 때 정지 경로를 보낼지. 기본 off, 켜면 연속 N 사이클 확인 후 정지
        self.stop_on_blocked = rospy.get_param('~stop_on_blocked', False)
        self.stop_confirm_cycles = rospy.get_param('~stop_confirm_cycles', 2)
        self._blocked_cycles = 0

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospy.Subscriber('/gps', GPSMessage, self.gps_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/jamming_mode_active', Bool, self.jamming_mode_callback)

        # 하이브리드 모드 (장애물 없으면 전역경로)
        self.hybrid_mode = Parameter.HYBRID_MODE_ENABLED
        self.use_global_path = True
        self.obstacle_detection_distance = Parameter.OBSTACLE_DETECTION_DISTANCE
        self.mode_switch_hysteresis = Parameter.MODE_SWITCH_HYSTERESIS
        self.last_avoidance_time = 0.0
        self.last_avoidance_position = None
        self.is_in_recovery_mode = False

        rospy.Subscriber('/aisw/obstacles', Detection3DArray, self.obs_callback)

        self.ref_path = load_ref_path()
        self.spline_ref = CubicSpline2D(np.asarray(self.ref_path.cx, np.float64),
                                        np.asarray(self.ref_path.cy, np.float64))

        self.ego_s_list = None
        self.ego_d_list = None
        self.gps_x = None
        self.gps_y = None
        self.gps_flag = False
        self.ego_yaw = None
        self.imu_flag = False

        self.obs = []           # 장애물 꼭짓점 [x1,y1, x2,y2, x3,y3, x4,y4] 목록 (월드)
        self.obs_tuples = []    # (track_id, 꼭짓점8, 상대속도 vx)
        self.obs_flag = False

        # 최근접 s 탐색용 전역경로 격자
        self.search_ds = 0.25
        s0, s1 = self.spline_ref.s[0], self.spline_ref.s[-1]
        self.ref_s_search = np.arange(s0, s1, self.search_ds)
        rx, ry = self.spline_ref.calc_position_vec(self.ref_s_search)
        self.ref_xy_search = np.vstack([rx, ry]).T
        self.last_s = None

        # 구간 인덱스 → s 범위 (맵 범위 밖이면 무시)
        n_pts = len(self.spline_ref.s)
        sections = ros_sections()

        def _ranges(key, with_side=False):
            out = []
            for z in get_section(sections, key, []):
                i0, i1 = int(z[0]), int(z[1])
                if i0 < n_pts and i1 < n_pts:
                    out.append((self.spline_ref.s[i0], self.spline_ref.s[i1]) + ((int(z[2]),) if with_side else ()))
                else:
                    rospy.logwarn('[lattice] %s 구간 %s 가 맵 범위(%d점) 밖이라 무시', key, z, n_pts)
            return out

        self.special_s_ranges = _ranges('lattice.special_zones')       # 허용 차로 2개
        self.curve_s_ranges = _ranges('lattice.curve_single_lane')     # 허용 차로 1개 (곡선)
        self.crossline_s_ranges = _ranges('lattice.crosswalk_zones')   # 횡단보도
        self.danger_ranges = _ranges('lattice.merge_zones', with_side=True)   # 끼어들기 (감시 방향: 우 0, 좌 1)

        self.initial_path_published = False
        self.jamming_mode_active = False

        # [d, s] 격자 비용, 횡이동 비용, DP 누적 비용/역추적
        self.lattice_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.ds_sampling_num])
        self.smooth_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.dd_sampling_num])
        self.dp_cost = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), float('inf'))
        self.backptr = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), -1, dtype=int)

        self.ego_s = 0
        self.blocked_nodes = {}                     # (j, s) → 마지막 차단 시각
        self.block_duration = Parameter.block_time
        self.circular_blocks = []                   # (cx, cy, t) 전역경로 기준 원형 차단
        self.stop_this_cycle = False

    # ------------------------------------------------------------------ 콜백
    def gps_callback(self, msg):
        utm_x, utm_y = self.proj_UTM(msg.longitude, msg.latitude)
        self.gps_x = utm_x - msg.eastOffset
        self.gps_y = utm_y - msg.northOffset
        self.gps_flag = True

    def imu_callback(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.ego_yaw = math.degrees(yaw)
        self.imu_flag = True

    def jamming_mode_callback(self, msg):
        self.jamming_mode_active = msg.data

    def obs_callback(self, msg):
        if not (self.gps_flag and self.imu_flag):
            return
        coords_list, tuples_list = [], []
        for det in msg.detections:
            cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
            dx = max(det.bbox.size.x / 2.0, 0.5)
            dy = max(det.bbox.size.y / 2.0, 0.5)
            q = det.bbox.center.orientation
            _, _, yaw_obj = euler_from_quaternion((q.x, q.y, q.z, q.w))
            c_o, s_o = math.cos(yaw_obj), math.sin(yaw_obj)
            R_obj = np.array([[c_o, -s_o], [s_o, c_o]])
            offsets = np.array([[dx, dy], [-dx, dy], [-dx, -dy], [dx, -dy]], dtype=float)
            corners_local = (offsets @ R_obj.T) + np.array([cx, cy])
            wc = np.array([self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, p[0], p[1])
                           for p in corners_local], dtype=float)
            cxm, cym = wc[:, 0].mean(), wc[:, 1].mean()
            wc = wc[np.argsort(np.arctan2(wc[:, 1] - cym, wc[:, 0] - cxm))]
            coords8 = []
            for X, Y in wc:
                coords8.extend([float(X), float(Y)])
            vx_rel = struct.unpack('f', det.source_cloud.data[0:4])[0]
            coords_list.append(coords8)
            tuples_list.append((det.results[0].id, coords8, vx_rel))

        # 장애물 기억: LiDAR 3 Hz·감지 깜빡임에 모드/조향이 매 주기 뒤집히지 않게 OBS_HOLD_S 동안 유지.
        # 이번 주기 감지와 중심이 1.5 m 이내면 같은 물체로 보고 새 값으로 대체
        now = rospy.get_time()
        mem = [(t, c, tup) for (t, c, tup) in getattr(self, '_obs_mem', []) if now - t <= OBS_HOLD_S]

        def _ctr(c8):
            return (sum(c8[0::2]) / 4.0, sum(c8[1::2]) / 4.0)
        cur_ctrs = [_ctr(c) for c in coords_list]
        kept = [(t, c, tup) for (t, c, tup) in mem
                if all(math.hypot(_ctr(c)[0] - a, _ctr(c)[1] - b) > 1.5 for (a, b) in cur_ctrs)]
        self._obs_mem = kept + [(now, c, tup) for c, tup in zip(coords_list, tuples_list)]
        self.obs = [c for (_, c, _) in self._obs_mem]
        self.obs_tuples = [tup for (_, _, tup) in self._obs_mem]
        self.obs_flag = bool(self.obs)

    # ------------------------------------------------------------------ 좌표
    def _path_sd(self, x, y):
        """전역경로 기준 (누적거리 s, 횡거리 d[좌+]). 하이브리드 판정용 간이 Frenet."""
        if not hasattr(self, '_gp_xy'):
            gx, gy = load_map_fields(DEFAULT_MAP)[:2]
            self._gp_xy = np.column_stack([gx, gy]).astype(np.float64)
            seg = np.hypot(*np.diff(self._gp_xy, axis=0).T)
            self._gp_s = np.concatenate([[0.0], np.cumsum(seg)])
            t = np.gradient(self._gp_xy, axis=0)
            self._gp_yaw = np.arctan2(t[:, 1], t[:, 0])
        i = int(np.argmin(np.sum((self._gp_xy - (x, y)) ** 2, axis=1)))
        dx, dy = x - self._gp_xy[i, 0], y - self._gp_xy[i, 1]
        c, sn = math.cos(self._gp_yaw[i]), math.sin(self._gp_yaw[i])
        return self._gp_s[i] + c * dx + sn * dy, -sn * dx + c * dy

    @staticmethod
    def local_to_world(x0, y0, yaw_deg, xv, yv):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        X, Y = np.array([x0, y0]) + np.array([[c, -s], [s, c]]) @ np.array([xv, yv])
        return float(X), float(Y)

    def find_nearest_s(self, x, y, mode='ego', center_s=None):
        """전역경로 위 최근접 s. ego 는 직전 s 주변(뒤 10~앞 30 m), obs 는 ego_s 주변만 탐색."""
        P = np.array([x, y])
        s_arr, XY = self.ref_s_search, self.ref_xy_search
        if mode == 'ego' and self.last_s is not None:
            s0 = max(self.last_s - 10.0, s_arr[0])
            s1 = min(self.last_s + 30.0, s_arr[-1])
            i0 = int((s0 - s_arr[0]) / self.search_ds)
            i1 = int((s1 - s_arr[0]) / self.search_ds) + 1
        elif mode == 'obs' and center_s is not None:
            s0 = max(center_s - 5.0, s_arr[0])
            s1 = min(center_s + Parameter.pre_window_s + 30.0, s_arr[-1])
            i0 = int((s0 - s_arr[0]) / self.search_ds)
            i1 = int((s1 - s_arr[0]) / self.search_ds) + 1
        else:
            i0, i1 = 0, len(s_arr)

        d = XY[i0:i1] - P
        d2 = (d * d).sum(axis=1)
        j = int(np.argmin(d2))
        best_i, best_d2 = i0 + j, float(d2[j])
        if mode == 'ego' and self.last_s is not None:
            # 순환 코스: 창이 경로 끝에 걸리면 시작부 30 m 도 탐색 (바퀴를 넘어도 s 가 끝에 묶이지 않게)
            if i1 >= len(s_arr) - 1:
                k1 = int(30.0 / self.search_ds) + 1
                d0 = XY[:k1] - P
                d0 = (d0 * d0).sum(axis=1)
                k = int(np.argmin(d0))
                if d0[k] < best_d2:
                    best_i, best_d2 = k, float(d0[k])
            # 창 안 최근접점이 5 m 넘게 멀면(위치 점프/재시작) 전체 재탐색
            if best_d2 > 25.0:
                dd = XY - P
                best_i = int(np.argmin((dd * dd).sum(axis=1)))
        s = s_arr[best_i]
        if mode == 'ego':
            self.last_s = s
        return s

    def get_frenet_d(self, s_ref, x, y):
        """s_ref 지점 법선 방향 횡편차 (왼쪽 +)."""
        xr, yr = self.spline_ref.calc_position(s_ref)
        yaw_r = math.radians(self.spline_ref.calc_yaw(s_ref))
        return (x - xr) * (-math.sin(yaw_r)) + (y - yr) * math.cos(yaw_r)

    @staticmethod
    def gps_point_to_rect_gaps(px, py, rect_corners, s_axis, d_axis):
        """점–사각형 거리 → (마진 적용 거리, 서명 거리[내부 음수])."""
        cx, cy = np.mean(rect_corners[:, 0]), np.mean(rect_corners[:, 1])
        A = rect_corners[np.argsort(np.arctan2(rect_corners[:, 1] - cy, rect_corners[:, 0] - cx))]
        B = np.roll(A, -1, axis=0)
        AB = B - A
        AP = np.stack([px, py]) - A
        cross = AB[:, 0] * AP[:, 1] - AB[:, 1] * AP[:, 0]
        inside = (np.all(cross >= 0.0) or np.all(cross <= 0.0))
        edge_len = np.einsum('ij,ij->i', AB, AB)
        proj_ratio = np.where(edge_len > 0.0,
                              np.clip((AP[:, 0] * AB[:, 0] + AP[:, 1] * AB[:, 1]) / edge_len, 0.0, 1.0), 0.0)
        closest = A + proj_ratio[:, None] * AB
        dmin = float(np.linalg.norm(closest - np.array([px, py]), axis=1).min())
        gap_raw = -dmin if inside else dmin

        vec = np.array([px - cx, py - cy])
        nrm = np.linalg.norm(vec)
        if nrm > 1e-9:
            u = vec / nrm
            margin_eff = abs(np.dot(u, s_axis)) * Parameter.safety_buf_s + abs(np.dot(u, d_axis)) * Parameter.safety_buf_d
        else:
            margin_eff = max(Parameter.safety_buf_s, Parameter.safety_buf_d)
        return gap_raw - margin_eff, gap_raw

    # ------------------------------------------------------------------ 모드 판단
    def check_obstacles_nearby(self):
        """전역경로 앞 OBS_S_RANGE, 횡 |d| <= OBS_D_GATE 안의 물체만 센다 (길가 가로등·경계석 제외).
        → (가까운 물체 있음, 최소 거리)"""
        if self.gps_x is None or not self.obs:
            return False, float('inf')
        ego_s, _ = self._path_sd(self.gps_x, self.gps_y)
        min_distance = float('inf')
        for obs in self.obs:
            center_x = np.mean(obs[0::2])
            center_y = np.mean(obs[1::2])
            os_, od = self._path_sd(center_x, center_y)
            if not (OBS_S_RANGE[0] <= os_ - ego_s <= OBS_S_RANGE[1] and abs(od) <= OBS_D_GATE):
                continue
            min_distance = min(min_distance, math.hypot(self.gps_x - center_x, self.gps_y - center_y))
        return (min_distance < self.obstacle_detection_distance), min_distance

    def update_hybrid_mode(self):
        """→ True 면 전역경로, False 면 래티스."""
        if not self.hybrid_mode:
            return False
        now = rospy.get_time()
        obstacles_nearby, min_distance = self.check_obstacles_nearby()
        current_pos = (self.gps_x, self.gps_y) if self.gps_x is not None else None

        # 회피 복귀 모드 종료: 시간·거리 모두 지나야
        time_passed = (now - self.last_avoidance_time) >= Parameter.AVOIDANCE_RECOVERY_TIME
        dist_passed = True
        if self.last_avoidance_position is not None and current_pos is not None:
            dist_passed = math.hypot(current_pos[0] - self.last_avoidance_position[0],
                                     current_pos[1] - self.last_avoidance_position[1]) >= Parameter.AVOIDANCE_RECOVERY_DISTANCE
        if self.is_in_recovery_mode and time_passed and dist_passed:
            self.is_in_recovery_mode = False

        if obstacles_nearby:
            self._clear_since = None
        if self.use_global_path and obstacles_nearby:
            self.use_global_path = False
            self.last_avoidance_time = now
            self.last_avoidance_position = current_pos
            self.is_in_recovery_mode = True
        elif not self.use_global_path and not obstacles_nearby:
            # 경로 중심 근처로 돌아온 뒤에만 전역 전환 (큰 횡편차에서 바로 전환하면 급복귀·오버슈트).
            # 장애물이 RETURN_TIMEOUT_S 넘게 안 보이면 횡편차와 무관하게 복귀 (옆 차로 장기 체류 방지)
            _, ego_d = self._path_sd(self.gps_x, self.gps_y) if current_pos is not None else (0.0, 0.0)
            self._clear_since = getattr(self, '_clear_since', None) or now
            timed_out = now - self._clear_since >= RETURN_TIMEOUT_S
            if not self.is_in_recovery_mode and (abs(ego_d) <= RETURN_D_MAX or timed_out):
                if min_distance > (self.obstacle_detection_distance + self.mode_switch_hysteresis):
                    self.use_global_path = True
        return self.use_global_path

    def nearest_forward_obstacle_vrel(self):
        """→ (vrel, stop_trigger)
        vrel: 곡선 단일차로 구간에서 같은 차로 앞차의 상대속도 [m/s] 또는 None
        stop_trigger: 끼어들기 구간에서 지정된 한쪽 옆 차로에 차가 있거나 다가오면 True
        """
        if not hasattr(self, '_prev_d_gap'):
            self._prev_d_gap = {}
        if not self.obs_tuples:
            return None, False

        ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')
        ego_d = self.get_frenet_d(ego_s, self.gps_x, self.gps_y)

        target_side, zone_idx = None, None   # 우측 0, 좌측 1
        for i, (s0, s1, side) in enumerate(self.danger_ranges):
            if s0 <= ego_s <= s1:
                target_side, zone_idx = side, i
                break
        in_curve_single_lane = any(s0 <= ego_s <= s1 for (s0, s1) in self.curve_s_ranges)

        max_ahead = Parameter.car_following_distance
        d_same_tol = Parameter.road_width * 0.5
        merge_band = Parameter.road_width * 6.0
        adj_ahead = Parameter.car_following_distance
        adj_d_min = Parameter.road_width * 0.8
        adj_d_max = Parameter.road_width * 6.0
        dec_thresh = max(Parameter.local_step_size, 0.3)
        if zone_idx is not None:   # 구간별 값이 있으면 덮어씀
            def pick(name, default):
                try:
                    v = getattr(Parameter, name)[zone_idx]
                    return default if v is None else v
                except Exception:
                    return default
            adj_ahead = pick('MERGE_ADJ_AHEAD', adj_ahead)
            adj_d_min = pick('MERGE_ADJ_D_MIN', adj_d_min)
            adj_d_max = pick('MERGE_ADJ_D_MAX', adj_d_max)
            merge_band = pick('MERGE_BAND', merge_band)
            dec_thresh = pick('MERGE_DEC_THR', dec_thresh)

        stop_trigger = False
        min_ds_same = float('inf')
        min_vrel = None
        for _, coords8, vx_rel in self.obs_tuples:
            cx = float(np.mean(coords8[0::2]))
            cy = float(np.mean(coords8[1::2]))
            s_p = self.find_nearest_s(cx, cy, mode='obs', center_s=ego_s)
            d_p = self.get_frenet_d(s_p, cx, cy)
            ds = s_p - ego_s
            if ds <= 0.0 or ds > max_ahead:
                continue
            d_diff = d_p - ego_d      # + 좌측, - 우측
            g = abs(d_diff)

            # 끼어들기 STOP: danger_ranges 안에서 지정된 한쪽만
            if target_side is not None:
                side_ok = (d_diff < 0.0) if target_side == 0 else (d_diff > 0.0)
                if side_ok:
                    if (0.0 < ds <= adj_ahead) and (adj_d_min <= g <= adj_d_max):
                        print(f'STOP by adjacent presence: ds={ds:.2f}, |d|={g:.2f} in [{adj_d_min:.1f},{adj_d_max:.1f}]')
                        stop_trigger = True
                    key = (round(s_p, 1), round(d_p, 1))
                    prev_g = self._prev_d_gap.get(key)
                    if g <= merge_band and prev_g is not None and g < prev_g - dec_thresh:
                        print(f'STOP by merge risk: prev_g={prev_g:.2f} -> g={g:.2f} (thr={dec_thresh:.2f})')
                        stop_trigger = True
                    self._prev_d_gap[key] = g

            # 앞차 상대속도: 곡선 단일차로 구간에서만
            if in_curve_single_lane and g <= d_same_tol and isinstance(vx_rel, (int, float)) and math.isfinite(vx_rel):
                if ds < min_ds_same:
                    min_ds_same = ds
                    min_vrel = float(vx_rel)
        return min_vrel, stop_trigger

    # ------------------------------------------------------------------ 래티스
    def lattice_node(self, gps_x, gps_y):
        ego_s = self.find_nearest_s(gps_x, gps_y, mode='ego')
        s_max = self.spline_ref.s[-1]
        self.lattice_cost_array.fill(0.0)

        self.ego_s_list = [min(ego_s + i * Parameter.ds_interval, s_max) for i in range(Parameter.ds_sampling_num)]
        # 좌측 최대 1개 차로(3.2 m) — 2개 차로는 K-City 편도 1~2차로에서 중앙선 침범
        self.ego_d_list = np.linspace(0.0, Parameter.road_width * LATERAL_LANES, Parameter.dd_sampling_num)

        # 횡방향 비용
        for i, d in enumerate(self.ego_d_list):
            self.lattice_cost_array[i, :] = abs(d) * Parameter.lateral_cost_weight \
                if abs(d) <= Parameter.road_width * 2.0 else Parameter.BIG

        s_vec = np.asarray(self.ego_s_list, np.float64)
        xr, yr = self.spline_ref.calc_position_vec(s_vec)
        yaw = np.deg2rad(self.spline_ref.calc_yaw_vec(s_vec))
        sin_yaw, cos_yaw = np.sin(yaw), np.cos(yaw)

        rects = []
        if self.obs_flag and self.obs is not None:
            for obs in self.obs:
                rects.append(np.array([[obs[i], obs[i + 1]] for i in range(0, 8, 2)]))

        # 장애물 비용 (차=점, 장애물=사각형+마진)
        if rects:
            for i, s in enumerate(self.ego_s_list):
                is_single_lane = any(s0 <= s <= s1 for (s0, s1) in self.curve_s_ranges)
                s_axis = np.array([cos_yaw[i], sin_yaw[i]])
                d_axis = np.array([-sin_yaw[i], cos_yaw[i]])
                for j, d in enumerate(self.ego_d_list):
                    px = xr[i] - sin_yaw[i] * d
                    py = yr[i] + cos_yaw[i] * d

                    if not is_single_lane:   # 단일차로 구간에서는 원형 차단 미적용
                        now_t = rospy.get_time()
                        self.circular_blocks = [(cx, cy, t0) for (cx, cy, t0) in self.circular_blocks
                                                if now_t - t0 < self.block_duration]
                        r2 = Parameter.HOTSPOT_RADIUS * Parameter.HOTSPOT_RADIUS
                        if any((px - cx) ** 2 + (py - cy) ** 2 <= r2 for (cx, cy, _) in self.circular_blocks):
                            self.lattice_cost_array[j, i] = Parameter.BIG
                            continue

                    min_gap_margin = float('inf')
                    min_gap_raw = float('inf')
                    for rc in rects:
                        g_m, g_r = self.gps_point_to_rect_gaps(px, py, rc, s_axis, d_axis)
                        min_gap_margin = min(min_gap_margin, g_m)
                        min_gap_raw = min(min_gap_raw, g_r)

                    # (1) 실물 충돌 / (2) 마진 침범 — 전역경로 칸이면 잠시 막아 둔다
                    if min_gap_raw <= 0.0 or min_gap_margin <= 0.0:
                        if abs(self.ego_d_list[j]) < 0.1:
                            self.blocked_nodes[(j, round(self.ego_s_list[i], 1))] = rospy.get_time()
                            if not is_single_lane:
                                self.circular_blocks.append((xr[i], yr[i], rospy.get_time()))
                        if min_gap_raw <= 0.0:
                            self.lattice_cost_array[j, i] = Parameter.BIG
                        else:
                            self.lattice_cost_array[j, i] += Parameter.MARGIN_BLOCK_COST
                        continue

                    # 블록 유지
                    if abs(self.ego_d_list[j]) < 0.1:
                        key = (j, round(self.ego_s_list[i], 1))
                        if key in self.blocked_nodes:
                            if rospy.get_time() - self.blocked_nodes[key] < self.block_duration:
                                self.lattice_cost_array[j, i] += Parameter.MARGIN_BLOCK_COST
                                continue
                            self.blocked_nodes.pop(key, None)

                    # 여유가 작을수록 페널티 (0~1)
                    buf_scale = max(Parameter.safety_buf_s, Parameter.safety_buf_d)
                    safety_norm = min(min_gap_margin / buf_scale, 1.0)
                    self.lattice_cost_array[j, i] += Parameter.obs_cost_weight * (1.0 - safety_norm)

            # 선제 회피: 장애물 앞쪽(s) x 장애물 차선 쪽(d, 가우시안)
            for rec in rects:
                cx, cy = float(np.mean(rec[:, 0])), float(np.mean(rec[:, 1]))
                s_obs = self.find_nearest_s(cx, cy, mode='obs', center_s=ego_s)
                d_obs = self.get_frenet_d(s_obs, cx, cy)
                for i, s in enumerate(self.ego_s_list):
                    ds = s_obs - s
                    if 0.0 < ds <= Parameter.pre_window_s:
                        ramp_s = (Parameter.pre_window_s - ds) / Parameter.pre_window_s
                        for j, d in enumerate(self.ego_d_list):
                            dd = d_obs - d
                            w_d = math.exp(-(dd * dd) / (2.0 * Parameter.sigma_d * Parameter.sigma_d))
                            self.lattice_cost_array[j, i] += Parameter.pre_cost_weight * ramp_s * w_d

        # 횡이동 비용 (3칸 이상 점프 금지)
        for j in range(Parameter.dd_sampling_num):
            for k in range(Parameter.dd_sampling_num):
                self.smooth_cost_array[j, k] = Parameter.BIG if abs(j - k) >= 3 else abs(j - k) * Parameter.smooth_cost_weight

        # 허용 차로 2개 구간: 바깥 3칸 막기
        for s0, s1 in self.special_s_ranges:
            for i, s in enumerate(self.ego_s_list):
                if s0 <= s <= s1:
                    for r in range(6, 9):
                        self.lattice_cost_array[r, i] = Parameter.BIG
        # 허용 차로 1개 구간(곡선): 전역경로 칸(0)만
        for s0, s1 in self.curve_s_ranges:
            for i, s in enumerate(self.ego_s_list):
                if s0 <= s <= s1:
                    for row in range(1, Parameter.dd_sampling_num):
                        self.lattice_cost_array[row, i] += Parameter.MARGIN_BLOCK_COST

        # DP: 자차 횡위치와 가장 가까운 칸에서 시작
        ego_d = self.get_frenet_d(ego_s, gps_x, gps_y)
        start_row = int(np.argmin(np.abs(self.ego_d_list - ego_d)))
        self.dp_cost[:, 0] = float('inf')
        self.dp_cost[start_row, 0] = self.lattice_cost_array[start_row, 0]
        for col in range(1, Parameter.ds_sampling_num):
            for row in range(Parameter.dd_sampling_num):
                candidates = []
                for prev in range(Parameter.dd_sampling_num):
                    base = self.dp_cost[prev, col - 1]
                    if not np.isfinite(base):
                        candidates.append(Parameter.BIG)
                        continue
                    c1 = self.smooth_cost_array[prev, row]
                    prevprev = self.backptr[prev, col - 1] if col >= 2 else prev
                    if prevprev < 0:
                        prevprev = prev
                    c2 = Parameter.second_diff_weight * abs(row - 2 * prev + prevprev)
                    candidates.append(base + c1 + c2 + self.lattice_cost_array[row, col])
                best_prev = int(np.argmin(candidates))
                self.dp_cost[row, col] = candidates[best_prev]
                self.backptr[row, col] = best_prev

        end_col = Parameter.ds_sampling_num - 1
        best_row = int(np.argmin(self.dp_cost[:, end_col]))
        end_costs = self.dp_cost[:, end_col]   # inf 는 -1 로 기록
        self.candidates_pub.publish(Float32MultiArray(data=[float(best_row)] + [
            float(c) if np.isfinite(c) else -1.0 for c in end_costs]))
        if not np.isfinite(self.dp_cost[best_row, end_col]):
            self.stop_this_cycle = True

        path_rows = [best_row]
        r = best_row
        for col in range(end_col, 0, -1):
            r = self.backptr[r, col]
            path_rows.append(r)
        d_final_list = [self.ego_d_list[idx] for idx in reversed(path_rows)]
        return d_final_list, self.ego_s_list, xr, yr, np.sin(yaw), np.cos(yaw)

    # ------------------------------------------------------------------ 경로 생성/발행
    def make_global_path_segment(self, ego_s):
        """전역경로 앞 lookahead_distance. 횡편차가 크면(회피 직후) 0 까지 코사인으로 줄어드는 경로."""
        s_arr, XY = self.ref_s_search, self.ref_xy_search
        i0 = max(0, int((ego_s - s_arr[0]) / self.search_ds))
        i1 = min(len(s_arr), int((ego_s + Parameter.lookahead_distance - s_arr[0]) / self.search_ds))
        xs = XY[i0:i1, 0].tolist()
        ys = XY[i0:i1, 1].tolist()
        # 순환 코스: 끝 근처면 시작부를 이어 붙임 (0~1점으로 퇴화하면 PP 가 시작점 한 점을 쫓아 이탈)
        need = int(Parameter.lookahead_distance / self.search_ds)
        if len(xs) < need:
            wrap = need - len(xs)
            xs += XY[:wrap, 0].tolist()
            ys += XY[:wrap, 1].tolist()
        # 부드러운 복귀 (평소 추종 오차 < 0.5 m 에는 적용 안 함)
        if self.gps_x is not None and len(xs) >= 3:
            _, d0 = self._path_sd(self.gps_x, self.gps_y)
            if abs(d0) > BLEND_D_MIN:
                ax, ay = np.asarray(xs), np.asarray(ys)
                sc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(ax), np.diff(ay)))])
                tx, ty = np.gradient(ax), np.gradient(ay)
                nrm = np.hypot(tx, ty)
                nrm[nrm < 1e-9] = 1.0
                off = d0 * np.where(sc < BLEND_M, 0.5 * (1.0 + np.cos(np.pi * sc / BLEND_M)), 0.0)
                xs = (ax - off * ty / nrm).tolist()
                ys = (ay + off * tx / nrm).tolist()
        return xs, ys

    @staticmethod
    def make_path_cached(d_list, xr, yr, sin_yaw, cos_yaw):
        d = np.asarray(d_list)
        return (xr - sin_yaw * d).tolist(), (yr + cos_yaw * d).tolist()

    def publish_path(self, xs, ys, stop=False):
        """stop=True 면 첫 점 z=-100 (normal_drive 가 제동 신호로 읽음)."""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = 'map'
        for idx, (x, y) in enumerate(zip(xs, ys)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = -100.0 if (stop and idx == 0) else 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def smooth(self, path_x, path_y):
        """중복점 제거 후 3차 스플라인으로 local_step_size 간격 재샘플."""
        try:
            arr = np.array([path_x, path_y]).T
            _, unique_idx = np.unique(arr, axis=0, return_index=True)
            unique_idx = np.sort(unique_idx)
            if len(unique_idx) < 2:
                rospy.logwarn('[LatticePlanner] Insufficient unique points for smoothing, using original path')
                return path_x, path_y
            sp = CubicSpline2D(np.asarray([path_x[i] for i in unique_idx], np.float64),
                               np.asarray([path_y[i] for i in unique_idx], np.float64))
            return sp.calc_position_vec(np.arange(sp.s[0], sp.s[-1], Parameter.local_step_size))
        except Exception as e:
            rospy.logwarn(f'[LatticePlanner] Smoothing failed: {e}, using original path')
            return path_x, path_y

    # ------------------------------------------------------------------ 메인 루프
    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.jamming_mode_active:   # AI 구간이면 대기
                rospy.loginfo_throttle(2.0, '[LatticePlanner] AI 구간 - 대기')
                rate.sleep()
                continue
            if not (self.gps_flag and self.imu_flag):
                rate.sleep()
                continue

            vrel, stop_trigger = self.nearest_forward_obstacle_vrel()
            self.vrel_pub.publish(Float32(data=float('nan') if vrel is None else float(vrel)))
            self.merge_stop_pub.publish(UInt8(data=1 if stop_trigger else 0))

            self.ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')

            # 시작 직후 전역경로 한 번 발행 (normal_drive 가 바로 출발할 수 있게)
            if not self.initial_path_published:
                path_x, path_y = self.make_global_path_segment(self.ego_s)
                if len(path_x) >= 3:
                    self.publish_path(path_x, path_y)
                self.initial_path_published = True

            use_global = self.update_hybrid_mode()
            if use_global:
                smooth_x, smooth_y = self.make_global_path_segment(self.ego_s)
            else:
                path_d_list, _, xr, yr, sin_yaw, cos_yaw = self.lattice_node(self.gps_x, self.gps_y)
                if path_d_list is None or any(d is None for d in path_d_list):
                    rate.sleep()
                    continue
                path_x, path_y = self.make_path_cached(path_d_list, xr, yr, sin_yaw, cos_yaw)

                # 회피 불가(모든 후보 inf) 정지 — ~stop_on_blocked, 연속 stop_confirm_cycles 사이클 확인
                blocked = self.stop_this_cycle
                self.stop_this_cycle = False
                self._blocked_cycles = self._blocked_cycles + 1 if blocked else 0
                if blocked:
                    rospy.logwarn_throttle(1.0, '[lattice] 모든 후보 경로 inf (%d사이클)%s', self._blocked_cycles,
                                           '' if self.stop_on_blocked else ' — stop_on_blocked=false 라 주행 유지')
                if self.stop_on_blocked and self._blocked_cycles >= self.stop_confirm_cycles:
                    self.publish_path([self.gps_x], [self.gps_y], stop=True)
                    self.path_mode.publish(UInt8(data=1))
                    rate.sleep()
                    continue
                smooth_x, smooth_y = self.smooth(path_x, path_y)

            self.publish_path(smooth_x, smooth_y)
            self.path_mode.publish(UInt8(data=0 if use_global else 1))   # 0 전역 / 1 로컬
            rate.sleep()


if __name__ == '__main__':
    LatticePlanner().main()
