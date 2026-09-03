#!/usr/bin/env python3
# -*- coding: utf-8 -*-
try:
    profile  # kernprof -l 로 실행하면 builtins에 주입됨
except NameError:
    def profile(func):  # IDE 노란줄 방지 + 일반 실행 시 no-op
        return func
    
import os
import rospy
import numpy as np
import math
import json
import bisect
import threading
import subprocess
import time
from tf.transformations import euler_from_quaternion
from pyproj import Proj
from sensor_msgs.msg import Imu
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage, EventInfo, CollisionData
from std_msgs.msg import UInt8, Bool
from vision_msgs.msg import Detection3DArray
from morai_msgs.srv import MoraiEventCmdSrv
from collections import deque
from nav_msgs.msg import Path
# cubic spline
from scipy.interpolate import CubicSpline
# RVIZ
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class Parameter:
    # 아이오닉5 모델 스펙
    vehicle_wheelbase = 3.000       # 차량 휠베이스 [m]
    vehicle_length = 4.635          # 차량 전체 길이 [m]
    vehicle_width = 1.892           # 차량 폭 [m]
    vehicle_height = 2.434          # 차량 전체 높이 [m]
    vehicle_front_overhang = 0.845  # 앞바퀴부터 앞 범퍼까지 거리 [m]
    vehicle_rear_overhang = 0.79    # 뒷바퀴부터 뒤 범퍼까지 거리 [m]
    vehicle_min_radius = 5.87       # 차량 최소회전반경 [m]

    max_wheel_angle = 40.0          # 차량 최대조향각 [deg]
    max_velocity = 40.0 / 3.6       # 최대 속도 [kph] : 단 throttle제어이므로 어느정도 여유둬야 함 (46.5 이상은 금지)
    max_local_velocity = 25.0 / 3.6        # 로컬경로 최대허용속도

    default_target_index = 30       # 0.2m 간격이므로 30개는 6m 전방

    # 추종할 맵 json파일 경로
    INPUT_JSONS = [
        # '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_1.json',
        # '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_2.json',
        # '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_3.json',
        # '/home/yhj/catkin_ws/src/hlfma_morai/map/sangam_bonseon_ver3.json'
        os.environ.get('HL_MAP_JSON') or os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map', 'kcity_map.json')
    ]
    # 초기 맵 설정
    INITIAL_MAP_INDEX = 0

    curve_range_start = 8          # 곡률판단 시작부 (ego ind 기준) : 4m
    curve_range_end = 12            # 곡률판단 종료부 (ego ind 기준) : 6m
    curve_filter = 8                # 이동평균필터 반영 개수
    mu = 0.55                       # 마찰계수

class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = [v / 3.6 for v in cv]
        # self.cv = cv
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)

class MissionState:
    mission_0 = "NORMAL_DRIVING"
    mission_1 = "E_STOP"
    mission_2 = "LANE_DETECTION_BASED_DRIVING"
    mission_3 = "PERPENDICULAR_PARKING"
    mission_4 = "PARALLEL_PARKING"
    mission_5 = "STATIC_OBSTACLE"
    mission_6 = "DYNAMIC_OBSTACLE"

class FrenetCoordinate:
    """독립적인 Frenet 좌표계 변환 클래스"""

    def __init__(self, ref_path):
        self.ref_path = ref_path
        self.ref_s = self._calculate_cumulative_s()

    def _calculate_cumulative_s(self):
        """경로를 따라 누적 s 거리 계산"""
        s_list = [0.0]
        for i in range(1, len(self.ref_path.cx)):
            dx = self.ref_path.cx[i] - self.ref_path.cx[i-1]
            dy = self.ref_path.cy[i] - self.ref_path.cy[i-1]
            ds = math.sqrt(dx*dx + dy*dy)
            s_list.append(s_list[-1] + ds)
        return s_list

    def cartesian_to_frenet(self, x, y):
        """절대좌표 (x, y) → Frenet 좌표 (s, d) 변환"""
        # 가장 가까운 경로점 찾기
        min_dist = float('inf')
        closest_idx = 0

        for i, (px, py) in enumerate(zip(self.ref_path.cx, self.ref_path.cy)):
            dist = (x - px)**2 + (y - py)**2
            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # s 좌표: 경로를 따라 누적된 거리
        s = self.ref_s[closest_idx]

        # d 좌표: 경로에 수직인 거리 (좌측 +, 우측 -)
        path_yaw_rad = math.radians(self.ref_path.cyaw[closest_idx])
        dx = x - self.ref_path.cx[closest_idx]
        dy = y - self.ref_path.cy[closest_idx]

        # 경로 수직 방향으로 투영
        d = -dx * math.sin(path_yaw_rad) + dy * math.cos(path_yaw_rad)

        return s, d

#################################################################################
class CubicSpline2D_fast:
    def __init__(self, x, y):
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        ds = np.hypot(np.diff(x), np.diff(y))
        self.s = np.empty(x.shape[0], np.float64); self.s[0] = 0.0
        self.s[1:] = np.cumsum(ds)

        # 기존 코드의 natural BC에 대응
        self.sx = CubicSpline(self.s, x, bc_type='natural')  # extrapolate=True(기본)
        self.sy = CubicSpline(self.s, y, bc_type='natural')

    # 벡터 평가
    def calc_position_vec(self, s_array):
        return self.sx(s_array), self.sy(s_array)

    def calc_yaw_vec(self, s_array):
        dx = self.sx(s_array, 1); dy = self.sy(s_array, 1)
        return np.rad2deg(np.arctan2(dy, dx))

    def calc_curvature_vec(self, s_array):
        dx  = self.sx(s_array, 1); ddx = self.sx(s_array, 2)
        dy  = self.sy(s_array, 1); ddy = self.sy(s_array, 2)
        return (ddy*dx - ddx*dy) / np.power(dx*dx + dy*dy, 1.5)

    # 단일값 헬퍼(현재 코드의 calc_position/ calc_yaw 호출 보완)
    def calc_position(self, s):
        x, y = self.calc_position_vec(np.array([s], np.float64))
        return float(x[0]), float(y[0])

    def calc_yaw(self, s):
        return float(self.calc_yaw_vec(np.array([s], np.float64))[0])

    def calc_curvature(self, s):
        return float(self.calc_curvature_vec(np.array([s], np.float64))[0])
#################################################################################

class PurePursuit_Control:
    def __init__(self, path):
        self.wb = Parameter.vehicle_wheelbase
        self.lfd = Parameter.default_target_index  # look-ahead 거리
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_gear = 4
        self.ego_ind = 0
        self.ego_vel = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_ind = 0
        self.target_vel = 0.0

        self.global_path = path
        self.path = None

    def normalize_rad_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    def update_path(self, path):
        self.path = path

    def run(self, ego_x, ego_y, ego_yaw, ego_gear, ego_ind, ego_vel, curvedvelocity, path_updated_flag):
        #print("Pure-Pursuit Control Running")
        if self.path is not None:
            self.ego_x = ego_x
            self.ego_y = ego_y
            self.ego_yaw = ego_yaw
            self.ego_gear = ego_gear
            self.ego_ind = ego_ind
            self.ego_vel = ego_vel
            self.target_vel = curvedvelocity

            if abs(self.ego_vel) > 40.0 / 3.6:
                self.lfd = 22   # 11.0m
            elif abs(self.ego_vel) > 20.0 / 3.6:
                self.lfd = 18   # 9.0m
            elif abs(self.ego_vel) > 18.0 / 3.6:
                self.lfd = 14   # 7.0m
            elif abs(self.ego_vel) > 15.0 / 3.6:
                self.lfd = 12   # 6.0m
            elif abs(self.ego_vel) > 10.0 / 3.6:
                self.lfd = 5   # 5.0m
     
            # self.target_vel = self.path.cv[self.ego_ind]
            self.target_ind = self.ego_ind + self.lfd

            # print(self.lfd)
            # print(self.target_ind)

            if self.target_ind >= self.path.length:
                self.target_ind = self.path.length - 1

            self.target_x = self.path.cx[self.target_ind]
            self.target_y = self.path.cy[self.target_ind]

            # heading error
            alpha = self.normalize_rad_angle(math.atan2(self.target_y - self.ego_y, self.target_x - self.ego_x) - math.radians(self.ego_yaw))

            lookahed_distance = math.hypot(self.target_x - self.ego_x, self.target_y - self.ego_y)
            if lookahed_distance <= 0.0:
                steering_rad = 0.0
            else:
                steering_rad = math.atan2(2.0 * self.wb * math.sin(alpha), lookahed_distance)

            steering_deg = self.normalize_180(math.degrees(steering_rad))
            # print(self.target_vel)
            # print(steering_deg)
            return np.clip(abs(self.target_vel), 0.2, Parameter.max_velocity), np.clip(steering_deg, -40.0, 40.0)
    
# Trottle제어기
class AccelCmd_Converter:
    def __init__(self, rate_hz):
        self.p_gain = 0.35
        self.i_gain = 0.05
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 1 / rate_hz
        self.output = 0.0

    def run(self, target_vel, current_vel, ego_gear):
        error = target_vel - current_vel
        # print(target_vel)
        # print(current_vel)
        # print(error)
        p_control = self.p_gain * error
        if error <= 5 and abs(self.output) < 1.0:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime
        self.output = p_control + self.i_control + d_control
        self.prev_error = error

        if self.output > 0:
            accel_cmd = self.output
            brake_cmd = 0.0
        else:
            accel_cmd = 0.0
            brake_cmd = -self.output
        # print(self.output)
        return accel_cmd, brake_cmd
    
class CurvedBased_Velocity:
    def __init__(self, path):
        # 큐를 사용하여 최근 곡률값 몇 개만 저장
        self.curve_history = deque(maxlen=Parameter.curve_filter)   # 이동평균필터
        self.ego_global_index = 0
        self.ego_local_index = 0
        self.global_path = path
        self.path = None

    def update_path(self, path):
        self.path = path

    def run(self, ego_global_ind, ego_local_ind):
        self.ego_global_index = ego_global_ind
        self.ego_local_index = ego_local_ind
        x_list = []
        y_list = []
        for box in range(Parameter.curve_range_start, Parameter.curve_range_end):
            idx = self.ego_local_index + box
            if idx < 0 or idx >= self.path.length:
                continue  # 경로 범위를 벗어나는 인덱스는 건너뜀
            x = self.path.cx[idx]
            y = self.path.cy[idx]
            x_list.append([-2*x, -2*y ,1])
            y_list.append((-x*x) - (y*y))
            
        x_matrix = np.array(x_list)
        y_matrix = np.array(y_list)
        # 점이 3개 미만이면 원 적합 불가 → 바로 큰 반경
        if x_matrix.shape[0] < 3:
            r = 1e6
        else:
            try:
                a_matrix, residuals, rank, s = np.linalg.lstsq(x_matrix, y_matrix, rcond=None)
                # 랭크 체크(거의 일직선인 경우 등)
                if rank < 3:
                    r = 1e6
                else:
                    a, b, c = a_matrix
                    r2 = a*a + b*b - c
                    r = math.sqrt(r2) if r2 > 0 else 1e6   # abs() 대신 음수면 실패 처리
            except np.linalg.LinAlgError:
                r = 1e6
        
        # print(f"Calculated radius: {r}")

        self.curve_history.append(r)

        #이동평균필터
        if len(self.curve_history) >= 2:
            smoothed_r = sum(self.curve_history) / len(self.curve_history)
        else:
            smoothed_r = r

        # print(f"Smoothed radius: {smoothed_r}")
            
        v_max = math.sqrt(smoothed_r * 9.81 * Parameter.mu)

        if v_max > self.global_path.cv[self.ego_global_index]:
            v_max = self.global_path.cv[self.ego_global_index]

        return float(v_max)

class Morai_Control_Node:
    def __init__(self, init_node=True):
        if init_node:
            rospy.init_node('morai_control_node', anonymous = True)

        # rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.odom_callback)
        rospy.Subscriber("/Competition_topic",EgoVehicleStatus, self.comp_callback)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.coll_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)
        rospy.Subscriber("/planner_mode", UInt8, self.planner_mode_callback)
        rospy.Subscriber("/jamming_mode_active", Bool, self.jamming_mode_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        # self.ctrl_cmd_msg.longlCmdType=2
        self.ctrl_cmd_msg.longlCmdType=1

        # ServiceProxy를 한 번만 생성
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self._event_cmd = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        # 시작 시 제어권 확보: automode(3) + 기어 D
        self.change_ctrl_mode(3)
        self.change_to_drive()

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.rate_hz = 15   # 메인루프 주기 설정

        # 초기 맵 설정
        self.initial_map_index = Parameter.INITIAL_MAP_INDEX
        # 모든 맵을 미리 불러와 저장
        self.all_paths = [ self.load_ref_map(p) for p in Parameter.INPUT_JSONS ]
        # 맵 인덱스 clamp & 현재 ref_path 설정
        self.map_index = max(0, min(self.initial_map_index, len(self.all_paths)-1))
        self.ref_path  = self.all_paths[self.map_index]

        # ================= RViz용 Publisher 추가 =================
        # self.map_marker_pub = rospy.Publisher("/ref_path_marker", MarkerArray, queue_size=1)
    
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_gear = 4
        self.ego_vel = 0.0
        self.ego_index_global = 0
        self.ego_index_local = 0

        self.velocity = 0.0
        self.steering = 0.0
        self.curvedvelocity = 0.0

        self.gps_fix = 0
        self.odom_flag = False
        self.gps_flag = False
        self.imu_flag = False
        self.coll_flag = False

        # 후진 관련 변수
        self.adjusted_yaw = 0.0
        self.prev_gear = self.ref_path.cgear[0]

        self.ego_mission = MissionState.mission_0

        # 장애물 회피 관련 변수
        self.local_path = None
        self.local_path_flag = False    # 일회성

        # 로컬맵 업데이트 플래그 (연속성 위한 플래그)
        self.path_updated = False

        self.planner_mode = 0
        self.local_map_update = 0

        # 스왑용 최소 변수들
        self.pending_local_path = None
        self.has_pending = False
        self.pending_stamp = 0.0

        self.SWAP_MIN_INTERVAL = 0.25   # 최소대기시간 [sec]
        self.SWAP_TIMEOUT  = 0.4    # 점프 커도 이 시간 지나면 강제 스왑 허용 [sec]
        self._last_swap_time   = 0.0

        # 신호등 제어 플래그
        self.traffic_light_brake = False

        # 외부에서 제한하는 속도 [m/s] : 하나의 차선에서 앞 차량 속도 기반
        self.external_speed_cap = None

        # 제밍 모드 상태
        self.jamming_mode_active = False

        # 인덱스 1633에서 장애물 토픽 기반 정지 관련
        self.stop_index_1633 = 1631
        self.is_stop_completed_1633 = False
        self.stop_started_1633 = False
        self.braking_started_1633 = False
        self.obstacles_detected = False
        self.last_obstacle_time = None

        # 모든 경로 inf시 정지명령 유지
        self._planner_stop_until = 0.0   # STOP 유지 종료 시각(ROS time)
        self._planner_stop_hold  = 0.2   # 최소 유지 시간 [sec]

        # 장애물 감지 토픽 구독
        rospy.Subscriber('/tracked_objects_3d', Detection3DArray, self._obstacle_callback)

        # 사용할 클래스 객체 초기화
        self.cmd_converter = AccelCmd_Converter(self.rate_hz)
        self.purepursuit_controller = PurePursuit_Control(self.ref_path)
        self.curvebased_vel = CurvedBased_Velocity(self.ref_path)

        # Lattice planner 참조 (s 계산용)
        self.lattice_planner = None  # master에서 설정

        # 독립적인 Frenet 좌표계 (1633 인덱스 장애물 체크용)
        self.frenet_coord = None  # 첫 사용 시 초기화
        self._latest_detections = []  # 최신 장애물 데이터 저장

    ########################################## SENSOR CALLBACK ##########################################

    def comp_callback(self, msg):
        self.ego_vel = msg.velocity.x   # 상대좌표계 기준 : 전진속도 / 후진시 음수로 출력됨 [m/s]
        self.odom_flag = True
        # print(self.ego_vel)

    def gps_callback(self, msg):
        """
        차량 후륜중심 (x:0.0, y:0.0, z:1.2) / 30Hz
        """
        # UTM기반 ENU좌표계 (WGS84->UTM->ENU)
        latitude = msg.latitude
        longitude = msg.longitude
        utm_x, utm_y = self.proj_UTM(longitude, latitude)
        self.ego_x = utm_x - msg.eastOffset
        self.ego_y = utm_y - msg.northOffset
        self.gps_fix = msg.status           # 0 : no fix / 1 : 2D fix / 2 : 3D fix / 3 : RTK fix / 4 : RTK Float

        self.gps_flag = True
        # print(f"GPS x: {self.ego_x}, y: {self.ego_y}, z: {self.gps_fix}")

    def imu_callback(self, msg):
        """
        차량 후륜중심 (x:0.0, y:0.0, z:1.3) / 30Hz
        """
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.ego_yaw = math.degrees(yaw)

        self.imu_flag = True
        # print(f"IMU yaw: {self.ego_yaw}")

    def coll_callback(self, msg):
        self.coll_flag = hasattr(msg, "collision_object") and len(msg.collision_object) > 0
        # print(f"collision: {len(msg.collision_object)}")
    
    @profile
    def local_path_callback(self, msg):
        # STOP 신호: 첫 pose의 z == -1.0
        if msg.poses and msg.poses[0].pose.position.z == -100.0:
            now = rospy.get_time()
            # 여러 번 들어와도 남은 시간이 짧으면 다시 연장
            self._planner_stop_until = max(self._planner_stop_until, now + self._planner_stop_hold)
            # 경로 갱신은 무시 (브레이크만 유지)
            return

        if len(msg.poses) < 3:
            # 너무 짧으면 스킵
            self.local_path_flag = False
            return

        # 1) 메시지 → numpy 배열 (리스트 만들지 말고 바로 fromiter)
        nposes = len(msg.poses)
        xs = np.fromiter((p.pose.position.x for p in msg.poses), dtype=np.float64, count=nposes)
        ys = np.fromiter((p.pose.position.y for p in msg.poses), dtype=np.float64, count=nposes)

        # 선택: 중복점 제거(같은 좌표가 연속으로 올 때 곡률 계산이 불안정해질 수 있음)
        # mask = np.r_[True, np.logical_or(np.diff(xs)!=0.0, np.diff(ys)!=0.0)]
        # xs, ys = xs[mask], ys[mask]
        if xs.size < 3:
            self.local_path_flag = False
            return

        # 2) 빠른 스플라인으로 한 번에 평가 (s 전체 벡터)
        sp = CubicSpline2D_fast(xs, ys)     # 또는 CubicSpline2D_nb / Cython 버전
        ss = sp.s[:-1]                      # 끝점 하나 제외(원래 코드와 동일한 샘플링)
        cx, cy = sp.calc_position_vec(ss)
        cyaw   = sp.calc_yaw_vec(ss)
        ck     = sp.calc_curvature_vec(ss)

        # 3) 메타 필드 길이 맞춰 한 번에 채우기
        n = ss.shape[0]
        cv = [Parameter.max_velocity * 3.6] * n   # PATH에서 /3.6 하므로 여기선 kph로
        cm = ["NORMAL_DRIVING"] * n
        cg = [4] * n

        new_path = PATH(cx, cy, cyaw, ck, cv, cm, cg)

        # 바로 교체/업데이트 금지 → pending 큐에 적재만
        self.pending_local_path = new_path
        self.has_pending = True
        self.pending_stamp = rospy.get_time()

        # 첫 경로가 아직 없으면 즉시 채택(시동 단계에서만)
        if self.local_path is None:
            self.local_path = self.pending_local_path
            self.has_pending = False
            self.local_path_flag = True
            self.purepursuit_controller.update_path(self.local_path)
            self.curvebased_vel.update_path(self.local_path)
            print("[local_path] first path applied immediately")

    def planner_mode_callback(self, msg):
        # 0이면 전역경로, 1이면 로컬경로
        self.planner_mode = int(msg.data)

    def jamming_mode_callback(self, msg):
        self.jamming_mode_active = msg.data

    def _obstacle_callback(self, msg: Detection3DArray):
        """장애물 감지 토픽 콜백 - 크기 2m 이하 장애물만 카운트"""
        small_obstacles = 0
        total_obstacles = len(msg.detections) if msg.detections else 0

        # Frenet 체크용 최신 데이터 저장
        self._latest_detections = msg.detections if msg.detections else []

        if msg.detections:
            for detection in msg.detections:
                # bbox 크기 확인 (x, y, z 중 하나라도 3m 넘으면 제외)
                size_x = detection.bbox.size.x
                size_y = detection.bbox.size.y
                size_z = detection.bbox.size.z
                max_size = max(size_x, size_y, size_z)

                if max_size <= 2.0:
                    small_obstacles += 1
                    rospy.loginfo_throttle(3.0, f"[소형 장애물] 크기: x={size_x:.2f}, y={size_y:.2f}, z={size_z:.2f}m")
                else:
                    rospy.loginfo_throttle(3.0, f"[대형 장애물 제외] 크기: x={size_x:.2f}, y={size_y:.2f}, z={size_z:.2f}m")

        if small_obstacles > 0:
            self.obstacles_detected = True
            self.last_obstacle_time = rospy.get_time()
            rospy.loginfo_throttle(2.0, f"[장애물 감지] 소형 장애물 {small_obstacles}개 감지 (전체: {total_obstacles}개)")
        else:
            self.obstacles_detected = False
            rospy.loginfo_throttle(2.0, f"[장애물 감지] 소형 장애물 없음 (전체: {total_obstacles}개, 모두 3m 이상)")

    def check_path_clear_with_frenet(self):
        """독립적인 Frenet 좌표계 기반 전방 경로 클리어 체크"""
        try:
            # 첫 사용 시 Frenet 좌표계 초기화
            if self.frenet_coord is None:
                self.frenet_coord = FrenetCoordinate(self.ref_path)
                rospy.loginfo("[1633 정지] FrenetCoordinate 초기화 완료")

            # 현재 차량의 Frenet 좌표
            ego_s, ego_d = self.frenet_coord.cartesian_to_frenet(self.ego_x, self.ego_y)

            # 전방 체크 거리 및 횡방향 감지 범위
            front_check_distance = 15.0  # 전방 15m
            lateral_detection_range = 5.0  # 횡방향 감지 범위 (±5m) - 다가오는 보행자 포함

            # Detection3DArray에서 받은 장애물들 체크
            front_obstacles = 0
            if hasattr(self, '_latest_detections') and self._latest_detections:
                for detection in self._latest_detections:

                    # 장애물 위치 (차량 좌표계 → 절대 좌표계)
                    obs_x_rel = detection.bbox.center.position.x
                    obs_y_rel = detection.bbox.center.position.y

                    # 차량 기준 절대좌표로 변환
                    ego_yaw_rad = math.radians(self.ego_yaw)
                    obs_x_abs = self.ego_x + obs_x_rel * math.cos(ego_yaw_rad) - obs_y_rel * math.sin(ego_yaw_rad)
                    obs_y_abs = self.ego_y + obs_x_rel * math.sin(ego_yaw_rad) + obs_y_rel * math.cos(ego_yaw_rad)

                    # 장애물의 Frenet 좌표
                    obs_s, obs_d = self.frenet_coord.cartesian_to_frenet(obs_x_abs, obs_y_abs)

                    # 전방에 있고 && 횡방향 감지 범위 내에 있고 && 가까운 거리
                    if (obs_s > ego_s and
                        (obs_s - ego_s) <= front_check_distance and
                        abs(obs_d - ego_d) <= lateral_detection_range):

                        front_obstacles += 1
                        rospy.loginfo_throttle(1.0,
                            f"[1633 정지] Frenet 전방 장애물 감지: "
                            f"ego(s={ego_s:.1f}, d={ego_d:.1f}), "
                            f"obs(s={obs_s:.1f}, d={obs_d:.1f}), "
                            f"거리={obs_s-ego_s:.1f}m")

            if front_obstacles > 0:
                rospy.loginfo_throttle(1.0, f"[1633 정지] Frenet 기준 전방 장애물 {front_obstacles}개 - 계속 정지")
                return False
            else:
                rospy.loginfo_throttle(2.0, f"[1633 정지] Frenet 기준 전방 경로 클리어 - ego(s={ego_s:.1f}, d={ego_d:.1f})")
                return True

        except Exception as e:
            rospy.logwarn_throttle(2.0, f"[1633 정지] Frenet 계산 오류: {e}")
            return not self.obstacles_detected

    #####################################################################################################

    # 기어 변경
    def change_gear(self, gear_value):
        try:
            event_msg = EventInfo()
            event_msg.option = 2  # gear 변경
            event_msg.gear = gear_value
            self._event_cmd(event_msg)
        
            gear_str = {1: 'P', 2: 'R', 3: 'N', 4: 'D'}.get(gear_value, str(gear_value))
            print(f"Gear changed to {gear_str}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def change_to_parking(self):
        self.change_gear(1)  # P

    def change_to_drive(self):
        self.change_gear(4)  # D

    def change_to_reverse(self):
        self.change_gear(2)  # R
    
    # 컨트롤 모드 변경
    def change_ctrl_mode(self, mode_value):
        """
        mode_value:
         1 Keyboard
         2 GameWheel
         3 automode
         4 cruisemode
        """
        try:
            evt = EventInfo()
            evt.option    = 1        # ctrl_mode 변경 옵션
            evt.ctrl_mode = mode_value
            self._event_cmd(evt)
            print(f"Control mode changed to {mode_value}")
        except rospy.ServiceException as e:
            print(f"Ctrl mode change failed: {e}")

    # json파일 불러와 ref 정보 PATH객체로 저장
    def load_ref_map(self, json_file):
        with open(json_file, 'r') as f:
                data = json.load(f)

        keys = sorted(data.keys(), key=lambda k: int(k))

        rx      = [data[k]['x']         for k in keys]
        ry      = [data[k]['y']         for k in keys]
        ryaw    = [data[k]['yaw']       for k in keys]
        rk      = [data[k]['curvature'] for k in keys]
        rvel    = [data[k]['velocity']  for k in keys]
        rm      = [data[k]['mission']   for k in keys]
        rgear   = [data[k]['gear']      for k in keys]

        self.ref_path = PATH(rx, ry, ryaw, rk, rvel, rm, rgear)

        return self.ref_path

    # 현재 위치의 경로에서의 인덱스 검색
    def nearest_index(self, path, ego_x, ego_y):
        dx = [ego_x - x for x in path.cx]
        dy = [ego_y - y for y in path.cy]
        dist = np.hypot(dx, dy)

        ind = int(np.argmin(dist))

        return ind

    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    # RViz Marker 생성 함수
    def publish_map_marker(self, path: PATH):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ref_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # 굵기/색/가시성
        marker.scale.x = 1.5           # 선 두께 [m]
        marker.color.r = 1.0           # 빨강
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.1   # z-fighting 방지

        for x, y in zip(path.cx, path.cy):
            p = Point(x=x, y=y, z=0.0)
            marker.points.append(p)

        marker_array.markers.append(marker)
        # self.map_marker_pub.publish(marker_array)

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                # 제밍 모드 활성화되면 제어 중단
                if self.jamming_mode_active:
                    rate.sleep()
                    continue
                
                # STOP 유지 타이머가 살아있으면 무조건 제동 유지
                if rospy.get_time() < self._planner_stop_until:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 0.7
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    rate.sleep()
                    continue

                # 인덱스 1632에서 장애물 기반 정지 체크
                current_index = self.ego_index_global
                
                # 디버깅: 현재 인덱스와 1633 근처 상황 로깅
                if current_index is not None and 1630 <= current_index <= 1640:
                    rospy.loginfo(f"[1633 정지 디버그] 현재 인덱스: {current_index}, 정지완료: {self.is_stop_completed_1633}, 제동시작: {self.braking_started_1633}, 정지시작: {self.stop_started_1633}, 장애물감지: {self.obstacles_detected}")
                
                # 1632 도달하면 풀 브레이크 - 속도 0까지
                if current_index is not None and current_index >= 1630 and not self.is_stop_completed_1633:
                    if not self.braking_started_1633:
                        self.braking_started_1633 = True
                        rospy.loginfo("="*50)
                        rospy.loginfo(f"==== 인덱스 {current_index}에서 풀 브레이크 시작! ====")
                        rospy.loginfo("="*50)
                    
                    # 현재 속도 확인
                    current_speed = self.ego_vel  # m/s
                    
                    # 풀 브레이크 - 속도 0까지
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    rospy.loginfo_throttle(0.5, f"[풀 브레이크] 인덱스 {current_index}, 속도: {current_speed:.2f}m/s")
                    
                    # 속도가 거의 0이 되면 Frenet s 기준으로 장애물 판단
                    if current_speed < 0.1:  # 0.1 m/s 이하면 정지로 판단
                        path_clear = self.check_path_clear_with_frenet()
                        if path_clear:
                            self.is_stop_completed_1633 = True
                            rospy.loginfo("="*50)
                            rospy.loginfo(f"==== 완전 정지 완료! Frenet s 기준 전방 경로 클리어 - 주행 재개! ====")
                            rospy.loginfo("="*50)
                        else:
                            rospy.loginfo_throttle(1.0, f"[완전 정지] Frenet s 기준 전방 장애물 감지로 계속 정지 중...")
                            rate.sleep()
                            continue
                    else:
                        # 아직 속도가 있으면 계속 제동
                        rate.sleep()
                        continue

                if not (self.odom_flag and self.gps_flag and self.imu_flag):
                    print("Waiting for GPS and IMU...")
                    rate.sleep()
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    continue

                if self.local_path is None:
                    print("Waiting for /local_path ...")
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    rate.sleep()
                    continue

                # 신호등 제동 확인 (최우선 처리)
                if self.traffic_light_brake:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    rate.sleep()
                    continue

                # pending ↔ current 스왑 시도 : 조건부 스왑 (모드=로컬일 때만)
                if self.has_pending and self.local_path is not None:
                    now = rospy.get_time()
                    if (now - self._last_swap_time) >= self.SWAP_MIN_INTERVAL:
                        # 너무 짧은 로컬 경로는 거름 (길이 10포인트 미만이면 불안정)
                        if self.pending_local_path.length >= 10 or (now - self.pending_stamp) > self.SWAP_TIMEOUT:
                            self.local_path = self.pending_local_path
                            self.has_pending = False
                            self.path_updated = True  # 이번 프레임에 경로가 갱신되었음을 컨트롤러에 알림
                            self.purepursuit_controller.update_path(self.local_path)
                            self.curvebased_vel.update_path(self.local_path)
                            self.local_map_update += 1
                            self._last_swap_time = now
                            # print(f"[local_path] swapped (len={self.local_path.length})")
                        else:
                            # print(f"[local_path] HOLD (len={self.pending_local_path.length})")
                            pass

                # ref_path  rviz 시각화
                # self.publish_map_marker(self.ref_path)

                self.ego_index_global = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)
                self.ego_index_local = self.nearest_index(self.local_path, self.ego_x, self.ego_y)
                self.curvedvelocity = self.curvebased_vel.run(self.ego_index_global, self.ego_index_local)
                # 로컬경로일 경우 속도 제한
                if self.planner_mode == 1:
                    if self.curvedvelocity > Parameter.max_local_velocity:
                        self.curvedvelocity = Parameter.max_local_velocity

                # 외부 속도 제한이 있을 경우 (None이면 무시)
                if self.external_speed_cap is not None:
                    self.curvedvelocity = min(self.curvedvelocity, self.external_speed_cap)

                # 맵 끝에 다다르면 다음 맵으로 전환 : 마지막 인덱스 2개 이내
                # if self.ego_index >= self.ref_path.length - 2 and self.map_index < len(self.all_paths) - 1:
                #     self.map_index += 1
                #     self.ref_path = self.all_paths[self.map_index]
                #     print(f">>> Switch to map #{self.map_index+1}")
                #     # 컨트롤러 맵 업데이트
                #     # self.purepursuit_controller.update_path(self.ref_path)
                #     # 인덱스 및 기어 다시 계산
                #     self.ego_index = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)
                #     self.prev_gear = self.ref_path.cgear[self.ego_index]

                # 기존/변경된 맵 기준 미션과 기어 업데이트 
                self.ego_mission = self.ref_path.cmission[self.ego_index_global]
                self.ego_gear = self.ref_path.cgear[self.ego_index_global]

                # 중간에 맵상에서 기어가 바뀌는 시점 감지
                if self.ego_gear != self.prev_gear:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    # 속도 멈출 때까지 기다림
                    if abs(self.ego_vel) < 0.2:
                        if self.ego_gear == 2:
                            self.change_to_reverse()
                        elif self.ego_gear == 4:
                            self.change_to_drive()
                        elif self.ego_gear == 1:
                            self.change_to_parking()
                        self.prev_gear = self.ego_gear
                    continue

                # # 종료 시점 정지 후 기어 P로 변경
                # if self.ego_index >= self.ref_path.length - 1:
                #     self.ctrl_cmd_msg.accel = 0.0
                #     self.ctrl_cmd_msg.brake = 1.0
                #     self.ctrl_cmd_msg.steering = 0.0
                #     self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                #     if abs(self.ego_vel) < 0.2:
                #         self.change_to_parking()
                #         print("=== End of Path Reached ===")
                #         break
                #     continue

                self.adjusted_yaw = self.ego_yaw
                # 후진 기어로 시작할 때만 아래 코드 주석처리 제거 (디폴트는 직진 시작)
                # if self.ego_gear == 2:
                #     self.change_to_reverse()
                #     self.adjusted_yaw = (self.ego_yaw + 180) % 360
                #
                #     # 아래 내용은 컨트롤러 내부에서 구현
                #     # self.target_velocity *= -1.0
                #     # self.target_curvature *= -1.0
                #     self.ego_vel *= -1.0    # ego 속도는 후진시 음수로 나오므로, 양수로 일관되게 처리 (제어기에서도 양수로 출력)

                # 신호등 제동 확인은 이미 위에서 처리됨
                
                # 미션별 적용
                if self.ego_mission == MissionState.mission_0:
                    self.velocity, self.steering = self.purepursuit_controller.run(self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_gear, self.ego_index_local, self.ego_vel, self.curvedvelocity, self.path_updated)
                elif self.ego_mission == MissionState.mission_1:
                    self.ctrl_cmd_msg.accel = 0.0
                    self.ctrl_cmd_msg.brake = 1.0
                    self.ctrl_cmd_msg.steering = 0.0
                    self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                    continue
                elif self.ego_mission == MissionState.mission_2:
                    pass
                elif self.ego_mission == MissionState.mission_3:
                    pass
                elif self.ego_mission == MissionState.mission_4:
                    pass
                elif self.ego_mission == MissionState.mission_5:
                    pass
                elif self.ego_mission == MissionState.mission_6:
                    pass

                # # 진동 제거
                # if abs(self.steering) < 0.5:
                #     self.steering = 0.0

                accel_cmd, brake_cmd = self.cmd_converter.run(self.velocity, self.ego_vel, self.ego_gear)

                self.ctrl_cmd_msg.accel = accel_cmd
                self.ctrl_cmd_msg.brake = brake_cmd
                self.ctrl_cmd_msg.steering = math.radians(self.normalize_180(self.steering))
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

                # print("-------------------------------------")
                # # print(f" Map        = # {self.map_index+1}")
                # print(" Local Map Following..." if self.planner_mode == 1 else " Global Map Following...")
                # print(f" Map Update   = # {self.local_map_update}")
                # print(f" Path Length  = {self.local_path.length}")
                # print(f" Glob Ind     = {self.ego_index_global}")
                # print(f" Local Ind    = {self.ego_index_local}")
                # print(f" Mission      = {self.ego_mission}")
                # print(f" Collision    = {self.coll_flag}")
                # print(f" Gear         = { {1: 'P', 2: 'R', 3: 'N', 4: 'D'}.get(self.ego_gear, str(self.ego_gear)) }")
                # print(f" Velocity     = {self.ego_vel:.3f}")
                print(f" Accel (%)    = {100 if self.ctrl_cmd_msg.accel * 100 >= 100 else self.ctrl_cmd_msg.accel * 100:.3f}")
                print(f" Brake (%)    = {100 if self.ctrl_cmd_msg.brake * 100 >= 100 else self.ctrl_cmd_msg.brake * 100:.3f}")
                # print(f" Steering     = {self.steering:.3f}")
                # print("-------------------------------------\n")

                rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down...")

if __name__ == "__main__":
    try:
        morai = Morai_Control_Node()
        morai.main()
    except KeyboardInterrupt:
        print("Program interrupted")