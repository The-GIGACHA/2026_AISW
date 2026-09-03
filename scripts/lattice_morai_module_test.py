#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import json
import math
import bisect
import numpy as np
from geometry_msgs.msg import PointStamped, PoseStamped, Point
from nav_msgs.msg import Path
from morai_msgs.msg import GPSMessage
from sensor_msgs.msg import Imu
from pyproj import Proj
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
# RVIZ 용도
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# Parameter 객체 (전역변수)
class Parameter:
    # 아이오닉5 모델 스펙
    vehicle_wheelbase = 3.000       # 차량 휠베이스 [m]
    vehicle_length = 4.635          # 차량 전체 길이 [m]
    vehicle_width = 1.892           # 차량 폭 [m]
    vehicle_height = 2.434          # 차량 전체 높이 [m]
    vehicle_front_overhang = 0.845  # 앞바퀴부터 앞 범퍼까지 거리 [m]
    vehicle_rear_overhang = 0.79    # 뒷바퀴부터 뒤 범퍼까지 거리 [m]
    vehicle_min_radius = 5.87       # 차량 최소회전반경 [m]
 
    road_width = 3.4           # 도로 폭 [m] # 3.4로 해야 좀 넉넉하게 회피함..
    dd_sampling_num = 3         # 도로 횡방향 샘플링 개수   (차선 포함)

    lookahead_distance = 25.0   # 경로 종방향 거리 [m] (더 멀리 미리 계획)
    ds_sampling_num = 10        # 경로 종방향 샘플링 개수 (현재 위치 포함)
    ds_interval = lookahead_distance / (ds_sampling_num - 1) # 경로 종방향 샘플링 간격 [m]
    
    INPUT_JSON = '/home/taegang/catkin_ws/src/2025_HL_MORAI_FINAL-ROUND/map/sangam_bonseon_hdmap.json' # json파일 경로
    local_step_size = 0.2   # 로컬경로 인덱스 간격 [m]

    # obs_radius 제거 - 장애물 정확한 크기 사용
    safety_buf = 5.0       # 장애물과의 최소 안전거리 [m] (과도한 거리 축소)

    pre_window_s = 20.0     # 장애물 앞쪽으로 미리 페널티를 뿌릴 s거리 [m] (더 일찍 회피 시작)
    sigma_d = 0.5          # 가우시안 표준편차 [m]
    pre_cost_weight = 3.0  # 장애물 차선 선제적 가중치

    lateral_cost_weight = 2.5         # 도로 횡방향 비용 가중치
    obs_cost_weight = 25                # 장애물 비용 가중치 (100->40으로 감소)
    smooth_cost_weight = 8.0           # 횡방향 차선 변경 비용 가중치

    # 하이브리드 모드 설정
    HYBRID_MODE_ENABLED = True          # 하이브리드 모드 사용 여부
    OBSTACLE_DETECTION_DISTANCE = 35.0  # 장애물 감지 거리 [m] (더 멀리서 감지)
    MODE_SWITCH_HYSTERESIS = 5.0        # 모드 전환 히스테리시스 [m]
    
    # 회피 후 복귀 안정화 설정
    AVOIDANCE_RECOVERY_TIME = 3.0       # 회피 후 래티스 모드 유지 시간 [초]
    AVOIDANCE_RECOVERY_DISTANCE = 10.0  # 회피 후 래티스 모드 유지 거리 [m]

    PLOT_FLAG = False

           # 시각화 여부

# PATH 객체
class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = cv
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)

class CubicSpline1D:
    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted in ascending order")

        self.a, self.b, self.c, self.d = [], [], [], []
        self.x = x
        self.y = y
        self.nx = len(x)  # dimension of x

        # calc coefficient a
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h, self.a)
        self.c = np.linalg.solve(A, B)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            d = (self.c[i + 1] - self.c[i]) / (3.0 * h[i])
            b = 1.0 / h[i] * (self.a[i + 1] - self.a[i]) \
                - h[i] / 3.0 * (2.0 * self.c[i] + self.c[i + 1])
            self.d.append(d)
            self.b.append(b)

    def calc_position(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        position = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return position

    def calc_first_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        dy = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return dy

    def calc_second_derivative(self, x):
        if x < self.x[0]:
            return None
        elif x > self.x[-1]:
            return None

        i = self.__search_index(x)
        dx = x - self.x[i]
        ddy = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return ddy

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        return A

    def __calc_B(self, h, a):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (a[i + 2] - a[i + 1]) / h[i + 1]\
                - 3.0 * (a[i + 1] - a[i]) / h[i]
        return B


class CubicSpline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc_position(s)
        y = self.sy.calc_position(s)

        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calc_first_derivative(s)
        ddx = self.sx.calc_second_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        ddy = self.sy.calc_second_derivative(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calc_first_derivative(s)
        dy = self.sy.calc_first_derivative(s)
        yaw = np.rad2deg(math.atan2(dy, dx))
        return yaw


class LatticePlanner:
    def __init__(self):
        rospy.init_node("lattice_planner", anonymous=True)
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/lattice_markers", MarkerArray, queue_size=1)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)

        # 하이브리드 모드 관련 변수
        self.hybrid_mode = Parameter.HYBRID_MODE_ENABLED  # 하이브리드 모드 활성화
        self.use_global_path = True  # 초기에는 글로벌 패스 사용
        self.obstacle_detection_distance = Parameter.OBSTACLE_DETECTION_DISTANCE  # 장애물 감지 거리 [m]
        self.mode_switch_hysteresis = Parameter.MODE_SWITCH_HYSTERESIS  # 모드 전환 히스테리시스 [m]
        
        # 회피 후 복귀 안정화 관련 변수
        self.last_avoidance_time = 0.0      # 마지막 회피 시점
        self.last_avoidance_position = None # 마지막 회피 위치
        self.is_in_recovery_mode = False    # 회피 복귀 모드 여부

        # MORAI 시뮬레이터
        # rospy.Subscriber("odom", Odometry, self.odom_callback)

        # rospy.Subscriber('/camera/obstacles_xy', Float32MultiArray, self.obs_callback)

        self.ref_path = self.load_ref_map(Parameter.INPUT_JSON)
        # CubicSpline2D 객체 생성
        self.spline_ref = CubicSpline2D(self.ref_path.cx, self.ref_path.cy)

        # 현재위치로부터의 후보경로 s,d
        self.ego_s_list = None
        self.ego_d_list = None

        self.gps_x = None
        self.gps_y = None

        self.obs = []
        self.obs_flag = False
        
        # 테스트용 임의 장애물 (기본값)
        self.test_obstacles = [
            # 장애물 1
            [647.63, -689.08, 649.32, -690.44, 657.37, -681.87, 655.28, -680.47],
            # 장애물 2
            [790.09, -657.23, 787.5, -658.04, 779.47, -649.53, 781.44, -647.98],
            # 장애물 3.x(차량)
            [818.84, -766.02, 817.44, -764.47, 819.45, -762.32, 820.97, -763.70],
            # 장애물 3
            [781.7, - 812.92, 779.11, -812.61, 784.92, -803.05, 787.94, -805.03],
            # 장애물 4
            [704.23, -905.66, 699.99, -904.64, 717.09, -882.98, 720.22, -886.13]
        ]
        
        # obstacle_planner에서 전달받을 장애물 좌표 (현재는 주석 처리)
        self.dynamic_obstacles = []  # obstacle_planner에서 업데이트됨

        self.gps_flag = False

        self.ego_yaw = None
        self.imu_flag = False

        # 차량 각 꼭짓점 좌표                                                                                                                       
        self.fl_corner = np.array([0.0, 0.0])
        self.fr_corner = np.array([0.0, 0.0])
        self.rl_corner = np.array([0.0, 0.0])
        self.rr_corner = np.array([0.0, 0.0])
        self.vehicle_corner = None

        # 현재인덱스 검색 함수 용도
        self.search_ds = 0.2   # 최근접 s 탐색 전용 간격(0.5~1.0 권장) : local_step_size랑 분리
        s0, s1 = self.spline_ref.s[0], self.spline_ref.s[-1]
        self.ref_s_search = np.arange(s0, s1, self.search_ds)
        self.ref_xy_search = np.array([[self.spline_ref.calc_position(si)[0], self.spline_ref.calc_position(si)[1]] for si in self.ref_s_search])
        self.last_s = None

        # 초기 글로벌 패스 발행 (Waiting for Local Path 문제 해결)
        self.publish_initial_global_path = True
        self.initial_path_published = False

        # [d,s]격자의 경로 비용 & 장애물 비용
        self.lattice_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.ds_sampling_num])
        # 횡방향 차선 변경 비용
        self.smooth_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.dd_sampling_num])

        # 최종 DP 알고리즘 노드별 비용 : DP[j,i] : 레이어 i, 칸 j 까지 올 때의 최소 누적 비용
        self.dp_cost = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), float('inf'))
        # backpointer 초기화
        self.backptr = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), -1, dtype=int)
        
        # 비용 시각화 초기화
        self.cost_fig = None
        self.cost_ax = None

        ################################ 그래프 plot
        if Parameter.PLOT_FLAG:
            # 이전 실행 때 떠있던 윈도우 닫고 시작
            plt.close('all')
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10,6))

            # --- 영구 artists 만들기: 루프에서는 set_* 로 "갱신만" 합니다 ---
            # 후보 경로 라인들 (dd_sampling_num 개수만큼)
            self.cand_lines = [self.ax.plot([], [], 'g--', linewidth=1)[0]
                            for _ in range(Parameter.dd_sampling_num)]

            # 최종(스플라인) 경로
            self.spline_line, = self.ax.plot([], [], color='red', linewidth=2, solid_capstyle='round')

            # 차량 위치(점) + 차체
            self.vehicle_point, = self.ax.plot([], [], 'bo')
            # 차체 외곽선(라인) — Polygon 대신 Line2D 사용
            self.vehicle_line, = self.ax.plot([], [], '-', linewidth=2, color='tab:blue', zorder=5)

            # 장애물 오브젝트(원/텍스트)는 처음 들어오는 순간 1회 생성 후 재사용
            self.obs_inited = False
            self.obs_body_patches = []
            self.obs_safe_patches = []
            self.obs_texts = []
        ################################

    # 전역경로 json파일 불러와 PATH객체로 저장
    def load_ref_map(self, json_file):
        with open(json_file, 'r') as f:
                data = json.load(f)

        keys = sorted(data.keys(), key=lambda k: int(k))

        # rx, ry, ryaw, rk, rvel, rm, rgear 읽어옴
        rx      = [data[k]['x']         for k in keys]
        ry      = [data[k]['y']         for k in keys]
        ryaw    = [data[k]['yaw']       for k in keys]
        rk      = [data[k]['curvature'] for k in keys]
        rvel    = [data[k]['velocity']  for k in keys]
        rm      = [data[k]['mission']   for k in keys]
        rgear   = [data[k]['gear']      for k in keys]

        self.ref_path = PATH(rx, ry, ryaw, rk, rvel, rm, rgear)

        return self.ref_path
    
    #################### SENSOR CALLBACK ####################
    def gps_callback(self, msg):
        utm_x, utm_y = self.proj_UTM(msg.longitude, msg.latitude)
        self.gps_x = utm_x - msg.eastOffset
        self.gps_y = utm_y - msg.northOffset
        self.gps_flag = True

    def imu_callback(self, msg):
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

    def obstacle_node(self):
        self.obs_flag = True
        
        # obstacle_planner에서 전달된 장애물이 있으면 사용, 없으면 test_obstacles 사용
        if self.dynamic_obstacles:
            self.obs = self.dynamic_obstacles.copy()
            rospy.loginfo_throttle(2.0, f"obstacle_planner에서 전달받은 장애물 {len(self.obs)}개 사용")
        else:
            # 기존 test_obstacles 사용 (기본값)
            self.obs = self.test_obstacles.copy()
        
        # 디버그: 현재 차량 위치와 장애물 거리 출력
        if hasattr(self, 'gps_x') and hasattr(self, 'gps_y'):
            print(f"차량 위치: ({self.gps_x:.1f}, {self.gps_y:.1f})")
            for i, obs in enumerate(self.obs):
                # 장애물의 중심점 계산 (4개 포인트의 평균)
                obs_points = np.array([[obs[j], obs[j+1]] for j in range(0, 8, 2)])
                center_x = np.mean(obs_points[:, 0])
                center_y = np.mean(obs_points[:, 1])
                dist = math.sqrt((self.gps_x - center_x)**2 + (self.gps_y - center_y)**2)
                print(f"장애물 {i+1} 중심({center_x:.1f}, {center_y:.1f}): 거리 {dist:.1f}m")
        
        return self.obs
    
    def update_obstacles(self, obstacles):
        """obstacle_planner에서 호출하는 장애물 업데이트 함수"""
        self.dynamic_obstacles = obstacles
        rospy.loginfo_throttle(3.0, f"장애물 업데이트: {len(obstacles)}개 받음")

    def check_obstacles_nearby(self):
        """현재 차량 주변에 장애물이 있는지 확인"""
        if not (hasattr(self, 'gps_x') and hasattr(self, 'gps_y') and self.obs):
            return False
        
        min_distance = float('inf')
        for obs in self.obs:
            # 장애물의 중심점 계산 (4개 포인트의 평균)
            obs_points = np.array([[obs[i], obs[i+1]] for i in range(0, 8, 2)])
            center_x = np.mean(obs_points[:, 0])
            center_y = np.mean(obs_points[:, 1])
            
            dist = math.sqrt((self.gps_x - center_x)**2 + (self.gps_y - center_y)**2)
            min_distance = min(min_distance, dist)
        
        return min_distance < self.obstacle_detection_distance

    def update_hybrid_mode(self):
        """하이브리드 모드에서 글로벌 패스와 래티스 패스 간의 모드 전환을 결정 (회피 후 안정화 포함)"""
        if not self.hybrid_mode:
            return False  # 하이브리드 모드가 아니면 항상 래티스 사용
        
        current_time = rospy.get_time()
        obstacles_nearby = self.check_obstacles_nearby()
        
        # 현재 위치
        current_pos = (self.gps_x, self.gps_y) if hasattr(self, 'gps_x') and hasattr(self, 'gps_y') else None
        
        # 회피 복귀 모드 상태 체크
        recovery_time_passed = (current_time - self.last_avoidance_time) >= Parameter.AVOIDANCE_RECOVERY_TIME
        recovery_distance_passed = True
        
        if self.last_avoidance_position is not None and current_pos is not None:
            distance_from_avoidance = math.sqrt(
                (current_pos[0] - self.last_avoidance_position[0])**2 + 
                (current_pos[1] - self.last_avoidance_position[1])**2
            )
            recovery_distance_passed = distance_from_avoidance >= Parameter.AVOIDANCE_RECOVERY_DISTANCE
        
        # 회피 복귀 모드 종료 조건
        if self.is_in_recovery_mode and (recovery_time_passed and recovery_distance_passed):
            self.is_in_recovery_mode = False
            rospy.loginfo("회피 복귀 모드 종료 (시간: {:.1f}초, 거리: {:.1f}m 경과)".format(
                current_time - self.last_avoidance_time, 
                distance_from_avoidance if self.last_avoidance_position else 0
            ))
        
        # 히스테리시스를 적용한 모드 전환 로직
        if self.use_global_path and obstacles_nearby:
            # 글로벌 패스 사용 중인데 장애물 발견 -> 래티스로 전환
            min_distance = float('inf')
            if hasattr(self, 'gps_x') and hasattr(self, 'gps_y') and self.obs:
                for obs in self.obs:
                    # 장애물의 중심점 계산 (4개 포인트의 평균)
                    obs_points = np.array([[obs[j], obs[j+1]] for j in range(0, 8, 2)])
                    center_x = np.mean(obs_points[:, 0])
                    center_y = np.mean(obs_points[:, 1])
                    dist = math.sqrt((self.gps_x - center_x)**2 + (self.gps_y - center_y)**2)
                    min_distance = min(min_distance, dist)
            
            if min_distance < self.obstacle_detection_distance:
                self.use_global_path = False
                self.last_avoidance_time = current_time
                self.last_avoidance_position = current_pos
                self.is_in_recovery_mode = True
                rospy.loginfo("모드 전환: 글로벌 패스 -> 래티스 패스 (장애물 감지, 거리: {:.1f}m)".format(min_distance))
                
        elif not self.use_global_path and not obstacles_nearby:
            # 래티스 사용 중인데 장애물 없음 -> 글로벌 패스로 전환 (단, 회피 복귀 모드가 아닐 때만)
            if not self.is_in_recovery_mode:
                min_distance = float('inf')
                if hasattr(self, 'gps_x') and hasattr(self, 'gps_y') and self.obs:
                    for obs in self.obs:
                        # 장애물의 중심점 계산 (4개 포인트의 평균)
                        obs_points = np.array([[obs[j], obs[j+1]] for j in range(0, 8, 2)])
                        center_x = np.mean(obs_points[:, 0])
                        center_y = np.mean(obs_points[:, 1])
                        dist = math.sqrt((self.gps_x - center_x)**2 + (self.gps_y - center_y)**2)
                        min_distance = min(min_distance, dist)
                
                # 히스테리시스: 장애물이 감지 거리 + 히스테리시스보다 멀어져야 전환
                if min_distance > (self.obstacle_detection_distance + self.mode_switch_hysteresis):
                    self.use_global_path = True
                    rospy.loginfo("모드 전환: 래티스 패스 -> 글로벌 패스 (장애물 없음, 거리: {:.1f}m)".format(min_distance))
            else:
                rospy.loginfo_throttle(2.0, "회피 복귀 모드 중 - 래티스 모드 유지 (남은 시간: {:.1f}초)".format(
                    max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (current_time - self.last_avoidance_time))
                ))
        
        return self.use_global_path

    # 카메라 토픽 콜백: data.data = [x1,y1, x2,y2, …]
    # def obs_callback(self, msg: Float32MultiArray):
    #     arr = msg.data
    #     if len(arr) >= 1:
    #         # 장애물 좌표가 있으면
    #         self.obs = [(arr[i], arr[i+1]) for i in range(0, len(arr), 2)]
    #         self.obs_flag = True
    #     else:
    #         # 빈 메시지면 장애물 없음
    #         self.obs = []
    #         self.obs_flag = False
    
    ##########################################################

    # 로컬좌표계 (xv,yv)를 월드좌표계 (X,Y)로 변환 : 기준좌표(x0,y0)
    def local_to_world(self, x0, y0, yaw_deg, xv, yv):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        R = np.array([[c, -s],
                      [s,  c]])
        X, Y = np.array([x0, y0]) + R @ np.array([xv, yv])
        return float(X), float(Y)

    # 경로상의 위치 s 계산
    def find_nearest_s(self, x, y, mode='ego', center_s=None):
        P = np.array([x, y])
        s_arr = self.ref_s_search
        XY = self.ref_xy_search

        # 검색 구간 결정
        if mode == 'ego' and self.last_s is not None:
            # ego 차량은 최근 s 주변만(뒤 10m ~ 앞 30m)
            s0 = max(self.last_s - 10.0, s_arr[0])
            s1 = min(self.last_s + 30.0, s_arr[-1])
            i0 = int((s0 - s_arr[0]) / self.search_ds); i1 = int((s1 - s_arr[0]) / self.search_ds) + 1
        elif mode == 'obs' and center_s is not None:
            # 장애물은 ego_s 근방만(선제 윈도우까지) : 뒤 5m ~ 앞 (pre_window_s+30)m
            s0 = max(center_s - 5.0, s_arr[0])
            s1 = min(center_s + Parameter.pre_window_s + 30.0, s_arr[-1])
            i0 = int((s0 - s_arr[0]) / self.search_ds)
            i1 = int((s1 - s_arr[0]) / self.search_ds) + 1
        else:
            i0, i1 = 0, len(s_arr)

        d = XY[i0:i1] - P                           # 차이
        j = int(np.argmin((d * d).sum(axis=1)))     # dx^2+dy^2 제곱거리 가장 작은 인덱스
        s = s_arr[i0 + j]                           # 경로상에서 몇 미터 지점인지
        if mode == 'ego':
            self.last_s = s
        return s

    # 횡방향 편차 d 계산 (법선벡터 내적)
    def get_frenet_d(self, s_ref, x, y):
        xr, yr = self.spline_ref.calc_position(s_ref)
        yaw_r = math.radians(self.spline_ref.calc_yaw(s_ref))

        # (dx, dy) 벡터
        dx = x - xr
        dy = y - yr
        # 도로 방향벡터는 (cos(yaw), sin(yaw))이므로, 법선벡터는 (-sin(yaw), cos(yaw)) : 왼쪽 + / 오른쪽 -
        d0 = dx * (-math.sin(yaw_r)) + dy * math.cos(yaw_r)
        return d0

    def frenet_to_world(self, s, d):
        """기준경로 s 지점에서 횡방향 d만큼 떨어진 월드좌표 (x,y)와 참조 yaw(deg)를 반환"""
        xr, yr = self.spline_ref.calc_position(s)
        yaw_deg = self.spline_ref.calc_yaw(s)
        yaw = math.radians(yaw_deg)
        # 법선벡터 이용하여 오프셋d 적용
        x = xr - math.sin(yaw) * d
        y = yr + math.cos(yaw) * d
        return x, y, yaw_deg

    # 차량과 장애물 간의 여유거리 계산
    def plot_lattice_cost_overlay(self):
        """기존 차량 plot에 격자 비용 오버레이 추가"""
        if not Parameter.PLOT_FLAG:
            return
            
        # 기존 plot이 있을 때만 격자 정보 추가
        if hasattr(self, 'ax') and self.ax is not None:
            # 격자별 비용을 색상으로 표시
            lattice_cost_clipped = np.where(self.lattice_cost_array == float('inf'), 100, self.lattice_cost_array)
            
            # 격자점들의 실제 월드 좌표 계산
            grid_x, grid_y = [], []
            grid_costs = []
            
            for i, s in enumerate(self.ego_s_list):
                for j, d in enumerate(self.ego_d_list):
                    # frenet to world 변환
                    x, y, _ = self.frenet_to_world(s, d)
                    grid_x.append(x)
                    grid_y.append(y)
                    grid_costs.append(lattice_cost_clipped[j, i])
            
            # 기존 scatter 제거 (업데이트를 위해)
            if hasattr(self, 'grid_scatter'):
                self.grid_scatter.remove()
            if hasattr(self, 'cost_texts'):
                for text in self.cost_texts:
                    text.remove()
                self.cost_texts = []
            
            # 격자점들을 scatter plot으로 표시 (비용에 따른 색상)
            self.grid_scatter = self.ax.scatter(grid_x, grid_y, c=grid_costs, cmap='RdYlBu_r', 
                                              s=80, alpha=0.8, vmin=0, vmax=50, zorder=2)
            
            # colorbar 추가 (한 번만)
            if not hasattr(self, 'cost_colorbar'):
                self.cost_colorbar = plt.colorbar(self.grid_scatter, ax=self.ax, shrink=0.8)
                self.cost_colorbar.set_label('Grid Cost', rotation=270, labelpad=20)
            
            # 격자점에 비용 수치 표시 (일부만)
            self.cost_texts = []
            for i in range(0, len(grid_x), 4):  # 4개마다 하나씩만 표시
                if grid_costs[i] < 90:  # 무한대가 아닌 경우만
                    text = self.ax.text(grid_x[i], grid_y[i], f'{grid_costs[i]:.1f}', 
                                      fontsize=7, ha='center', va='center', 
                                      bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.8),
                                      zorder=5)
                    self.cost_texts.append(text)
            
            # 최적 경로를 굵은 선으로 표시
            if hasattr(self, 'best_path_d') and hasattr(self, 'best_path_s'):
                path_x, path_y = [], []
                for s, d in zip(self.best_path_s, self.best_path_d):
                    x, y, _ = self.frenet_to_world(s, d)
                    path_x.append(x)
                    path_y.append(y)
                
                # 최적 경로를 굵은 파란색 선으로 표시
                if hasattr(self, 'optimal_path_line'):
                    self.optimal_path_line.set_data(path_x, path_y)
                else:
                    self.optimal_path_line, = self.ax.plot(path_x, path_y, 'b-', 
                                                         linewidth=4, alpha=0.8, zorder=3,
                                                         label='Optimal Path')
            
            # 범례에 격자 정보 추가
            if not hasattr(self, 'cost_legend_added'):
                self.ax.legend(loc='upper right')
                self.cost_legend_added = True
                
            # 제목에 비용 정보 추가
            ego_d_idx = len(self.ego_d_list) // 2  # 중앙 차선 인덱스
            center_cost = lattice_cost_clipped[ego_d_idx, 0]  # 시작점 비용
            self.ax.set_title(f'Lattice Path Planning (Center Lane Cost: {center_cost:.1f})')
            
            plt.pause(0.01)
    
    def obstacle_clearance(self, vehicle_corners, obstacle_corners):
        """
        SAT(Separating Axis Theorem)을 사용한 사각형 vs 사각형 충돌 검사
        vehicle_corners: shape (4,2) 차량의 4개 꼭짓점 (FL, FR, RR, RL 순서)
        obstacle_corners: shape (4,2) 장애물의 4개 꼭짓점
        return: gap >= 0 (0이면 접촉/충돌, >0이면 최단거리)
        """
        def get_projection_range(corners, axis):
            """주어진 축에 대한 투영 범위 계산"""
            projections = np.dot(corners, axis)
            return np.min(projections), np.max(projections)
        
        def get_edges(corners):
            """사각형의 변 벡터들을 반환 (시계방향)"""
            edges = []
            for i in range(4):
                edge = corners[(i+1)%4] - corners[i]
                edges.append(edge)
            return edges
        
        def get_normals(edges):
            """변 벡터들에 수직인 법선 벡터들을 반환"""
            normals = []
            for edge in edges:
                # 2D에서 수직 벡터: (x,y) -> (-y,x)
                normal = np.array([-edge[1], edge[0]])
                # 정규화
                length = np.linalg.norm(normal)
                if length > 1e-10:
                    normal = normal / length
                    normals.append(normal)
            return normals
        
        # 두 사각형이 분리되어 있는지 확인
        vehicle_edges = get_edges(vehicle_corners)
        obstacle_edges = get_edges(obstacle_corners)
        
        # 모든 법선 벡터들 (차량과 장애물의 모든 변에 수직인 벡터들)
        all_normals = get_normals(vehicle_edges) + get_normals(obstacle_edges)
        
        min_overlap = float('inf')
        separating_axis = None
        
        for normal in all_normals:
            # 각 사각형을 이 축에 투영
            vehicle_min, vehicle_max = get_projection_range(vehicle_corners, normal)
            obstacle_min, obstacle_max = get_projection_range(obstacle_corners, normal)
            
            # 겹치는지 확인
            if vehicle_max < obstacle_min or obstacle_max < vehicle_min:
                # 분리된 축을 찾음 -> 충돌하지 않음
                gap = min(abs(vehicle_max - obstacle_min), abs(obstacle_max - vehicle_min))
                return gap
            
            # 겹치는 정도 계산
            overlap = min(vehicle_max - obstacle_min, obstacle_max - vehicle_min)
            if overlap < min_overlap:
                min_overlap = overlap
                separating_axis = normal
        
        # 모든 축에서 겹침 -> 충돌
        return 0.0

    def lattice_node(self, gps_x, gps_y):
        # 현재위치 ind, s, d 계산 : 종방향s만 최대 0.25m 오차 존재 / 횡방향d는 오차 거의 X
        ego_s = self.find_nearest_s(gps_x, gps_y, mode='ego')
        ego_d = self.get_frenet_d(ego_s, gps_x, gps_y)
        s_max = self.spline_ref.s[-1]

        # 현재위치로부터의 격자 [m]
        self.ego_s_list = [min(ego_s + i * Parameter.ds_interval, s_max) for i in range(Parameter.ds_sampling_num)]     # s는 현재위치 기준 (단, 끝 지점 도달시 그냥 끝 인덱스 대입 -> 마지막에 대해 반복계산할 뿐 오류나지 X)
        self.ego_d_list = np.linspace(0.0, Parameter.road_width * 2.0, Parameter.dd_sampling_num)                       # d는 도로기준 고정 경로 : 왼쪽 + / 오른쪽 -
        
        # 횡방향 cost
        for i, d in enumerate(self.ego_d_list):
            if abs(d) <= Parameter.road_width * 2.0:
                self.lattice_cost_array[i, :] = abs(d) * Parameter.lateral_cost_weight
            else:
                self.lattice_cost_array[i, :] = float('inf')

        # 차량 거동을 고려한 장애물 회피 비용
        xr      = np.array([self.spline_ref.calc_position(s)[0] for s in self.ego_s_list])
        yr      = np.array([self.spline_ref.calc_position(s)[1] for s in self.ego_s_list])
        yaw_deg = np.array([self.spline_ref.calc_yaw(s) for s in self.ego_s_list])
        yaw     = np.deg2rad(yaw_deg)
        sin_yaw = np.sin(yaw)
        cos_yaw = np.cos(yaw)
        fx = Parameter.vehicle_front_overhang + Parameter.vehicle_wheelbase   # +x 전방
        rx = -Parameter.vehicle_rear_overhang                                 # -x 후방
        hy = Parameter.vehicle_width / 2.0     
        if self.obs_flag and self.obs is not None and len(self.obs) > 0:
            for i, s in enumerate(self.ego_s_list):
                for j, d in enumerate(self.ego_d_list):
                    # (s, d)에서 차량 중심
                    x_c = xr[i] - sin_yaw[i] * d
                    y_c = yr[i] + cos_yaw[i] * d
                    c, s_ = cos_yaw[i], sin_yaw[i]  # 회전 재사용

                    # 코너 4점 (로컬 -> 월드, 한 번의 회전/이동만)
                    fl = np.array([x_c + c*fx - s_*hy, y_c + s_*fx + c*hy])
                    fr = np.array([x_c + c*fx + s_*hy, y_c + s_*fx - c*hy])
                    rl = np.array([x_c + c*rx - s_*hy, y_c + s_*rx + c*hy])
                    rr = np.array([x_c + c*rx + s_*hy, y_c + s_*rx - c*hy])
                    rect = np.vstack([fl, fr, rr, rl])  # 시계방향 (4,2)

                    # 이 셀에서의 최소 gap
                    min_gap = float('inf')
                    for obs in self.obs:  # 장애물은 4개 포인트 사각형
                        # 장애물 사각형 생성 [x1,y1, x2,y2, x3,y3, x4,y4] -> (4,2) array
                        obs_corners = np.array([[obs[i], obs[i+1]] for i in range(0, 8, 2)])
                        gap = self.obstacle_clearance(rect, obs_corners)
                        if gap < min_gap:
                            min_gap = gap

                    # 충돌/내부이면 셀 배제
                    if min_gap <= 0.0:
                        self.lattice_cost_array[j, i] = float('inf')
                        continue
                        
                    # safety_buf 이내일 때만 비용 부과 (수정된 부분)
                    if min_gap < Parameter.safety_buf:
                        scale = (Parameter.safety_buf - min_gap) / Parameter.safety_buf
                        penalty = scale * scale  # 제곱으로 더 부드럽게
                        self.lattice_cost_array[j, i] += Parameter.obs_cost_weight * penalty  # *10 제거!

        # 장애물 비용
        if self.obs_flag and self.obs is not None:
            for obs in self.obs:
                # 장애물의 중심점 계산 (4개 포인트의 평균)
                obs_points = np.array([[obs[i], obs[i+1]] for i in range(0, 8, 2)])
                obs_x = np.mean(obs_points[:, 0])
                obs_y = np.mean(obs_points[:, 1])
                
                # 장애물 중심좌표도 Frenet으로 변환
                s_obs = self.find_nearest_s(obs_x, obs_y, mode='obs', center_s=ego_s)
                d_obs = self.get_frenet_d(s_obs, obs_x, obs_y)
                # 이 장애물이 영향을 미치는 셀만 찾아서 비용 추가
                for i, s in enumerate(self.ego_s_list):
                    # 실제 장애물 비용
                    for j, d in enumerate(self.ego_d_list):
                        # lattice point를 실제 좌표로 변환
                        lattice_x, lattice_y, _ = self.frenet_to_world(s, d)
                        
                        # 차량을 lattice point에 위치시켰을 때의 사각형 생성
                        half_length = Parameter.vehicle_length / 2.0
                        half_width = Parameter.vehicle_width / 2.0
                        
                        # 현재 차량의 heading 사용 (lattice는 현재 자세 기준으로 계획됨)
                        ref_heading = math.radians(self.ego_yaw if self.ego_yaw is not None else 0.0)
                        
                        # 차량 사각형의 4개 모서리 계산
                        vehicle_corners = []
                        corners_local = [
                            (half_length, half_width),
                            (half_length, -half_width), 
                            (-half_length, -half_width),
                            (-half_length, half_width)
                        ]
                        
                        for dx, dy in corners_local:
                            x = lattice_x + dx * math.cos(ref_heading) - dy * math.sin(ref_heading)
                            y = lattice_y + dx * math.sin(ref_heading) + dy * math.cos(ref_heading)
                            vehicle_corners.extend([x, y])
                        
                        # SAT를 사용하여 충돌 검사  
                        obs_corners_array = np.array([[obs[k], obs[k+1]] for k in range(0, 8, 2)])
                        vehicle_corners_array = np.array([[vehicle_corners[k], vehicle_corners[k+1]] for k in range(0, 8, 2)])
                        collision_gap = self.obstacle_clearance(vehicle_corners_array, obs_corners_array)
                        if collision_gap <= 0.0:
                            # 충돌이 감지되면 최대 비용 적용
                            self.lattice_cost_array[j, i] += Parameter.obs_cost_weight
                        else:
                            # 충돌하지 않는 경우 거리 기반 비용 계산
                            dis = math.hypot(s - s_obs, d - d_obs)
                            safety_margin = Parameter.safety_buf
                            if dis <= safety_margin:
                                self.lattice_cost_array[j, i] += Parameter.obs_cost_weight * (safety_margin - dis) / safety_margin
                            
                    ds = s_obs - s
                    if 0.0 < ds <= 20.0:  # 20m 전방부터 감지
                        for j, d in enumerate(self.ego_d_list):
                            # 장애물이 경로 상에 있을 때만
                            if abs(d_obs) < Parameter.road_width * 1.5:
                                dd = abs(d - d_obs)
                                
                                # 거리 기반 팩터 (가까울수록 강함)
                                distance_factor = (20.0 - ds) / 20.0  # 0~1
                                distance_factor = distance_factor ** 1.5  # 지수적 증가
                                
                                # 횡방향 팩터 (같은 차선일수록 강함)
                                lateral_factor = max(0, 1.0 - dd / Parameter.road_width)
                                
                                # 최종 비용 (더 강하게)
                                cost = 5.0 * distance_factor * lateral_factor
                                self.lattice_cost_array[j, i] += cost
                            
        # 차선변경 스무딩 cost : 시작j => 종료k (급격한 횡방향 변화 막기 위해)
        for j in range(Parameter.dd_sampling_num):
            for k in range(Parameter.dd_sampling_num):
                self.smooth_cost_array[j, k] = abs(j - k) * Parameter.smooth_cost_weight

        ##### 동적계획법 DP 시작 #####

        # 출발 레이어(col=0)는 전이비용이 없으므로 노드비용만
        self.dp_cost[:, 0] = self.lattice_cost_array[:, 0]

        for col in range(1, Parameter.ds_sampling_num):
            for row in range(Parameter.dd_sampling_num):
                candidates = []
                for prev in range(Parameter.dd_sampling_num):
                    cost = (self.dp_cost[prev, col-1] + self.smooth_cost_array[prev, row] + self.lattice_cost_array[row, col])
                    candidates.append(cost)

                best_prev = int(np.argmin(candidates))
                # 가장 싸게 올 수 있는 이전 칸(prev) → 현재 칸(row, col) 전이 비용
                self.dp_cost[row, col] = candidates[best_prev]
                # 최소 비용을 내는 이전 칸(prev) 의 인덱스 : 경로 역추적할 때 사용
                self.backptr[row, col] = best_prev

        # 7) 최종 레이어에서 최저비용 행(row) 찾고 역추적 (전이비용 고려하기 위해 역추적 진행)
        end_col = Parameter.ds_sampling_num - 1
        best_row = int(np.argmin(self.dp_cost[:, end_col]))
        path_rows = [best_row]
        current_row = best_row
        for col in range(end_col, 0, -1):
            r = self.backptr[current_row, col]
            path_rows.append(r)
            current_row = r

        # 역추적해서 나온 d 인덱스 리스트
        d_idx_list = list(reversed(path_rows))  # col 순서대로 정렬

        # 인덱스를 실제 거리값으로 매핑 [m]
        d_final_list = [self.ego_d_list[idx] for idx in d_idx_list]
        print(f"Selected path d values: {d_final_list}")

        ############################
        
        # 최적 경로 저장 (시각화용)
        self.best_path_d = d_final_list.copy()
        self.best_path_s = self.ego_s_list.copy()
        
        # 기존 plot에 격자 비용 오버레이 (3Hz 주기)
        if not hasattr(self, '_viz_counter'):
            self._viz_counter = 0
        self._viz_counter += 1
        if self._viz_counter % 3 == 0:  # 3번마다 한 번씩 시각화
            self.plot_lattice_cost_overlay()

        return d_final_list, self.ego_s_list

    def make_global_path_segment(self, ego_s):
        """글로벌 경로에서 현재 위치 기준으로 앞쪽 경로 세그먼트를 생성"""
        path_x = []
        path_y = []
        
        # 현재 위치에서 앞쪽으로 lookahead_distance만큼의 경로 생성
        s_max = self.spline_ref.s[-1]
        s_end = min(ego_s + Parameter.lookahead_distance, s_max)
        
        # 0.5m 간격으로 경로 점 생성
        s_samples = np.arange(ego_s, s_end, 0.5)
        if len(s_samples) == 0 or s_samples[-1] < s_end - 0.1:
            s_samples = np.append(s_samples, s_end)
        
        for s in s_samples:
            x, y = self.spline_ref.calc_position(s)
            path_x.append(x)
            path_y.append(y)
            
        return path_x, path_y

    # (s, d) 최종경로를 (x, y)로 변환
    def make_path(self, d_list, s_list):
        """
        - d_list: DP에서 찾은 각 단계의 d 값 리스트
        - s_list: 각 단계의 s 값 리스트
        """
        path_x   = []
        path_y   = []

        path_d_list = d_list
        path_s_list = s_list

        # 각 (s, d_idx) → (x, y, yaw)
        for i, s in enumerate(path_s_list):
            # 2.1) 기준 경로 상의 점
            xr, yr   = self.spline_ref.calc_position(s)
            yaw_ref  = self.spline_ref.calc_yaw(s)

            # 법선벡터[−sin(yaw), cos(yaw)]에 d를 곱해 주면, 그 축으로부터 횡방향(도로 너비 방향) 오프셋
            x = xr - math.sin(math.radians(yaw_ref)) * path_d_list[i]
            y = yr + math.cos(math.radians(yaw_ref)) * path_d_list[i]

            path_x.append(x)
            path_y.append(y)

        return path_x, path_y

    # 경로를 publish
    def publish_path(self, xs, ys):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for x, y in zip(xs, ys):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
    
    def publish_initial_global_path_segment(self):
        """초기 글로벌 패스 세그먼트 발행 (Controller 대기 시간 단축)"""
        try:
            if not (hasattr(self, 'gps_x') and hasattr(self, 'gps_y')):
                rospy.logwarn("GPS 정보 없어서 초기 글로벌 패스 발행 불가")
                return
                
            # 현재 위치 기준으로 앞쪽 글로벌 패스 세그먼트 생성
            ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')
            path_x, path_y = self.make_global_path_segment(ego_s)
            
            if len(path_x) >= 3:
                self.publish_path(path_x, path_y)
                rospy.loginfo("🚀 초기 글로벌 패스 발행 완료 (Controller 대기 시간 단축)")
            else:
                rospy.logwarn("초기 글로벌 패스 세그먼트가 너무 짧음")
                
        except Exception as e:
            rospy.logerr(f"초기 글로벌 패스 발행 실패: {e}")
            import traceback
            traceback.print_exc()

    # RVIZ 시각화
    def publish_markers(self, cand_paths, local_path):
        marker_array = MarkerArray()
        mid = 0

        # 후보경로 3개 (연녹, z=0.00)
        for cx, cy in cand_paths:
            m = Marker()
            m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
            m.ns = "candidates"; m.id = mid; mid += 1
            m.type = Marker.LINE_STRIP; m.action = Marker.ADD
            m.scale.x = 0.4
            m.color = ColorRGBA(0.1, 0.9, 0.1, 0.85)
            m.pose.orientation.w = 1.0
            m.pose.position.z = 0.00
            m.points = [Point(x=x, y=y, z=0.00) for x, y in zip(cx, cy)]
            marker_array.markers.append(m)

        # 최종 경로 (파랑, z=0.04) — 스무딩 후
        m = Marker()
        m.header.frame_id = "map"; m.header.stamp = rospy.Time.now()
        m.ns = "local_final"; m.id = mid; mid += 1
        m.type = Marker.LINE_STRIP; m.action = Marker.ADD
        m.scale.x = 0.8
        m.color = ColorRGBA(0.1, 0.1, 1.0, 0.95)
        m.pose.orientation.w = 1.0
        m.pose.position.z = 0.04
        m.points = [Point(x=x, y=y, z=0.04) for x, y in zip(local_path[0], local_path[1])]
        marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    # 메인 루프
    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t0 = time.perf_counter()
            self.obstacle_node()

            if not (self.gps_flag and self.imu_flag):
                rospy.logwarn("Waiting for GPS and IMU...")
                rate.sleep()
                continue
            
            # 초기 글로벌 패스 발행 (Controller가 바로 시작할 수 있도록)
            if self.publish_initial_global_path and not self.initial_path_published:
                self.publish_initial_global_path_segment()
                self.initial_path_published = True
            
            ############################ 차량 시각화 필요시 활성화 ############################
            if Parameter.PLOT_FLAG:
                # 차량 각 꼭짓점 상대좌표를 절대좌표로 변환
                t1 = time.perf_counter()
                self.fl_corner = np.array(self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, Parameter.vehicle_front_overhang + Parameter.vehicle_wheelbase, Parameter.vehicle_width / 2))
                self.fr_corner = np.array(self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, Parameter.vehicle_front_overhang + Parameter.vehicle_wheelbase, -Parameter.vehicle_width / 2))
                self.rl_corner = np.array(self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, -Parameter.vehicle_rear_overhang, Parameter.vehicle_width / 2))
                self.rr_corner = np.array(self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, -Parameter.vehicle_rear_overhang, -Parameter.vehicle_width / 2))
                # shape (4,2)
                self.vehicle_corner = np.vstack([self.fl_corner, self.fr_corner, self.rr_corner, self.rl_corner])
                corners_ms = (time.perf_counter() - t1) * 1000.0
            ################################################################################

            t2 = time.perf_counter()
            
            # 하이브리드 모드 업데이트
            use_global = self.update_hybrid_mode()
            
            if use_global:
                # 글로벌 패스 사용
                ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')
                path_x, path_y = self.make_global_path_segment(ego_s)
                lattice_ms = 0.0  # 래티스 계산 시간 없음
                # 글로벌 모드에서는 path_s_list가 없으므로 더미 값 설정
                path_s_list = None
                path_d_list = None
                rospy.loginfo_throttle(2.0, "하이브리드 모드: 글로벌 패스 사용 중")
            else:
                # 래티스 플래닝 사용
                path_d_list, path_s_list = self.lattice_node(self.gps_x, self.gps_y)
                lattice_ms = (time.perf_counter() - t2) * 1000.0

                if path_d_list is None or any(d is None for d in path_d_list):
                    rospy.logwarn("No feasible path found yet, retrying...")
                    rate.sleep()
                    continue
                
                t3 = time.perf_counter()
                path_x, path_y = self.make_path(path_d_list, path_s_list)
                make_ms = (time.perf_counter() - t3) * 1000.0
                
                # 래티스 모드 로그에 회피 복귀 상태 포함
                recovery_info = ""
                if hasattr(self, 'is_in_recovery_mode') and self.is_in_recovery_mode:
                    remaining_time = max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (rospy.get_time() - self.last_avoidance_time))
                    recovery_info = f" (복귀모드: {remaining_time:.1f}초)"
                rospy.loginfo_throttle(2.0, f"하이브리드 모드: 래티스 패스 사용 중{recovery_info}")
            
            if use_global:
                make_ms = (time.perf_counter() - t2) * 1000.0 - lattice_ms

            # 보간하여 부드럽게 만들어준 최종경로
            t4 = time.perf_counter()
            
            # 성능 개선: 점 개수가 충분하면 스무딩 스킵
            if len(path_x) >= 10:
                smooth_x, smooth_y = path_x, path_y
            else:
                path_smoothed = CubicSpline2D(path_x, path_y)
                smooth_s = np.arange(path_smoothed.s[0], path_smoothed.s[-1], Parameter.local_step_size)
                smooth_positions = np.array([path_smoothed.calc_position(ss) for ss in smooth_s])
                smooth_x = smooth_positions[:, 0]
                smooth_y = smooth_positions[:, 1]
            
            self.publish_path(smooth_x, smooth_y)
            smoothing_ms = (time.perf_counter() - t4) * 1000.0

            ################################### RVIZ 시각화 ###################################
            if use_global:
                # 글로벌 패스 모드에서는 후보경로 없음
                cand_paths = []
            else:
                # 래티스 모드에서만 후보경로들 표시
                cand_paths = []
                if path_s_list is not None:
                    for d in self.ego_d_list:
                        cx, cy = self.make_path([d]*len(path_s_list), path_s_list)
                        cand_paths.append((cx, cy))

            # local_path (publish_path에서 쓰는 동일 데이터)
            local_path = (smooth_x, smooth_y)

            self.publish_markers(cand_paths, local_path)
            ################################################################################

            ################그래프 plot################
            if Parameter.PLOT_FLAG:
                # Figure가 닫혔는지 확인하고 재생성
                if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number):
                    print("Plot window closed, recreating...")
                    plt.ion()
                    self.fig, self.ax = plt.subplots(figsize=(10,6))
                    # 다시 모든 plot objects 생성
                    self.cand_lines = [self.ax.plot([], [], 'g--', linewidth=1)[0] for _ in range(Parameter.dd_sampling_num)]
                    self.spline_line, = self.ax.plot([], [], color='red', linewidth=2, solid_capstyle='round')
                    self.vehicle_point, = self.ax.plot([], [], 'bo')
                    self.vehicle_line, = self.ax.plot([], [], '-', linewidth=2, color='tab:blue', zorder=5)
                    self.obs_inited = False
                    self.obs_body_patches = []
                    self.obs_safe_patches = []
                    self.obs_texts = []
                
                try:
                    t5 = time.perf_counter()
                    # 1) 후보 경로들 갱신 (초록 점선) - 래티스 모드에서만
                    if use_global or path_s_list is None:
                        # 글로벌 패스 모드에서는 후보 경로 숨김
                        for i in range(len(self.cand_lines)):
                            self.cand_lines[i].set_data([], [])
                    else:
                        # 래티스 모드에서는 후보 경로 표시
                        for i, d in enumerate(self.ego_d_list):
                            cx, cy = self.make_path([d]*len(path_s_list), path_s_list)
                            self.cand_lines[i].set_data(cx, cy)

                    # 2) 스플라인 최종경로 갱신 (빨강 실선)
                    self.spline_line.set_data(smooth_x, smooth_y)

                    # 3) 차량 위치/차체 갱신
                    self.vehicle_point.set_data([self.gps_x], [self.gps_y])
                    vx = [self.fl_corner[0], self.fr_corner[0], self.rr_corner[0], self.rl_corner[0], self.fl_corner[0]]
                    vy = [self.fl_corner[1], self.fr_corner[1], self.rr_corner[1], self.rl_corner[1], self.fl_corner[1]]
                    self.vehicle_line.set_data(vx, vy)

                    # 4) 장애물 원/세이프티/텍스트 갱신
                    if self.obs_flag and self.obs:
                        # 최초 1회 또는 개수 변화 시에만 패치 생성/재생성
                        if (not self.obs_inited) or (len(self.obs_body_patches) != len(self.obs)):
                            # 기존 것 정리
                            for p in (self.obs_body_patches + self.obs_safe_patches):
                                p.remove()
                            for t in self.obs_texts:
                                t.remove()
                        self.obs_body_patches, self.obs_safe_patches, self.obs_texts = [], [], []

                        # 새로 만들기
                        for _ in self.obs:
                            # 사각형 장애물을 위한 Polygon 패치 생성 (dummy 좌표)
                            dummy_corners = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])
                            p_body = patches.Polygon(dummy_corners, fill=True, 
                                                   color='tab:orange', alpha=0.5, zorder=3)
                            p_safe = patches.Polygon(dummy_corners, fill=False, 
                                                   linestyle='--', linewidth=1.0,
                                                   edgecolor='tab:red', alpha=0.9, zorder=3)
                            self.ax.add_patch(p_body)
                            self.ax.add_patch(p_safe)
                            self.obs_body_patches.append(p_body)
                            self.obs_safe_patches.append(p_safe)
                            self.obs_texts.append(self.ax.text(0,0,'', fontsize=8,
                                                            ha='center', va='bottom', zorder=4))
                        self.obs_inited = True

                        # 위치/텍스트만 갱신
                        for i, obs in enumerate(self.obs):
                            # 장애물 사각형 좌표 생성 [x1,y1, x2,y2, x3,y3, x4,y4] -> (4,2) array
                            obs_corners = np.array([[obs[j], obs[j+1]] for j in range(0, 8, 2)])
                            
                            # 장애물 본체 사각형 업데이트
                            self.obs_body_patches[i].set_xy(obs_corners)
                            
                            # Safety buffer를 위한 확장된 사각형 생성
                            center_x = np.mean(obs_corners[:, 0])
                            center_y = np.mean(obs_corners[:, 1])
                            
                            # 각 꼭짓점을 중심에서 safety_buf만큼 확장
                            safe_corners = obs_corners.copy()
                            for j in range(4):
                                # 중심에서 꼭짓점으로의 방향벡터
                                direction = obs_corners[j] - np.array([center_x, center_y])
                                direction_norm = np.linalg.norm(direction)
                                if direction_norm > 0:
                                    # 방향은 유지하되 거리를 safety_buf만큼 증가
                                    direction_unit = direction / direction_norm
                                    safe_corners[j] = obs_corners[j] + direction_unit * Parameter.safety_buf
                            
                            self.obs_safe_patches[i].set_xy(safe_corners)
                            
                            # 충돌 검사
                            gap = self.obstacle_clearance(self.vehicle_corner, obs_corners)
                            
                            # 텍스트 위치 및 내용 업데이트
                            self.obs_texts[i].set_position((center_x, center_y))
                            self.obs_texts[i].set_text(f"gap {gap:.2f} m" if gap > 0.0 else "COLLISION")
                            self.obs_texts[i].set_color('red' if gap <= 0.0 else 'black')

                    # 5) 축/렌더 갱신
                    all_x = path_x + [self.gps_x]
                    all_y = path_y + [self.gps_y]
                    pad = 2.0
                    self.ax.set_xlim(min(all_x)-pad, max(all_x)+pad)
                    self.ax.set_ylim(min(all_y)-pad, max(all_y)+pad)

                    # 현재 모드를 제목에 표시
                    mode_str = "글로벌 패스" if use_global else "래티스 플래닝"
                    obstacles_count = len(self.obs) if self.obs else 0
                    
                    # 회피 복귀 모드 상태 표시
                    recovery_status = ""
                    if hasattr(self, 'is_in_recovery_mode') and self.is_in_recovery_mode:
                        remaining_time = max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (rospy.get_time() - self.last_avoidance_time))
                        recovery_status = f" [복귀모드: {remaining_time:.1f}초]"
                    
                    self.ax.set_title(f'하이브리드 패스 플래닝 - {mode_str}{recovery_status} (장애물: {obstacles_count}개)')

                    # 안정적인 draw 방법 사용
                    plt.figure(self.fig.number)  # figure 활성화
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    plt.pause(0.01)
                    plot_ms = (time.perf_counter() - t5) * 1000.0
                
                except Exception as e:
                    print(f"Plot error occurred: {e}")
                    plot_ms = 0.0
            ########################################

            total_ms = (time.perf_counter() - t0) * 1000.0
            rospy.loginfo_throttle(
                1.0,
                f"corners {('NONE' if (c:=locals().get('corners_ms')) is None else f'{c:.1f} ms')} | "
                f"lattice {lattice_ms:.1f} | make {make_ms:.1f} | smooth {smoothing_ms:.1f} | "
                f"plot {('NONE' if (p:=locals().get('plot_ms')) is None else f'{p:.1f} ms')} | "
                f"total {total_ms:.1f}"
)
            rate.sleep()

if __name__ == '__main__':
    path_planner_node = LatticePlanner().main()