#!/home/inji2/.local/rospython/python3
# -*- coding: utf-8 -*-
try:
    profile  # kernprof -l 로 실행하면 builtins에 주입됨
except NameError:
    def profile(func):  # IDE 노란줄 방지 + 일반 실행 시 no-op
        return func
    
import os
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
from std_msgs.msg import UInt8, Float32, Bool
from vision_msgs.msg import Detection3DArray
from collections import deque
import struct
# numba
from numba import njit
# cubic spline
from scipy.interpolate import CubicSpline
# RVIZ 용도
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


# Parameter 객체 (전역변수)
class Parameter:
    PLOT_FLAG = False               # 시각화 여부

    # 아이오닉5 모델 스펙
    vehicle_wheelbase = 3.000       # 차량 휠베이스 [m]
    vehicle_length = 4.635          # 차량 전체 길이 [m]
    vehicle_width = 1.892           # 차량 폭 [m]
    vehicle_height = 2.434          # 차량 전체 높이 [m]
    vehicle_front_overhang = 0.845  # 앞바퀴부터 앞 범퍼까지 거리 [m]
    vehicle_rear_overhang = 0.79    # 뒷바퀴부터 뒤 범퍼까지 거리 [m]
    vehicle_min_radius = 5.87       # 차량 최소회전반경 [m]
 
    road_width = 3.2           # 도로 폭 [m]
    dd_sampling_num = 9         # 도로 횡방향 샘플링 개수   (차선 포함)

    lookahead_distance = 30.0   # 경로 종방향 거리 [m] (더 멀리 미리 계획)
    ds_sampling_num = 16        # 경로 종방향 샘플링 개수 (현재 위치 포함)
    ds_interval = lookahead_distance / (ds_sampling_num - 1) # 경로 종방향 샘플링 간격 [m]
    
    INPUT_JSON = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map', 'kcity_map.json') # json파일 경로
    local_step_size = 0.5   # 로컬경로 인덱스 간격 [m]

    # obs_radius 제거 - 장애물 정확한 크기 사용
    # safety_buf = 1.22       # 장애물과의 최소 안전거리 [m]
    safety_buf_s = 1.3      # 장애물 s방향 안전거리 [m]
    safety_buf_d = 1.2      # 장애물 d방향 안전거리 [m]

    pre_window_s = 18.0      # 장애물 앞쪽으로 미리 페널티를 뿌릴 s거리 [m] (더 일찍 회피 시작)
    sigma_d = 1.7            # 가우시안 표준편차 [m]
    pre_cost_weight = 200.0  # 장애물 차선 선제적 가중치

    # post_window_s = 25.0        # 장애물 뒤쪽 가중치 거리 [m]
    # post_cost_weight = 50.0     # 장애물 이후 가중치

    lateral_cost_weight = 3.0           # 도로 전역경로 비용 가중치
    obs_cost_weight = 100               # 장애물 비용 가중치
    smooth_cost_weight = 2.5            # 횡방향 차선 변경 비용 가중치 (1차)
    second_diff_weight = 9.0            # 2차 차분(지그재그 억제) 비용

    # 하이브리드 모드 설정
    HYBRID_MODE_ENABLED = True          # 하이브리드 모드 사용 여부
    OBSTACLE_DETECTION_DISTANCE = lookahead_distance  # 장애물 감지 거리 [m]
    MODE_SWITCH_HYSTERESIS = 1.0        # 전역경로 모드 진입 히스테리시스 [m]
    
    # 회피 후 복귀 안정화 설정
    AVOIDANCE_RECOVERY_TIME = 2.3       # 회피 후 래티스 모드 유지 시간 [초]
    AVOIDANCE_RECOVERY_DISTANCE = 9.0  # 회피 후 래티스 모드 유지 거리 [m]
    
    # 허용경로 2개 차선 시작/종료 지점 인덱스
    SPECIAL_IDX_START_1 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    SPECIAL_IDX_END_1   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)

    
    # (곡선구간) 혀용경로 1개 시작/종료
    CUREVE_IDX_START_1 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_END_1   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_START_2 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    # CUREVE_IDX_END_2   = 970
    CUREVE_IDX_END_2   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_START_3 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_END_3   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_START_4 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_END_4   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)

    CUREVE_IDX_START_5 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CUREVE_IDX_END_5   = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)

    # 횡단보도 구간 
    CROSSLINE_IDX_START_0 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    #CROSSLINE_IDX_END_0 = 
    CROSSLINE_IDX_START_1 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CROSSLINE_IDX_END_1 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CROSSLINE_IDX_START_2 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CROSSLINE_IDX_END_2 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CROSSLINE_IDX_START_3 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    CROSSLINE_IDX_END_3 = 9999999  # [2026_AISW] 옛 상암맵 인덱스 → 무효화(새 kcity 4392pt에서 오발동 방지)
    

    # U턴 끼어들기 0 (도로기준 좌측)
    DANGER_IDX_START_0 = 10
    DANGER_IDX_END_0 = 24
    # 우측 끼어들기 1 (도로기준 우측)
    DANGER_IDX_START_1 = 370
    DANGER_IDX_END_1 = 385
    # U턴 끼어들기 2 (도로기준 좌측)
    # DANGER_IDX_START_2 = 
    # DANGER_IDX_END_2 = 
    # U턴 끼어들기 3 (도로기준 좌측)
    DANGER_IDX_START_3 = 925
    DANGER_IDX_END_3 = 938
    # 우측 끼어들기 4 (도로기준 우측)
    DANGER_IDX_START_4 = 1038
    DANGER_IDX_END_4 = 1050
    # U턴 끼어들기 5 (도로기준 좌측)
    # DANGER_IDX_START_5 = 1530
    # DANGER_IDX_END_5 = 1542


    car_following_distance = 18.0      # 카팔로잉 인식 최대 거리 [m]

    BIG = float('inf')        # 매우 큰 값 [ float('inf') 대신 사용 : 센서값 튀어서 모두 inf 나와 경로 임의생성 방지 ]
    INF = float('inf')
    MARGIN_BLOCK_COST = 1e8   # 마진영역 전용 큰 유한 코스트

    block_time = 1.10         # 장애물과 겹칠 경우 전역경로 막을 시간
    HOTSPOT_RADIUS = 1.3      # 장애물 전역경로 겹칠 경우, 원형존 반지름 [m]

    # 끼어들기 구간별 튜닝 파라미터 [끼어들기 구간 : 0, 1, 3, 4, 5]
    # 각 리스트의 i번째 값이 self.danger_ranges[i]에 대응 (없거나 None이면 기본값 사용)
    MERGE_ADJ_AHEAD = [car_following_distance-2.3, car_following_distance, car_following_distance-2.3, car_following_distance-2.3, car_following_distance-2.3]          # 옆차선 존재 감시 전방거리[m]
    MERGE_ADJ_D_MIN = [road_width * 0.6, road_width * 0.7, road_width * 0.6, road_width * 0.6, road_width * 0.7]                      # 옆차선으로 간주할 최소 |d|
    MERGE_ADJ_D_MAX = [road_width * 3.5, road_width * 5.5, road_width * 4.5, road_width * 4.5, road_width * 5.5]                      # 옆차선으로 간주할 최대 |d|
    MERGE_BAND     = [road_width * 3.5, road_width * 5.5, road_width * 4.5, road_width * 4.5, road_width * 5.5]                       # d-gap 급감 감시 밴드
    MERGE_DEC_THR  = [local_step_size-0.1, local_step_size, local_step_size-0.1, local_step_size-0.1, local_step_size]                         # d-gap 감소 임계값 (옵션)

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

class LatticePlanner:
    def __init__(self, init_node = True):
        if init_node:
            rospy.init_node("lattice_planner", anonymous=True)
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.marker_pub = rospy.Publisher("/lattice_markers", MarkerArray, queue_size=1)
        self.path_mode = rospy.Publisher('/planner_mode', UInt8, queue_size=1, latch=True)
        self.vrel_pub = rospy.Publisher('/nearest_vrel', Float32, queue_size=1)
        self.merge_stop_pub = rospy.Publisher('/merge_stop_flag', UInt8, queue_size=1)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/jamming_mode_active", Bool, self.jamming_mode_callback)
        # rospy.Subscriber("/iou_fusion_markers", MarkerArray, self.camera_callback)

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
        rospy.Subscriber('/tracked_objects_3d', Detection3DArray, self.obs_callback)

        self.ref_path = self.load_ref_map(Parameter.INPUT_JSON)

        cx = np.asarray(self.ref_path.cx, np.float64)
        cy = np.asarray(self.ref_path.cy, np.float64)
        self.spline_ref = CubicSpline2D_fast(cx, cy)

        # 현재위치로부터의 후보경로 s,d
        self.ego_s_list = None
        self.ego_d_list = None

        self.gps_x = None
        self.gps_y = None

        self.obs = [] # 장애물 좌표 리스트 [(x1,y1, x2,y2, x3,y3, x4,y4), ...]
        self.obs_tuples = [] # 장애물 (절대x, 절대y, 상대vx) 튜플
        self.obs_flag = False
        
        # 테스트용 임의 장애물 [x1, y1, x2, y2, x3, y3, x4, y4] : 시계방향으로 기입
        self.test_obstacles = [
        #     # 장애물 1
        #     [647.63, -689.08, 649.32, -690.44, 657.37, -681.87, 655.28, -680.47],
        #     # 장애물 2
        #     [790.09, -657.23, 787.5, -658.04, 779.47, -649.53, 781.44, -647.98],
        #     # # 장애물 3.x(차량)
        #     # [818.84, -766.02, 817.44, -764.47, 819.45, -762.32, 820.97, -763.70],
        #     # 장애물 3
        #     [782.03, - 813.40, 779.11, -812.61, 784.92, -803.05, 787.96, -805.03],
        #     # 장애물 4
        #     [704.23, -905.66, 699.98, -904.36, 717.09, -882.98, 721.06, -886.13]
        ]

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
        self.search_ds = 0.25   # 최근접 s 탐색 전용 간격(0.5~1.0 권장) : local_step_size랑 분리
        s0, s1 = self.spline_ref.s[0], self.spline_ref.s[-1]
        self.ref_s_search = np.arange(s0, s1, self.search_ds)
        rx, ry = self.spline_ref.calc_position_vec(self.ref_s_search)
        self.ref_xy_search = np.vstack([rx, ry]).T   # shape (N,2)
        self.last_s = None
        
        # 구간 인덱스가 현재 맵 범위를 벗어나면 해당 구간은 비활성화 (맵별 인덱스 하드코딩 호환용)
        n_pts = len(self.spline_ref.s)
        def _s_range(i0, i1, *extra):
            if i0 < n_pts and i1 < n_pts:
                return (self.spline_ref.s[i0], self.spline_ref.s[i1]) + tuple(extra)
            return None

        # 허용경로 2개 차선 시작/종료 지점 s
        self.special_s_ranges = [r for r in [
            _s_range(Parameter.SPECIAL_IDX_START_1, Parameter.SPECIAL_IDX_END_1)
        ] if r is not None]

        # 허용경로 1개 시작/종료 지점 s (곡선구간)
        self.curve_s_ranges = [r for r in [
            _s_range(Parameter.CUREVE_IDX_START_1, Parameter.CUREVE_IDX_END_1),
            _s_range(Parameter.CUREVE_IDX_START_2, Parameter.CUREVE_IDX_END_2),
            _s_range(Parameter.CUREVE_IDX_START_3, Parameter.CUREVE_IDX_END_3),
            _s_range(Parameter.CUREVE_IDX_START_4, Parameter.CUREVE_IDX_END_4),
            _s_range(Parameter.CUREVE_IDX_START_5, Parameter.CUREVE_IDX_END_5)
        ] if r is not None]

        # 힝단보도 구간
        self.crossline_s_ranges = [r for r in [
            _s_range(Parameter.CROSSLINE_IDX_START_1, Parameter.CROSSLINE_IDX_END_1)
        ] if r is not None]

        # 끼어들기 구간 목록 (우측 0, 좌측 1)
        self.danger_ranges = [r for r in [
            _s_range(Parameter.DANGER_IDX_START_0, Parameter.DANGER_IDX_END_0, 1),
            _s_range(Parameter.DANGER_IDX_START_1, Parameter.DANGER_IDX_END_1, 0),
            _s_range(Parameter.DANGER_IDX_START_3, Parameter.DANGER_IDX_END_3, 1),
            _s_range(Parameter.DANGER_IDX_START_4, Parameter.DANGER_IDX_END_4, 0)
        ] if r is not None]

        # 초기 글로벌 패스 발행 (Waiting for Local Path 문제 해결)
        self.publish_initial_global_path = True
        self.initial_path_published = False

        # 제밍 모드 상태
        self.jamming_mode_active = False

        # [d,s]격자의 경로 비용 & 장애물 비용
        self.lattice_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.ds_sampling_num])
        # 횡방향 차선 변경 비용
        self.smooth_cost_array = np.zeros([Parameter.dd_sampling_num, Parameter.dd_sampling_num])

        # 최종 DP 알고리즘 노드별 비용 : DP[j,i] : 레이어 i, 칸 j 까지 올 때의 최소 누적 비용
        self.dp_cost = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), float('inf'))
        # backpointer 초기화
        self.backptr = np.full((Parameter.dd_sampling_num, Parameter.ds_sampling_num), -1, dtype=int)

        self.ego_s = 0
        self.ego_vel = 0.0

        self.blocked_nodes = {}                     # key=(j,i), value=마지막 차단 시각
        self.block_duration = Parameter.block_time  # [sec] 전역경로 블럭 유지 시간
        self.circular_blocks = []  # (cx, cy, t_created) : 전역경로-기준 원형 블록

        self.test_obs_flag = True
        self.stop_this_cycle = False

        ################################ 그래프 plot
        if Parameter.PLOT_FLAG:
            # 이전 실행 때 떠있던 윈도우 닫고 시작
            plt.close('all')
            plt.ion()
            self.fig, self.ax = plt.subplots(figsize=(10,6))

            # --- 영구 artists 만들기: 루프에서는 set_* 로 "갱신만" 합니다 ---
            # 후보 경로 라인들 (dd_sampling_num 개수만큼)
            self.cand_lines = [self.ax.plot([], [], 'g--', linewidth=1)[0] for _ in range(Parameter.dd_sampling_num)]

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
            self.block_circles = []        # 원형 no-go 존 (patches.Circle) 보관
            self.block_circle_texts = []   # 남은 시간 등 텍스트(원하면)

            self.cost_scatter = self.ax.scatter([], [], s=80, c=[], cmap='viridis')
            self.block_scatter = self.ax.scatter([], [], s=60, marker='x', c='k', linewidths=2.0)
            self.fig.colorbar(self.cost_scatter, ax=self.ax, fraction=0.046, pad=0.04).set_label('Lattice cost')
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

    def comp_callback(self, msg):
        self.ego_vel = msg.velocity.x   # 상대좌표계 기준 : 전진속도 / 후진시 음수로 출력됨 [m/s]
        # print(self.ego_vel)

    def jamming_mode_callback(self, msg):
        """제밍 모드 상태 콜백"""
        self.jamming_mode_active = msg.data
        # if msg.data:
            # rospy.loginfo("[LatticePlanner] 제밍 모드 활성화 - lattice planner 일시 중단")
        # else:
            # rospy.loginfo("[LatticePlanner] 제밍 모드 비활성화 - lattice planner 재개")

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

    # def camera_callback(self, msg):


    def obs_callback(self, msg: Detection3DArray):
        if not (self.gps_flag and self.imu_flag):
            return

        coords_list = []
        tuples_list = []

        for det in msg.detections:
            # --- 좌표 계산 부분 (기존 코드 동일) ---
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            dx = max(det.bbox.size.x / 2.0, 0.5)
            dy = max(det.bbox.size.y / 2.0, 0.5)

            q = det.bbox.center.orientation
            _, _, yaw_obj = euler_from_quaternion((q.x, q.y, q.z, q.w))
            c_o, s_o = math.cos(yaw_obj), math.sin(yaw_obj)
            R_obj = np.array([[c_o, -s_o],[s_o, c_o]])

            offsets = np.array([[ dx,  dy],
                                [-dx,  dy],
                                [-dx, -dy],
                                [ dx, -dy]], dtype=float)
            corners_local = (offsets @ R_obj.T) + np.array([cx, cy])
            world_corners = [self.local_to_world(self.gps_x, self.gps_y, self.ego_yaw, p[0], p[1]) for p in corners_local]
            
            # 코너 출력 
            # print(f'corner 1: {world_corners[0]}') 
            # print(f'corner 2: {world_corners[1]}') 
            # print(f'corner 3: {world_corners[2]}') 
            # print(f'corner 4: {world_corners[3]}')

            wc = np.array(world_corners, dtype=float)
            cxm, cym = wc[:,0].mean(), wc[:,1].mean()
            # print(f"cx : {cxm}")
            # print(f"cy : {cym}")
            ang = np.arctan2(wc[:,1]-cym, wc[:,0]-cxm)
            wc = wc[np.argsort(ang)]
            coords8 = []
            for X, Y in wc:
                coords8.extend([float(X), float(Y)])

            track_id = det.results[0].id
            vx_rel = struct.unpack('f', det.source_cloud.data[0:4])[0]

            coords_list.append(coords8)
            tuples_list.append((track_id, coords8, vx_rel))

        # 테스트용 정적장애물과 병합 (테스트는 속도 None으로)
        if coords_list:
            base_coords = getattr(self, "test_obstacles", [])
            self.obs = base_coords + coords_list
            self.obs_tuples = [(None, c, None) for c in base_coords] + tuples_list
            self.obs_flag = True
        else:
            base_coords = getattr(self, "test_obstacles", [])
            self.obs = base_coords.copy()
            self.obs_tuples = [(None, c, None) for c in base_coords]
            self.obs_flag = bool(self.obs)

    
    ##########################################################

    def obstacle_node(self):
        self.obs_flag = True
        # test_obstacles 사용 (더 간단한 테스트용) - 주석처리: 라이다 데이터 사용
        # self.obs = self.test_obstacles.copy()
        # self.obs = [(coords8, (0.0, 0.0)) for coords8 in self.test_obstacles] # 장애물 (좌표xy, 속도vxvy) 튜플
        
        return self.obs

    def check_obstacles_nearby(self):
        """가장 가까운 장애물 거리와, 임계값 이내 여부를 함께 반환"""
        if not (hasattr(self, 'gps_x') and hasattr(self, 'gps_y') and self.obs):
            return False, float('inf')

        min_distance = float('inf')
        for obs in self.obs:
            # 사각형 4점의 평균 = 중심
            obs_points = np.array([[obs[i], obs[i+1]] for i in range(0, 8, 2)])
            center_x = np.mean(obs_points[:, 0])
            center_y = np.mean(obs_points[:, 1])
            dist = math.hypot(self.gps_x - center_x, self.gps_y - center_y)
            min_distance = min(min_distance, dist)

        return (min_distance < self.obstacle_detection_distance), min_distance

    def update_hybrid_mode(self):
        if not self.hybrid_mode:
            return False

        current_time = rospy.get_time()
        obstacles_nearby, min_distance = self.check_obstacles_nearby()
        current_pos = (self.gps_x, self.gps_y) if hasattr(self, 'gps_x') and hasattr(self, 'gps_y') else None

        # 회피 복귀 모드 종료 체크
        recovery_time_passed = (current_time - self.last_avoidance_time) >= Parameter.AVOIDANCE_RECOVERY_TIME
        recovery_distance_passed = True
        if self.last_avoidance_position is not None and current_pos is not None:
            distance_from_avoidance = math.hypot(
                current_pos[0] - self.last_avoidance_position[0],
                current_pos[1] - self.last_avoidance_position[1]
            )
            recovery_distance_passed = distance_from_avoidance >= Parameter.AVOIDANCE_RECOVERY_DISTANCE
        if self.is_in_recovery_mode and (recovery_time_passed and recovery_distance_passed):
            self.is_in_recovery_mode = False
            # rospy.loginfo("회피 복귀 모드 종료 (시간: {:.1f}초, 거리: {:.1f}m 경과)".format(
            #     current_time - self.last_avoidance_time,
            #     distance_from_avoidance if self.last_avoidance_position else 0
            # ))

        # 히스테리시스 적용 모드 전환
        if self.use_global_path and obstacles_nearby:
            self.use_global_path = False
            self.last_avoidance_time = current_time
            self.last_avoidance_position = current_pos
            self.is_in_recovery_mode = True
            # rospy.loginfo("모드 전환: 글로벌 패스 -> 래티스 패스 (장애물 감지, 거리: {:.1f}m)".format(min_distance))

        elif not self.use_global_path and not obstacles_nearby:
            if not self.is_in_recovery_mode:
                if min_distance > (self.obstacle_detection_distance + self.mode_switch_hysteresis):
                    self.use_global_path = True
                    # rospy.loginfo("모드 전환: 래티스 패스 -> 글로벌 패스 (장애물 없음, 거리: {:.1f}m)".format(min_distance))
            # else:
                # rospy.loginfo_throttle(2.0, "회피 복귀 모드 중 - 래티스 모드 유지 (남은 시간: {:.1f}초)".format(
                #     max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (current_time - self.last_avoidance_time))
                # ))

        return self.use_global_path

    # 로컬좌표계 (xv,yv)를 월드좌표계 (X,Y)로 변환 : 기준좌표(x0,y0)
    def local_to_world(self, x0, y0, yaw_deg, xv, yv):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        R = np.array([[c, -s],
                      [s,  c]])
        X, Y = np.array([x0, y0]) + R @ np.array([xv, yv])
        return float(X), float(Y)
    
    #  월드좌표계 (X,Y)를 로컬좌표계 (xv,yv)로 변환 : 기준좌표(x0,y0)
    def world_to_local(self, x0, y0, yaw_deg, X, Y):
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        R_T = np.array([[ c, s],
                        [-s, c]])  # R^T
        vec = np.array([X - x0, Y - y0])
        xv, yv = R_T @ vec
        return float(xv), float(yv)
    
    # 상대좌표계 속도 → 절대좌표계 속도
    def rotate_vel_local_to_world(self, vx_local, vy_local, yaw_deg):
        yaw = math.radians(yaw_deg)
        c, s = math.cos(yaw), math.sin(yaw)
        vxw = c * vx_local - s * vy_local
        vyw = s * vx_local + c * vy_local
        return vxw, vyw
    
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

    # def frenet_to_world(self, s, d):
    #     """기준경로 s 지점에서 횡방향 d만큼 떨어진 월드좌표 (x,y)와 참조 yaw(deg)를 반환"""
    #     xr, yr = self.spline_ref.calc_position(s)
    #     yaw_deg = self.spline_ref.calc_yaw(s)
    #     yaw = math.radians(yaw_deg)
    #     # 법선벡터 이용하여 오프셋d 적용
    #     x = xr - math.sin(yaw) * d
    #     y = yr + math.cos(yaw) * d
    #     return x, y, yaw_deg

    # 사각형 차량과 원형 장애물 간의 여유거리 계산
    # def obstacle_clearance(self, corners, cx, cy, radius):
    #     """
    #     corners: shape (4,2) (FL, FR, RR, RL 순서)
    #     (cx, cy): 장애물 원 중심
    #     radius:   장애물 반경
    #     return:   gap >= 0 (0이면 접촉/내부)
    #     """
    #     A = corners                     # 꼭짓점들 (FL, FR, RR, RL)
    #     B = np.roll(A, -1, axis=0)      # A의 꼭짓점들 한 칸씩 민 것 : (FR, RR, RL, FL)
    #     AB = B - A                      # 각 변의 벡터 (FL→FR, FR→RR, RR→RL, RL→FL) : 시계방향
    #     AP = np.array([cx, cy]) - A     # 각 꼭짓점에서 원 중심까지의 벡터

    #     # 장애물 차량 경계 내부 존재 여부 판단 (외적 : 부호 모두 같으면 내부)
    #     cross = AB[:,0]*AP[:,1] - AB[:,1]*AP[:,0]
    #     inside = (np.all(cross >= 0.0) or np.all(cross <= 0.0))

    #     edge_length = np.einsum('ij,ij->i', AB, AB)     # |AB|^2
    #     # AB 전체를 1로 봤을 때, A에서 몇 비율만큼 가면 P의 정사영 지점에 닿는가 (0~1)
    #     proj_ratio = np.where(edge_length > 0.0, np.clip((AP[:,0]*AB[:,0] + AP[:,1]*AB[:,1]) / edge_length, 0.0, 1.0), 0.0)
    #     # 최단직선거리 점 후보
    #     closest_points = A + proj_ratio[:, None] * AB
    #     distances = np.linalg.norm(closest_points - np.array([cx, cy]), axis=1)
    #     dmin = float(distances.min())

    #     # 여유거리 = 최단직선거리 - 장애물 반경
    #     gap = 0.0 if inside else max(0.0, dmin - radius)
    #     return gap
    
    def gps_point_to_rect_gaps(self, px: float, py: float, rect_corners: np.ndarray, s_axis: np.ndarray, d_axis: np.ndarray):
        """
        반환:
        gap_margin : 마진 적용 거리
        gap_raw    : 마진 미적용 거리
        """
        cx, cy = np.mean(rect_corners[:, 0]), np.mean(rect_corners[:, 1])
        angles = np.arctan2(rect_corners[:, 1] - cy, rect_corners[:, 0] - cx)
        rect_corners = rect_corners[np.argsort(angles)]

        A = rect_corners
        B = np.roll(A, -1, axis=0)
        AB = B - A
        AP = np.stack([px, py]) - A

        cross = AB[:, 0] * AP[:, 1] - AB[:, 1] * AP[:, 0]
        inside = (np.all(cross >= 0.0) or np.all(cross <= 0.0))

        edge_len = np.einsum('ij,ij->i', AB, AB)
        proj_ratio = np.where(edge_len > 0.0,
                            np.clip((AP[:, 0]*AB[:, 0] + AP[:, 1]*AB[:, 1]) / edge_len, 0.0, 1.0),
                            0.0)
        closest_point = A + proj_ratio[:, None] * AB
        dists = np.linalg.norm(closest_point - np.array([px, py]), axis=1)
        dmin = float(dists.min())

        gap_raw = -dmin if inside else dmin  # (마진 미적용) 서명거리

        vec = np.array([px - cx, py - cy])
        nrm = np.linalg.norm(vec)
        if nrm > 1e-9:
            u = vec / nrm
            margin_eff = abs(np.dot(u, s_axis)) * Parameter.safety_buf_s + \
                        abs(np.dot(u, d_axis)) * Parameter.safety_buf_d
        else:
            margin_eff = max(Parameter.safety_buf_s, Parameter.safety_buf_d)

        gap_margin = gap_raw - margin_eff
        return gap_margin, gap_raw
    
    def gps_point_to_rect_gap(self, px, py, rect_corners, s_axis, d_axis):
        # 플롯에서 쓰는 단일 gap 호출용 래퍼 (마진 적용 거리만 반환)
        return self.gps_point_to_rect_gaps(px, py, rect_corners, s_axis, d_axis)[0]

    def nearest_forward_obstacle_vrel(self):
        """
        (끼어들기 구간 한쪽 방향 감시 + 단일차선 곡선구간 vrel 산출)
        반환:
        (vrel, stop_trigger)
            - vrel: 같은 차로 앞차의 상대속도(m/s) 또는 None (곡선 단일차선 구간에서만 계산 유지)
            - stop_trigger: 끼어들기 STOP 필요 여부 (danger_ranges 내에서만, 지정된 한쪽만 감시)
        """
        if not hasattr(self, "_prev_d_gap"):
            self._prev_d_gap = {}

        if not self.obs_tuples:
            return None, False

        ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')
        ego_d = self.get_frenet_d(ego_s, self.gps_x, self.gps_y)

        # 1) 끼어들기 구간에서 활성화할 방향 (우측=0, 좌측=1)
        target_side = None
        zone_idx = None
        for i, (s0, s1, side) in enumerate(self.danger_ranges):
            if s0 <= ego_s <= s1:
                target_side = side      # 0: right, 1: left
                zone_idx = i
                break

        # 2) vrel(앞차) 계산은 기존 로직 유지: "단일차선 곡선 구간"에서만
        in_curve_single_lane = any(s0 <= ego_s <= s1 for (s0, s1) in self.curve_s_ranges)

        # ======= 파라미터 기본값 =======
        max_ahead  = Parameter.car_following_distance       # 전방 탐색 최대 거리 [m]
        d_same_tol = Parameter.road_width * 0.5             # 같은 차로로 간주할 d 허용 오차 [m]
        merge_band = Parameter.road_width * 6.0             # 끼어들기 합류 감시 d 범위 [m]

        # 옆차선-존재만으로 STOP 밴드(끼어들기 전용)
        adj_ahead  = Parameter.car_following_distance       # 옆차선 존재 감시 전방 거리 [m]
        adj_d_min  = Parameter.road_width * 0.8             # 옆차선으로 간주되는 최소 d차이 [m]
        adj_d_max  = Parameter.road_width * 6.0             # 옆차선으로 간주되는 최대 d차이 [m]

        dec_thresh = max(Parameter.local_step_size, 0.3)    # d갭 급감으로 합류 판단 임계값 [m]
        # ============================
        # 구간별 파라미터 오버라이드 (있으면 덮어씀)
        if zone_idx is not None:
            def pick(arr, default):
                try:
                    v = getattr(Parameter, arr)[zone_idx]
                    return default if (v is None) else v
                except Exception:
                    return default
            adj_ahead  = pick("MERGE_ADJ_AHEAD", adj_ahead)
            adj_d_min  = pick("MERGE_ADJ_D_MIN", adj_d_min)
            adj_d_max  = pick("MERGE_ADJ_D_MAX", adj_d_max)
            merge_band = pick("MERGE_BAND",     merge_band)
            dec_thresh = pick("MERGE_DEC_THR",  dec_thresh)

        # 상태 프린트(1회)
        side_str = "RIGHT(0)" if target_side == 0 else ("LEFT(1)" if target_side == 1 else "NONE")
        print(f"danger_side={side_str}, in_curve_single_lane={in_curve_single_lane}")
        
        stop_trigger = False
        min_ds_same = float('inf')
        min_vrel = None

        for _, coords8, vx_rel in self.obs_tuples:
            pts = np.array([[coords8[i], coords8[i+1]] for i in range(0, 8, 2)], dtype=float)
            cx = float(pts[:, 0].mean()); cy = float(pts[:, 1].mean())

            s_p = self.find_nearest_s(cx, cy, mode='obs', center_s=ego_s)
            d_p = self.get_frenet_d(s_p, cx, cy)
            ds  = s_p - ego_s
            if ds <= 0.0 or ds > max_ahead:
                continue

            # 좌/우 판정: +면 좌측, -면 우측(도로 기준)
            d_diff = d_p - ego_d
            g = abs(d_diff)

            # -------- (A) 끼어들기 STOP 로직: danger_ranges 안에서만, 지정된 한쪽만 본다 --------
            if target_side is not None:
                side_ok = (d_diff < 0.0) if (target_side == 0) else (d_diff > 0.0)
                if side_ok:
                    # (A-1) 옆 차선에 물체 존재: 즉시 STOP
                    if (0.0 < ds <= adj_ahead) and (adj_d_min <= g <= adj_d_max):
                        print(f"STOP by adjacent presence: ds={ds:.2f}, |d|={g:.2f} in [{adj_d_min:.1f},{adj_d_max:.1f}]")
                        stop_trigger = True

                    # (A-2) 합류(갭 감소) 징후: d-gap이 빠르게 줄어들면 STOP
                    key = (round(s_p, 1), round(d_p, 1))
                    prev_g = self._prev_d_gap.get(key, None)
                    if g <= merge_band and (prev_g is not None) and (g < prev_g - dec_thresh):
                        print(f"STOP by merge risk: prev_g={prev_g:.2f} -> g={g:.2f} (thr={dec_thresh:.2f})")
                        stop_trigger = True
                    # 한쪽만 추적하게 prev 갱신도 side_ok일 때만
                    self._prev_d_gap[key] = g
            # -------------------------------------------------------------------------

            # -------- (B) vrel(앞차) 후보: 기존과 동일, 곡선 단일차선 구간에서만 --------
            if in_curve_single_lane:
                if (g <= d_same_tol) and (vx_rel is not None) and isinstance(vx_rel, (int, float)) and math.isfinite(vx_rel):
                    if ds < min_ds_same:
                        min_ds_same = ds
                        min_vrel = float(vx_rel)
                        print(f"Car Following : {min_vrel} m/s")
            # -------------------------------------------------------------------------

        return min_vrel, stop_trigger


    def lattice_node(self, gps_x, gps_y):
        # 현재위치 ind, s, d 계산 : 종방향s는 오차 존재 / 횡방향d는 오차 거의 X
        ego_s = self.find_nearest_s(gps_x, gps_y, mode='ego')
        ego_d = self.get_frenet_d(ego_s, gps_x, gps_y)
        s_max = self.spline_ref.s[-1]
        self.lattice_cost_array.fill(0.0)

        # 현재위치로부터의 격자 [m]
        self.ego_s_list = [min(ego_s + i * Parameter.ds_interval, s_max) for i in range(Parameter.ds_sampling_num)]      # s는 현재위치 기준 (단, 끝 지점 도달시 그냥 끝 인덱스 대입 -> 마지막에 대해 반복계산할 뿐 오류나지 X)
        self.ego_d_list = np.linspace(0.0, Parameter.road_width * 2.0, Parameter.dd_sampling_num)     # d는 도로기준 고정 경로 : 왼쪽 + / 오른쪽 -

        # 횡방향 cost
        for i, d in enumerate(self.ego_d_list):
            if abs(d) <= Parameter.road_width * 2.0:
                self.lattice_cost_array[i, :] = abs(d) * Parameter.lateral_cost_weight
            else:
                self.lattice_cost_array[i, :] = Parameter.BIG

        # 장애물 비용 (차=점, 장애물=사각형+gap)
        # 벡터 한 번에 호출
        s_vec = np.asarray(self.ego_s_list, np.float64)
        xr, yr = self.spline_ref.calc_position_vec(s_vec)
        yaw = np.deg2rad(self.spline_ref.calc_yaw_vec(s_vec))
        sin_yaw, cos_yaw = np.sin(yaw), np.cos(yaw)
        # kr  = self.spline_ref.calc_curvature_vec(s_vec)
        # k_max = 1.0 / max(1e-6, Parameter.vehicle_min_radius) * 0.95  # 차량 회전반경에 따른 허용 최대 곡률

        # 사각형 장애물 모음
        rects = []
        if self.obs_flag and self.obs is not None:
            for obs in self.obs:  # obs = [x1,y1,x2,y2,x3,y3,x4,y4]
                rects.append(np.array([[obs[i], obs[i+1]] for i in range(0, 8, 2)]))

        if rects:
            for i, s in enumerate(self.ego_s_list):
               # 단일 차선인지 미리 확인 
                is_single_lane = any(s0 <= s <= s1 for (s0, s1) in self.curve_s_ranges)
                # s/d 축: 이 인덱스 i에 대해 1번만 계산 : 마진 s,d축 구분 용도
                s_axis = np.array([cos_yaw[i],  sin_yaw[i]])
                d_axis = np.array([-sin_yaw[i],  cos_yaw[i]])
                for j, d in enumerate(self.ego_d_list):
                    # 차량 : Frenet (s,d) -> 월드 좌표
                    px = xr[i] - sin_yaw[i] * d
                    py = yr[i] + cos_yaw[i] * d

                    if not is_single_lane:   # 추가 ↓ 단일차선 구간에서는 원 체크/적용 OFF
                        # 전역경로 기준 원형 no-go 존 적용
                        now_t = rospy.get_time()
                        # 만료된 원 제거
                        self.circular_blocks = [(cx, cy, t0) for (cx, cy, t0) in self.circular_blocks if now_t - t0 < self.block_duration]
                        # 반지름^2
                        r2 = Parameter.HOTSPOT_RADIUS * Parameter.HOTSPOT_RADIUS
                        # 원 내부면 즉시 차단
                        blocked_here = False
                        for (cx, cy, t0) in self.circular_blocks:
                            dx = px - cx; dy = py - cy
                            if dx*dx + dy*dy <= r2:
                                self.lattice_cost_array[j, i] = Parameter.BIG  # inf
                                blocked_here = True
                                break
                        if blocked_here:
                            continue

                    # 마진 고려/미고려 모든 사각형과의 최소 gap 계산
                    min_gap_margin = float('inf')
                    min_gap_raw = float('inf')
                    for rc in rects:
                        g_m, g_r = self.gps_point_to_rect_gaps(px, py, rc, s_axis, d_axis)
                        if g_m < min_gap_margin: min_gap_margin = g_m
                        if g_r < min_gap_raw:    min_gap_raw = g_r

                    # (1) 실물 사각형 충돌
                    if min_gap_raw <= 0.0:
                        if abs(self.ego_d_list[j]) < 0.1:
                            key = (j, round(self.ego_s_list[i], 1))
                            self.blocked_nodes[key] = rospy.get_time()
                            if not is_single_lane:                       # 추가 ↓
                                self.circular_blocks.append((xr[i], yr[i], rospy.get_time()))
                        self.lattice_cost_array[j, i] = Parameter.BIG
                        continue

                    # (2) 마진영역 침범
                    if min_gap_margin <= 0.0:
                        if abs(self.ego_d_list[j]) < 0.1:
                            key = (j, round(self.ego_s_list[i], 1))
                            self.blocked_nodes[key] = rospy.get_time()
                            if not is_single_lane:                       # 추가 ↓
                                self.circular_blocks.append((xr[i], yr[i], rospy.get_time()))
                        self.lattice_cost_array[j, i] += Parameter.MARGIN_BLOCK_COST
                        continue

                    # --- 블럭 유지 검사 ---
                    if abs(self.ego_d_list[j]) < 0.1:
                        key = (j, round(self.ego_s_list[i], 1))
                        if key in self.blocked_nodes:
                            if rospy.get_time() - self.blocked_nodes[key] < self.block_duration:
                                self.lattice_cost_array[j, i] += Parameter.MARGIN_BLOCK_COST
                                continue
                            else:
                                self.blocked_nodes.pop(key, None)

                    # 여유 작을수록 패널티 부여 (0~1) : 이방성
                    buf_scale = max(Parameter.safety_buf_s, Parameter.safety_buf_d)
                    safety_norm = min(min_gap_margin / buf_scale, 1.0)  # 0~1
                    penalty = (1.0 - safety_norm)
                    self.lattice_cost_array[j, i] += Parameter.obs_cost_weight * penalty

        # 선제 회피 가중치 : 장애물 앞쪽(s축) + 장애물과 같은 차선 쪽(d축) 비용
        if rects:
            for rec in rects:
                # 장애물 중심
                cx = float(np.mean(rec[:, 0]))
                cy = float(np.mean(rec[:, 1]))
                s_obs = self.find_nearest_s(cx, cy, mode='obs', center_s=ego_s)
                d_obs = self.get_frenet_d(s_obs, cx, cy)
                for i, s in enumerate(self.ego_s_list):
                    ds = s_obs - s
                    if 0.0 < ds <= Parameter.pre_window_s:
                        # 장애물 앞쪽 (s축) 비용
                        ramp_s = (Parameter.pre_window_s - ds) / Parameter.pre_window_s  # 0~1
                        for j, d in enumerate(self.ego_d_list):
                            dd = d_obs - d
                            # 장애물 차선 (d축) 비용 : 가우시안 (가까울수록 1에 가깝고, 멀어질수록 0에 가까움) 
                            w_d_gauss = math.exp(-(dd * dd) / (2.0 * Parameter.sigma_d * Parameter.sigma_d))

                            self.lattice_cost_array[j, i] += Parameter.pre_cost_weight * ramp_s * w_d_gauss

                    # elif -Parameter.post_window_s <= ds < 0.0:
                    #     # 뒤쪽(통과 직후) 페널티: 살짝만 남겨서 급복귀/근접 방지
                    #     ds_back = -ds  # 양수화
                    #     ramp_s = (Parameter.post_window_s - ds_back) / Parameter.post_window_s  # 0~1
                    #     # 너무 멀리까지 영향 안 주도록 살짝 더 완만(필요시 제곱)
                    #     ramp_s *= ramp_s  # 근처에서 더 세고 빠르게 감쇄
                    #     for j, d in enumerate(self.ego_d_list):
                    #         dd = d_obs - d
                    #         w_d_gauss = math.exp(-(dd * dd) / (2.0 * Parameter.sigma_d * Parameter.sigma_d))
                    #         self.lattice_cost_array[j, i] += Parameter.post_cost_weight * ramp_s * w_d_gauss


        # 차선변경 스무딩 cost : 시작j => 종료k (급격한 횡방향 변화 막기 위해)
        for j in range(Parameter.dd_sampling_num):
            for k in range(Parameter.dd_sampling_num):
                # 끝에서 끝은 불가능하도록 처리
                if abs(j - k) >= 3:
                    self.smooth_cost_array[j, k] = Parameter.BIG
                else:
                    self.smooth_cost_array[j, k] = abs(j - k) * Parameter.smooth_cost_weight
        
        # 차선 2개 제한
        for s0, s1 in self.special_s_ranges:
            rows_to_block = range(6,9)  # 해상도 5 : [4,3,2,1,0] 중 막아야 할 차선 4,3 [8,7,6,5,4,3,2,1,0]
            for i, s in enumerate(self.ego_s_list):
                if s0 <= s <= s1:
                    for r in rows_to_block:
                        self.lattice_cost_array[r, i] = Parameter.BIG

        # 차선 1개 제한
        for s0, s1 in self.curve_s_ranges:
            allowed_row = 0  # 곡선구간에서 허용할 전역경로 차선 : [4,3,2,1,0] 중 0
            for i, s in enumerate(self.ego_s_list):
                if s0 <= s <= s1:
                    for row in range(Parameter.dd_sampling_num):
                        if row != allowed_row:
                            self.lattice_cost_array[row, i] += Parameter.MARGIN_BLOCK_COST

        ##### 동적계획법 DP 시작 #####

        # ego_d와 가장 가까운 격자에서 경로 시작 (나머지는 inf)
        ego_d = self.get_frenet_d(ego_s, gps_x, gps_y)
        # ego_d = self.get_frenet_d(ego_s, gps_x, gps_y)
        start_row = int(np.argmin(np.abs(self.ego_d_list - ego_d)))
        self.dp_cost[:, 0] = float('inf')
        self.dp_cost[start_row, 0] = self.lattice_cost_array[start_row, 0]

        for col in range(1, Parameter.ds_sampling_num):
            for row in range(Parameter.dd_sampling_num):
                candidates = []
                for prev in range(Parameter.dd_sampling_num):
                    base = self.dp_cost[prev, col-1]
                    if not np.isfinite(base):
                        candidates.append(Parameter.BIG)
                        continue

                    # 1차(연속 이동) 비용: |row - prev|
                    c1 = self.smooth_cost_array[prev, row]  # = abs(prev-row) * smooth_cost_weight

                    # 2차 차분(지그재그 억제) 비용: |row - 2*prev + prevprev|
                    if col >= 2:
                        prevprev = self.backptr[prev, col-1]
                        if prevprev < 0:
                            prevprev = prev   # 초기 구간 방어
                    else:
                        prevprev = prev       # t=1에서는 2차 항 정의 어렵기 때문에 0 또는 완화

                    c2 = Parameter.second_diff_weight * abs(row - 2*prev + prevprev)

                    cost = base + c1 + c2 + self.lattice_cost_array[row, col]
                    candidates.append(cost)

                best_prev = int(np.argmin(candidates))
                # 가장 싸게 올 수 있는 이전 칸(prev) → 현재 칸(row, col) 전이 비용
                self.dp_cost[row, col] = candidates[best_prev]
                # 최소 비용을 내는 이전 칸(prev) 의 인덱스 : 경로 역추적할 때 사용
                self.backptr[row, col] = best_prev

        # 7) 최종 레이어에서 최저비용 행(row) 찾고 역추적 (전이비용 고려하기 위해 역추적 진행)
        end_col = Parameter.ds_sampling_num - 1
        best_row = int(np.argmin(self.dp_cost[:, end_col]))

        # === [추가] 최종 선택 경로 비용이 inf면 즉시 STOP ===
        if not np.isfinite(self.dp_cost[best_row, end_col]):
            self.stop_this_cycle = True

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

        ############################

        # 계산량 줄이기 위해 상태벡터도 같이 반환
        return d_final_list, self.ego_s_list, xr, yr, np.sin(yaw), np.cos(yaw)

    def make_global_path_segment(self, ego_s):
        # 미리 만든 전역경로 격자(ref_s_search/ref_xy_search)에서 인덱스만 잘라서 반환
        s_arr = self.ref_s_search
        XY    = self.ref_xy_search  # shape (N,2)

        s_end = ego_s + Parameter.lookahead_distance
        # 범위를 s_arr 인덱스로 변환
        i0 = max(0, int((ego_s - s_arr[0]) / self.search_ds))
        i1 = min(len(s_arr), int((s_end - s_arr[0]) / self.search_ds))

        # 바로 x,y 슬라이싱
        xs = XY[i0:i1, 0].tolist()
        ys = XY[i0:i1, 1].tolist()
        return xs, ys

    # (s, d) 최종경로를 (x, y)로 변환
    # def make_path(self, d_list, s_list):
    #     path_x, path_y = [], []
    #     for s, d in zip(s_list, d_list):
    #         # 1회 호출이므로 frenet_to_world 함수 그대로 사용
    #         x, y, _ = self.frenet_to_world(s, d)
    #         path_x.append(x)
    #         path_y.append(y)
    #     return path_x, path_y

    def make_path_cached(self, d_list, xr, yr, sin_yaw, cos_yaw):
        # d_list와 xr/yr 길이는 동일(= s_list 길이)
        d = np.asarray(d_list)
        x = xr - sin_yaw * d
        y = yr + cos_yaw * d
        return x.tolist(), y.tolist()

    # 경로를 publish
    def publish_path(self, xs, ys, stop=False):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"

        for idx, (x, y) in enumerate(zip(xs, ys)):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            # (수정) stop=True일 때 첫 포인트만 -100.0
            pose.pose.position.z = -100.0 if (stop and idx == 0) else 0.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def publish_initial_global_path_segment(self, ego_s):
        """초기 글로벌 패스 세그먼트 발행 (Controller 대기 시간 단축)"""
        try:
            if not (hasattr(self, 'gps_x') and hasattr(self, 'gps_y')):
                # rospy.logwarn("GPS 정보 없어서 초기 글로벌 패스 발행 불가")
                return
                
            # 현재 위치 기준으로 앞쪽 글로벌 패스 세그먼트 생성
            path_x, path_y = self.make_global_path_segment(ego_s)
            
            if len(path_x) >= 3:
                self.publish_path(path_x, path_y)
                # rospy.loginfo("초기 글로벌 패스 발행 완료 (Controller 대기 시간 단축)")
            # else:
                # rospy.logwarn("초기 글로벌 패스 세그먼트가 너무 짧음")
                
        except Exception as e:
            # rospy.logerr(f"초기 글로벌 패스 발행 실패: {e}")
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

    @profile
    # 메인 루프
    def main(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            t0 = time.perf_counter()

            # 제밍 모드 활성화시 lattice planner 중단
            if self.jamming_mode_active:
                rospy.loginfo_throttle(2.0, "[LatticePlanner] 제밍 모드 중 - lattice planner 대기")
                rate.sleep()
                continue

            if self.test_obs_flag:
                self.obstacle_node()
                self.test_obs_flag = False

            if not (self.gps_flag and self.imu_flag):
                # rospy.logwarn("Waiting for GPS and IMU...")
                rate.sleep()
                continue
            
            # vrel 계산 (곡선 단일차선 구간에서만 유효)
            vrel, stop_trigger = self.nearest_forward_obstacle_vrel()
            out = Float32()
            out.data = float('nan') if vrel is None else float(vrel)
            self.vrel_pub.publish(out)

            # 끼어드는 차량 : 즉시 정지
            if stop_trigger:
                self.merge_stop_pub.publish(UInt8(data=1))
            else:
                # STOP 해제
                self.merge_stop_pub.publish(UInt8(data=0))
            
            self.ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')

            # 초기 글로벌 패스 발행 (Controller가 바로 시작할 수 있도록)
            if self.publish_initial_global_path and not self.initial_path_published:
                self.publish_initial_global_path_segment(self.ego_s)
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
                path_x, path_y = self.make_global_path_segment(self.ego_s)
                lattice_ms = 0.0  # 래티스 계산 시간 없음
                # 글로벌 모드에서는 path_s_list가 없으므로 더미 값 설정
                path_s_list = None
                path_d_list = None
                # rospy.loginfo_throttle(2.0, "하이브리드 모드: 글로벌 패스 사용 중")
            else:
                # 래티스 플래닝 사용
                path_d_list, path_s_list, xr, yr, sin_yaw, cos_yaw = self.lattice_node(self.gps_x, self.gps_y)
                lattice_ms = (time.perf_counter() - t2) * 1000.0

                if path_d_list is None or any(d is None for d in path_d_list):
                    # rospy.logwarn("No feasible path found yet, retrying...")
                    rate.sleep()
                    continue
                
                t3 = time.perf_counter()
                path_x, path_y = self.make_path_cached(path_d_list, xr, yr, sin_yaw, cos_yaw)
                make_ms = (time.perf_counter() - t3) * 1000.0

                # if self.stop_this_cycle:
                #     # 컨트롤러가 STOP을 인식하도록 첫 점 z=-100만 가진 최소 경로 전송
                #     self.publish_path([self.gps_x], [self.gps_y], stop=True)
                #     self.stop_this_cycle = False  # 다음 프레임 대비 리셋

                #     # 모드도 래티스로 명시
                #     mode_msg = UInt8(); mode_msg.data = 1
                #     self.path_mode.publish(mode_msg)

                #     rate.sleep()
                #     continue
                                
                # 래티스 모드 로그에 회피 복귀 상태 포함
                recovery_info = ""
                if hasattr(self, 'is_in_recovery_mode') and self.is_in_recovery_mode:
                    remaining_time = max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (rospy.get_time() - self.last_avoidance_time))
                    recovery_info = f" (복귀모드: {remaining_time:.1f}초)"
                # rospy.loginfo_throttle(2.0, f"하이브리드 모드: 래티스 패스 사용 중{recovery_info}")
            
            if use_global:
                make_ms = (time.perf_counter() - t2) * 1000.0 - lattice_ms

            # 보간하여 부드럽게 만들어준 최종경로
            t4 = time.perf_counter()
            
            if use_global:
                smooth_x, smooth_y = path_x, path_y   # 전역 모드는 그대로 사용
            else:
                ################### CubicSpline 에러 방지 처리 시작 ###################
                try:
                    # 중복점 제거 및 순서 확인
                    path_array = np.array([path_x, path_y]).T
                    # 중복점 제거
                    _, unique_idx = np.unique(path_array, axis=0, return_index=True)
                    unique_idx = np.sort(unique_idx)
                    path_x_clean = [path_x[i] for i in unique_idx]
                    path_y_clean = [path_y[i] for i in unique_idx]

                    # 최소 2개 점이 필요
                    if len(path_x_clean) < 2:
                        rospy.logwarn("[LatticePlanner] Insufficient unique points for smoothing, using original path")
                        smooth_x, smooth_y = path_x, path_y
                    else:
                        path_smoothed = CubicSpline2D_fast(np.asarray(path_x_clean, np.float64), np.asarray(path_y_clean, np.float64))
                        smooth_s = np.arange(path_smoothed.s[0], path_smoothed.s[-1], Parameter.local_step_size)
                        smooth_x, smooth_y = path_smoothed.calc_position_vec(smooth_s)

                except Exception as e:
                    rospy.logwarn(f"[LatticePlanner] Smoothing failed: {e}, using original path")
                    smooth_x, smooth_y = path_x, path_y
                ################### CubicSpline 에러 방지 처리 끝 ###################

                ################### 기존 코드 (삭제 예정) ###################
                # path_smoothed = CubicSpline2D_fast(np.asarray(path_x, np.float64), np.asarray(path_y, np.float64))
                # smooth_s = np.arange(path_smoothed.s[0], path_smoothed.s[-1], Parameter.local_step_size)
                # smooth_x, smooth_y = path_smoothed.calc_position_vec(smooth_s)
                ################### 기존 코드 끝 ###################
            
            # # 현재 내 실제 위치로 경로 첫 위치 치환 (근접 격자에서 옮겨간거라 큰 문제 X)
            # if len(smooth_x) > 0 and len(smooth_y) > 0:
            #     smooth_x[0], smooth_y[0] = self.gps_x, self.gps_y
            
            self.publish_path(smooth_x, smooth_y)
            smoothing_ms = (time.perf_counter() - t4) * 1000.0
            
            # 모드 발행 (전역경로:0 , 로컬경로:1)
            mode_msg = UInt8()
            mode_msg.data = 0 if use_global else 1
            self.path_mode.publish(mode_msg)


            ################################### RVIZ 시각화 ###################################
            # if use_global:
            #     # 글로벌 패스 모드에서는 후보경로 없음
            #     cand_paths = []
            # else:
            #     # 래티스 모드에서만 후보경로 표시 (스플라인 호출 없이 캐시 사용)
            #     cand_paths = []
            #     if (path_s_list is not None):
            #         # lattice_node()에서 이미 계산된 xr, yr, sin_yaw, cos_yaw 사용
            #         for d in self.ego_d_list:
            #             cx = (xr - sin_yaw * d).tolist()
            #             cy = (yr + cos_yaw * d).tolist()
            #             cand_paths.append((cx, cy))

            # # local_path (publish_path에서 쓰는 동일 데이터)
            # local_path = (smooth_x, smooth_y)

            # self.publish_markers(cand_paths, local_path)
            ################################################################################

            ################그래프 plot################
            if Parameter.PLOT_FLAG:
                # Figure가 닫혔는지 확인하고 재생성
                if not hasattr(self, 'fig') or not plt.fignum_exists(self.fig.number):
                    # print("Plot window closed, recreating...")
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
                    self.block_circles = []
                    self.block_circle_texts = []
                    self.cost_scatter = self.ax.scatter([], [], s=80, c=[], cmap='viridis')
                    self.block_scatter = self.ax.scatter([], [], s=60, marker='x', c='k', linewidths=2.0)

                try:
                    t5 = time.perf_counter()

                    # 1) 후보 경로들 갱신 (초록 점선)
                    #    - 래티스 모드일 때만, 그리고 이번 사이클에서 얻은 xr/yr/sin/cos를 이용해서 직접 계산
                    if use_global or (path_s_list is None):
                        # 글로벌 패스 모드 or 래티스 데이터 없음 → 후보 경로 숨김
                        for i in range(len(self.cand_lines)):
                            self.cand_lines[i].set_data([], [])
                    else:
                        # 래티스 모드: 같은 s에 대해 d를 고정하여 월드좌표 계산 (캐시 사용, 스플라인 호출 없음)
                        # 주의: xr, yr, sin_yaw, cos_yaw는 위에서 lattice_node 리턴값으로 이미 계산됨
                        for i, d in enumerate(self.ego_d_list):
                            cx = (xr - sin_yaw * d).tolist()
                            cy = (yr + cos_yaw * d).tolist()
                            self.cand_lines[i].set_data(cx, cy)

                    # 2) 최종경로 갱신 (빨강 실선)
                    self.spline_line.set_data(smooth_x, smooth_y)

                    # 3) 차량 위치/차체 갱신
                    self.vehicle_point.set_data([self.gps_x], [self.gps_y])
                    vx = [self.fl_corner[0], self.fr_corner[0], self.rr_corner[0], self.rl_corner[0], self.fl_corner[0]]
                    vy = [self.fl_corner[1], self.fr_corner[1], self.rr_corner[1], self.rl_corner[1], self.fl_corner[1]]
                    self.vehicle_line.set_data(vx, vy)

                    yaw_deg_plot = self.spline_ref.calc_yaw(self.ego_s)
                    yaw_plot = math.radians(yaw_deg_plot)
                    s_axis = np.array([math.cos(yaw_plot),  math.sin(yaw_plot)])
                    d_axis = np.array([-math.sin(yaw_plot), math.cos(yaw_plot)])

                    # 4) 장애물(사각형) + 세이프티 버퍼 + 텍스트 갱신
                    if self.obs_flag and self.obs:
                        # 최초 1회 또는 개수 변화 시에만 패치들 재생성
                        if (not self.obs_inited) or (len(self.obs_body_patches) != len(self.obs)):
                            # 기존 패치/텍스트 제거
                            for p in (self.obs_body_patches + self.obs_safe_patches):
                                try:
                                    p.remove()
                                except Exception:
                                    pass
                            for t in self.obs_texts:
                                try:
                                    t.remove()
                                except Exception:
                                    pass
                            self.obs_body_patches, self.obs_safe_patches, self.obs_texts = [], [], []

                            # 새 패치 생성
                            for _ in self.obs:
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
                                self.obs_texts.append(self.ax.text(0, 0, '', fontsize=8,
                                                                   ha='center', va='bottom', zorder=4))
                            self.obs_inited = True

                        # 위치/텍스트 갱신
                        for i, obs in enumerate(self.obs):
                            obs_corners = np.array([[obs[j], obs[j+1]] for j in range(0, 8, 2)])
                            self.obs_body_patches[i].set_xy(obs_corners)

                            # 세이프티 버퍼(모서리 방향으로 safety_buf 확장) 단순 근사
                            center_x = np.mean(obs_corners[:, 0])
                            center_y = np.mean(obs_corners[:, 1])
                            buf_vis = max(Parameter.safety_buf_s, Parameter.safety_buf_d)  # 표시용은 최대치로 등거리 확장
                            safe_corners = obs_corners.copy()
                            for j in range(4):
                                direction = obs_corners[j] - np.array([center_x, center_y])
                                nrm = np.linalg.norm(direction)
                                if nrm > 1e-9:
                                    safe_corners[j] = obs_corners[j] + (direction / nrm) * buf_vis
                            self.obs_safe_patches[i].set_xy(safe_corners)
                            # 텍스트 (gap / 충돌) 갱신
                            gap = self.gps_point_to_rect_gap(self.gps_x, self.gps_y, obs_corners, s_axis, d_axis)
                            self.obs_texts[i].set_position((center_x, center_y))
                            self.obs_texts[i].set_text(f"gap {gap:.2f} m" if gap > 0.0 else "COLLISION")
                            self.obs_texts[i].set_color('red' if gap <= 0.0 else 'black')
                    else:
                        # 장애물 없으면 패치들 제거
                        for p in (self.obs_body_patches + self.obs_safe_patches):
                            try: p.remove()
                            except: pass
                        for t in self.obs_texts:
                            try: t.remove()
                            except: pass
                        self.obs_body_patches, self.obs_safe_patches, self.obs_texts = [], [], []
                        self.obs_inited = False

                    # 4.5) 원형 no-go 존(전역경로 차단 원) 갱신
                    try:
                        # 현재 유효한 원 목록
                        hotspots = getattr(self, "circular_blocks", [])
                        # 패치 개수 맞추기: 부족하면 생성, 많으면 제거
                        # 생성
                        while len(self.block_circles) < len(hotspots):
                            c = patches.Circle((0.0, 0.0),
                                            radius=Parameter.HOTSPOT_RADIUS,
                                            fill=False,
                                            linewidth=2.0,
                                            linestyle='-',
                                            edgecolor='tab:red',
                                            alpha=0.8,
                                            zorder=3)
                            self.ax.add_patch(c)
                            self.block_circles.append(c)
                            # 텍스트(선택) — 남은 시간 표기
                            t = self.ax.text(0, 0, '',
                                            fontsize=7, color='tab:red',
                                            ha='center', va='top', zorder=4)
                            self.block_circle_texts.append(t)
                        # 제거
                        while len(self.block_circles) > len(hotspots):
                            try:
                                self.block_circles[-1].remove()
                            except Exception:
                                pass
                            self.block_circles.pop()
                            try:
                                self.block_circle_texts[-1].remove()
                            except Exception:
                                pass
                            self.block_circle_texts.pop()

                        # 위치/텍스트 업데이트
                        now_t = rospy.get_time()
                        for i, (cx, cy, t0) in enumerate(hotspots):
                            self.block_circles[i].center = (cx, cy)
                            self.block_circles[i].set_radius(Parameter.HOTSPOT_RADIUS)

                            # 남은 시간 표기(선택): 0.1s 해상도
                            remain = max(0.0, self.block_duration - (now_t - t0))
                            self.block_circle_texts[i].set_position((cx, cy - Parameter.HOTSPOT_RADIUS - 0.2))
                            self.block_circle_texts[i].set_text(f"{remain:0.1f}s")
                    except Exception:
                        # 원 그리기 실패해도 전체 파이프라인 안 멈추게
                        pass

                    # 5) 래티스 비용/차단(inf) 점 찍기 — 캐시만 사용 (스플라인 호출 금지)
                    if (not use_global) and (self.ego_s_list is not None) and (self.ego_d_list is not None):
                        # xr, yr, sin_yaw, cos_yaw는 위 lattice_node에서 이미 계산됨
                        pts = []
                        vals = []
                        blk = []  # inf(충돌/불가) 셀 좌표

                        for i_s, _ in enumerate(self.ego_s_list):
                            # 각 d에 대해 점 생성
                            for j_d, d in enumerate(self.ego_d_list):
                                c = self.lattice_cost_array[j_d, i_s]
                                px = xr[i_s] - sin_yaw[i_s] * d
                                py = yr[i_s] + cos_yaw[i_s] * d

                                if not np.isfinite(c):
                                    blk.append([px, py])
                                else:
                                    pts.append([px, py]); vals.append(c)

                        # 유효 코스트 점 갱신
                        if pts:
                            pts = np.array(pts); vals = np.array(vals)
                            self.cost_scatter.set_offsets(pts)
                            self.cost_scatter.set_array(vals)
                            vmax = float(np.percentile(vals, 95))
                            vmin = float(np.min(vals))
                            if vmax <= vmin:
                                vmax = vmin + 1e-6
                            self.cost_scatter.set_clim(vmin=vmin, vmax=vmax)
                        else:
                            self.cost_scatter.set_offsets(np.empty((0, 2)))
                            self.cost_scatter.set_array(np.array([]))

                        # inf(충돌/불가) 셀 X표 갱신
                        if blk:
                            self.block_scatter.set_offsets(np.array(blk))
                        else:
                            self.block_scatter.set_offsets(np.empty((0, 2)))
                    else:
                        # 글로벌 모드일 땐 비용/블럭 점 비움
                        self.cost_scatter.set_offsets(np.empty((0, 2)))
                        self.cost_scatter.set_array(np.array([]))
                        self.block_scatter.set_offsets(np.empty((0, 2)))

                    # 6) 축/렌더 갱신
                    all_x = path_x + [self.gps_x]
                    all_y = path_y + [self.gps_y]
                    pad = 2.0
                    self.ax.set_xlim(min(all_x)-pad, max(all_x)+pad)
                    self.ax.set_ylim(min(all_y)-pad, max(all_y)+pad)

                    # 제목(모드/복귀 상태)
                    mode_str = "글로벌 패스" if use_global else "래티스 플래닝"
                    obstacles_count = len(self.obs) if self.obs else 0
                    recovery_status = ""
                    if hasattr(self, 'is_in_recovery_mode') and self.is_in_recovery_mode:
                        remaining_time = max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (rospy.get_time() - self.last_avoidance_time))
                        recovery_status = f" [복귀모드: {remaining_time:.1f}초]"
                    self.ax.set_title(f'하이브리드 패스 플래닝 - {mode_str}{recovery_status} (장애물: {obstacles_count}개)')

                    # 안정적인 draw
                    plt.figure(self.fig.number)
                    self.fig.canvas.draw()
                    self.fig.canvas.flush_events()
                    plt.pause(0.01)
                    plot_ms = (time.perf_counter() - t5) * 1000.0

                except Exception as e:
                    # print(f"Plot error occurred: {e}")
                    plot_ms = 0.0
            ########################################

            total_ms = (time.perf_counter() - t0) * 1000.0
            inst_hz = 1000.0 / total_ms if total_ms > 1e-6 else float('inf')  # 현재 처리 속도 기준 최대가능 Hz
            # rospy.loginfo_throttle(
            #     1.0,
            #     f"corners {('NONE' if (c:=locals().get('corners_ms')) is None else f'{c:.1f} ms')} | "
            #     f"lattice {lattice_ms:.1f} | make {make_ms:.1f} | smooth {smoothing_ms:.1f} | "
            #     f"plot {('NONE' if (p:=locals().get('plot_ms')) is None else f'{p:.1f} ms')} | "
            #     f"total {total_ms:.1f} ms | Cap hz {inst_hz:.1f} Hz"
            # )
            rate.sleep()

if __name__ == '__main__':
    path_planner_node = LatticePlanner().main()