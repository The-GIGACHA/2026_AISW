#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import json
import tf
import bisect
from tf.transformations import euler_from_quaternion
from pyproj import Proj
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage, EventInfo, CollisionData
from morai_msgs.srv import MoraiEventCmdSrv
from nav_msgs.msg import Path


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
    max_velocity = 9.5              # 최대 속도 [m/s] : 단 throttle제어이므로 어느정도 여유둬야 함

    default_target_index = 10       # 0.2m 간격이므로 10개는 2m 전방

    # 추종할 맵 json파일 경로
    INPUT_JSONS = [
        '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_1.json',
        '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_2.json',
        '/home/yhj/catkin_ws/src/alpha_one/scripts/wonju_map_final_3.json',
    ]
    # 초기 맵 설정
    INITIAL_MAP_INDEX = 0

    road_width = 3.5                # 도로 너비 폭

    LOCAL_PATH_SPEED = 0.5

class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        # self.cv = [v / 3.6 for v in cv]
        self.cv = cv
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

#####################################################################################################
# Spline 경로 생성을 위한 클래스 2개
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
#####################################################################################################

class Kanayama_Control: 
    def __init__(self, path):
        self.wb = Parameter.vehicle_wheelbase
        self.Kx = 0.685
        self.Ky = 0.400
        self.Ktheta = math.sqrt(4 * self.Ky * 0.225)

        self.path = path
        self.ego_gear = 4
        self.ego_pose = np.array([0.0, 0.0, 0.0])
        self.ego_ind = 0
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.target_ind = 0
        self.target_velocity = 0.0
        self.target_curvature = 0.0
        self.global_error = np.array([0.0, 0.0, 0.0])
        self.local_error = np.array([0.0, 0.0, 0.0])

    def global_to_local(self, theta, global_error):
        rotation_matrix = np.array([[math.cos(theta), math.sin(theta), 0],
                                     [-math.sin(theta), math.cos(theta), 0],
                                     [0, 0, 1]])
        local_error = rotation_matrix @ global_error
        return local_error
    
    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    def angle_diff(self, target, current):
    # 두 각도의 차이를 [-pi, pi]로 정규화
        return math.atan2(math.sin(target - current), math.cos(target - current))
    
    def update_path(self, path):
        self.path = path
    
    def run(self, ego_x, ego_y, ego_yaw, ego_gear, ego_ind):
        print("Kanayama Control Running")
        if self.path is not None:
            self.ego_gear = ego_gear
            self.ego_ind = ego_ind
            self.target_ind = self.ego_ind + Parameter.default_target_index
            if self.target_ind >= self.path.length:
                        self.target_ind = self.path.length - 1

            self.ego_pose = np.array([ego_x, ego_y, math.radians(ego_yaw)])
            self.target_pose = np.array([self.path.cx[self.target_ind], self.path.cy[self.target_ind], math.radians(self.path.cyaw[self.target_ind])])
            self.target_velocity = self.path.cv[self.target_ind]
            self.target_curvature = self.path.ck[self.target_ind]
            # 후진의 경우
            if self.ego_gear == 2:
                self.target_velocity *= -1
                self.target_curvature *= -1

            # 속도에 따른 별도의 게인값
            if abs(self.target_velocity) > 9.0:     # 32.4kph
                self.Kx = 0.700
                self.Ky = 0.150
                self.Ktheta = math.sqrt(4 * self.Ky * 0.100)
            elif abs(self.target_velocity) > 5.0:     # 18kph
                self.Kx = 0.685
                self.Ky = 0.150
                self.Ktheta = math.sqrt(4 * self.Ky * 0.150)
            elif abs(self.target_velocity) > 4.0:   # 14.4kph
                self.Kx = 0.685
                self.Ky = 0.250
                self.Ktheta = math.sqrt(4 * self.Ky * 0.200)
            elif abs(self.target_velocity) > 3.0:   # 10.8kph
                self.Kx = 0.685
                self.Ky = 0.400
                self.Ktheta = math.sqrt(4 * self.Ky * 0.250)
            elif abs(self.target_velocity) > 1.39:   # 5kph
                self.Kx = 0.685
                self.Ky = 0.600
                self.Ktheta = math.sqrt(4 * self.Ky * 0.300)
            else:
                self.Kx = 0.685
                self.Ky = 0.600
                self.Ktheta = math.sqrt(4 * self.Ky * 0.300)

            self.global_error = np.array([self.target_pose[0]-self.ego_pose[0], self.target_pose[1]-self.ego_pose[1], self.angle_diff(self.target_pose[2],self.ego_pose[2])])
            self.local_error = self.global_to_local(self.ego_pose[2], self.global_error)

            velocity = self.target_velocity * math.cos(self.local_error[2]) + self.Kx * self.local_error[0]
            angular_velocity = self.target_velocity * self.target_curvature + self.target_velocity * (self.Ky * self.local_error[1] + self.Ktheta * math.sin(self.local_error[2]))
            steering = self.normalize_180(math.degrees(math.atan2(self.wb * angular_velocity, max(abs(self.target_velocity), 0.1))))
            
            return np.clip(abs(velocity), 0.2, Parameter.max_velocity), np.clip(steering, -Parameter.max_wheel_angle, Parameter.max_wheel_angle)

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

        self.path = path

    def normalize_rad_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    def update_path(self, path):
        self.path = path

    def run(self, ego_x, ego_y, ego_yaw, ego_gear, ego_ind, ego_vel):
        print("Pure-Pursuit Control Running")
        if self.path is not None:
            self.ego_x = ego_x
            self.ego_y = ego_y
            self.ego_yaw = ego_yaw
            self.ego_gear = ego_gear
            self.ego_ind = ego_ind
            self.ego_vel = ego_vel
            if abs(self.ego_vel) > 4.0:
                self.lfd = 10   # 2.0m
            else:
                self.lfd = 5    # 1.0m
            self.target_vel = self.path.cv[self.ego_ind]
            self.target_ind = self.ego_ind + self.lfd
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
            print(self.target_vel)
            print(steering_deg)
            return np.clip(abs(self.target_vel), 0.2, Parameter.max_velocity), np.clip(steering_deg, -40.0, 40.0)
    
# Trottle제어기
class AccelCmd_Converter:
    def __init__(self, rate_hz):
        self.p_gain = 0.35
        self.i_gain = 0.06
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

class Perpendicular_ParkingPlanner:
    def __init__(self):
        # 입력받아야 하는 변수
        self.corner_far = np.array([0.0, 0.0])                      # 차량 진입로 기준 먼 주차공간 모서리 (x,y)
        self.corner_near = np.array([0.0, 0.0])                     # 차량 진입로 기준 가까운 주차공간 모서리 (x,y)
        self.parking_depth = 0.0                                    # 주차공간의 깊이 (세로 길이) [m]
        self.minturn_circle_radius = Parameter.vehicle_min_radius   # 최소회전반경 원 O2의 반지름 [m] : 완만한 경로 필요하면 늘리기
        self.entry_offset = Parameter.road_width / 2                # 차량 진입선과 주차공간 경계선 사이의 수직거리 d : 필요시 경로에 맞게 수정

        self.corner_half = np.array([0.0, 0.0])                     # 주차공간 진입 모서리 중간지점
        self.alignment_circle_center = np.array([0.0, 0.0])         # 주차공간 진입 정렬용 큰 원 O1의 중심점
        self.alignment_circle_radius = 0.0                          # 정렬용 원 O1의 반지름 [m]
        self.minturn_circle_center = np.array([0.0, 0.0])           # 최소회전반경 원 O2의 중심점
        self.direction_vector = np.array([0.0, 0.0])                # 주차공간 모서리의 2개를 잇는 직선의 방향벡터
        self.unit_direction = np.array([0.0, 0.0])                  # 단위 방향벡터
        self.unit_normal = np.array([0.0, 0.0])                     # 단위 법선벡터

        self.turning_point_1 = np.array([0.0, 0.0])                 # 직선 -> 최소회전반경 원 O2 진입점
        self.turning_point_2 = np.array([0.0, 0.0])                 # 최소회전반경 원 O2 -> 정렬용 원 O1 진입점
    
    def update(self, corner_far, corner_near, parking_depth):
        self.corner_far = corner_far
        self.corner_near = corner_near
        self.parking_depth = parking_depth
        self.corner_half = (self.corner_near + self.corner_far) / 2
        self.direction_vector = self.corner_near - self.corner_far
        self.unit_direction = self.direction_vector / np.linalg.norm(self.direction_vector)
        self.unit_normal = np.array([-self.unit_direction[1], self.unit_direction[0]])
        self.minturn_circle_center = self.corner_half - (self.minturn_circle_radius + self.entry_offset) * self.unit_normal

        self.alignment_circle_radius = self.entry_offset + (self.entry_offset ** 2) / (2 * self.minturn_circle_radius)
        self.alignment_circle_center = self.corner_half - self.alignment_circle_radius * self.unit_direction

        self.turning_point_1 = self.minturn_circle_center + self.minturn_circle_radius * self.unit_normal
        theta = math.atan2(self.alignment_circle_radius, self.minturn_circle_radius + self.entry_offset)
        self.turning_point_2 = self.minturn_circle_center + self.minturn_circle_radius * (self.unit_normal * np.cos(theta) - self.unit_direction * np.sin(theta))

        if not np.all(np.isfinite(self.turning_point_1)):
            rospy.logerr("ParkingPlanner: turning_point_1 is invalid...")
            return False
        if not np.all(np.isfinite(self.turning_point_2)):
            rospy.logerr("ParkingPlanner: turning_point_2 is invalid...")
            return False
        return True

    def shortest_arc_angles(self, a0, a1):
        # 두 각도 차를 (-pi,pi]로 정규화하여 최단경로 호 결정 (방향)
        delta = (a1 - a0 + math.pi) % (2*math.pi) - math.pi
        return a0, a0 + delta

    def sample_circle_arc(self, center, radius, p_start, p_end, ds=0.2, forward=True):
        # center: 원 중심, radius: 반지름
        # p_start, p_end: 호의 시작/끝점 (둘 다 원 위에 있어야 함)
        # ds: 호 길이 샘플 간격
        a0 = math.atan2(p_start[1]-center[1], p_start[0]-center[0])
        a1 = math.atan2(p_end[1]-center[1], p_end[0]-center[0])
        if forward:
            a_s, a_e = self.shortest_arc_angles(a0, a1)
        else:
            # reverse일 땐 호 진행을 반대로 뒤집기 (yaw 반전)
            a_s, a_e = self.shortest_arc_angles(a1, a0)
        span = a_e - a_s        # 호의 각도 범위
        L = abs(span * radius)  # 호 길이
        N = max(1, int(L/ds))   # 샘플링 포인트 개수
        thetas = np.linspace(a_s, a_e, N+1)
        pts = []
        for th in thetas:
            x = center[0] + radius * math.cos(th)
            y = center[1] + radius * math.sin(th)
            pts.append((x, y))
        return pts

    def generate_trajectory_spline(self, ds=0.2, velocity=1.0, approach_length=2.0):
        P1 = self.turning_point_1               # 직선 진입로 → 작은 원
        P2 = self.turning_point_2               # 작은 원 → 큰 원
        C_small = self.minturn_circle_center
        R_small = self.minturn_circle_radius
        C_big   = self.alignment_circle_center
        R_big   = self.alignment_circle_radius

        raw = []

        # (a) 접근 직선: P1 approach_length만큼 앞쪽에서 경로 시작
        start_pt = P1 + self.unit_direction * approach_length
        N = max(1, int(approach_length / ds))
        for i in range(N+1):
            dist = ds * i
            x = start_pt[0] - self.unit_direction[0] * dist
            y = start_pt[1] - self.unit_direction[1] * dist
            raw.append((x, y))

        # (b) 작은 원호 전진
        small_start = P1
        small_end   = P2
        raw += self.sample_circle_arc(C_small, R_small, small_start, small_end, ds, forward=True)

        idx_forward_end = len(raw) - 1      # 전진 구간이 끝난 지점(P2) 인덱스 저장

        # (c) 큰 원호 후진
        big_start = P2
        big_end   = self.corner_half
        raw += self.sample_circle_arc(C_big, R_big, big_start, big_end, ds, forward=False)

        # (d) 최종 후진 직선: corner_half → corner_half + (unit_normal)*parking_depth
        n_steps = max(1, int(self.parking_depth / ds))  # parking_depth 만큼 후진 직선 생성

        for k in range(1, n_steps + 1):
            # corner_half에서 k*ds 만큼 unit_normal 방향으로 이동
            x = self.corner_half[0] + self.unit_normal[0] * ds * k
            y = self.corner_half[1] + self.unit_normal[1] * ds * k
            raw.append((x, y))

        # (e) spline 보간 & 재샘플링
        xs = [p[0] for p in raw]
        ys = [p[1] for p in raw]
        try:
            spline2d = CubicSpline2D(xs, ys)
        except Exception as e:
            rospy.logerr("ParkingPlanner: CubicSpline failed... : %s", e)
            return None
        total_s   = spline2d.s[-1]
        s_samples = np.arange(0, total_s, ds)

        # 전진이 끝난 지점의 s 계산
        s_split = spline2d.s[idx_forward_end]

        # PATH 생성에 필요한 값들
        cx, cy, cyaw, ck, cv, cmission, cgear = [], [], [], [], [], [], []
        for s in s_samples:
            x, y = spline2d.calc_position(s)
            yaw  = spline2d.calc_yaw(s)
            gear = 4 if s < s_split else 2

            cx.append(x)
            cy.append(y)
            cyaw.append(yaw)
            ck.append(spline2d.calc_curvature(s))
            cv.append(velocity)
            cmission.append(MissionState.mission_3)        # 전부 ‘PERPENDICULAR_PARKING’
            cgear.append(gear)

        print(f"Parking PATH created: {len(cx)} points")

        return PATH(cx, cy, cyaw, ck, cv, cmission, cgear)

class Morai_Control_Node:
    def __init__(self):
        rospy.init_node('morai_control_node', anonymous = True)

        # rospy.Subscriber("/Ego_topic",EgoVehicleStatus, self.odom_callback)
        rospy.Subscriber("/Competition_topic",EgoVehicleStatus, self.comp_callback)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.coll_callback)
        rospy.Subscriber("/local_path", Path, self.local_path_callback)

        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        # self.ctrl_cmd_msg.longlCmdType=2
        self.ctrl_cmd_msg.longlCmdType=1

        # ServiceProxy를 한 번만 생성
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self._event_cmd = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.rate_hz = 30   # 메인루프 주기 설정

        # 초기 맵 설정
        self.initial_map_index = Parameter.INITIAL_MAP_INDEX
        # 모든 맵을 미리 불러와 저장
        self.all_paths = [ self.load_ref_map(p) for p in Parameter.INPUT_JSONS ]
        # 맵 인덱스 clamp & 현재 ref_path 설정
        self.map_index = max(0, min(self.initial_map_index, len(self.all_paths)-1))
        self.ref_path  = self.all_paths[self.map_index]
    
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_gear = 4
        self.ego_vel = 0.0
        self.ego_index = 0

        self.velocity = 0.0
        self.steering = 0.0

        self.gps_fix = 0
        self.odom_flag = False
        self.gps_flag = False
        self.imu_flag = False
        self.coll_flag = False

        self.local_path_flag = False

        # 후진 관련 변수
        self.adjusted_yaw = 0.0
        self.prev_gear = self.ref_path.cgear[0]

        self.ego_mission = MissionState.mission_0

        # 주차미션 맵
        self.parking_path = None

        # 사용할 클래스 객체 초기화
        self.cmd_converter = AccelCmd_Converter(self.rate_hz)
        self.kanayama_controller = Kanayama_Control(self.ref_path)
        # self.purepursuit_controller = PurePursuit_Control(self.ref_path)
        # self.perpedicular_parking = Perpendicular_ParkingPlanner()

    # def odom_callback(self, msg):
    #     # br = tf.TransformBroadcaster()
    #     # br.sendTransform((msg.position.x, msg.position.y, msg.position.z),
    #     #                 tf.transformations.quaternion_from_euler(0, 0, msg.heading/180*math.pi),
    #     #                 rospy.Time.now(),
    #     #                 "gps",
    #     #                 "map")

    #     # self.ego_x = msg.position.x
    #     # self.ego_y = msg.position.y
    #     # self.ego_yaw = msg.heading          # deg 단위
    #     self.ego_vel = msg.velocity.x       # 상대좌표계 기준 : 전진속도 / 후진시 음수로 출력됨 [m/s]

    #     # print(self.ego_vel)

    #     self.odom_flag = True

    # 대회용 토픽
    def comp_callback(self, msg):
        self.ego_vel = msg.velocity.x       # 상대좌표계 기준 : 전진속도 / 후진시 음수로 출력됨 [m/s]
        # print(self.ego_vel)
        self.odom_flag = True

    def gps_callback(self, msg):
        """
        차량 상대좌표계 중심은 기본적으로 후륜중심으로 되어있으므로 (x:0.0, y:0.0, z:1.2) / 30Hz
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
        차량 후륜중심에 장착하는게 이론과 정확 (x:0.0, y:0.0, z:1.3) / 30Hz
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

    def local_path_callback(self, msg):
        if len(msg.poses) < 3:
            print("local_path too short (<3). Holding last path.")
            self.local_path_flag = False; 
            return
        xs = [p.pose.position.x for p in msg.poses]
        ys = [p.pose.position.y for p in msg.poses]
        sp = CubicSpline2D(xs, ys)
        ss = np.array(sp.s[:-1])
        cx, cy, cyaw, ck, cv, cm, cg = [], [], [], [], [], [], []
        for s in ss:
            x,y = sp.calc_position(s); yaw = sp.calc_yaw(s); k = sp.calc_curvature(s)
            cx.append(x); cy.append(y); cyaw.append(yaw); ck.append(k)
            cv.append(Parameter.LOCAL_PATH_SPEED); cm.append("NORMAL_DRIVING"); cg.append(4)
        self.ref_path = PATH(cx, cy, cyaw, ck, cv, cm, cg)
        self.kanayama_controller.update_path(self.ref_path)
        self.local_path_flag = True

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

    def main(self):
        rate = rospy.Rate(self.rate_hz)
        
        # 주차경로 생성
        # parking_geo = self.perpedicular_parking.update(np.array([0.0, 1.0]), np.array([5.0, 5.0]), 7.0)
        # if not parking_geo:
        #     print("Failed to compute parking geometry...")
        #     return
        # self.parking_path = self.perpedicular_parking.generate_trajectory_spline()
        # if self.parking_path is None:
        #     print("No Parking Path Generated...")
        #     return
        
        # 기어 D단 및 자율주행 모드로 변경
        self.change_to_drive()
        self.change_ctrl_mode(3)

        while not rospy.is_shutdown():
            if not (self.odom_flag and self.gps_flag and self.imu_flag):
                rate.sleep()
                continue

            if not self.local_path_flag:
                print("Waiting for Local Path...")
                rate.sleep()
                continue

            self.ego_index = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)

            if not self.local_path_flag:
                # 맵 끝에 다다르면 다음 맵으로 전환 : 마지막 인덱스 2개 이내
                if self.ego_index >= self.ref_path.length - 2 and self.map_index < len(self.all_paths) - 1:
                    self.map_index += 1
                    self.ref_path = self.all_paths[self.map_index]
                    print(f">>> Switch to map #{self.map_index+1}")
                    # 컨트롤러 맵 업데이트
                    self.kanayama_controller.update_path(self.ref_path)
                    # self.purepursuit_controller.update_path(self.ref_path)
                    # 인덱스 및 기어 다시 계산
                    self.ego_index = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)
                    self.prev_gear = self.ref_path.cgear[self.ego_index]

                # 기존/변경된 맵 기준 미션과 기어 업데이트 
                self.ego_mission = self.ref_path.cmission[self.ego_index]
                self.ego_gear = self.ref_path.cgear[self.ego_index]

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
            else:
                self.ego_mission = MissionState.mission_0
                self.ego_gear    = 4
            

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

            
            # 미션별 적용
            if self.ego_mission == MissionState.mission_0:
                self.velocity, self.steering = self.kanayama_controller.run(self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_gear, self.ego_index)
                # self.velocity, self.steering = self.purepursuit_controller.run(self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_gear, self.ego_index, self.ego_vel)
            elif self.ego_mission == MissionState.mission_1:
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = 1.0
                self.ctrl_cmd_msg.steering = 0.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                continue
            elif self.ego_mission == MissionState.mission_2:
                pass
            elif self.ego_mission == MissionState.mission_3:
                self.kanayama_controller.update_path(self.parking_path)
                # 주차경로 기준 인덱스 재계산
                parking_ind = self.nearest_index(self.parking_path, self.ego_x, self.ego_y)
                self.velocity, self.steering = self.kanayama_controller.run(self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_gear, parking_ind)
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

            print("-------------------------------------")
            print(f" Map       = #{self.map_index+1}")
            print(f" Index     = {self.ego_index}")
            print(f" Mission   = {self.ego_mission}")
            print(f" Collision = {self.coll_flag}")
            print(f" Gear      = { {1: 'P', 2: 'R', 3: 'N', 4: 'D'}.get(self.ego_gear, str(self.ego_gear)) }")
            print(f" Accel (%) = {100 if self.ctrl_cmd_msg.accel * 100 >= 100 else self.ctrl_cmd_msg.accel * 100:.3f}")
            print(f" Brake (%) = {100 if self.ctrl_cmd_msg.brake * 100 >= 100 else self.ctrl_cmd_msg.brake * 100:.3f}")
            print(f" Steering  = {self.steering:.3f}")
            print("-------------------------------------\n")

            rate.sleep()


if __name__ == "__main__":
    morai = Morai_Control_Node()
    morai.main()
