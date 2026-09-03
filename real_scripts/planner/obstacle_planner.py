#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from morai_msgs.msg import CollisionData
from vision_msgs.msg import Detection3DArray


class ObstaclePlanner:
    """
    라이다 장애물 분리, 신호등 처리, 회피 전략을 통합하는 플래너 클래스
    """
    
    def __init__(self, lattice_planner=None, enable_subscribers=True, master=None):
        # lattice planner 직접 참조
        self.lattice_planner = lattice_planner
        # master 참조 (controller 접근용)
        self.master = master
        
        # 장애물 데이터 (주석 처리 - 사용하지 않음)
        # self.static_obstacles = []
        # self.dynamic_obstacles = []
        # self.static_obs_flag = False
        # self.dynamic_obs_flag = False
        
        # 자차 정보 (절대속도 변환용)
        self.ego_velocity = 0.0  # m/s
        self.ego_yaw = 0.0      # deg
        
        # 신호등 상태
        self.traffic_light_red = False
        self.traffic_light_yellow = False  
        self.traffic_light_green = False
        
        # 제동 관련 파라미터 실제 측정
        self.deceleration_rate = 8.516  # m/s^2
        self.speed_distance_map = {
            49: 14.5, 40: 10.5, 30: 7, 20: 4.2
        }
        
        # self.speed_distance_map = {
        #     49: 15.0, 40: 11.0, 30: 7.5, 20: 4.5
        # }
        # 판단 기준 파라미터 (주석 처리 - 사용하지 않음)
        # self.avoidance_distance_threshold = 15.0  # 회피 시작 거리 [m]
        # self.movement_threshold = 3  # m/s, 동적 장애물 판단 기준
        # self.lateral_clearance = 2.5  # 측면 여유 거리 [m]
        # self.following_distance = 10.0  # 추종 거리 [m]
        # 
        # # 장애물 추적 (동적 판단용)
        # self.obstacle_tracking = {}
        # self.tracking_window_size = 5
        
        # 정지선 좌표 리스트
        self.stop_lines = [
            {'x': 620, 'y': -722.1},  # 첫 번째 정지선
            {'x': 768.34, 'y': -829.0},  # 두 번째 정지선
            #{'x': 770.81, 'y': -830.96}  # tmp 번째 정지선

        ]
        
        # 상태 관리
        # 상태 관리 (주석 처리 - 사용하지 않음)
        # self.current_mode = "normal"
        # self.target_obstacle = None
        # self.following_target_id = None  # Following 중인 장애물 ID
        # self.following_start_time = None  # Following 시작 시간
        
        # 토픽 구독 (조건부) - 신호등만 구독
        if enable_subscribers:
            self._setup_subscribers()
    
    def update_ego_info(self, ego_velocity, ego_yaw):
        """자차 정보 업데이트 (master.py에서 호출)"""
        self.ego_velocity = ego_velocity  # m/s
        self.ego_yaw = ego_yaw           # deg
        rospy.loginfo_throttle(1.0, f"자차 정보 업데이트: 속도={ego_velocity:.2f}m/s, yaw={ego_yaw:.1f}deg")
    
    def _setup_subscribers(self):
        """토픽 구독 설정 - 신호등만"""
        # 장애물 토픽 (주석 처리 - 사용하지 않음)
        # rospy.Subscriber("/tracked_objects_3d", Detection3DArray, self.tracked_objects_callback)
        
        # 신호등 토픽만 구독
        rospy.Subscriber('/traffic_light_red', Bool, self.traffic_light_red_callback)
        rospy.Subscriber('/traffic_light_yellow', Bool, self.traffic_light_yellow_callback)
        rospy.Subscriber('/traffic_light_green', Bool, self.traffic_light_green_callback)
    
    
    # ==================== 신호등 콜백 함수 ====================
    def traffic_light_red_callback(self, msg):
        self.traffic_light_red = msg.data
        if msg.data:
            rospy.loginfo("빨간불 신호 감지")
    
    def traffic_light_yellow_callback(self, msg):
        self.traffic_light_yellow = msg.data
        if msg.data:
            rospy.loginfo("노란불 신호 감지")
    
    def traffic_light_green_callback(self, msg):
        self.traffic_light_green = msg.data
        if msg.data:
            rospy.loginfo("초록불 신호 감지")
    
    # ==================== 유틸리티 함수 ====================
    
    def get_traffic_light_state(self):
        """신호등 상태 반환"""
        state = 'unknown'
        if self.traffic_light_red:
            state = 'red'
        elif self.traffic_light_yellow:
            state = 'yellow'
        elif self.traffic_light_green:
            state = 'green'

        rospy.loginfo_throttle(2.0, f"[ObstaclePlanner] 신호등 상태: Red={self.traffic_light_red}, Yellow={self.traffic_light_yellow}, Green={self.traffic_light_green} → {state}")
        return state
    
    def calculate_braking_distance(self, current_speed_kmh):
        """제동거리 계산 - 언제부터 브레이킹을 시작해야 하는지"""
        current_speed_ms = current_speed_kmh / 3.6
        
        if current_speed_kmh in self.speed_distance_map:
            return self.speed_distance_map[current_speed_kmh]
        
        speeds = sorted(self.speed_distance_map.keys())
        
        if current_speed_kmh < speeds[0]:
            return self._calculate_physics_braking_distance(current_speed_ms)
        elif current_speed_kmh > speeds[-1]:
            return self._calculate_physics_braking_distance(current_speed_ms)
        else:
            # 선형 보간
            for i in range(len(speeds) - 1):
                if speeds[i] <= current_speed_kmh <= speeds[i + 1]:
                    ratio = (current_speed_kmh - speeds[i]) / (speeds[i + 1] - speeds[i])
                    distance = (self.speed_distance_map[speeds[i]] + 
                              ratio * (self.speed_distance_map[speeds[i + 1]] - 
                                     self.speed_distance_map[speeds[i]]))
                    return distance
        
        return self._calculate_physics_braking_distance(current_speed_ms)
    
    def _calculate_physics_braking_distance(self, speed_ms):
        """물리 공식 기반 제동거리"""
        if speed_ms <= 0:
            return 0.0
        braking_distance = (speed_ms ** 2) / (2 * self.deceleration_rate)
        return braking_distance * 1.1  # 10% 안전 마진
    
    def can_safely_pass_yellow(self, current_speed_kmh, distance_to_stopline):
        """노란불 3초 내 통과 가능 여부 (앞바퀴 기준)"""
        if current_speed_kmh <= 0:
            return False
        
        speed_ms = current_speed_kmh / 3.6
        time_to_reach = distance_to_stopline / speed_ms
        return time_to_reach < 3.0  # 3초 내 통과 가능
    
    def get_distance_to_nearest_stopline(self, ego_x, ego_y, ego_yaw=0.0):
        """가장 가까운 정지선까지의 거리 계산 (주행 방향 고려)"""
        if not self.stop_lines:
            return None
        
        # 후륜에서 앞바퀴까지의 거리
        front_wheel_offset = 3.0  # 후륜에서 앞바퀴까지 거리 [m]
        
        # 후륜 좌표(GPS)에서 앞바퀴 좌표로 변환
        yaw_rad = math.radians(ego_yaw)
        front_x = ego_x + front_wheel_offset * math.cos(yaw_rad)
        front_y = ego_y + front_wheel_offset * math.sin(yaw_rad)
        
        # 주행 방향 벡터
        drive_dir_x = math.cos(yaw_rad)
        drive_dir_y = math.sin(yaw_rad)
        
        # 가장 가까운 정지선 찾기 (실제 거리 기준)
        nearest_stopline = None
        min_actual_distance = float('inf')
        
        for stop_line in self.stop_lines:
            dx = stop_line['x'] - front_x
            dy = stop_line['y'] - front_y
            actual_distance = math.sqrt(dx**2 + dy**2)
            
            if actual_distance < min_actual_distance:
                min_actual_distance = actual_distance
                nearest_stopline = stop_line
        
        if nearest_stopline is None:
            return None
            
        # 가장 가까운 정지선이 앞에 있는지 확인
        dx = nearest_stopline['x'] - front_x  
        dy = nearest_stopline['y'] - front_y
        distance_along_path = dx * drive_dir_x + dy * drive_dir_y
        
        # 정지선이 뒤에 있으면 None (이미 지남)
        if distance_along_path <= 0:
            rospy.loginfo_throttle(2.0, f"[ObstaclePlanner] 정지선 이미 지남: distance_along_path={distance_along_path:.1f}m")
            return None

        # 정지선이 앞에 있으면 주행 방향 거리 반환 (안전거리 1m 고려)
        safety_margin = 0  # 정지선에서 0.5m 앞에서 멈추기 위한 안전거리
        adjusted_distance = distance_along_path - safety_margin
        rospy.loginfo_throttle(2.0, f"[ObstaclePlanner] 정지선까지 거리: {distance_along_path:.1f}m → 안전거리 적용: {adjusted_distance:.1f}m")
        return adjusted_distance
    
    def get_braking_distance_for_path(self, ego_x, ego_y, ego_yaw, current_speed_kmh):
        """주행 방향을 고려한 제동거리 계산"""
        if not self.stop_lines:
            return None
            
        # 가장 가까운 정지선 찾기
        min_distance = float('inf')
        target_stop_line = None
        for stop_line in self.stop_lines:
            distance = math.sqrt((stop_line['x'] - ego_x)**2 + (stop_line['y'] - ego_y)**2)
            if distance < min_distance:
                min_distance = distance
                target_stop_line = stop_line
        
        if target_stop_line is None:
            return None
        
        # 차량에서 정지선으로의 벡터
        dx = target_stop_line['x'] - ego_x
        dy = target_stop_line['y'] - ego_y
        
        # 차량의 주행 방향 벡터 (yaw 기준)
        yaw_rad = math.radians(ego_yaw)
        drive_dir_x = math.cos(yaw_rad)
        drive_dir_y = math.sin(yaw_rad)
        
        # 정지선까지의 거리를 주행 방향으로 투영 (내적)
        distance_along_path = dx * drive_dir_x + dy * drive_dir_y
        
        # 음수면 정지선을 지나쳤거나 반대 방향
        if distance_along_path <= 0:
            return 0.0
        
        # 기본 제동거리 계산
        base_braking_distance = self.calculate_braking_distance(current_speed_kmh)
        
        # 실제 주행 경로 거리와 직선 거리의 비율로 보정
        straight_distance = min_distance
        if straight_distance > 0:
            path_ratio = distance_along_path / straight_distance
            corrected_braking_distance = base_braking_distance * path_ratio
        else:
            corrected_braking_distance = base_braking_distance
        
        return distance_along_path, corrected_braking_distance
    
    # ==================== 좌표 변환 및 lattice 연동 함수 ====================
    def transform_relative_to_absolute(self, relative_obstacles, ego_x, ego_y, ego_yaw):
        """상대좌표를 절대좌표로 변환"""
        absolute_obstacles = []
        
        for rel_obs in relative_obstacles:
            # 상대좌표 (차량 기준) -> 절대좌표 (월드 좌표계) 변환
            rel_x, rel_y = rel_obs['position']
            
            # 회전 변환
            yaw_rad = math.radians(ego_yaw)
            cos_yaw = math.cos(yaw_rad)
            sin_yaw = math.sin(yaw_rad)
            
            abs_x = ego_x + rel_x * cos_yaw - rel_y * sin_yaw
            abs_y = ego_y + rel_x * sin_yaw + rel_y * cos_yaw
            
            # lattice에서 사용하는 8개 좌표 형태로 변환 (사각형 가정)
            size_x = rel_obs.get('size', [2.0, 1.0, 1.5])[0] / 2
            size_y = rel_obs.get('size', [2.0, 1.0, 1.5])[1] / 2
            
            # 사각형 4개 모서리 계산
            corners = [
                (abs_x - size_x, abs_y - size_y),  # 좌하
                (abs_x + size_x, abs_y - size_y),  # 우하
                (abs_x + size_x, abs_y + size_y),  # 우상
                (abs_x - size_x, abs_y + size_y)   # 좌상
            ]
            
            # 8개 좌표로 변환 [x1,y1, x2,y2, x3,y3, x4,y4]
            obstacle_coords = []
            for corner in corners:
                obstacle_coords.extend(corner)
            
            absolute_obstacles.append(obstacle_coords)
        
        return absolute_obstacles
    
    def update_lattice_obstacles(self, ego_x, ego_y, ego_yaw):
        """처리된 장애물 정보를 lattice에 직접 전달"""
        if self.lattice_planner is None:
            return
        
        # 모든 장애물 수집
        all_obstacles = self.static_obstacles + self.dynamic_obstacles
        
        if not all_obstacles:
            # 장애물이 없으면 빈 리스트 전달
            self.lattice_planner.update_obstacles([])
            return
        
        # 상대좌표 -> 절대좌표 변환
        absolute_obstacles = self.transform_relative_to_absolute(
            all_obstacles, ego_x, ego_y, ego_yaw)
        
        # lattice에 직접 전달
        self.lattice_planner.update_obstacles(absolute_obstacles)
        
        rospy.loginfo_throttle(2.0, f"장애물 {len(absolute_obstacles)}개를 lattice에 전달")
    
    # ==================== 메인 플래닝 함수 ====================
    def plan_driving_strategy(self, ego_position, ego_velocity_kmh, ego_yaw, 
                            distance_to_traffic_light=None):
        """통합 주행 전략 결정"""
        strategy = {
            'mode': 'normal',
            'target_speed': ego_velocity_kmh,
            'lateral_offset': 0.0,
            'stop_required': False,
            'reason': '',
            'target_obstacle': None,
            'use_lattice': False
        }
        
        # 1. 신호등 처리 (최우선)
        if distance_to_traffic_light is not None:
            traffic_strategy = self._plan_traffic_light_strategy(
                ego_velocity_kmh, distance_to_traffic_light)
            if traffic_strategy['stop_required']:
                strategy.update(traffic_strategy)
                strategy['mode'] = 'stopping_traffic'
                return strategy
            elif traffic_strategy['target_speed'] < ego_velocity_kmh * 0.8:
                strategy['target_speed'] = traffic_strategy['target_speed']
                strategy['reason'] = 'traffic_light_deceleration'
    
        return strategy
    
    # ==================== 세부 전략 함수 ====================
    def _plan_traffic_light_strategy(self, ego_velocity_kmh, distance_to_light):
        """신호등 기반 전략"""
        strategy = {
            'stop_required': False,
            'target_speed': ego_velocity_kmh,
            'reason': ''
        }
        
        traffic_state = self.get_traffic_light_state()
        
        if traffic_state == 'green' or traffic_state == 'unknown':
            return strategy
        
        if traffic_state == 'red':
            # 앞바퀴가 정지선을 넘었으면 무조건 통과 (이미 정지선 진입)
            if distance_to_light <= 0:
                strategy['reason'] = 'red_light_front_wheel_passed'
                rospy.loginfo_throttle(1.0, f"[ObstaclePlanner] 빨간불이지만 앞바퀴 정지선 통과 - 계속 진행")
                return strategy

            # 측정된 실제 제동거리 사용
            required_braking_distance = self.calculate_braking_distance(ego_velocity_kmh)

            rospy.loginfo_throttle(1.0, f"[ObstaclePlanner] 빨간불 거리체크: 거리={distance_to_light:.1f}m, "
                        f"필요제동거리={required_braking_distance:.1f}m, 속도={ego_velocity_kmh:.1f}km/h, "
                        f"조건={'제동' if distance_to_light <= required_braking_distance else '통과'}")

            if distance_to_light <= required_braking_distance:
                strategy['stop_required'] = True
                strategy['target_speed'] = 0.0
                strategy['reason'] = 'traffic_light_red'
                rospy.loginfo(f"[ObstaclePlanner]  빨간불 제동 결정!")
            else:
                # 제동거리보다 멀면 무시하고 통과
                rospy.loginfo_throttle(1.0, f"[ObstaclePlanner] 빨간불 무시 통과: 제동거리 {required_braking_distance:.1f}m < 신호등거리 {distance_to_light:.1f}m")
                            
        elif traffic_state == 'yellow':
            # 노란불: 3초 내 통과 가능하면 통과, 아니면 감속
            if self.can_safely_pass_yellow(ego_velocity_kmh, distance_to_light):
                strategy['reason'] = 'yellow_light_pass'
                rospy.loginfo(f"노란불 통과: 거리={distance_to_light:.1f}m")
            else:
                # 감속해서 빨간불 전에 정지선 앞에서 멈춤
                strategy['stop_required'] = False  # 급정지가 아닌 감속
                strategy['target_speed'] = min(ego_velocity_kmh * 0.6, 20.0)  # 현재 속도의 60% 또는 20km/h 중 작은 값
                strategy['reason'] = 'traffic_light_yellow_decel'
                rospy.loginfo(f"노란불 감속: 거리={distance_to_light:.1f}m, 목표속도={strategy['target_speed']:.1f}km/h")
        
        return strategy
