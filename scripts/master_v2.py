#!/home/inji2/.local/rospython/python3
# -*- coding: utf-8 -*-

import os
import rospy
import threading
from controller import Morai_Control_Node
from planner.obstacle_planner import ObstaclePlanner
from jamming_zone_controller import JammingZoneController
from lattice_planner_v2 import LatticePlanner
from std_msgs.msg import Float32, Bool, UInt8
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import MarkerArray
import math
import numpy as np
import json


class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = [v / 3.6 for v in cv]
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)


class MasterController:
    """
    Controller와 ObstaclePlanner를 통합 관리하는 마스터 노드
    """
    
    def __init__(self):
        # ROS 노드 초기화 (master에서 먼저 초기화)
        rospy.init_node('master_controller_node', anonymous=True)

        # Controller 초기화 (init_node=False로 중복 호출 방지)
        self.controller = Morai_Control_Node(init_node=False)
        
        # ObstaclePlanner 초기화 (master 참조 전달)
        self.obstacle_planner = ObstaclePlanner(enable_subscribers=True, master=self)

        # Lattice Planner 초기화 (s 계산용, init_node=False로 중복 방지)
        self.lattice_planner = LatticePlanner(init_node=False)

        # Jamming Zone Controller 초기화 (rospy.init_node는 이미 호출되었으므로 스킵)
        self.jamming_controller = JammingZoneController(init_node=False)

        # Controller에 lattice planner 참조 설정
        self.controller.lattice_planner = self.lattice_planner
        
        # 제밍모드 신호 퍼블리셔 (controller에게 제어권 양도 신호)
        self.jamming_mode_pub = rospy.Publisher("/jamming_mode_active", Bool, queue_size=1)
        
        # 제어 주기
        self.control_rate = rospy.Rate(15)  # 30Hz
        
        # 통합 제어 플래그
        self.traffic_light_control_active = True
        
        # 제동 상태 유지를 위한 플래그
        self.braking_started = False

        # 래치/홀드용 상태변수 추가
        self.waiting_for_green = False
        self.full_stop_counter = 0
        self.last_distance_to_stopline = None
        self.last_traffic_state = 'unknown'
        self.min_stop_hold_frames = int(15 * 0.3)  # 0.3초 (15Hz 기준)
        self._green_since = None

        
        # 주행 거리 추적을 위한 위치 히스토리
        self.position_history = []
        self.max_history_size = 50  # 최근 50개 위치 저장
        
        #self.jamming_zone_start = 1657  # 제밍구역 시작 인덱스
        self.jamming_zone_start = rospy.get_param('~jamming_zone_start', 9999999)  # [2026_AISW] 기본 비활성
        
        self.jamming_zone_end = rospy.get_param('~jamming_zone_end', 9999999)
        self.is_in_jamming_zone = False
        self.jamming_zone_mode_active = False

        # 음영구간 장애물 감지 및 e-stop 관련
        self.shadow_zone_estop_active = False

        # 마스터에서 관리하는 인덱스 (제밍 모드와 무관하게 지속적으로 업데이트)
        self.master_ego_index_global = 0
        self.master_ego_x = 0.0
        self.master_ego_y = 0.0
        self.master_ego_yaw = 0.0

        # 마스터에서 독립적으로 ref_path 로드
        self.master_ref_path = self.load_ref_map()
        rospy.loginfo(f"Master ref_path 로드 완료: {self.master_ref_path.length}개 포인트")
        
        # 한 차로만 존재하고 전방에 차량이 있을 경우, 카팔로잉 용도
        rospy.Subscriber('/nearest_vrel', Float32, self._vrel_cb)
        self._vrel_stamp = 0.0
        self.nearest_vrel = float('nan')   

        rospy.Subscriber('/merge_stop_flag', UInt8, self.stop_vrel)
        self.stop_vrel_flag = None

        # 음영구간 장애물 감지용 토픽 구독
        rospy.Subscriber('/drivable_area_objects', MarkerArray, self.shadow_zone_obstacle_callback)
        
        rospy.loginfo("MasterController 초기화 완료")

    def load_ref_map(self):
        """마스터에서 독립적으로 ref_path 로드"""
        json_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map', 'kcity_map.json')

        with open(json_file, 'r') as f:
            data = json.load(f)

        keys = sorted(data.keys(), key=lambda k: int(k))

        rx = [data[k]['x'] for k in keys]
        ry = [data[k]['y'] for k in keys]
        ryaw = [data[k]['yaw'] for k in keys]
        rk = [data[k]['curvature'] for k in keys]
        rvel = [data[k]['velocity'] for k in keys]
        rm = [data[k]['mission'] for k in keys]
        rgear = [data[k]['gear'] for k in keys]

        return PATH(rx, ry, ryaw, rk, rvel, rm, rgear)

    def nearest_index(self, path, ego_x, ego_y):
        """현재 위치의 경로에서의 인덱스 검색 (controller.py와 동일)"""
        dx = [ego_x - x for x in path.cx]
        dy = [ego_y - y for y in path.cy]
        dist = np.hypot(dx, dy)
        ind = int(np.argmin(dist))
        return ind

    def _vrel_cb(self, msg: Float32):
        self.nearest_vrel = msg.data
        self._vrel_stamp = rospy.get_time()

    def stop_vrel(self, msg):
        # 1이면 정지, 0이면 정지X
        self.stop_vrel_flag = int(msg.data)

    def shadow_zone_obstacle_callback(self, msg: MarkerArray):
        """음영구간 장애물 감지 콜백 - iou_fusion_markers 토픽"""
        # 제밍구역(음영구간) 내에서만 처리
        if not self.is_in_jamming_zone:
            return

        # 마커가 있으면 장애물 감지 - 즉시 정지
        if msg.markers and len(msg.markers) > 0:
            # 첫 번째 마커 정보 추출
            first_marker = msg.markers[0]
            obstacle_x = first_marker.pose.position.x
            obstacle_y = first_marker.pose.position.y
            obstacle_distance = math.sqrt(obstacle_x**2 + obstacle_y**2)

            if not self.shadow_zone_estop_active:
                rospy.logwarn(f"[음영구간 E-STOP] 장애물 감지! 거리: {obstacle_distance:.1f}m")
                rospy.logwarn("==== 음영구간 E-STOP 활성화! ====")

            self.shadow_zone_estop_active = True
        else:
            # 마커가 없으면 장애물 없음 - 즉시 주행 재개
            if self.shadow_zone_estop_active:
                rospy.loginfo("==== 음영구간 E-STOP 해제 - 주행 재개 ====")

            self.shadow_zone_estop_active = False

    def _compute_speed_cap_from_vrel(self):
        # 정지명령 수신시 정지 최우선
        if self.stop_vrel_flag in (1, True):
            return 0.0

        # 최신데이터 여부 확인 (0.5초 이내만 사용)
        if (rospy.get_time() - self._vrel_stamp) > 0.5:
            return None
        if math.isnan(self.nearest_vrel):
            return None
        
        # ego 속도(m/s)
        ego_v = getattr(self.controller, 'ego_vel', 0.0)

        # v_rel 정의 가정: 로컬 x축(전방) 상대속도 [m/s].
        # 절대(차선 방향) 장애물 속도 추정
        v_obs = ego_v + self.nearest_vrel

        # 속도 제한 (최소 0.0kph, 최대 35km/h)
        v_obs = np.clip(v_obs, 0.0, 35.0/3.6)

        # 안전 버퍼(속도 약간 더 낮추도록)
        buffer = 2.5 / 3.6
        cap = max(0.0, v_obs - buffer)
        
        print(f"cap : {cap}")

        return cap
    
    def update_position_history(self, ego_x, ego_y):
        """위치 히스토리 업데이트"""
        current_pos = (ego_x, ego_y)
        self.position_history.append(current_pos)
        
        # 히스토리 크기 제한
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)
    
    def calculate_actual_braking_distance(self, current_speed_kmh):
        """실제 주행 경로를 고려한 제동거리 계산"""
        if len(self.position_history) < 10:  # 최소 10개 위치 필요
            # 기본 제동거리 사용
            base_distance = self.obstacle_planner.calculate_braking_distance(current_speed_kmh)
            return base_distance * 1.2  # 안전 계수
        
        # 최근 10개 위치에서 실제 주행 거리 계산
        recent_positions = self.position_history[-10:]
        actual_distance = 0.0
        
        for i in range(1, len(recent_positions)):
            prev_x, prev_y = recent_positions[i-1]
            curr_x, curr_y = recent_positions[i]
            segment_distance = ((curr_x - prev_x)**2 + (curr_y - prev_y)**2)**0.5
            actual_distance += segment_distance
        
        if actual_distance <= 0:
            base_distance = self.obstacle_planner.calculate_braking_distance(current_speed_kmh)
            return base_distance * 1.3
        
        # 직선 거리 계산
        start_x, start_y = recent_positions[0]
        end_x, end_y = recent_positions[-1]
        straight_distance = ((end_x - start_x)**2 + (end_y - start_y)**2)**0.5
        
        if straight_distance <= 0:
            base_distance = self.obstacle_planner.calculate_braking_distance(current_speed_kmh)
            return base_distance * 1.2
        
        # 경로 보정 계수 (실제 주행 거리 / 직선 거리) - 최대 1.5배로 제한
        path_correction = actual_distance / straight_distance
        
        # 기본 제동거리에 경로 보정 적용
        base_braking_distance = self.obstacle_planner.calculate_braking_distance(current_speed_kmh)
        corrected_distance = base_braking_distance * path_correction * 1.2  # 20% 안전 마진
        
        return corrected_distance
    
    def check_jamming_zone(self):
        """제밍구역 진입/탈출 체크 (마스터가 관리하는 전역 인덱스 사용)"""
        # 마스터에서 관리하는 인덱스 사용 (제밍 모드와 무관하게 지속 업데이트)
        current_index = self.master_ego_index_global

        if current_index is None:
            rospy.logwarn_throttle(2.0, "[Jamming] master ego index not ready")
            return self.is_in_jamming_zone

        try:
            current_index = int(current_index)
        except Exception:
            rospy.logwarn_throttle(2.0, f"[Jamming] invalid master index: {current_index}")
            return self.is_in_jamming_zone

        # 디버깅 로그 추가
        #rospy.loginfo_throttle(1.0, f"[Jamming Debug] Current index: {current_index}, Start: {self.jamming_zone_start}, End: {self.jamming_zone_end}, In zone: {self.is_in_jamming_zone}")

        # 1. 제밍구역 진입: GPS 인덱스가 1657에 도달하면 제밍구역 진입
        if (not self.is_in_jamming_zone) and (current_index >= self.jamming_zone_start):
            rospy.loginfo("="*50)
            rospy.loginfo(f"==== 제밍구역 진입! (인덱스: {current_index}) ====")
            rospy.loginfo("==== GPS 신호 차단 - 카메라 경로 추종 시작 ====")
            rospy.loginfo("="*50)
            self.is_in_jamming_zone = True

        # 2. 제밍구역 탈출: GPS 인덱스가 1825에 도달하면 제밍구역 탈출
        if self.is_in_jamming_zone and (current_index >= self.jamming_zone_end):
            rospy.loginfo("="*50)
            rospy.loginfo(f"==== 제밍구역 탈출! (인덱스: {current_index}) ====")
            rospy.loginfo("==== GPS 신호 복구 - GPS 경로 추종 재개 ====")
            rospy.loginfo("="*50)
            self.is_in_jamming_zone = False
            self.deactivate_jamming_zone_mode()

        # 3. 제밍구역 내에서는 GPS 인덱스 완전 무시
        elif self.is_in_jamming_zone:
            rospy.loginfo_throttle(5.0, f"[Jamming Mode] GPS 무시, 카메라 경로 추종 중 (현재 GPS 인덱스: {current_index})")

        return self.is_in_jamming_zone

    def update_master_ego_status(self):
        """마스터에서 차량 위치와 인덱스를 지속적으로 업데이트 (제밍 모드와 무관)"""
        # controller에서 차량 위치 정보 가져오기
        if hasattr(self.controller, 'ego_x') and hasattr(self.controller, 'ego_y'):
            self.master_ego_x = self.controller.ego_x
            self.master_ego_y = self.controller.ego_y

        if hasattr(self.controller, 'ego_yaw'):
            self.master_ego_yaw = self.controller.ego_yaw
        elif hasattr(self.controller, 'adjusted_yaw'):
            self.master_ego_yaw = self.controller.adjusted_yaw

        # 마스터의 ref_path를 사용하여 인덱스 계산 (controller와 무관하게 독립적으로)
        if self.master_ref_path is not None and self.master_ego_x != 0.0 and self.master_ego_y != 0.0:
            self.master_ego_index_global = self.nearest_index(
                self.master_ref_path, self.master_ego_x, self.master_ego_y)

        # 디버깅용 로그
        rospy.loginfo_throttle(5.0,
            f"[Master] 위치=({self.master_ego_x:.1f}, {self.master_ego_y:.1f}), "
            f"인덱스={self.master_ego_index_global}, yaw={self.master_ego_yaw:.1f}")

    def activate_jamming_zone_mode(self):
        """제밍구역 모드 활성화"""
        rospy.loginfo("제밍구역 모드 활성화:")
        rospy.loginfo("  - GPS 신호 무시")
        rospy.loginfo("  - 다른 센서 기반 주행 시작")
        rospy.loginfo("  - 장애물/신호등 제어 비활성화")
        
        # 여기에 제밍구역 모드에 필요한 설정 추가
        # 예: self.controller.use_gps = False
        #     self.controller.use_jamming_mode = True
    
    def deactivate_jamming_zone_mode(self):
        """제밍구역 모드 비활성화"""
        rospy.loginfo("일반 모드 복귀:")
        rospy.loginfo("  - GPS 신호 재활성화")  
        rospy.loginfo("  - 장애물/신호등 제어 재활성화")
        
        # 여기에 일반 모드 복귀에 필요한 설정 추가
        # 예: self.controller.use_gps = True
        #     self.controller.use_jamming_mode = False
    
    def get_traffic_light_strategy(self):
        """신호등 기반 주행 전략 계산"""
        # 차량 위치 확인
        if not hasattr(self.controller, 'ego_x') or not hasattr(self.controller, 'ego_y'):
            rospy.logwarn_throttle(5.0, "차량 위치 정보 없음")
            return None
            
        # 현재 차량 위치와 속도
        ego_x = self.controller.ego_x
        ego_y = self.controller.ego_y
        ego_velocity = getattr(self.controller, 'ego_vel', 0.0) * 3.6  # m/s -> km/h

        # 현재 인덱스 확인 - 신호등 처리 범위 제한 (141-221, 1100-1169)
        # 단, 이미 제동 중이면 인덱스와 관계없이 초록불 해제 로직은 실행
        current_index = self.master_ego_index_global
        _tz = rospy.get_param('~traffic_zones', [])  # [2026_AISW] [[s,e],...] 미설정 시 신호등 로직 꺼짐
        traffic_zone_1 = any(z[0] <= current_index <= z[1] for z in _tz)
        traffic_zone_2 = False

        if not self.braking_started and (current_index is None or not (traffic_zone_1 or traffic_zone_2)):
            rospy.loginfo_throttle(5.0, f"[신호등] 현재 인덱스 {current_index} - 신호등 처리 범위(141-221, 1100-1169) 밖이므로 무시")
            return None
        
        # 위치 히스토리 업데이트
        self.update_position_history(ego_x, ego_y)
        
        # 정지선까지 거리 계산 (앞바퀴 기준)
        ego_yaw = getattr(self.controller, 'adjusted_yaw', 0.0)
        distance_to_stopline = self.obstacle_planner.get_distance_to_nearest_stopline(ego_x, ego_y, ego_yaw)
        
        # 신호등 상태 확인
        traffic_state = self.obstacle_planner.get_traffic_light_state()

        # 인덱스 기반 강제 신호 설정 및 제동 해제
        if False:  # [2026_AISW] 옛 상암맵 강제 green 구간 비활성
            traffic_state = 'green'  # 초록불 강제 설정
            rospy.loginfo_throttle(2.0, f"[신호등] 인덱스 {current_index} - 초록불 강제 설정")
            # 이미 제동 중이어도 강제 해제
            if self.braking_started:
                self.braking_started = False
                self.waiting_for_green = False
                self._green_since = None
                self.controller.traffic_light_brake = False  # 제동 플래그 해제
                rospy.loginfo(f"[신호등] 인덱스 {current_index} - 강제 제동 해제하고 직진!")
        elif False:  # [2026_AISW] 옛 상암맵 강제 green 구간 비활성
            traffic_state = 'green'  # 초록불 강제 설정
            rospy.loginfo_throttle(2.0, f"[신호등] 인덱스 {current_index} - 초록불 강제 설정")
            # 이미 제동 중이어도 강제 해제
            if self.braking_started:
                self.braking_started = False
                self.waiting_for_green = False
                self._green_since = None
                self.controller.traffic_light_brake = False  # 제동 플래그 해제
                rospy.loginfo(f"[신호등] 인덱스 {current_index} - 강제 제동 해제하고 직진!")
        
        # 디버그 출력
        rospy.loginfo_throttle(2.0,
            f"[신호등 디버그] 위치=({ego_x:.1f}, {ego_y:.1f}), "
            f"신호등={traffic_state}, 거리={distance_to_stopline}, "
            f"속도={ego_velocity:.1f}km/h, yaw={ego_yaw:.1f}")
        
        if distance_to_stopline is None:
            rospy.logwarn_throttle(5.0, "정지선 거리 계산 실패")
            return None

        # 제동거리 계산 및 비교 (제동 상태가 아닐 때만)
        if not self.braking_started:
            braking_distance = self.obstacle_planner.calculate_braking_distance(ego_velocity)

            # 제동거리 < 신호등까지 거리면 토픽 무시 (너무 멀어서 제동할 필요 없음)
            if braking_distance < distance_to_stopline:
                rospy.loginfo_throttle(5.0, f"[신호등] 제동거리({braking_distance:.1f}m) < 신호등거리({distance_to_stopline:.1f}m) - 토픽 무시")
                return None

        if traffic_state == 'unknown':
            return None
        
        # 신호등 전략 계산
        strategy = self.obstacle_planner._plan_traffic_light_strategy(
            ego_velocity, distance_to_stopline)
        
        rospy.loginfo_throttle(1.0,
            f"[신호등] 위치=({ego_x:.1f},{ego_y:.1f}), 거리={distance_to_stopline:.1f}m, "
            f"속도={ego_velocity:.1f}km/h, 신호={traffic_state}, 제동={strategy.get('stop_required', 'N/A')}")

        # 매 주기 최신값 저장
        self.last_distance_to_stopline = distance_to_stopline
        self.last_traffic_state = traffic_state

        return strategy
    
    def apply_traffic_control(self, strategy):
        """신호등 제어 적용"""
        if strategy is None:
            rospy.loginfo_throttle(3.0, "[Master] 신호등 전략이 None - 제어 건너뜀")
            return

        rospy.loginfo_throttle(1.0, f"[Master] 신호등 제어 적용: stop_required={strategy.get('stop_required', False)}, reason={strategy.get('reason', 'None')}")

        # 현재 신호/속도
        traffic_state = self.obstacle_planner.get_traffic_light_state()
        current_speed_ms = getattr(self.controller, 'ego_vel', 0.0)

        # ① 이미 커밋된 상태라면: 무조건 유지 (strategy 무시)
        if self.braking_started:
            self.controller.traffic_light_brake = True
            self.controller.velocity = 0.0

            # 완전 정지 검출 (연속 프레임 카운팅)
            if current_speed_ms < 0.1:
                self.full_stop_counter += 1
            else:
                self.full_stop_counter = 0

            # 완전 정지 완료 → 초록불 대기 상태 전환
            if self.full_stop_counter >= self.min_stop_hold_frames:
                self.waiting_for_green = True
                rospy.loginfo_throttle(1.0, f"[Master] 완전 정지 완료 - 초록불 대기 중")

            # 해제 조건: 초록불이 안정적으로 들어온 뒤 소폭 지연
            if self.waiting_for_green and traffic_state == 'green':
                if self._green_since is None:
                    self._green_since = rospy.Time.now()
                if rospy.Time.now() - self._green_since >= rospy.Duration(0.2):
                    # RELEASE
                    self.braking_started = False
                    self.waiting_for_green = False
                    self._green_since = None
                    self.controller.traffic_light_brake = False
                    rospy.loginfo("[Master] 초록불 안정화 - 제동 해제하고 출발!")
                    # 출발 시 살살 가속 제한 (선택)
                    self.controller.external_speed_cap = 5.0/3.6
                else:
                    rospy.loginfo_throttle(1.0, f"[Master] 초록불 감지 - 안정화 대기 중")
            else:
                self._green_since = None
                if not self.waiting_for_green:
                    rospy.loginfo_throttle(1.0, f"[Master] 제동 지속 중 (속도: {current_speed_ms*3.6:.1f}km/h)")

            return  # 커밋된 동안엔 strategy의 stop_required를 보지 않음

        # ② 아직 커밋 전이라면: 이번 프레임에 '한 번만' 판단
        want_stop = bool(strategy.get('stop_required', False))

        if want_stop:
            # STOP 커밋 (한 번만)
            self.braking_started = True
            self.waiting_for_green = False
            self.full_stop_counter = 0
            self._green_since = None
            self.controller.traffic_light_brake = True
            self.controller.velocity = 0.0
            rospy.loginfo(f"[Master] 신호등 제동 커밋: {strategy['reason']} - 완전 정지까지 계속!")
            return

        # ③ 통과 케이스: 그냥 정상 주행(감속 명령은 그대로 활용)
        self.controller.traffic_light_brake = False
        rospy.loginfo_throttle(1.0, "[Master] 통과 판정 - 정상 주행 지속")

        # 노란불 감속 처리
        if strategy.get('reason') == 'traffic_light_yellow_decel' and 'target_speed' in strategy:
            target_speed_ms = strategy['target_speed'] / 3.6  # km/h -> m/s
            self.controller.external_speed_cap = target_speed_ms
            rospy.loginfo_throttle(1.0, f"노란불 감속: {strategy['target_speed']:.1f}km/h")
        else:
            # 노란불 감속이 아닌 경우 speed_cap 해제
            if hasattr(self.controller, 'external_speed_cap'):
                self.controller.external_speed_cap = None
    
    def control_loop(self):
        """메인 제어 루프"""
        try:
            while not rospy.is_shutdown():
                # 0. 마스터 차량 상태 업데이트 (제밍 모드와 무관하게 지속)
                self.update_master_ego_status()

                if hasattr(self.controller, 'ego_index_global'):
                    rospy.loginfo(f"[Master Check] Controller Index = {self.controller.ego_index_global}")
                rospy.loginfo(f"[Master Check] Master Index = {self.master_ego_index_global}")

                # 1. 제밍구역 상태를 "한 번만" 체크
                in_jamming_zone = self.check_jamming_zone()
                
                # 제밍모드 신호를 controller.py에게 전송
                jamming_signal = Bool()
                jamming_signal.data = in_jamming_zone
                self.jamming_mode_pub.publish(jamming_signal)

                # 2. 체크된 상태값(in_jamming_zone)을 사용하여 분기
                if in_jamming_zone:
                    rospy.loginfo_throttle(2.0, f"제밍구역 내 주행 중 - 마스터 인덱스: {self.master_ego_index_global}")

                    # 음영구간 e-stop 체크 (최우선)
                    if self.shadow_zone_estop_active:
                        rospy.logwarn_throttle(1.0, "[음영구간 E-STOP] 장애물로 인한 긴급정지!")
                        # 즉시 정지
                        v, w = 0.0, 0.0
                    else:
                        # 정상 제밍구역 제어
                        # 제밍구역에서는 jamming_zone_controller가 직접 제어
                        self.jamming_controller.jamming_mode_active = True
                        rospy.loginfo("[Master Debug] Starting jamming zone control...")

                        # jamming_controller의 odom 정보 업데이트 (마스터에서 관리하는 정보 사용)
                        #rospy.loginfo("[JAMMING DEBUG] Checking master attributes...")

                        # 마스터에서 관리하는 위치 정보 전달
                        self.jamming_controller.odom_x = self.master_ego_x
                        self.jamming_controller.odom_y = self.master_ego_y
                        self.jamming_controller.odom_yaw = self.master_ego_yaw
                        #rospy.loginfo(f"[JAMMING DEBUG] Updated odom from master: x={self.master_ego_x:.3f}, y={self.master_ego_y:.3f}, yaw={self.master_ego_yaw:.3f}")

                        # jamming controller 한 번 실행 (cmd_vel 퍼블리시)
                        rospy.loginfo_throttle(1.0, "[JAMMING CONTROL] About to call compute_cmd...")
                        try:
                            v, w = self.jamming_controller.compute_cmd()
                            rospy.loginfo_throttle(1.0, f"[JAMMING CONTROL] SUCCESS: v={v:.3f} m/s, w={w:.3f} rad/s")
                        except Exception as e:
                            rospy.logerr(f"[JAMMING CONTROL ERROR] compute_cmd failed: {e}")
                            import traceback
                            rospy.logerr(f"[JAMMING CONTROL ERROR] Traceback: {traceback.format_exc()}")
                            v, w = 0.0, 0.0
                        rospy.loginfo_throttle(1.0, "[JAMMING CONTROL] compute_cmd completed, proceeding to vehicle control...")
                    
                    # v(m/s)를 accel/brake로 변환 (controller의 AccelCmd_Converter 방식 사용)
                    current_vel = getattr(self.controller, 'ego_vel', 0.0)  # m/s
                    #rospy.loginfo(f"[JAMMING DEBUG] Current velocity: {current_vel:.3f} m/s, target velocity: {v:.3f} m/s")
                    accel_cmd, brake_cmd = self.controller.cmd_converter.run(v, current_vel, 4)  # 기어는 D(4) 고정
                    
                    self.controller.ctrl_cmd_msg.accel = accel_cmd
                    self.controller.ctrl_cmd_msg.brake = brake_cmd
                    self.controller.ctrl_cmd_msg.steering = w  # w는 rad/s이므로 그대로 사용
                    self.controller.ctrl_cmd_pub.publish(self.controller.ctrl_cmd_msg)
                    
                    #rospy.loginfo(f"[JAMMING CONTROL] Vehicle command sent: accel={accel_cmd:.3f}, brake={brake_cmd:.3f}, steering={w:.3f}")
                    #rospy.loginfo("[JAMMING CONTROL] ===== 카메라 경로 추종 중 =====")
                    
                    self.control_rate.sleep()
                    continue
                else:
                    # 제밍구역이 아닐 때는 jamming controller 비활성화
                    self.jamming_controller.jamming_mode_active = False
                
                # --- 제밍구역이 아닐 때만 아래 로직 실행 ---
                # ====================[ 수정된 부분 종료 ]====================
                
                # 1. 자차 정보를 obstacle_planner에 전달 (절대속도 변환용)
                if hasattr(self.controller, 'ego_vel') and hasattr(self.controller, 'adjusted_yaw'):
                    ego_velocity_ms = getattr(self.controller, 'ego_vel', 0.0)  # m/s
                    ego_yaw_deg = getattr(self.controller, 'adjusted_yaw', 0.0)  # deg
                    self.obstacle_planner.update_ego_info(ego_velocity_ms, ego_yaw_deg)
                
                # 1. 신호등 제어
                if self.traffic_light_control_active:
                    rospy.loginfo_throttle(2.0, "[Master] 신호등 제어 활성화 - 전략 계산 중")
                    traffic_strategy = self.get_traffic_light_strategy()
                    rospy.loginfo_throttle(2.0, f"[Master] 신호등 전략: {traffic_strategy}")
                    self.apply_traffic_control(traffic_strategy)
                else:
                    rospy.loginfo_throttle(5.0, "[Master] 신호등 제어 비활성화")

                # 2. 차선 하나일 경우, 카팔로잉
                cap = self._compute_speed_cap_from_vrel()

                self.controller.external_speed_cap = cap

                self.control_rate.sleep()
                
        except KeyboardInterrupt:
            rospy.loginfo("Master Controller 종료")
        except Exception as e:
            rospy.logerr(f"제어 루프 오류: {e}")
                
        except KeyboardInterrupt:
            rospy.loginfo("Master Controller 종료")
        except Exception as e:
            rospy.logerr(f"제어 루프 오류: {e}")
    
    def start_controller_thread(self):
        """Controller를 별도 스레드에서 실행"""
        controller_thread = threading.Thread(target=self.controller.main, daemon=True)
        controller_thread.start()
        return controller_thread
    
    def run(self):
        """마스터 컨트롤러 실행"""
        rospy.loginfo("Master Controller 시작")
        
        # Controller를 별도 스레드에서 실행
        controller_thread = self.start_controller_thread()
        
        # 메인 제어 루프 시작
        self.control_loop()
        
        # 정리
        self.obstacle_planner = None
        if hasattr(self.controller, 'stop_lattice_planner'):
            self.controller.stop_lattice_planner()


if __name__ == "__main__":
    try:
        master = MasterController()
        master.run()
    except KeyboardInterrupt:
        rospy.loginfo("프로그램 종료")
    except Exception as e:  
        rospy.logerr(f"마스터 컨트롤러 오류: {e}")
    finally:
        rospy.loginfo("정리 완료")