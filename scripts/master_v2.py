#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import threading
from controller import Morai_Control_Node
from planner.obstacle_planner import ObstaclePlanner
from aisw_common import PKG_DIR, load_map_fields, NearestIndexer, ros_sections, get_section
from ai.dead_reckoning import DeadReckoning
from ai.zone_controller import AIZoneController
from std_msgs.msg import Float32, Bool, UInt8, String
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import PoseStamped
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

        # [2026_AISW] 쓰이지 않는 LatticePlanner 인스턴스 제거 — lattice_planner 노드와
        # 같은 토픽을 중복 구독해 콜백만 두 배로 돌고 있었다.

        # [2026_AISW] AI 구간 제어권 신호 (True = master 가 직접 제어, controller/lattice 는 대기).
        # 토픽 이름은 기존 노드 호환을 위해 /jamming_mode_active 유지.
        self.jamming_mode_pub = rospy.Publisher("/jamming_mode_active", Bool, queue_size=1)
        self.drive_mode_pub = rospy.Publisher('/aisw/drive_mode', String, queue_size=1)
        self.mission_pub = rospy.Publisher('/aisw/mission', String, queue_size=1)
        self.pose_pub = rospy.Publisher('/aisw/ego_pose', PoseStamped, queue_size=1)
        
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
        
        # [2026_AISW] 구간 설정: config/kcity_sections.yaml (~파라미터가 있으면 우선)
        self.sections = ros_sections()
        self.traffic_zones = rospy.get_param('~traffic_zones', get_section(self.sections, 'traffic_zones', []))
        # 정지선: 옛 상암맵 하드코딩 좌표 대신 설정 파일 값 사용
        self.obstacle_planner.stop_lines = [
            {'x': float(p[0]), 'y': float(p[1])} for p in get_section(self.sections, 'stop_lines', [])]
        self.is_in_jamming_zone = False   # = AI 구간 제어 중 (구 이름 유지)

        # 마스터에서 관리하는 인덱스 (제밍 모드와 무관하게 지속적으로 업데이트)
        self.master_ego_index_global = 0
        self.master_ego_x = 0.0
        self.master_ego_y = 0.0
        self.master_ego_yaw = 0.0

        # 마스터에서 독립적으로 ref_path 로드
        self.master_ref_path = self.load_ref_map()
        self.master_indexer = NearestIndexer(self.master_ref_path.cx, self.master_ref_path.cy)
        rospy.loginfo(f"Master ref_path 로드 완료: {self.master_ref_path.length}개 포인트")

        # [2026_AISW] AI 구간 (config ai_zones): GPS 음영 / 회전교차로 → ai_zone_controller
        self.ai_zones = get_section(self.sections, 'ai_zones', [])
        self.missions = get_section(self.sections, 'missions', [])
        speed_limit = float(get_section(self.sections, 'speed_limit_kph', 55.0)) / 3.6
        self.ai_ctrl = AIZoneController(
            self.master_ref_path.cx, self.master_ref_path.cy,
            model_dir=os.path.expanduser(rospy.get_param('~model_dir', os.path.join(PKG_DIR, 'models'))),
            speed_limit_mps=speed_limit,
            use_ai=bool(rospy.get_param('~ai_enable', True)),
            log=rospy.loginfo)
        self.active_zone = None
        self.dr = DeadReckoning()
        self._last_fix_t = None
        self.gps_age = float('inf')
        self.yaw_rate = 0.0
        self.scan = None
        self._scan_t = 0.0
        self._last_loop_t = None
        rospy.Subscriber('/aisw/lidar_scan', LaserScan, self._scan_cb, queue_size=1)
        rospy.Subscriber('/imu', Imu, self._imu_rate_cb, queue_size=1)
        
        # 한 차로만 존재하고 전방에 차량이 있을 경우, 카팔로잉 용도
        rospy.Subscriber('/nearest_vrel', Float32, self._vrel_cb)
        self._vrel_stamp = 0.0
        self.nearest_vrel = float('nan')   

        rospy.Subscriber('/merge_stop_flag', UInt8, self.stop_vrel)
        self.stop_vrel_flag = None

        rospy.loginfo("MasterController 초기화 완료")

    def load_ref_map(self):
        """마스터에서 독립적으로 ref_path 로드"""
        json_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map', 'kcity_map.json')
        return PATH(*load_map_fields(json_file))

    def _vrel_cb(self, msg: Float32):
        self.nearest_vrel = msg.data
        self._vrel_stamp = rospy.get_time()

    def stop_vrel(self, msg):
        # 1이면 정지, 0이면 정지X
        self.stop_vrel_flag = int(msg.data)

    def _scan_cb(self, msg):
        self.scan = msg.ranges
        self._scan_t = rospy.get_time()

    def _imu_rate_cb(self, msg):
        self.yaw_rate = msg.angular_velocity.z

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
    
    def update_master_ego_status(self):
        """GPS 가 신선하면 GPS, 끊기면 추측항법으로 자차 위치/인덱스를 갱신 (제어 모드와 무관하게 매 주기)."""
        c = self.controller
        now = rospy.get_time()
        gps_t = getattr(c, '_last_gps_t', 0.0)
        self.gps_age = (now - gps_t) if gps_t > 0.0 else float('inf')
        self.master_ego_yaw = getattr(c, 'ego_yaw', 0.0)
        if gps_t > 0.0 and gps_t != self._last_fix_t and self.gps_age < 0.25:
            self.dr.fix(c.ego_x, c.ego_y, now)
            self._last_fix_t = gps_t
        else:
            self.dr.predict(getattr(c, 'ego_vel', 0.0), math.radians(self.master_ego_yaw), now)

        if self.dr.ready:
            self.master_ego_x, self.master_ego_y = self.dr.x, self.dr.y
            self.master_ego_index_global = self.master_indexer.find(self.master_ego_x, self.master_ego_y)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = self.master_ego_x, self.master_ego_y
            pose.pose.position.z = 0.0 if self.gps_age < 0.5 else 1.0   # z=1: 추측항법 위치 (로그 구분용)
            half = math.radians(self.master_ego_yaw) / 2.0
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(half), math.cos(half)
            self.pose_pub.publish(pose)

        idx = self.master_ego_index_global
        names = [m['name'] for m in self.missions if int(m['start']) <= idx <= int(m['end'])]
        self.mission_pub.publish(String(','.join(names)))

        rospy.loginfo_throttle(5.0,
            f"[Master] 위치=({self.master_ego_x:.1f}, {self.master_ego_y:.1f}), "
            f"인덱스={idx}, yaw={self.master_ego_yaw:.1f}, GPS age={self.gps_age:.1f}s, 미션={names}")

    def update_ai_zone(self):
        """AI 구간 진입/탈출 판정. GPS 음영 중에는 추측항법 인덱스로 진행을 따라간다.

        - ai_zones 의 [enter, end] 안이면 해당 모드
        - 어디서든 GPS 가 0.5초 넘게 끊기면 shaded 모드 (예상 밖 두절 대비.
          controller 의 GPS 두절 크리프/정지(0.7초)보다 먼저 제어권을 가져온다)
        - shaded 구간 끝을 지나도 GPS 가 복구되기 전까지는 shaded 유지
        """
        idx = self.master_ego_index_global
        zone = None
        if self.dr.ready:
            for z in self.ai_zones:
                if int(z['enter']) <= idx <= int(z['end']):
                    zone = z
                    break
            if self.gps_age > 0.5 and (zone is None or zone.get('mode') != 'shaded'):
                if self.active_zone and self.active_zone.get('mode') == 'shaded':
                    zone = self.active_zone
                else:
                    zone = {'name': 'gps_lost', 'mode': 'shaded', 'enter': idx, 'end': idx}

        prev = self.active_zone['name'] if self.active_zone else None
        cur = zone['name'] if zone else None
        if cur != prev:
            if zone is not None:
                rospy.loginfo('=' * 50)
                rospy.loginfo('==== AI 구간 진입: %s (mode=%s, 인덱스 %d, GPS age %.1fs) ====',
                              cur, zone.get('mode'), idx, self.gps_age)
                # gps_lost → gps_shaded 처럼 같은 모드 안에서 이름만 바뀌면 회피 상태 유지
                if self.active_zone is None or self.active_zone.get('mode') != zone.get('mode'):
                    self.ai_ctrl.reset()
            else:
                rospy.loginfo('==== AI 구간 탈출: %s (인덱스 %d) → 룰베이스 복귀 ====', prev, idx)
                # 룰베이스 복귀 시 신호등 래치 등 이전 상태가 남지 않게
                self.braking_started = False
                self.controller.traffic_light_brake = False
        self.active_zone = zone
        self.is_in_jamming_zone = zone is not None
        return zone

    def ai_zone_control(self, zone):
        """AI 구간 1주기 제어: ai_zone_controller 출력 → /ctrl_cmd 직접 발행."""
        now = rospy.get_time()
        dt = 1.0 / 15 if self._last_loop_t is None else min(max(now - self._last_loop_t, 0.01), 0.2)
        ego_vel = getattr(self.controller, 'ego_vel', 0.0)
        scan_fresh = (now - self._scan_t) < 0.5
        v, steer, source = self.ai_ctrl.step(
            zone.get('mode', 'shaded'), self.master_ego_index_global,
            self.master_ego_x, self.master_ego_y, math.radians(self.master_ego_yaw),
            ego_vel, self.yaw_rate, self.scan if scan_fresh else None, dt)
        if not scan_fresh:
            # LiDAR 없이는 통로 감시가 불가능 → 서행
            v = min(v, 8.0 / 3.6)
            source += '+no_lidar'
            rospy.logwarn_throttle(2.0, '[AI] /aisw/lidar_scan 0.5초 이상 끊김 — 8 kph 서행 (lidar_obstacles:=true 확인)')

        accel_cmd, brake_cmd = self.controller.cmd_converter.run(v, ego_vel, 4)  # 기어 D(4)
        msg = self.controller.ctrl_cmd_msg
        msg.accel = min(max(accel_cmd, 0.0), 1.0)
        msg.brake = min(max(brake_cmd, 0.0), 1.0)
        msg.steering = steer   # [rad], 브리지에서 steer_scale 보상
        self.controller.ctrl_cmd_pub.publish(msg)
        self.drive_mode_pub.publish(String('AI_%s:%s' % (zone.get('mode'), source)))
        rospy.loginfo_throttle(1.0, '[AI %s] idx=%d v=%.1f→%.1fkph steer=%.1f° src=%s GPS age=%.1fs DR %.0fm',
                               zone.get('name'), self.master_ego_index_global, ego_vel * 3.6, v * 3.6,
                               math.degrees(steer), source, self.gps_age, self.dr.dist_since_fix)

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
        _tz = self.traffic_zones  # [2026_AISW] [[s,e],...] 비어 있으면 신호등 로직 꺼짐
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


                # 1. AI 구간 판정 (GPS 음영 / 회전교차로)
                zone = self.update_ai_zone()
                self.jamming_mode_pub.publish(Bool(data=zone is not None))

                if zone is not None:
                    self.ai_zone_control(zone)
                    self._last_loop_t = rospy.get_time()
                    self.control_rate.sleep()
                    continue
                self._last_loop_t = rospy.get_time()
                self.drive_mode_pub.publish(String('RULE'))

                # --- 이하 룰베이스 (AI 구간이 아닐 때) ---
                
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