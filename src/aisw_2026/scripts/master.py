#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""메인 판단·제어 노드.

매 주기(15 Hz):
  1. 자차 위치 갱신 — GPS 가 있으면 GPS, 끊기면 추측항법 (/aisw/ego_pose)
  2. AI 구간 판정 — ai_zones(회전교차로·GPS 음영) 안이면 control/roundabout_shaded 로 직접 /ctrl_cmd
  3. 룰 구간 — 신호등·앞차·합류·플래너 속도 상한을 정해 control/normal_drive(별도 스레드)에 넘긴다
"""
import math
import os
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8
from vision_msgs.msg import Detection3DArray

from control.ai_input import detections_to_objects
from control.path_utils import PKG_DIR, NearestIndexer, get_section, load_ref_path, ros_sections
from control.gps_jamming import DeadReckoning
from control.normal_drive import RuleController
from control.traffic_light import TrafficLight
from control.roundabout_shaded import ZoneController

DR_MAX_DIST = 250.0   # GPS 없이 추측항법으로 갈 수 있는 최대 거리 [m] (음영 박스 ≈120 m)
GPS_LOST_S = 1.2      # AI 구간 밖에서 GPS 두절로 판단해 추측항법 shaded 모드로 넘기는 시간 [s]


class Master:
    def __init__(self):
        rospy.init_node('master', anonymous=True)

        self.controller = RuleController()
        self.traffic = TrafficLight()

        # AI 구간 제어권 신호 (True = master 가 직접 제어, normal_drive/플래너는 대기)
        self.jamming_mode_pub = rospy.Publisher('/jamming_mode_active', Bool, queue_size=1)
        self.drive_mode_pub = rospy.Publisher('/aisw/drive_mode', String, queue_size=1)
        self.mission_pub = rospy.Publisher('/aisw/mission', String, queue_size=1)
        self.pose_pub = rospy.Publisher('/aisw/ego_pose', PoseStamped, queue_size=1)
        # 회전교차로 [전문가 속도, AI 속도] (m/s) — data_recorder 가 기록해 DAgger 재학습에 쓴다
        self.ai_label_pub = rospy.Publisher('/aisw/ai_label', Float32MultiArray, queue_size=1)
        self.control_rate = rospy.Rate(15)

        # 신호등 제동 래치
        self.braking_started = False
        self.waiting_for_green = False
        self.full_stop_counter = 0
        self.min_stop_hold_frames = int(15 * 0.3)   # 0.3초
        self._green_since = None

        # 구간 설정: config/kcity_sections.yaml (~파라미터가 있으면 우선)
        self.sections = ros_sections()
        self.traffic_zones = rospy.get_param('~traffic_zones', get_section(self.sections, 'traffic_zones', []))
        self.traffic.stop_lines = [{'x': float(p[0]), 'y': float(p[1])}
                                   for p in get_section(self.sections, 'stop_lines', [])]
        self.ai_zones = get_section(self.sections, 'ai_zones', [])
        self.missions = get_section(self.sections, 'missions', [])

        # 자차 상태 (AI 구간 여부와 무관하게 매 주기 갱신)
        self.master_ego_index_global = 0
        self.master_ego_x = 0.0
        self.master_ego_y = 0.0
        self.master_ego_yaw = 0.0
        self.master_ref_path = load_ref_path()
        self.master_indexer = NearestIndexer(self.master_ref_path.cx, self.master_ref_path.cy)
        rospy.loginfo(f'Master ref_path 로드 완료: {self.master_ref_path.length}개 포인트')

        speed_limit = float(get_section(self.sections, 'speed_limit_kph', 55.0)) / 3.6
        self.ai_ctrl = ZoneController(
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
        self.detections = []    # /aisw/obstacles (자차 기준 장애물)
        self._obs_t = 0.0
        self._last_loop_t = None
        rospy.Subscriber('/aisw/obstacles', Detection3DArray, self._obs_cb, queue_size=1)
        rospy.Subscriber('/imu', Imu, self._imu_rate_cb, queue_size=1)

        # 단일 차로 앞차 상대속도(lattice) / 합류 정지(lattice) / 경로상 충돌 앞 속도 상한(frenet)
        rospy.Subscriber('/nearest_vrel', Float32, self._vrel_cb)
        self._vrel_stamp = 0.0
        self.nearest_vrel = float('nan')
        rospy.Subscriber('/merge_stop_flag', UInt8, self._merge_stop_cb)
        self.stop_vrel_flag = None
        rospy.Subscriber('/aisw/speed_cap', Float32, self._planner_cap_cb)
        self._planner_cap = None
        self._planner_cap_t = 0.0

        rospy.loginfo('Master 초기화 완료')

    # ------------------------------------------------------------------ 콜백
    def _planner_cap_cb(self, msg):
        self._planner_cap = float(msg.data)
        self._planner_cap_t = rospy.get_time()

    def _vrel_cb(self, msg):
        self.nearest_vrel = msg.data
        self._vrel_stamp = rospy.get_time()

    def _merge_stop_cb(self, msg):
        self.stop_vrel_flag = int(msg.data)   # 1 = 정지

    def _obs_cb(self, msg):
        self.detections = msg.detections
        self._obs_t = rospy.get_time()

    def _imu_rate_cb(self, msg):
        self.yaw_rate = msg.angular_velocity.z

    # ------------------------------------------------------------------ 자차 상태
    def update_master_ego_status(self):
        """GPS 가 신선하면 GPS, 끊기면 추측항법으로 자차 위치/인덱스를 갱신."""
        c = self.controller
        now = rospy.get_time()
        gps_t = getattr(c, '_last_gps_t', 0.0)
        self.gps_age = (now - gps_t) if gps_t > 0.0 else float('inf')
        self.master_ego_yaw = c.ego_yaw
        # 매 주기 적분 → 새 GPS 가 오면 위치 덮어쓰기 + 속도 보정 갱신
        self.dr.predict(c.ego_vel, math.radians(self.master_ego_yaw), now)
        if gps_t > 0.0 and gps_t != self._last_fix_t and self.gps_age < 0.25:
            self.dr.fix(getattr(c, '_gps_x', c.ego_x), getattr(c, '_gps_y', c.ego_y), now)
            self._last_fix_t = gps_t
        c.speed_scale = self.dr.scale   # normal_drive 의 GPS 사이 위치 예측도 같은 보정 사용
        rospy.loginfo_throttle(10.0, '[Master] 속도 보정 scale=%.3f (샘플 %d)', self.dr.scale, self.dr.scale_samples)

        if self.dr.ready:
            self.master_ego_x, self.master_ego_y = self.dr.x, self.dr.y
            self.master_ego_index_global = self.master_indexer.find(self.master_ego_x, self.master_ego_y)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'map'
            pose.pose.position.x, pose.pose.position.y = self.master_ego_x, self.master_ego_y
            pose.pose.position.z = 0.0 if self.gps_age < 0.5 else 1.0   # z=1: 추측항법 위치
            half = math.radians(self.master_ego_yaw) / 2.0
            pose.pose.orientation.z, pose.pose.orientation.w = math.sin(half), math.cos(half)
            self.pose_pub.publish(pose)

        idx = self.master_ego_index_global
        names = [m['name'] for m in self.missions if int(m['start']) <= idx <= int(m['end'])]
        self.mission_pub.publish(String(','.join(names)))
        rospy.loginfo_throttle(5.0,
            f'[Master] 위치=({self.master_ego_x:.1f}, {self.master_ego_y:.1f}), '
            f'인덱스={idx}, yaw={self.master_ego_yaw:.1f}, GPS age={self.gps_age:.1f}s, 미션={names}')

    # ------------------------------------------------------------------ AI 구간
    def update_ai_zone(self):
        """AI 구간 진입/탈출 판정.

        - ai_zones 의 [enter, end] 안이면 해당 모드
        - 어디서든 GPS 가 GPS_LOST_S 넘게 끊기면 shaded 모드 (normal_drive 의 두절 크리프(1.5초)보다 먼저)
        - shaded 구간 끝을 지나도 GPS 가 복구되기 전까지는 shaded 유지
        """
        idx = self.master_ego_index_global
        zone = None
        if self.dr.ready:
            for z in self.ai_zones:
                if int(z['enter']) <= idx <= int(z['end']):
                    zone = z
                    break
            # shaded 구간 안: 0.5초 / 밖: 1.2초 (부하 시 /gps 6~8 Hz 간격 오탐 방지)
            lost_thr = 0.5 if (zone is not None and zone.get('mode') == 'shaded') else GPS_LOST_S
            if self.gps_age > lost_thr and (zone is None or zone.get('mode') != 'shaded'):
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
                self.braking_started = False
                self.controller.traffic_light_brake = False
        self.active_zone = zone
        return zone

    def ai_zone_control(self, zone):
        """AI 구간 1주기: roundabout_shaded 출력 → /ctrl_cmd 직접 발행."""
        now = rospy.get_time()
        dt = 1.0 / 15 if self._last_loop_t is None else min(max(now - self._last_loop_t, 0.01), 0.2)
        ego_vel = self.controller.ego_vel
        obs_fresh = (now - self._obs_t) < 1.0   # 부하 시 LiDAR 3 Hz
        v, steer, source = self.ai_ctrl.step(
            zone.get('mode', 'shaded'), self.master_ego_index_global,
            self.master_ego_x, self.master_ego_y, math.radians(self.master_ego_yaw),
            ego_vel, self.yaw_rate,
            detections_to_objects(self.detections, math.radians(self.master_ego_yaw)) if obs_fresh else None, dt)
        if zone.get('mode') == 'roundabout':
            self.ai_label_pub.publish(Float32MultiArray(data=[self.ai_ctrl.last['v_label'], self.ai_ctrl.last['v_ai']]))
        if zone.get('mode') == 'shaded' and self.gps_age > 0.5 and self.dr.dist_since_fix > DR_MAX_DIST:
            # 추측항법만으로 너무 멀리 옴 → 위치를 믿을 수 없으니 정지
            v = 0.0
            source += '+dr_limit'
            rospy.logerr_throttle(2.0, '[AI] GPS 없이 %.0f m 추측항법 — 위치 신뢰 불가, 정지', self.dr.dist_since_fix)
        if not obs_fresh:
            # LiDAR 장애물 없이는 통로 감시 불가 → 서행
            v = min(v, 8.0 / 3.6)
            source += '+no_lidar'
            rospy.logwarn_throttle(2.0, '[AI] /aisw/obstacles 1초 이상 끊김 — 8 kph 서행 (LiDAR_perception main.launch 확인)')

        accel_cmd, brake_cmd = self.controller.cmd_converter.run(v, ego_vel)
        self.controller._publish(min(max(accel_cmd, 0.0), 1.0), min(max(brake_cmd, 0.0), 1.0),
                                 steer)   # [rad], 브리지에서 steer_scale 보상
        self.drive_mode_pub.publish(String('AI_%s:%s' % (zone.get('mode'), source)))
        rospy.loginfo_throttle(1.0, '[AI %s] idx=%d v=%.1f→%.1fkph steer=%.1f° src=%s GPS age=%.1fs DR %.0fm',
                               zone.get('name'), self.master_ego_index_global, ego_vel * 3.6, v * 3.6,
                               math.degrees(steer), source, self.gps_age, self.dr.dist_since_fix)

    # ------------------------------------------------------------------ 룰 구간 속도 상한
    def speed_cap_from_vrel(self):
        """합류 정지 → 0, 단일 차로 앞차 → 앞차 속도 - 2.5 kph, 없으면 None. [m/s]"""
        if self.stop_vrel_flag in (1, True):
            return 0.0
        if (rospy.get_time() - self._vrel_stamp) > 0.5 or math.isnan(self.nearest_vrel):
            return None
        v_obs = np.clip(self.controller.ego_vel + self.nearest_vrel, 0.0, 35.0 / 3.6)   # 앞차 절대속도
        return max(0.0, v_obs - 2.5 / 3.6)

    def traffic_light_strategy(self):
        c = self.controller
        speed_kmh = c.ego_vel * 3.6
        idx = self.master_ego_index_global
        in_zone = any(z[0] <= idx <= z[1] for z in self.traffic_zones)   # 비어 있으면 신호등 로직 꺼짐
        if not self.braking_started and not in_zone:
            return None
        distance = self.traffic.distance_to_stopline(c.ego_x, c.ego_y, c.adjusted_yaw)
        state = self.traffic.get_state()
        rospy.loginfo_throttle(2.0, f'[신호등] 위치=({c.ego_x:.1f}, {c.ego_y:.1f}), 신호등={state}, '
                                    f'거리={distance}, 속도={speed_kmh:.1f}km/h, yaw={c.adjusted_yaw:.1f}')
        if distance is None:
            rospy.logwarn_throttle(5.0, '정지선 거리 계산 실패')
            return None
        # 아직 제동 전이면, 제동거리보다 멀 때는 무시
        if not self.braking_started and self.traffic.braking_distance(speed_kmh) < distance:
            return None
        if state == 'unknown':
            return None
        return self.traffic.plan(speed_kmh, distance)

    def apply_traffic_control(self, strategy):
        if strategy is None:
            return
        c = self.controller
        state = self.traffic.get_state()

        # ① 이미 정지 커밋됨: 완전 정지 → 초록불 0.2초 안정 후 해제
        if self.braking_started:
            c.traffic_light_brake = True
            c.velocity = 0.0
            self.full_stop_counter = self.full_stop_counter + 1 if c.ego_vel < 0.1 else 0
            if self.full_stop_counter >= self.min_stop_hold_frames:
                self.waiting_for_green = True
                rospy.loginfo_throttle(1.0, '[Master] 완전 정지 완료 - 초록불 대기 중')
            if self.waiting_for_green and state == 'green':
                if self._green_since is None:
                    self._green_since = rospy.Time.now()
                if rospy.Time.now() - self._green_since >= rospy.Duration(0.2):
                    self.braking_started = False
                    self.waiting_for_green = False
                    self._green_since = None
                    c.traffic_light_brake = False
                    c.external_speed_cap = 5.0 / 3.6   # 출발 시 살살
                    rospy.loginfo('[Master] 초록불 안정화 - 제동 해제하고 출발!')
            else:
                self._green_since = None
            return

        # ② 커밋 전: 이번 주기에 한 번 판단
        if strategy.get('stop_required', False):
            self.braking_started = True
            self.waiting_for_green = False
            self.full_stop_counter = 0
            self._green_since = None
            c.traffic_light_brake = True
            c.velocity = 0.0
            rospy.loginfo(f'[Master] 신호등 제동 커밋: {strategy["reason"]} - 완전 정지까지 계속!')
            return

        # ③ 통과 (노란불이면 감속 상한)
        c.traffic_light_brake = False
        if strategy.get('reason') == 'traffic_light_yellow_decel' and 'target_speed' in strategy:
            c.external_speed_cap = strategy['target_speed'] / 3.6
        else:
            c.external_speed_cap = None

    # ------------------------------------------------------------------ 메인 루프
    def control_loop(self):
        try:
            while not rospy.is_shutdown():
                self.update_master_ego_status()

                zone = self.update_ai_zone()
                self.jamming_mode_pub.publish(Bool(data=zone is not None))
                if zone is not None:
                    self.ai_zone_control(zone)
                    self._last_loop_t = rospy.get_time()
                    self.control_rate.sleep()
                    continue
                self._last_loop_t = rospy.get_time()
                self.drive_mode_pub.publish(String('RULE'))

                # 룰 구간: 신호등 → 속도 상한(앞차/합류/플래너)
                self.apply_traffic_control(self.traffic_light_strategy())
                cap = self.speed_cap_from_vrel()
                if self._planner_cap is not None and rospy.get_time() - self._planner_cap_t < 0.5:
                    cap = self._planner_cap if cap is None else min(cap, self._planner_cap)
                self.controller.external_speed_cap = cap

                self.control_rate.sleep()
        except Exception as e:
            rospy.logerr(f'제어 루프 오류: {e}')

    def run(self):
        rospy.loginfo('Master 시작')
        threading.Thread(target=self.controller.main, daemon=True).start()
        self.control_loop()


if __name__ == '__main__':
    try:
        Master().run()
    except rospy.ROSInterruptException:
        pass
