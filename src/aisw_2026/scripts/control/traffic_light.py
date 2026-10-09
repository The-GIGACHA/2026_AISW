# -*- coding: utf-8 -*-
"""신호등 판단: /traffic_light_{red,yellow,green} + 정지선 거리 + 제동거리 → 정지/감속/통과.

master.py 가 traffic_zones(config) 안에서만 쓴다. 신호등 인지 노드와 정지선 좌표(config stop_lines)가
아직 없어서 현재는 동작하지 않는다.
"""
import math

import rospy
from std_msgs.msg import Bool


class TrafficLight:
    def __init__(self):
        self.red = False
        self.yellow = False
        self.green = False

        # 실측 제동거리 [kph → m]
        self.deceleration_rate = 8.516  # m/s^2
        self.speed_distance_map = {49: 14.5, 40: 10.5, 30: 7, 20: 4.2}

        self.stop_lines = []   # [{'x':, 'y':}, ...] — master 가 config stop_lines 로 채운다

        rospy.Subscriber('/traffic_light_red', Bool, self._red_cb)
        rospy.Subscriber('/traffic_light_yellow', Bool, self._yellow_cb)
        rospy.Subscriber('/traffic_light_green', Bool, self._green_cb)

    def _red_cb(self, msg):
        self.red = msg.data
        if msg.data:
            rospy.loginfo('빨간불 신호 감지')

    def _yellow_cb(self, msg):
        self.yellow = msg.data
        if msg.data:
            rospy.loginfo('노란불 신호 감지')

    def _green_cb(self, msg):
        self.green = msg.data
        if msg.data:
            rospy.loginfo('초록불 신호 감지')

    def get_state(self):
        state = 'red' if self.red else 'yellow' if self.yellow else 'green' if self.green else 'unknown'
        rospy.loginfo_throttle(2.0, f'[TrafficLight] Red={self.red}, Yellow={self.yellow}, Green={self.green} → {state}')
        return state

    def braking_distance(self, speed_kmh):
        """현재 속도에서 정지까지 필요한 거리 [m] (실측표 선형보간, 범위 밖은 물리식)."""
        if speed_kmh in self.speed_distance_map:
            return self.speed_distance_map[speed_kmh]
        speeds = sorted(self.speed_distance_map)
        if speeds[0] <= speed_kmh <= speeds[-1]:
            for a, b in zip(speeds[:-1], speeds[1:]):
                if a <= speed_kmh <= b:
                    da, db = self.speed_distance_map[a], self.speed_distance_map[b]
                    return da + (speed_kmh - a) / (b - a) * (db - da)
        v = speed_kmh / 3.6
        return 0.0 if v <= 0 else v * v / (2 * self.deceleration_rate) * 1.1   # 10% 여유

    def distance_to_stopline(self, ego_x, ego_y, ego_yaw_deg):
        """앞바퀴 기준, 가장 가까운 정지선까지 주행 방향 거리 [m]. 없거나 이미 지났으면 None."""
        if not self.stop_lines:
            return None
        yaw = math.radians(ego_yaw_deg)
        fx, fy = ego_x + 3.0 * math.cos(yaw), ego_y + 3.0 * math.sin(yaw)   # 후륜(GPS) → 앞바퀴 3 m
        line = min(self.stop_lines, key=lambda p: math.hypot(p['x'] - fx, p['y'] - fy))
        along = (line['x'] - fx) * math.cos(yaw) + (line['y'] - fy) * math.sin(yaw)
        if along <= 0:
            rospy.loginfo_throttle(2.0, f'[TrafficLight] 정지선 이미 지남: {along:.1f}m')
            return None
        rospy.loginfo_throttle(2.0, f'[TrafficLight] 정지선까지 거리: {along:.1f}m')
        return along

    def plan(self, speed_kmh, distance):
        """→ {'stop_required', 'target_speed'[kph], 'reason'}"""
        strategy = {'stop_required': False, 'target_speed': speed_kmh, 'reason': ''}
        state = self.get_state()
        if state in ('green', 'unknown'):
            return strategy

        if state == 'red':
            if distance <= 0:   # 앞바퀴가 이미 정지선을 넘음 → 통과
                strategy['reason'] = 'red_light_front_wheel_passed'
                return strategy
            need = self.braking_distance(speed_kmh)
            rospy.loginfo_throttle(1.0, f'[TrafficLight] 빨간불: 거리={distance:.1f}m, 필요제동거리={need:.1f}m, '
                                        f'속도={speed_kmh:.1f}km/h → {"제동" if distance <= need else "통과"}')
            if distance <= need:
                strategy.update(stop_required=True, target_speed=0.0, reason='traffic_light_red')
                rospy.loginfo('[TrafficLight] 빨간불 제동 결정!')

        elif state == 'yellow':
            # 3초 안에 정지선 통과 가능하면 통과, 아니면 감속
            if speed_kmh > 0 and distance / (speed_kmh / 3.6) < 3.0:
                strategy['reason'] = 'yellow_light_pass'
                rospy.loginfo(f'노란불 통과: 거리={distance:.1f}m')
            else:
                strategy['target_speed'] = min(speed_kmh * 0.6, 20.0)
                strategy['reason'] = 'traffic_light_yellow_decel'
                rospy.loginfo(f'노란불 감속: 거리={distance:.1f}m, 목표속도={strategy["target_speed"]:.1f}km/h')
        return strategy
