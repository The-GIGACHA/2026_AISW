#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] Frenet 샘플링 플래너 노드 — lattice_planner_v2 대체(같은 출력 토픽).

입력: /aisw/ego_pose (master 융합 위치·방향), /tracked_objects_3d (자차 좌표 + 추적 속도),
      /aisw/road_edges (LiDAR 도로 경계), /jamming_mode_active (AI 구간이면 대기)
출력: /local_path, /planner_mode, /aisw/speed_cap (경로상 충돌 지점 앞 정지 속도 [m/s], 제한 없으면 미발행)
실행: roslaunch aisw_2026 aisw_midterm.launch planner:=frenet
"""
import math
import struct
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, Float32MultiArray, UInt8
from vision_msgs.msg import Detection3DArray

from aisw_common import DEFAULT_MAP, load_map_fields
from planner.frenet_planner import FrenetPlanner, RefPath


class Node:
    def __init__(self):
        rospy.init_node('lattice_planner')   # 기존 노드 이름 유지 (launch/모니터 호환)
        rx, ry = load_map_fields(DEFAULT_MAP)[:2]
        self.planner = FrenetPlanner(RefPath(rx, ry))
        self.pose = None; self.pose_t = 0.0; self.prev = None; self.ego_v = 0.0
        self.dets = []; self.dets_t = 0.0
        self.edges = (float('nan'), float('nan'))
        self.ai_active = False
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.mode_pub = rospy.Publisher('/planner_mode', UInt8, queue_size=1, latch=True)
        self.cap_pub = rospy.Publisher('/aisw/speed_cap', Float32, queue_size=1)
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._pose, queue_size=1)
        rospy.Subscriber('/tracked_objects_3d', Detection3DArray, self._objs, queue_size=1)
        rospy.Subscriber('/aisw/road_edges', Float32MultiArray, self._edges, queue_size=1)
        rospy.Subscriber('/jamming_mode_active', Bool, self._ai, queue_size=1)
        self.mode_pub.publish(UInt8(0))

    def _pose(self, m):
        now = rospy.get_time()
        x, y = m.pose.position.x, m.pose.position.y
        yaw = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
        if self.prev is not None and now - self.prev[2] > 0.05:
            v = math.hypot(x - self.prev[0], y - self.prev[1]) / (now - self.prev[2])
            if v < 40.0:
                self.ego_v = 0.7 * self.ego_v + 0.3 * v
        self.prev = (x, y, now); self.pose = (x, y, yaw); self.pose_t = now

    def _objs(self, msg):
        if self.pose is None:
            return
        x, y, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for d in msg.detections:
            lx, ly = d.bbox.center.position.x, d.bbox.center.position.y
            vx = vy = 0.0
            if len(d.source_cloud.data) == 16:
                _, vx, vy, _ = struct.unpack('ffff', d.source_cloud.data)
            out.append((x + c * lx - s * ly, y + s * lx + c * ly, vx, vy,
                        d.bbox.size.x / 2.0, d.bbox.size.y / 2.0))
        self.dets = out; self.dets_t = rospy.get_time()

    def _edges(self, m):
        if len(m.data) >= 2:
            self.edges = (m.data[0], m.data[1])

    def _ai(self, m):
        self.ai_active = m.data

    def run(self):
        rate = rospy.Rate(10)
        mode_prev = None
        while not rospy.is_shutdown():
            rate.sleep()
            now = rospy.get_time()
            if self.ai_active or self.pose is None or now - self.pose_t > 0.5:
                continue
            objs = self.dets if now - self.dets_t < 1.0 else []
            t0 = time.time()
            res = self.planner.plan(*self.pose, self.ego_v, objs, self.edges)
            dt_ms = (time.time() - t0) * 1000
            path = Path(); path.header.stamp = rospy.Time.now(); path.header.frame_id = 'map'
            for px, py in zip(res['x'], res['y']):
                ps = PoseStamped(); ps.header = path.header
                ps.pose.position.x, ps.pose.position.y = float(px), float(py)
                ps.pose.orientation.w = 1.0
                path.poses.append(ps)
            self.path_pub.publish(path)
            if res['mode'] != mode_prev:
                self.mode_pub.publish(UInt8(res['mode'])); mode_prev = res['mode']
            if math.isfinite(res['speed_cap']):
                self.cap_pub.publish(Float32(res['speed_cap']))
            info = self.planner.last
            rospy.loginfo_throttle(1.0, '[frenet] s=%.0f d0=%+.2f dT=%+.2f L=%.0f obj=%d hit=%s cap=%s %.0fms',
                                   info['s0'], info['d0'], info['dT'], info['L'], info['n_obj'],
                                   'none' if info['s_hit'] is None else '%.1f' % info['s_hit'],
                                   'inf' if not math.isfinite(res['speed_cap']) else '%.1f' % res['speed_cap'], dt_ms)


if __name__ == '__main__':
    Node().run()
