#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""경로계획 노드 (Frenet 샘플링, control/frenet_core.py).

입력: /aisw/ego_pose (master 융합 위치·방향), /aisw/obstacles (자차 기준 장애물 + 속도),
      /jamming_mode_active (AI 구간이면 대기)
출력: /local_path, /planner_mode, /aisw/speed_cap (경로상 충돌·양보 지점 앞 속도 상한 [m/s], 제한 없으면 미발행)
"""
import math
import struct
import time

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Float32, UInt8
from vision_msgs.msg import Detection3DArray

from control.path_utils import DEFAULT_MAP, load_map_fields
from control.frenet_core import FrenetPlanner, RefPath


HOLD_S = 2.0   # 장애물 기억 시간 [s]


class Node:
    def __init__(self):
        rospy.init_node('frenet_planner')
        rx, ry = load_map_fields(DEFAULT_MAP)[:2]
        self.planner = FrenetPlanner(RefPath(rx, ry))
        self.pose = None; self.pose_t = 0.0; self.prev = None; self.ego_v = 0.0
        self.dets = []; self.dets_t = 0.0; self.mem = []
        self.ai_active = False
        self.path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.mode_pub = rospy.Publisher('/planner_mode', UInt8, queue_size=1, latch=True)
        self.cap_pub = rospy.Publisher('/aisw/speed_cap', Float32, queue_size=1)
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._pose, queue_size=1)
        rospy.Subscriber('/aisw/obstacles', Detection3DArray, self._objs, queue_size=1)
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
            conf = 1.0
            if len(d.source_cloud.data) == 16:
                _, vx, vy, conf = struct.unpack('ffff', d.source_cloud.data)
            out.append((x + c * lx - s * ly, y + s * lx + c * ly, vx, vy,
                        d.bbox.size.x / 2.0, d.bbox.size.y / 2.0, conf > 0.5))
        # 장애물 기억: LiDAR 3 Hz 깜빡임/근접 시 소실로 정지 상한이 풀려 장애물로 굴러가지 않게.
        # 이번 감지와 1.5 m 이내 겹치는 기억은 새 값으로 대체, 나머지는 HOLD_S 동안 유지(월드 좌표라 정지 물체에 정확).
        now = rospy.get_time()
        keep = [(t, o) for (t, o) in self.mem if now - t <= HOLD_S
                and all(math.hypot(o[0] - n[0], o[1] - n[1]) > 1.5 for n in out)]
        self.mem = keep + [(now, o) for o in out]
        self.dets = [o for _, o in self.mem]; self.dets_t = now

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
            objs = [o for t, o in self.mem if now - t <= HOLD_S]
            t0 = time.time()
            res = self.planner.plan(*self.pose, self.ego_v, objs)
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
