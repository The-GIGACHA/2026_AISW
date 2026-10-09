#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MORAI Object Info → 장애물 (시뮬 개발용).

  받는 것       MORAI Object Info UDP (기본 7505, '#MoraiObjInfo$') — NPC·보행자·물체의 위치·크기·속도 정답
  내보내는 토픽 /aisw/obstacles (vision_msgs/Detection3DArray, 자차 기준) — lidar_clusters 와 같은 형식

LiDAR 쪽에서 장애물 속도까지 보내 주기 전까지 판단·제어를 시뮬에서 돌리기 위한 노드.
Object Info 는 대회 허용 입력이 아니므로 본선에서는 쓰지 않는다 (launch obstacles:=lidar).

패킷: 헤더 14 + 길이 4 + aux 12 + 데이터(타임스탬프 8 + 객체 106 B x 20) + 꼬리 2
객체: id int16, type int16(-1 자차, 0 보행자, 1 차량, 2 물체), 위치 xyz f32[m], heading f32[deg],
      크기 xyz f32[m], overhang·wheelbase·rear overhang f32, 속도 xyz f32[km/h], 가속도 xyz f32, link id 38 B
"""
import math
import socket
import struct

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

HEADER = b'#MoraiObjInfo$'
OBJ_SIZE = 106
N_OBJ = 20
POSE_MAX_AGE = 0.5      # 자차 자세가 이보다 오래되면 발행하지 않음 [s]


def parse(raw):
    """→ [(id, type, x, y, heading_rad, length, width, height, vx, vy)] — vx, vy 는 map 기준 [m/s]."""
    if not raw.startswith(HEADER):
        return []
    n = struct.unpack_from('<i', raw, 14)[0]
    data = raw[30:30 + n]
    out = []
    for k in range(N_OBJ):
        o = 8 + OBJ_SIZE * k
        if o + OBJ_SIZE > len(data):
            break
        oid, typ = struct.unpack_from('<hh', data, o)
        px, py, _, hd, sx, sy, sz, _, _, _, vx, vy, _ = struct.unpack_from('<13f', data, o + 4)
        if typ == -1 or (oid == 0 and typ == 0 and px == 0.0 and py == 0.0):
            continue                     # 자차 / 빈 칸
        out.append((oid, typ, px, py, math.radians(hd), sx, sy, sz, vx / 3.6, vy / 3.6))
    return out


class MoraiObjects:
    def __init__(self):
        rospy.init_node('morai_objects')
        self.port = int(rospy.get_param('~port', 7505))
        # 속도 기준: local = 객체 자신의 진행방향 기준(x 전방) → heading 으로 map 기준으로 돌림 / world = 이미 map 기준
        self.vel_frame = rospy.get_param('~vel_frame', 'local')
        self.range_x = (float(rospy.get_param('~min_x', -5.0)), float(rospy.get_param('~max_x', 40.0)))
        self.range_y = float(rospy.get_param('~max_y', 25.0))
        self.pose = None
        self.pose_t = 0.0
        self.pub = rospy.Publisher('/aisw/obstacles', Detection3DArray, queue_size=1)
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._pose_cb, queue_size=1)

    def _pose_cb(self, m):
        yaw = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
        x, y, now = m.pose.position.x, m.pose.position.y, rospy.get_time()
        # 자차 속도 (map 기준) — 상대속도 계산용
        if self.pose is not None and now - self.pose_t > 0.05:
            dt = now - self.pose_t
            self.ego_v = (math.hypot(x - self.pose[0], y - self.pose[1]) / dt)
        self.pose = (x, y, yaw)
        self.pose_t = now

    def run(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(('0.0.0.0', self.port))
        s.settimeout(1.0)
        rospy.loginfo('[morai_objects] Object Info UDP %d 수신 (속도 기준 %s)', self.port, self.vel_frame)
        while not rospy.is_shutdown():
            try:
                raw = s.recv(65535)
            except socket.timeout:
                continue
            if self.pose is None or rospy.get_time() - self.pose_t > POSE_MAX_AGE:
                continue
            self.publish(parse(raw))

    def publish(self, objs):
        x0, y0, yaw = self.pose
        c, s = math.cos(yaw), math.sin(yaw)
        ego_v = getattr(self, 'ego_v', 0.0)
        out = Detection3DArray()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = 'ego'
        for oid, typ, px, py, hd, sx, sy, sz, vx, vy in objs:
            dx, dy = px - x0, py - y0
            ex, ey = c * dx + s * dy, -s * dx + c * dy
            if not (self.range_x[0] < ex < self.range_x[1] and abs(ey) < self.range_y):
                continue
            if self.vel_frame == 'local':
                vx, vy = vx * math.cos(hd) - vy * math.sin(hd), vx * math.sin(hd) + vy * math.cos(hd)
            rel_yaw = hd - yaw
            det = Detection3D()
            det.bbox.center.position.x = ex
            det.bbox.center.position.y = ey
            det.bbox.center.orientation.z = math.sin(rel_yaw / 2.0)
            det.bbox.center.orientation.w = math.cos(rel_yaw / 2.0)
            det.bbox.size.x, det.bbox.size.y, det.bbox.size.z = max(sx, 0.3), max(sy, 0.3), max(sz, 0.3)
            h = ObjectHypothesisWithPose()
            h.id = oid
            h.score = 1.0
            det.results.append(h)
            det.source_cloud = PointCloud2()
            det.source_cloud.data = struct.pack('ffff', vx * c + vy * s - ego_v, vx, vy, 1.0)
            out.detections.append(det)
        self.pub.publish(out)
        rospy.loginfo_throttle(2.0, '[morai_objects] 장애물 %d개', len(out.detections))


if __name__ == '__main__':
    MoraiObjects().run()
