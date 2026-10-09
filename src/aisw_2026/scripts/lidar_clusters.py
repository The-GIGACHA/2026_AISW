#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""LiDAR 클러스터 → 장애물.

  받는 토픽     /Clustered_cloud  (sensor_msgs/PointCloud2, map 좌표)
                — 26aisw_lidar_ws 의 LiDAR_perception main.launch 가 내보내는 클러스터 점
  내보내는 토픽 /aisw/obstacles   (vision_msgs/Detection3DArray, 자차 기준)
                — lattice/frenet 플래너, master 가 읽는 장애물 목록

점들을 물체 단위로 묶어 자차(후륜축) 기준 박스로 바꾼다. 속도는 계산하지 않는다(LiDAR 쪽에서 받을 예정, 지금은 0).
/aisw/obstacles 형식 (frame 'ego'):
  bbox.center/size = 자차(후륜축) 기준 좌표 [m], results[0].id = 물체 번호
  source_cloud.data = float32 [vx_rel, vx, vy(map), 확정(1/0)]
자차 자세: master 의 /aisw/ego_pose
"""
import math
import struct

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose

GRID = 0.5              # 클러스터 분리 격자 [m] (이웃 칸이 이어지면 같은 물체)
MIN_PTS = 3             # 물체로 보는 최소 점 수
MAX_EXTENT = 8.0        # 이보다 긴 덩어리(벽·가드레일)는 제외 [m]
WINDOW_X = (0.5, 25.0)  # 자차 기준 전방 범위 [m]
WINDOW_Y = 4.0          # 자차 기준 측면 범위 [m]
POSE_MAX_AGE = 0.5      # 자차 자세가 이보다 오래되면 변환하지 않음 [s]


class LidarClusters:
    def __init__(self):
        rospy.init_node('lidar_clusters')
        self.pose = None
        self.pose_t = 0.0
        self.pub = rospy.Publisher('/aisw/obstacles', Detection3DArray, queue_size=1)
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._pose_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param('~cloud_topic', '/Clustered_cloud'), PointCloud2, self._cloud_cb,
                         queue_size=1, buff_size=2 ** 24)

    def _pose_cb(self, m):
        yaw = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)
        self.pose_t = rospy.get_time()

    def _cloud_cb(self, msg):
        out = Detection3DArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'ego'
        if self.pose is None or rospy.get_time() - self.pose_t > POSE_MAX_AGE:
            self.pub.publish(out)
            return
        pts = np.array(list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)), np.float64)
        if len(pts):
            # map → 자차(후륜축) 좌표
            x0, y0, yaw = self.pose
            c, s = math.cos(yaw), math.sin(yaw)
            dx, dy = pts[:, 0] - x0, pts[:, 1] - y0
            ego = np.column_stack([c * dx + s * dy, -s * dx + c * dy, pts[:, 2]])
            for sel in self._group(ego):
                lo, hi = sel.min(0), sel.max(0)
                if hi[0] - lo[0] > MAX_EXTENT or hi[1] - lo[1] > MAX_EXTENT:
                    continue
                cx, cy = float((lo[0] + hi[0]) / 2), float((lo[1] + hi[1]) / 2)
                if not (WINDOW_X[0] < cx < WINDOW_X[1] and abs(cy) < WINDOW_Y):
                    continue
                det = Detection3D()
                det.bbox.center.position.x = cx
                det.bbox.center.position.y = cy
                det.bbox.center.orientation.w = 1.0
                det.bbox.size.x = float(max(hi[0] - lo[0], 0.3))
                det.bbox.size.y = float(max(hi[1] - lo[1], 0.3))
                det.bbox.size.z = float(max(hi[2] - lo[2], 0.3))
                h = ObjectHypothesisWithPose()
                h.id = len(out.detections)
                h.score = 1.0
                det.results.append(h)
                det.source_cloud = PointCloud2()
                det.source_cloud.data = struct.pack('ffff', 0.0, 0.0, 0.0, 1.0)
                out.detections.append(det)
        self.pub.publish(out)
        rospy.loginfo_throttle(2.0, '[lidar_clusters] 장애물 %d개', len(out.detections))

    @staticmethod
    def _group(pts):
        """XY 격자 연결요소로 점을 물체별로 나눈다."""
        ij = np.floor(pts[:, :2] / GRID).astype(np.int64)
        keys = ij[:, 0] * 100000 + ij[:, 1]
        order = np.argsort(keys)
        keys, pts = keys[order], pts[order]
        uk, start = np.unique(keys, return_index=True)
        cells = {k: (start[n], start[n + 1] if n + 1 < len(uk) else len(keys)) for n, k in enumerate(uk)}
        seen = set()
        for k in uk:
            if k in seen:
                continue
            stack, comp = [k], []
            while stack:
                cell = stack.pop()
                if cell in seen or cell not in cells:
                    continue
                seen.add(cell)
                comp.append(cell)
                ci, cj = cell // 100000, cell % 100000
                for di in (-1, 0, 1):
                    for dj in (-1, 0, 1):
                        nb = (ci + di) * 100000 + (cj + dj)
                        if nb in cells and nb not in seen:
                            stack.append(nb)
            sel = np.concatenate([pts[cells[cc][0]:cells[cc][1]] for cc in comp])
            if len(sel) >= MIN_PTS:
                yield sel


if __name__ == '__main__':
    LidarClusters()
    rospy.spin()
