#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 도로 경계 지도 기록: /aisw/road_edges([좌, 우] 자차 기준 횡거리) + /aisw/ego_pose
→ 전역경로 인덱스별 좌/우 경계 거리 CSV. 여러 바퀴 돌린 뒤 tools/build_edge_map.py 로 지도화.

  rosrun 없이: python3 tools/edge_logger.py   (~/aisw_logs/edges_YYYYmmdd_HHMMSS.csv)
"""
import csv, math, os, sys, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from aisw_common import DEFAULT_MAP, NearestIndexer, load_map_fields

rx, ry = load_map_fields(DEFAULT_MAP)[:2]
idxr = NearestIndexer(rx, ry)
state = {'pose': None}
path = os.path.expanduser(time.strftime('~/aisw_logs/edges_%Y%m%d_%H%M%S.csv'))
f = open(path, 'w', newline=''); w = csv.writer(f); w.writerow(['t', 'idx', 'x', 'y', 'yaw', 'left', 'right', 'path_off'])


def pose_cb(m):
    state['pose'] = (m.pose.position.x, m.pose.position.y, 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w))


def edge_cb(m):
    p = state['pose']
    if p is None:
        return
    i = idxr.find(p[0], p[1])
    j = min(i + 1, len(rx) - 1)
    yaw = math.atan2(ry[j] - ry[i], rx[j] - rx[i])
    off = -math.sin(yaw) * (p[0] - rx[i]) + math.cos(yaw) * (p[1] - ry[i])   # 자차의 경로 기준 횡편차(좌+)
    w.writerow([round(rospy.get_time(), 3), i, round(p[0], 2), round(p[1], 2), round(p[2], 3),
                m.data[0], m.data[1], round(off, 3)])


rospy.init_node('aisw_edge_logger', anonymous=True)
rospy.Subscriber('/aisw/ego_pose', PoseStamped, pose_cb, queue_size=1)
rospy.Subscriber('/aisw/road_edges', Float32MultiArray, edge_cb, queue_size=5)
rospy.loginfo('[edge_logger] %s', path)
rospy.on_shutdown(f.close)
rospy.spin()
