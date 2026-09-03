#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MORAI /Object_topic (ground-truth NPC/보행자/장애물) → /tracked_objects_3d (Detection3DArray) 브릿지.

lattice_planner_v2 / obstacle_planner는 LiDAR 인지 결과(/tracked_objects_3d, ego 로컬좌표)를
기대하지만, 시뮬 단독 실행 시 인지 스택이 없으므로 시뮬의 참값 객체 정보를 같은 형식으로 변환해준다.
LiDAR 인지 스택을 실제로 돌릴 때는 이 노드를 끄면 된다.
"""
import math

import rospy
from morai_msgs.msg import GPSMessage, ObjectStatusList
from sensor_msgs.msg import Imu
from vision_msgs.msg import Detection3D, Detection3DArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from pyproj import Proj

MAX_RANGE = 60.0    # ego로부터 이 거리[m] 이내의 객체만 전달


class ObstacleBridge:
    def __init__(self):
        rospy.init_node('obstacle_bridge', anonymous=True)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self.ego_x = None
        self.ego_y = None
        self.ego_yaw = None   # [deg]

        self.pub = rospy.Publisher('/tracked_objects_3d', Detection3DArray, queue_size=1)
        rospy.Subscriber('/gps', GPSMessage, self.gps_cb)
        rospy.Subscriber('/imu', Imu, self.imu_cb)
        rospy.Subscriber('/Object_topic', ObjectStatusList, self.obj_cb)

        rospy.loginfo("[ObstacleBridge] /Object_topic -> /tracked_objects_3d 변환 시작")

    def gps_cb(self, msg):
        ux, uy = self.proj_UTM(msg.longitude, msg.latitude)
        self.ego_x = ux - msg.eastOffset
        self.ego_y = uy - msg.northOffset

    def imu_cb(self, msg):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.ego_yaw = math.degrees(yaw)

    def obj_cb(self, msg):
        if self.ego_x is None or self.ego_yaw is None:
            return

        out = Detection3DArray()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = 'ego'

        c = math.cos(math.radians(self.ego_yaw))
        s = math.sin(math.radians(self.ego_yaw))

        for obj in list(msg.npc_list) + list(msg.pedestrian_list) + list(msg.obstacle_list):
            dx = obj.position.x - self.ego_x
            dy = obj.position.y - self.ego_y
            if math.hypot(dx, dy) > MAX_RANGE:
                continue
            # 월드 → ego 로컬 (x: 전방, y: 좌측)
            lx = c * dx + s * dy
            ly = -s * dx + c * dy

            det = Detection3D()
            det.header = out.header
            det.bbox.center.position.x = lx
            det.bbox.center.position.y = ly
            det.bbox.center.position.z = 0.0
            yaw_rel = math.radians(obj.heading - self.ego_yaw)
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw_rel)
            det.bbox.center.orientation.x = qx
            det.bbox.center.orientation.y = qy
            det.bbox.center.orientation.z = qz
            det.bbox.center.orientation.w = qw
            det.bbox.size.x = max(obj.size.x, 0.5)
            det.bbox.size.y = max(obj.size.y, 0.5)
            det.bbox.size.z = max(obj.size.z, 0.5)
            out.detections.append(det)

        self.pub.publish(out)


if __name__ == '__main__':
    ObstacleBridge()
    rospy.spin()
