#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] RViz 시각화 도우미 (주행에는 영향 없음).

- TF: map → base_link(후륜축, /aisw/ego_pose), base_link → lidar(x 0.58, z 1.55) → LiDAR 스캔을 RViz 에 표시
- /tracked_objects_3d → /aisw/viz/objects (MarkerArray): 정지=회색, 이동=주황 상자 + "id 속도" 글자
- 자차 상자 /aisw/viz/ego, 현재 모드·미션·속도상한 글자 /aisw/viz/status

  python3 tools/viz_node.py   그리고   rviz -d config/aisw.rviz
"""
import math
import os
import struct
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'scripts'))
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

LIDAR_X, LIDAR_Z = 0.58, 1.55
st = {'mode': '', 'mission': '', 'cap': float('inf'), 'cap_t': 0.0}


def tfmsg(parent, child, x, y, z, yaw):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now(); t.header.frame_id = parent; t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
    t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return t


def on_pose(m):
    yaw = 2 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
    br.sendTransform([tfmsg('map', 'base_link', m.pose.position.x, m.pose.position.y, 0.0, yaw),
                      tfmsg('base_link', 'lidar', LIDAR_X, 0.0, LIDAR_Z, 0.0),
                      tfmsg('base_link', 'ego', 0.0, 0.0, 0.0, 0.0)])
    ego = Marker(); ego.header.frame_id = 'base_link'; ego.header.stamp = rospy.Time.now()
    ego.ns, ego.id, ego.type = 'ego', 0, Marker.CUBE
    ego.pose.position.x, ego.pose.position.z = (3.845 - 0.79) / 2, 0.8; ego.pose.orientation.w = 1.0
    ego.scale.x, ego.scale.y, ego.scale.z = 4.635, 1.892, 1.5
    ego.color.r, ego.color.g, ego.color.b, ego.color.a = 0.1, 0.5, 1.0, 0.8
    txt = Marker(); txt.header = ego.header; txt.ns, txt.id, txt.type = 'status', 1, Marker.TEXT_VIEW_FACING
    txt.pose.position.z = 4.0; txt.pose.orientation.w = 1.0; txt.scale.z = 1.2
    txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
    cap = st['cap'] if rospy.get_time() - st['cap_t'] < 0.5 else float('inf')
    txt.text = '%s | %s | cap %s' % (st['mode'], st['mission'] or '-', 'none' if not math.isfinite(cap) else '%.0f kph' % (cap * 3.6))
    ego_pub.publish(MarkerArray(markers=[ego, txt]))


def on_objs(msg):
    arr = MarkerArray()
    clr = Marker(); clr.action = Marker.DELETEALL; arr.markers.append(clr)
    for k, d in enumerate(msg.detections):
        vx = vy = 0.0; conf = 1.0
        if len(d.source_cloud.data) == 16:
            _, vx, vy, conf = struct.unpack('ffff', d.source_cloud.data)
        sp = math.hypot(vx, vy)
        b = Marker(); b.header.frame_id = 'base_link'; b.header.stamp = rospy.Time.now()
        b.ns, b.id, b.type = 'obj', 2 * k, Marker.CUBE
        b.pose = d.bbox.center; b.pose.position.z = d.bbox.size.z / 2
        b.scale.x, b.scale.y, b.scale.z = max(d.bbox.size.x, 0.2), max(d.bbox.size.y, 0.2), max(d.bbox.size.z, 0.2)
        moving = sp > 1.2
        b.color.r, b.color.g, b.color.b = (1.0, 0.5, 0.0) if moving else (0.7, 0.7, 0.7)
        b.color.a = 0.9 if conf > 0.5 else 0.35            # 미확정 트랙은 흐리게
        b.lifetime = rospy.Duration(0.6)
        t = Marker(); t.header = b.header; t.ns, t.id, t.type = 'obj_txt', 2 * k + 1, Marker.TEXT_VIEW_FACING
        t.pose.position.x, t.pose.position.y, t.pose.position.z = d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size.z + 0.8
        t.pose.orientation.w = 1.0; t.scale.z = 0.7; t.color = b.color; t.color.a = 1.0
        t.text = '#%d %.1fm/s' % (d.results[0].id if d.results else -1, sp); t.lifetime = rospy.Duration(0.6)
        arr.markers += [b, t]
    obj_pub.publish(arr)


rospy.init_node('aisw_viz')
br = tf2_ros.TransformBroadcaster()
ego_pub = rospy.Publisher('/aisw/viz/ego', MarkerArray, queue_size=1)
obj_pub = rospy.Publisher('/aisw/viz/objects', MarkerArray, queue_size=1)
rospy.Subscriber('/aisw/ego_pose', PoseStamped, on_pose, queue_size=1)
rospy.Subscriber('/tracked_objects_3d', Detection3DArray, on_objs, queue_size=1)
rospy.Subscriber('/aisw/drive_mode', String, lambda m: st.update(mode=m.data), queue_size=1)
rospy.Subscriber('/aisw/mission', String, lambda m: st.update(mission=m.data), queue_size=1)
rospy.Subscriber('/aisw/speed_cap', Float32, lambda m: st.update(cap=m.data, cap_t=rospy.get_time()), queue_size=1)
rospy.spin()
