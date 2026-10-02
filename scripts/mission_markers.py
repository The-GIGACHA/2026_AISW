#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 규정집 미션 위치를 RViz 마커로 표시 (config/kcity_sections.yaml 의 missions / ai_zones).

발행: /aisw/mission_markers (visualization_msgs/MarkerArray, latch, frame 'map' = 지역 ENU 좌표)
  - 전역경로: 회색 선
  - 미션 구간: 종류별 색 굵은 선 + "이름 [시작-끝]" 글자 (estimate 는 끝에 '?')
  - AI 구간: 보라 반투명 띠
  - 100 인덱스마다 번호
RViz: Fixed Frame = map, Add → By topic → /aisw/mission_markers
"""
import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

from aisw_common import DEFAULT_MAP, load_map_fields, ros_sections, get_section

TYPE_COLOR = {
    'checkpoint':   (0.1, 0.9, 0.1),
    'lane_keep':    (0.6, 0.6, 0.0),
    'obstacle':     (1.0, 0.2, 0.2),
    'intersection': (1.0, 0.6, 0.0),
    'roundabout':   (0.2, 0.6, 1.0),
    'merge':        (0.9, 0.2, 0.9),
    'speed_limit':  (0.0, 0.9, 0.9),
    'gps_shaded':   (0.3, 0.3, 0.3),
}


def _marker(mid, ns, mtype, rgb, alpha, scale):
    m = Marker()
    m.header.frame_id = 'map'
    m.header.stamp = rospy.Time.now()
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = scale
    m.color.r, m.color.g, m.color.b = rgb
    m.color.a = alpha
    return m


def _segment(rx, ry, a, b, z):
    a, b = max(0, int(a)), min(len(rx) - 1, int(b))
    if a == b:   # 한 점짜리(체크포인트)는 앞뒤로 조금 늘려 보이게
        a, b = max(0, a - 4), min(len(rx) - 1, b + 4)
    return [Point(x=rx[i], y=ry[i], z=z) for i in range(a, b + 1)]


def build(rx, ry, sections):
    arr = MarkerArray()
    mid = 0

    path = _marker(mid, 'path', Marker.LINE_STRIP, (0.7, 0.7, 0.7), 0.8, 0.4)
    path.points = [Point(x=x, y=y, z=0.0) for x, y in zip(rx[::2], ry[::2])]
    arr.markers.append(path); mid += 1

    for i in range(0, len(rx), 100):
        t = _marker(mid, 'index', Marker.TEXT_VIEW_FACING, (0.8, 0.8, 1.0), 0.9, 2.5)
        t.pose.position = Point(x=rx[i], y=ry[i], z=2.0)
        t.text = str(i)
        arr.markers.append(t); mid += 1

    for z in get_section(sections, 'ai_zones', []):
        band = _marker(mid, 'ai_zone', Marker.LINE_STRIP, (0.6, 0.2, 1.0), 0.35, 6.0)
        band.points = _segment(rx, ry, z['enter'], z['end'], -0.1)
        arr.markers.append(band); mid += 1

    for m in get_section(sections, 'missions', []):
        rgb = TYPE_COLOR.get(m.get('type'), (1.0, 1.0, 1.0))
        line = _marker(mid, 'mission', Marker.LINE_STRIP, rgb, 1.0, 1.5)
        line.points = _segment(rx, ry, m['start'], m['end'], 0.3 + 0.2 * (mid % 3))
        arr.markers.append(line); mid += 1

        label = _marker(mid, 'mission_label', Marker.TEXT_VIEW_FACING, rgb, 1.0, 3.0)
        i = int(m['start'])
        label.pose.position = Point(x=rx[i], y=ry[i], z=6.0)
        label.text = '%s [%d-%d]%s' % (m['name'], m['start'], m['end'],
                                       '?' if m.get('source') == 'estimate' else '')
        arr.markers.append(label); mid += 1
    return arr


def main():
    rospy.init_node('aisw_mission_markers')
    rx, ry = load_map_fields(rospy.get_param('~map_file', DEFAULT_MAP))[:2]
    sections = ros_sections()
    pub = rospy.Publisher('/aisw/mission_markers', MarkerArray, queue_size=1, latch=True)
    arr = build(rx, ry, sections)
    pub.publish(arr)
    rospy.loginfo('[mission_markers] 미션 %d개, AI 구간 %d개 표시',
                  len(get_section(sections, 'missions', [])), len(get_section(sections, 'ai_zones', [])))
    rospy.spin()


if __name__ == '__main__':
    main()
