#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, json, rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from morai_msgs.msg import GPSMessage
from pyproj import Proj

"""
RViz에 HD 맵(ENU)과 실시간 GPS를 동일한 로컬 ENU 좌표계로 정렬해 시각화하는 노드 (단일 파일용).

좌표 파이프라인(맵):
  JSON(로컬 ENU; base_lla를 원점, AEQD 투영)
    ─[AEQD inverse]→ (lon, lat; WGS84)
    ─[UTM]→ (E, N; meters)
    ─[오프셋 보정]→ (E - eastOffset, N - northOffset)  ==> RViz 'map' 프레임

좌표 파이프라인(GPS):
  (lon, lat; WGS84)
    ─[UTM]→ (E, N; meters)
    ─[오프셋 보정]→ (E - eastOffset, N - northOffset)  ==> RViz 'map' 프레임

색상 규칙(맵 레이어 Legend):
  - 'center' (초록): 차선의 중심선(waypoints)
  - 'left'   (파랑): 왼쪽 경계선(leftBound)
  - 'right'  (빨강): 오른쪽 경계선(rightBound)

GNSS 상태 표시(차량 구체 색):
  - 0: No Fix(빨강), 1: 2D(주황), 2: 3D(노랑), 3/4: RTK/Float(초록)
"""

# ====== 설정 ======
FRAME_ID = "map"     # RViz 고정 프레임
UTM_ZONE = 52        # UTM 존
MAP_JSON = "/home/yhj/catkin_ws/src/hlfma_morai/map/Sangam_1.json"  # HD맵 경로
COMPARE_JSON = "/home/yhj/catkin_ws/src/hlfma_morai/map/sangam_bonseon_ver3.json" # 제공맵 경로
# =================

# ============= 각 레이어별 pts 저장 =============
DUMP_CENTER = False     # center 레이어 저장 여부
DUMP_LEFT   = False     # left   레이어 저장 여부
DUMP_RIGHT  = False     # right  레이어 저장 여부

PTS_CENTER_OUT = os.path.splitext(MAP_JSON)[0] + "_center_pts.txt"
PTS_LEFT_OUT   = os.path.splitext(MAP_JSON)[0] + "_left_pts.txt"
PTS_RIGHT_OUT  = os.path.splitext(MAP_JSON)[0] + "_right_pts.txt"
# =============================================

class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = [v / 3.6 for v in cv]
        # self.cv = cv
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)

def fix_color(status: int) -> ColorRGBA:
    """GNSS status를 색으로 표현 (빨강→노랑→초록)."""
    if status >= 3:  return ColorRGBA(0.1, 0.8, 0.1, 0.95)  # RTK(고정/Float) ≈ 초록
    if status == 2:  return ColorRGBA(1.0, 0.8, 0.1, 0.95)  # 3D Fix         ≈ 노랑
    if status == 1:  return ColorRGBA(1.0, 0.5, 0.1, 0.95)  # 2D Fix         ≈ 주황
    return ColorRGBA(0.9, 0.1, 0.1, 0.95)                   # No Fix         ≈ 빨강

def make_marker(mid, pts, color, ns, frame_id=FRAME_ID):
    """
    Polyline(선분) 마커 생성.
    - type: LINE_STRIP
    - scale.x: 선 두께 [m] (값을 키우면 더 '두껍게' 보임)
    - color: (r,g,b,a) 튜플 0~1
    - ns:    RViz에서 필터링용 네임스페이스 (center/left/right)
    """
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time.now()
    m.ns = ns
    m.id = mid
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.6
    m.color = ColorRGBA(*color)
    m.pose.orientation.w = 1.0
    for p in pts:
        x, y = float(p[0]), float(p[1])
        z = float(p[2]) if isinstance(p, (list, tuple)) and len(p) > 2 else 0.0
        m.points.append(Point(x=x, y=y, z=z))
    m.lifetime = rospy.Duration(0)
    return m

def enu_to_local_polyline(seq, proj_local_aeqd, proj_utm, east_off, north_off):
    """
    JSON에 저장된 ENU좌표를 모라이 ENU로 변환
      1) (x,y)[m] ─[AEQD inverse]→ (lon, lat) : AEQD 역변환 (ENU→위경도)
      2) (lon,lat) ─[UTM]→ (E,N)
      3) (E - east_off, N - north_off) → RViz 'map'의 모라이 ENU
    """
    out = []
    for p in seq:
        x, y = float(p[0]), float(p[1])
        lon, lat = proj_local_aeqd(x, y, inverse=True)
        E, N = proj_utm(lon, lat)
        if isinstance(p, (list, tuple)) and len(p) > 2:
            out.append([E - east_off, N - north_off, p[2]])
        else:
            out.append([E - east_off, N - north_off])
    return out

def _dump_pts(fh, layer, sid_i, seg_idx, pts):
    """
    layer: 'center' | 'left' | 'right'
    sid_i: 레인 인덱스
    seg_idx: center는 0 고정, left/right는 세그먼트 인덱스
    pts: [[x,y(,z)], ...] (모라이 ENU)
    """
    fh.write(f"[{layer}] sid={sid_i}, seg={seg_idx}, num={len(pts)}\n")
    for p in pts:
        if len(p) > 2:
            fh.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
        else:
            fh.write(f"{p[0]:.6f} {p[1]:.6f}\n")
    fh.write("\n")

def publish_map_once(path, arr, east_off, north_off, proj_utm):
    """
    HD맵 JSON을 읽어 base_lla 기준 AEQD를 만들고,
    레인 정보를 로컬 ENU로 변환하여 MarkerArray에 채운다.
    """
    if not os.path.isfile(path):
        rospy.logwarn("HD map file not found: %s", path)
        return 0

    with open(path) as f:
        data = json.load(f)

    base = data.get("base_lla", None)
    if not base or len(base) < 2:
        rospy.logwarn("base_lla missing in %s -> skip", os.path.basename(path))
        return 0

    base_lat, base_lon = float(base[0]), float(base[1])
    proj_local_aeqd = Proj(proj="aeqd", lat_0=base_lat, lon_0=base_lon, ellps="WGS84", units="m")

    lanes = data.get("lanelets", {})
    count = 0

    # 레이어별 덤프 파일 오픈(플래그 켜진 것만)
    fh_center = fh_left = fh_right = None
    try:
        if DUMP_CENTER:
            fh_center = open(PTS_CENTER_OUT, "w", encoding="utf-8")
            fh_center.write(f"# center pts dump from {os.path.basename(path)} (morai ENU)\n\n")
        if DUMP_LEFT:
            fh_left = open(PTS_LEFT_OUT, "w", encoding="utf-8")
            fh_left.write(f"# left pts dump from {os.path.basename(path)} (morai ENU)\n\n")
        if DUMP_RIGHT:
            fh_right = open(PTS_RIGHT_OUT, "w", encoding="utf-8")
            fh_right.write(f"# right pts dump from {os.path.basename(path)} (morai ENU)\n\n")
    except Exception as e:
        rospy.logwarn("Failed to open dump files: %s", e)

    for sid, lane in lanes.items():
        sid_i = int(sid)    # sid는 인덱스 정보 / lane은 실제 정보

        # ────────── 색상 레전드 ──────────
        # center : 초록 (0.1, 0.8, 0.1, 1.0)
        # left   : 파랑 (0.1, 0.1, 1.0, 0.8)
        # right  : 빨강 (1.0, 0.1, 0.1, 0.8)
        # ──────────────────────────────

        if lane.get("waypoints"):
            pts = enu_to_local_polyline(lane["waypoints"], proj_local_aeqd, proj_utm, east_off, north_off)
            arr.markers.append(make_marker(sid_i, pts, (0.1, 0.8, 0.1, 1.0), "center", FRAME_ID))
            if fh_center: _dump_pts(fh_center, "center", sid_i, 0, pts)
            count += 1

        for i, seg in enumerate(lane.get("leftBound", [])):
            pts = enu_to_local_polyline(seg, proj_local_aeqd, proj_utm, east_off, north_off)
            arr.markers.append(make_marker(100000 + sid_i*10 + i, pts, (0.1, 0.1, 1.0, 0.8), "left", FRAME_ID))
            if fh_left: _dump_pts(fh_left, "left", sid_i, i, pts)
            count += 1

        for i, seg in enumerate(lane.get("rightBound", [])):
            pts = enu_to_local_polyline(seg, proj_local_aeqd, proj_utm, east_off, north_off)
            arr.markers.append(make_marker(200000 + sid_i*10 + i, pts, (1.0, 0.1, 0.1, 0.8), "right", FRAME_ID))
            if fh_right: _dump_pts(fh_right, "right", sid_i, i, pts)
            count += 1
    
    # 파일 닫기 및 경로 로그
    if fh_center:
        fh_center.close()
        rospy.loginfo("Saved center pts: %s", PTS_CENTER_OUT)
    if fh_left:
        fh_left.close()
        rospy.loginfo("Saved left pts: %s", PTS_LEFT_OUT)
    if fh_right:
        fh_right.close()
        rospy.loginfo("Saved right pts: %s", PTS_RIGHT_OUT)
    
    return count

########################## 제공맵 publish ##########################
def publish_compare_map_marker(pub, path_obj):
    """
    제공맵(centerline만) 비교 표시:
    - 굵기: 1.5
    - 색:   시안
    - 토픽: /mgeo/markers_compare
    """
    marker_array = MarkerArray()
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "compare"
    marker.id = 900000  # 충돌 방지용 큰 ID
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD

    # 굵기/색/가시성
    marker.scale.x = 1.5           # 선 두께 [m] (컨트롤러와 동일)
    marker.color.r = 0.0           # Cyan색
    marker.color.g = 0.9
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position.z = 0.1   # z-fighting 방지

    for x, y in zip(path_obj.cx, path_obj.cy):
        marker.points.append(Point(x=float(x), y=float(y), z=0.0))

    marker_array.markers.append(marker)
    pub.publish(marker_array)

# json파일 불러와 ref 정보 PATH객체로 저장
def load_ref_map(json_file):
    with open(json_file, 'r') as f:
        data = json.load(f)

    keys = sorted(data.keys(), key=lambda k: int(k))

    rx      = [data[k]['x']         for k in keys]
    ry      = [data[k]['y']         for k in keys]
    ryaw    = [data[k]['yaw']       for k in keys]
    rk      = [data[k]['curvature'] for k in keys]
    rvel    = [data[k]['velocity']  for k in keys]
    rm      = [data[k]['mission']   for k in keys]
    rgear   = [data[k]['gear']      for k in keys]

    return PATH(rx, ry, ryaw, rk, rvel, rm, rgear)
##################################################################

class MapPublisher:
    """
      - /gps 첫 메시지에서 eastOffset/northOffset 확보
      - 맵을 로컬 ENU로 변환해 /mgeo/markers에 1회(latch) 퍼블리시
      - GPS는 (UTM - offset)으로 변환해 /ego/marker에 실시간 퍼블리시
    """
    def __init__(self):
        rospy.init_node("lane_json_to_rviz_single_map")
        # HD맵
        self.pub_map = rospy.Publisher("/mgeo/markers", MarkerArray, queue_size=1, latch=True)
        # 실시간 GPS
        self.pub_ego = rospy.Publisher("/ego/marker", Marker, queue_size=1, latch=True)
        # 제공맵
        self.pub_map_compare = rospy.Publisher("/mgeo/markers_compare", MarkerArray, queue_size=1, latch=True)

        # 위경도 <-> UTM 변환기 (지역 Zone 고정)
        self.proj_UTM = Proj(proj='utm', zone=UTM_ZONE, ellps='WGS84', preserve_units=False)

        self.map_published = False
        self._east_off = None
        self._north_off = None

        rospy.Subscriber("/gps", GPSMessage, self.gps_cb, queue_size=30)
        rospy.loginfo("Fixed frame=%s. Waiting first /gps for offsets...", FRAME_ID)

    def gps_cb(self, msg: GPSMessage):
        """
        /gps 콜백.
        - 첫 수신: offsets 저장 후 맵을 변환·퍼블리시(1회)
        - 매 수신: 차량 위치를 로컬 ENU로 변환해 EGO 마커 갱신
        """
        if not self.map_published:
            self._east_off  = float(msg.eastOffset)
            self._north_off = float(msg.northOffset)

            arr = MarkerArray()
            total = publish_map_once(MAP_JSON, arr, self._east_off, self._north_off, self.proj_UTM)
            self.pub_map.publish(arr)
            rospy.loginfo("Published %d markers on /mgeo/markers (frame_id=%s)", total, FRAME_ID)
            try:
                compare_path = load_ref_map(COMPARE_JSON)
                publish_compare_map_marker(self.pub_map_compare, compare_path)
                rospy.loginfo("Published compare marker on /mgeo/markers_compare")
            except Exception as e:
                rospy.logwarn("Compare map publish failed: %s", e)
            self.map_published = True

        # GPS 마커: (lon,lat)->UTM -> (UTM - offset)
        E, N = self.proj_UTM(float(msg.longitude), float(msg.latitude))
        ego_x = E - self._east_off
        ego_y = N - self._north_off

        m = Marker()
        m.header.frame_id = FRAME_ID
        m.header.stamp = rospy.Time.now()
        m.ns = "ego"
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position = Point(x=ego_x, y=ego_y, z=1.0)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 5.0    # 구(SPHERE) 지름 [m]
        m.color = fix_color(int(msg.status))  # GNSS 상태별 색
        m.lifetime = rospy.Duration(0)
        self.pub_ego.publish(m)

def main():
    MapPublisher()
    rospy.spin()

if __name__ == "__main__":
    main()



# HD맵 JSON파일 구조
# {
#   "base_lla": [37.xxx, 126.xxx, 29.18],   // ENU 원점이 되는 위경도고도
#   "lanelets": {
#     "0": {
#       "waypoints": [[x,y], [x,y], ...],   // 중심선(연속한 점들 → 한 줄)
#       "leftBound":  [                     // 왼쪽 경계선(여러 조각으로 쪼개질 수 있음)
#         [[x,y], [x,y], ...],              // segment 0
#         [[x,y], [x,y], ...]               // segment 1
#       ],
#       "rightBound": [                     // 오른쪽 경계선(여러 조각으로 쪼개질 수 있음)
#         [[x,y], [x,y], ...]               // segment 0
#       ]
#     },
#     "1": { ... },
#     "2": { ... }
#   }
# }