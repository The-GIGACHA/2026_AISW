#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
MORAI ONLY: Global map(json) tracking with Pure Pursuit + Speed PID
+ RViz Markers (map / ego / target / trail)

입력(구독)
- /gps   (morai_msgs/GPSMessage) : latitude, longitude, eastOffset, northOffset
- /imu   (sensor_msgs/Imu)       : orientation -> yaw
- /Competition_topic (morai_msgs/EgoVehicleStatus) : velocity -> speed

출력(발행)
- /ctrl_cmd (morai_msgs/CtrlCmd) : steering(rad), accel(0~1), brake(0~1)

RViz 출력(발행)
- /global_tracker/markers (visualization_msgs/MarkerArray)
"""

import os
import json
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple, Deque
from collections import deque

import rospy
import tf
from pyproj import Proj
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from sensor_msgs.msg import Imu
from morai_msgs.msg import GPSMessage, EgoVehicleStatus, CtrlCmd, ObjectStatusList

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA


# =========================
# Utils
# =========================
def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a

def dist2(ax: float, ay: float, bx: float, by: float) -> float:
    dx = ax - bx
    dy = ay - by
    return dx * dx + dy * dy


# =========================
# Data
# =========================
@dataclass
class Waypoint:
    x: float
    y: float
    yaw: Optional[float] = None
    curvature: float = 0.0
    velocity: Optional[float] = None


# =========================
# Parameters
# =========================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

class P:
    MAP_JSON = os.path.join(SCRIPT_DIR, "map", "kcity_map.json")
    MAP_VEL_UNIT = "kph"

    UTM_ZONE = 52
    UTM_ELLPS = "WGS84"

    # Pure Pursuit
    WHEEL_BASE = 0.8
    STEER_LIMIT_DEG = 30.0
    LOOKAHEAD_MIN  = 2.5
    LOOKAHEAD_GAIN = 0.20
    LOOKAHEAD_MAX  = 8.0

    # Speed PID
    USE_MAP_SPEED = False
    TARGET_SPEED_KPH = 20.0
    KP_V = 0.6
    KI_V = 0.05
    KD_V = 0.02
    ACCEL_LIMIT = 2.0
    DECEL_LIMIT = -4.0
    THR_MAX = 1.0
    BRK_MAX = 1.0

    # Topics
    TOPIC_GPS = "/gps"
    TOPIC_IMU = "/imu"
    TOPIC_EGO = "/Competition_topic"
    TOPIC_CMD = "/ctrl_cmd"

    LOOP_HZ = 50.0

    # ─── 장애물 회피 (MORAI /Object_topic 기반) ───
    #  동적(움직이는 NPC/보행자) = 정지 대기(WAIT), 정적 = 횡방향 오프셋 회피(AVOID)
    TOPIC_OBJECTS    = "/Object_topic"
    OB_AHEAD_M       = 18.0    # 경로 전방 이 거리까지 장애물 검사
    OB_HALF_W        = 1.3     # 주행 통로 반폭 [m] (차폭/2 + 여유) — 이 안에 들어오면 '막힘'
    OB_MARGIN        = 0.3     # 회피 시 추가 여유 [m]
    OB_DYN_KMH       = 2.0     # 이 속도 이상 움직이면 동적 장애물
    OB_STOP_GAP_M    = 7.0     # 동적: 경로상 이 거리 안이면 정지
    OB_SLOW_GAP_M    = 14.0    # 동적: 이 거리 안이면 감속 시작
    AVOID_OFFSETS    = (0.0, 1.0, -1.0, 2.0, -2.0, 3.0, -3.0)  # 정적 회피 후보 오프셋 [m] (+좌 / −우)
    AVOID_SPEED_KPH  = 10.0    # 회피 중 속도 상한
    WAIT_CLEAR_SEC   = 1.0     # 동적 장애물 소실 후 재출발까지 유지
    AVOID_CLEAR_SEC  = 1.5     # 정적 장애물 통과 후 경로 복귀까지 오프셋 유지

    # RViz markers
    TOPIC_MARKERS = "/global_tracker/markers"
    FRAME_ID = "map"          # RViz Fixed Frame
    TRAIL_LEN = 300
    MARKER_RATE_HZ = 10.0     # RViz 갱신 주기


# =========================
# Map loader
# =========================
def load_map_json(path_in: str) -> List[Waypoint]:
    if not os.path.isabs(path_in):
        path = os.path.join(SCRIPT_DIR, path_in)
    else:
        path = path_in

    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    if isinstance(data, dict) and len(data) > 0 and all(str(k).isdigit() for k in data.keys()):
        keys = sorted(data.keys(), key=lambda k: int(k))
        pts = [data[k] for k in keys]
    elif isinstance(data, list):
        pts = data
    elif isinstance(data, dict):
        for key in ["points", "waypoints", "path", "data"]:
            if key in data and isinstance(data[key], list):
                pts = data[key]
                break
        else:
            raise ValueError(f"Unsupported map json dict keys: {list(data.keys())}")
    else:
        raise ValueError("Unsupported map json type")

    wpts: List[Waypoint] = []
    for p in pts:
        x = float(p["x"]); y = float(p["y"])
        yaw = float(p["yaw"]) if "yaw" in p else None
        curv = float(p.get("curvature", 0.0))
        vel = p.get("velocity", None)
        vel = float(vel) if vel is not None else None
        wpts.append(Waypoint(x=x, y=y, yaw=yaw, curvature=curv, velocity=vel))

    if len(wpts) < 5:
        raise ValueError("Map waypoints too small.")
    return wpts


# =========================
# Obstacle avoidance helpers (순수 함수 — 오프라인 테스트 가능)
# =========================
@dataclass
class ObsInfo:
    s:   float   # 경로를 따라 잰 전방 거리 [m] (near_i 기준)
    lat: float   # 경로 기준 횡방향 오프셋 [m] (+좌 / −우)
    dyn: bool    # 동적 여부
    x:   float
    y:   float
    r:   float   # 장애물 반경 [m]


def analyze_obstacles(wpts, near_i, obstacles, ahead_m):
    """각 장애물을 경로 좌표계(s: 전방 거리, lat: 횡 오프셋)로 사영.
    obstacles: [(x, y, speed_mps, radius)] — 전방 ahead_m 이내만 반환."""
    out = []
    if near_i >= len(wpts) - 1:
        return out
    # 전방 윈도우의 누적 거리 s 와 각 지점 헤딩 계산
    win = []          # (idx, s, hx, hy)
    s_acc = 0.0
    for i in range(near_i, len(wpts) - 1):
        dx = wpts[i + 1].x - wpts[i].x
        dy = wpts[i + 1].y - wpts[i].y
        seg = math.hypot(dx, dy)
        if seg < 1e-6:
            continue
        win.append((i, s_acc, dx / seg, dy / seg))
        s_acc += seg
        if s_acc > ahead_m:
            break
    for (ox, oy, sp, orad) in obstacles:
        best = None   # (dist2, s, lat)
        for (i, s0, hx, hy) in win:
            rx, ry = ox - wpts[i].x, oy - wpts[i].y
            lon = rx * hx + ry * hy            # 세그먼트 진행 방향 성분
            lat = -rx * hy + ry * hx           # 좌(+)/우(−) 성분
            d2 = rx * rx + ry * ry
            if best is None or d2 < best[0]:
                best = (d2, s0 + max(0.0, lon), lat)
        if best is not None and best[1] <= ahead_m:
            out.append(ObsInfo(s=best[1], lat=best[2],
                               dyn=(sp * 3.6 >= P.OB_DYN_KMH),
                               x=ox, y=oy, r=orad))
    return out


def pick_avoid_offset(infos, half_w, margin, candidates):
    """정적 장애물을 피하는 최소 |오프셋| 선택.
    오프셋 o 적용 시 모든 장애물이 |lat − o| > half_w + r + margin 이면 통과 가능.
    없으면 None (→ 정지)."""
    for o in candidates:
        ok = True
        for ob in infos:
            if abs(ob.lat - o) <= half_w + ob.r + margin:
                ok = False
                break
        if ok:
            return o
    return None


# =========================
# Controllers
# =========================
class SpeedPID:
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.i = 0.0
        self.prev_e = 0.0

    def step(self, v_ref_mps: float, v_mps: float, dt: float) -> float:
        e = v_ref_mps - v_mps
        self.i += e * dt
        de = (e - self.prev_e) / max(dt, 1e-3)
        self.prev_e = e
        a = self.kp * e + self.ki * self.i + self.kd * de
        return clamp(a, P.DECEL_LIMIT, P.ACCEL_LIMIT)

def accel_to_throttle_brake(a_cmd: float) -> Tuple[float, float]:
    if a_cmd >= 0.0:
        thr = clamp(a_cmd / max(P.ACCEL_LIMIT, 1e-6), 0.0, P.THR_MAX)
        brk = 0.0
    else:
        thr = 0.0
        brk = clamp((-a_cmd) / max((-P.DECEL_LIMIT), 1e-6), 0.0, P.BRK_MAX)
    return thr, brk


# =========================
# RViz Marker helper
# =========================
def _pt(x, y, z=0.0) -> Point:
    p = Point()
    p.x = float(x); p.y = float(y); p.z = float(z)
    return p

def _color(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r = float(r); c.g = float(g); c.b = float(b); c.a = float(a)
    return c

class RvizViz:
    def __init__(self, wpts: List[Waypoint]):
        self.pub = rospy.Publisher(P.TOPIC_MARKERS, MarkerArray, queue_size=1)
        self.wpts = wpts

        # map points marker는 고정이라 한번 만들어두고 재사용
        self.map_marker = Marker()
        self.map_marker.header.frame_id = P.FRAME_ID
        self.map_marker.ns = "map"
        self.map_marker.id = 0
        self.map_marker.type = Marker.LINE_STRIP
        self.map_marker.action = Marker.ADD
        self.map_marker.scale.x = 0.25  # 선 두께
        self.map_marker.color = _color(0.3, 0.8, 1.0, 1.0)
        self.map_marker.pose.orientation.w = 1.0
        self.map_marker.points = [_pt(p.x, p.y, 0.0) for p in self.wpts]

    def publish(self,
                ego_xy: Tuple[float, float],
                tgt_xy: Tuple[float, float],
                trail: Deque[Tuple[float, float]],
                obstacles=None,
                state: str = ""):
        ma = MarkerArray()

        # (1) Map (LINE_STRIP)
        m_map = self.map_marker
        m_map.header.stamp = rospy.Time.now()
        ma.markers.append(m_map)

        # (2) Ego point (SPHERE)
        m_ego = Marker()
        m_ego.header.frame_id = P.FRAME_ID
        m_ego.header.stamp = rospy.Time.now()
        m_ego.ns = "ego"
        m_ego.id = 1
        m_ego.type = Marker.SPHERE
        m_ego.action = Marker.ADD
        m_ego.pose.position.x = float(ego_xy[0])
        m_ego.pose.position.y = float(ego_xy[1])
        m_ego.pose.position.z = 0.0
        m_ego.pose.orientation.w = 1.0
        m_ego.scale.x = 1.0
        m_ego.scale.y = 1.0
        m_ego.scale.z = 1.0
        m_ego.color = _color(1.0, 1.0, 0.2, 1.0)
        ma.markers.append(m_ego)

        # (3) Target point (SPHERE)
        m_tgt = Marker()
        m_tgt.header.frame_id = P.FRAME_ID
        m_tgt.header.stamp = rospy.Time.now()
        m_tgt.ns = "target"
        m_tgt.id = 2
        m_tgt.type = Marker.SPHERE
        m_tgt.action = Marker.ADD
        m_tgt.pose.position.x = float(tgt_xy[0])
        m_tgt.pose.position.y = float(tgt_xy[1])
        m_tgt.pose.position.z = 0.0
        m_tgt.pose.orientation.w = 1.0
        m_tgt.scale.x = 0.9
        m_tgt.scale.y = 0.9
        m_tgt.scale.z = 0.9
        m_tgt.color = _color(1.0, 0.2, 0.2, 1.0)
        ma.markers.append(m_tgt)

        # (4) Trail (LINE_STRIP)
        m_tr = Marker()
        m_tr.header.frame_id = P.FRAME_ID
        m_tr.header.stamp = rospy.Time.now()
        m_tr.ns = "trail"
        m_tr.id = 3
        m_tr.type = Marker.LINE_STRIP
        m_tr.action = Marker.ADD
        m_tr.pose.orientation.w = 1.0
        m_tr.scale.x = 0.15
        m_tr.color = _color(0.9, 0.9, 0.9, 0.8)
        m_tr.points = [_pt(x, y, 0.0) for (x, y) in trail]
        ma.markers.append(m_tr)

        # (5) 장애물 (CYLINDER — 동적 주황 / 정적 빨강)
        if obstacles is not None:
            for k, (ox, oy, sp, r) in enumerate(obstacles[:40]):
                m = Marker()
                m.header.frame_id = P.FRAME_ID
                m.header.stamp = rospy.Time.now()
                m.ns = "obstacles"; m.id = 10 + k
                m.type = Marker.CYLINDER; m.action = Marker.ADD
                m.pose.position.x = ox; m.pose.position.y = oy; m.pose.position.z = 0.5
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = max(0.6, 2.0 * r); m.scale.z = 1.0
                dyn = sp * 3.6 >= P.OB_DYN_KMH
                m.color = _color(1.0, 0.55, 0.1, 0.85) if dyn else _color(0.9, 0.15, 0.15, 0.85)
                m.lifetime = rospy.Duration(0.5)
                ma.markers.append(m)

        # (6) 회피 상태 텍스트 (ego 위)
        if state:
            m = Marker()
            m.header.frame_id = P.FRAME_ID
            m.header.stamp = rospy.Time.now()
            m.ns = "avoid_state"; m.id = 5
            m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
            m.pose.position.x = float(ego_xy[0]); m.pose.position.y = float(ego_xy[1]); m.pose.position.z = 2.2
            m.scale.z = 1.1
            m.color = _color(1.0, 1.0, 1.0, 1.0) if state.startswith("CLEAR") else _color(1.0, 0.4, 0.2, 1.0)
            m.text = state
            ma.markers.append(m)

        self.pub.publish(ma)


# =========================
# Node
# =========================
class MoraiGlobalTracker:
    def __init__(self):
        rospy.init_node("morai_global_tracker", anonymous=True)

        # params override
        self.map_json = rospy.get_param("~map_json", P.MAP_JSON)
        P.USE_MAP_SPEED = bool(rospy.get_param("~use_map_speed", P.USE_MAP_SPEED))
        P.TARGET_SPEED_KPH = float(rospy.get_param("~target_speed_kph", P.TARGET_SPEED_KPH))
        P.MAP_VEL_UNIT = rospy.get_param("~map_vel_unit", P.MAP_VEL_UNIT)

        P.WHEEL_BASE = float(rospy.get_param("~wheel_base", P.WHEEL_BASE))
        P.LOOKAHEAD_MIN = float(rospy.get_param("~lookahead_min", P.LOOKAHEAD_MIN))
        P.LOOKAHEAD_MAX = float(rospy.get_param("~lookahead_max", P.LOOKAHEAD_MAX))
        P.LOOKAHEAD_GAIN = float(rospy.get_param("~lookahead_gain", P.LOOKAHEAD_GAIN))
        P.STEER_LIMIT_DEG = float(rospy.get_param("~steer_limit_deg", P.STEER_LIMIT_DEG))

        P.FRAME_ID = rospy.get_param("~frame_id", P.FRAME_ID)
        P.TRAIL_LEN = int(rospy.get_param("~trail_len", P.TRAIL_LEN))
        P.MARKER_RATE_HZ = float(rospy.get_param("~marker_rate_hz", P.MARKER_RATE_HZ))

        self.steer_limit = math.radians(P.STEER_LIMIT_DEG)

        # map load
        self.wpts = load_map_json(self.map_json)
        rospy.loginfo(f"[MORAI Tracker] loaded map: {len(self.wpts)} points, file={self.map_json}")

        # UTM projector
        self.proj_utm = Proj(proj="utm", zone=P.UTM_ZONE, ellps=P.UTM_ELLPS, preserve_units=True)

        # ego state
        self.ego_x: Optional[float] = None
        self.ego_y: Optional[float] = None
        self.ego_yaw: Optional[float] = None
        self.ego_v: float = 0.0
        self.last_time = rospy.Time.now()

        self.last_nearest = 0
        self.pid = SpeedPID(P.KP_V, P.KI_V, P.KD_V)
        self.br = tf.TransformBroadcaster() # 추가: TF 방송국 초기화

        self.trail: Deque[Tuple[float, float]] = deque(maxlen=max(50, P.TRAIL_LEN))
        self.viz = RvizViz(self.wpts)
        self.last_marker_pub = rospy.Time(0)

        # pubs/subs
        self.pub_cmd = rospy.Publisher(P.TOPIC_CMD, CtrlCmd, queue_size=1)
        rospy.Subscriber(P.TOPIC_GPS, GPSMessage, self.cb_gps, queue_size=1)
        rospy.Subscriber(P.TOPIC_IMU, Imu, self.cb_imu, queue_size=1)
        rospy.Subscriber(P.TOPIC_EGO, EgoVehicleStatus, self.cb_ego, queue_size=1)
        rospy.Subscriber(P.TOPIC_OBJECTS, ObjectStatusList, self.cb_objects, queue_size=1)

        # ── 회피 상태 머신: CLEAR / WAIT(동적 정지) / AVOID(정적 오프셋) ──
        self.obstacles: List[Tuple[float, float, float, float]] = []   # (x, y, speed_mps, radius)
        self.av_state = "CLEAR"
        self.av_offset = 0.0
        self.last_dyn_t = 0.0      # 동적 장애물 마지막 관측 시각
        self.last_sta_t = 0.0      # 정적 장애물 마지막 관측 시각

        self.timer = rospy.Timer(rospy.Duration(1.0 / P.LOOP_HZ), self.loop)

    def cb_gps(self, msg: GPSMessage):
        utm_x, utm_y = self.proj_utm(msg.longitude, msg.latitude)
        self.ego_x = utm_x - float(msg.eastOffset)
        self.ego_y = utm_y - float(msg.northOffset)

    def cb_imu(self, msg: Imu):
        q = msg.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.ego_yaw = yaw

    def cb_ego(self, msg: EgoVehicleStatus):
        try:
            vx = float(msg.velocity.x)
            vy = float(msg.velocity.y)
            self.ego_v = math.hypot(vx, vy)
        except Exception:
            pass

    def cb_objects(self, msg: ObjectStatusList):
        """MORAI 오브젝트(NPC 차량·보행자·장애물) — 위치는 ego 와 같은 map 좌표계.
        velocity.x 는 km/h (오브젝트 전방 기준)."""
        obs = []
        for lst in (msg.npc_list, msg.pedestrian_list, msg.obstacle_list):
            for o in lst:
                r = max(abs(o.size.x), abs(o.size.y)) * 0.5
                r = max(0.4, min(r, 2.5))
                sp = abs(float(o.velocity.x)) / 3.6
                obs.append((float(o.position.x), float(o.position.y), sp, r))
        self.obstacles = obs

    def apply_avoidance(self, near_i, tgt_i, lfd, vref):
        """장애물 반영: (수정된 목표점 x, y, 수정 vref, 강제 브레이크 여부) 반환.
        동적 → WAIT(정지), 정적 → AVOID(경로 법선 방향 오프셋), 회피 불가 → 정지."""
        now = rospy.get_time()
        tx, ty = self.wpts[tgt_i].x, self.wpts[tgt_i].y
        infos = analyze_obstacles(self.wpts, near_i, self.obstacles, P.OB_AHEAD_M)
        blocking = [ob for ob in infos if abs(ob.lat) <= P.OB_HALF_W + ob.r]
        dyn = [ob for ob in blocking if ob.dyn]
        sta = [ob for ob in blocking if not ob.dyn]

        # ── 동적: 앞에서 움직이는 것 → 거리 따라 감속/정지, 지나가면 재출발 ──
        if dyn:
            self.last_dyn_t = now
            d = min(ob.s for ob in dyn)
            if d <= P.OB_STOP_GAP_M:
                self.av_state = "WAIT"
                return tx, ty, 0.0, True
            if d <= P.OB_SLOW_GAP_M:      # 감속 접근
                k = (d - P.OB_STOP_GAP_M) / max(1e-3, P.OB_SLOW_GAP_M - P.OB_STOP_GAP_M)
                return tx, ty, min(vref, vref * k), False
        elif self.av_state == "WAIT":
            if now - self.last_dyn_t < P.WAIT_CLEAR_SEC:
                return tx, ty, 0.0, True   # 소실 직후 잠깐 더 대기 (재등장 대비)
            self.av_state = "CLEAR"

        # ── 정적: 최소 오프셋 후보로 통로 확보 → 목표점을 경로 법선으로 평행이동 ──
        if sta:
            self.last_sta_t = now
            cands = P.AVOID_OFFSETS
            if self.av_state == "AVOID":   # 이미 회피 중이면 현재 오프셋 우선 (좌우 흔들림 방지)
                cands = (self.av_offset,) + tuple(o for o in P.AVOID_OFFSETS if o != self.av_offset)
            off = pick_avoid_offset(infos, P.OB_HALF_W, P.OB_MARGIN, cands)
            if off is None:                # 어디로도 못 피함 → 정지
                self.av_state = "WAIT"
                return tx, ty, 0.0, True
            self.av_state, self.av_offset = "AVOID", off
        elif self.av_state == "AVOID" and now - self.last_sta_t > P.AVOID_CLEAR_SEC:
            self.av_state, self.av_offset = "CLEAR", 0.0   # 통과 완료 → 경로 복귀

        if self.av_state == "AVOID" and abs(self.av_offset) > 1e-6:
            # 목표점에서의 경로 헤딩 법선 방향으로 오프셋
            j = min(tgt_i + 1, len(self.wpts) - 1)
            hx, hy = self.wpts[j].x - self.wpts[tgt_i].x, self.wpts[j].y - self.wpts[tgt_i].y
            n = math.hypot(hx, hy)
            if n > 1e-6:
                tx += (-hy / n) * self.av_offset
                ty += ( hx / n) * self.av_offset
            return tx, ty, min(vref, P.AVOID_SPEED_KPH / 3.6), False

        return tx, ty, vref, False

    def nearest_idx(self, x: float, y: float) -> int:
        start = max(0, self.last_nearest - 50)
        end = min(len(self.wpts), self.last_nearest + 300)
        best_i = start
        best_d2 = float("inf")
        for i in range(start, end):
            d2 = dist2(x, y, self.wpts[i].x, self.wpts[i].y)
            if d2 < best_d2:
                best_d2 = d2
                best_i = i
        self.last_nearest = best_i
        return best_i

    def find_target_by_lfd(self, near_i: int, lfd: float) -> int:
        acc = 0.0
        i = near_i
        while i + 1 < len(self.wpts) and acc < lfd:
            dx = self.wpts[i + 1].x - self.wpts[i].x
            dy = self.wpts[i + 1].y - self.wpts[i].y
            acc += math.hypot(dx, dy)
            i += 1
        return i

    def pure_pursuit(self, x: float, y: float, yaw: float, v: float) -> Tuple[float, int, int, float]:
        near_i = self.nearest_idx(x, y)

        lfd = P.LOOKAHEAD_GAIN * max(0.0, v) + P.LOOKAHEAD_MIN
        lfd = clamp(lfd, P.LOOKAHEAD_MIN, P.LOOKAHEAD_MAX)
        tgt_i = self.find_target_by_lfd(near_i, lfd)

        tx, ty = self.wpts[tgt_i].x, self.wpts[tgt_i].y
        alpha = wrap_pi(math.atan2(ty - y, tx - x) - yaw)

        steer = math.atan2(2.0 * P.WHEEL_BASE * math.sin(alpha), max(1e-3, lfd))
        steer = clamp(steer, -self.steer_limit, self.steer_limit)
        return steer, near_i, tgt_i, lfd

    def get_vref_mps(self, idx_for_speed: int) -> float:
        if P.USE_MAP_SPEED and self.wpts[idx_for_speed].velocity is not None:
            v = float(self.wpts[idx_for_speed].velocity)
            return (v / 3.6) if (P.MAP_VEL_UNIT == "kph") else v
        return P.TARGET_SPEED_KPH / 3.6

    def publish_markers(self, tgt_i: int):
        now = rospy.Time.now()
        if (now - self.last_marker_pub).to_sec() < (1.0 / max(P.MARKER_RATE_HZ, 1.0)):
            return
        self.last_marker_pub = now

        ego_xy = (self.ego_x, self.ego_y)
        tgt_xy = getattr(self, "_last_avoid_tgt", None) or (self.wpts[tgt_i].x, self.wpts[tgt_i].y)
        self.viz.publish(ego_xy, tgt_xy, self.trail,
                         obstacles=self.obstacles,
                         state=f"{self.av_state}"
                               f"{f' {self.av_offset:+.1f}m' if self.av_state == 'AVOID' else ''}")

    def loop(self, _evt):
        if self.ego_x is None or self.ego_y is None or self.ego_yaw is None:
            return

        self.trail.append((self.ego_x, self.ego_y))
        
        # --- 추가된 TF 발행 로직 ---
        self.br.sendTransform(
            (self.ego_x, self.ego_y, 0),
            quaternion_from_euler(0, 0, self.ego_yaw),
            rospy.Time.now(),
            "base_link",  # 자식 좌표계 (차량)
            "map"         # 부모 좌표계 (지도)
        )
        # ------------------------

        now = rospy.Time.now()
        # ... (이후 기존 코드)
        now = rospy.Time.now()
        dt = max((now - self.last_time).to_sec(), 1e-3)
        self.last_time = now

        steer, near_i, tgt_i, lfd = self.pure_pursuit(self.ego_x, self.ego_y, self.ego_yaw, self.ego_v)

        vref = self.get_vref_mps(tgt_i)

        # ── 장애물 회피: 목표점·속도 보정 (동적=WAIT, 정적=오프셋 AVOID) ──
        tx, ty, vref, hard_stop = self.apply_avoidance(near_i, tgt_i, lfd, vref)
        if self.av_state == "AVOID":
            alpha = wrap_pi(math.atan2(ty - self.ego_y, tx - self.ego_x) - self.ego_yaw)
            steer = clamp(math.atan2(2.0 * P.WHEEL_BASE * math.sin(alpha), max(1e-3, lfd)),
                          -self.steer_limit, self.steer_limit)

        if hard_stop:
            a_cmd, accel, brake = P.DECEL_LIMIT, 0.0, P.BRK_MAX
            self.pid.i = 0.0                      # 정지 중 적분 와인드업 방지
        else:
            a_cmd = self.pid.step(vref, self.ego_v, dt)
            accel, brake = accel_to_throttle_brake(a_cmd)

        cmd = CtrlCmd()
        cmd.longlCmdType = 1
        cmd.steering = steer
        cmd.accel = accel
        cmd.brake = brake
        self.pub_cmd.publish(cmd)
        self._last_avoid_tgt = (tx, ty)

        self.publish_markers(tgt_i)

        rospy.loginfo_throttle(
            0.5,
            f"[Track] near={near_i} tgt={tgt_i} "
            f"ego=({self.ego_x:.2f},{self.ego_y:.2f}) yaw={self.ego_yaw:.2f} v={self.ego_v:.2f} "
            f"lfd={lfd:.2f} steer={math.degrees(steer):.2f}deg "
            f"vref={vref:.2f} a={a_cmd:.2f} acc={accel:.2f} brk={brake:.2f} "
            f"AV={self.av_state}{f'({self.av_offset:+.1f}m)' if self.av_state == 'AVOID' else ''} "
            f"obs={len(self.obstacles)}"
        )


if __name__ == "__main__":
    try:
        MoraiGlobalTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
