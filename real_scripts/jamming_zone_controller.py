#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, math
from collections import deque

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import MarkerArray
from tf.transformations import euler_from_quaternion

def clamp(v, lo, hi): return max(lo, min(hi, v))

class JammingZoneController:
    """
    제밍구역 내 주행 제어 (center_path MarkerArray 포인트 추종)
    - 입력: /traffic_perception/center_path (visualization_msgs/MarkerArray)
            marker.points[*]의 (x,y,z) 사용. 총 20개 포인트로 균등 샘플링하여 추종.
    - 프레임: marker.header.frame_id가 base_link/lidar 등 로컬이면 그대로,
              map 등 전역이면 현재 odom 기준으로 로컬(SE2) 변환.
    - 제어: Pure Pursuit (가변 lookahead)
    - 인덱스/정지 로직: 기존 manager/controller가 처리 (수정 없음)
    """

    def __init__(self, init_node=True):
        if init_node:
            rospy.init_node("jamming_zone_controller", anonymous=True)

        # --- 설정 ---
        self.center_path_topic = "/traffic_perception/center_path"
        self.num_points = 20

        # --- Publishers ---
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.target_speed_pub = rospy.Publisher("/jamming_target_speed", Float32, queue_size=1)

        # --- Params ---
        self.base_lookahead = rospy.get_param("~base_lookahead", 5.0)   # [m]
        self.max_lookahead  = rospy.get_param("~max_lookahead", 7.0)    # [m]
        self.k_v_gain       = rospy.get_param("~lookahead_speed_gain", 0.4)
        self.target_speed   = rospy.get_param("~target_speed", 10.0)     # [m/s]
        self.max_speed      = rospy.get_param("~max_speed", 12.0)
        self.k_yaw_rate     = rospy.get_param("~yaw_rate_gain", 0.2)    # ω 스케일
        self.max_yaw_rate   = rospy.get_param("~max_yaw_rate", 0.3)     # [rad/s]
        self.auto_enable_on_zero_odom = rospy.get_param("~auto_enable_on_zero_odom", True)

        # --- State ---
        self.jamming_mode_active = False
        self._seen_nonzero_odom = False  # 0,0 리셋 감지용
        self.odom_x = 0.0; self.odom_y = 0.0; self.odom_yaw = 0.0

        # 포인트(로컬 프레임) / 누적거리
        self.pts_local = []  # [(x,y)]
        self.cum_s = []

        # 속도 추정
        self._last_odom_t = None
        self._last_odom_x = None
        self._last_odom_y = None
        self._est_speed = 0.0
        self._vel_hist = deque(maxlen=10)

        # --- Subscribers (표준 ROS1 시그니처) ---
        rospy.Subscriber("/kiss/odometry", Odometry, self.kiss_odometry_callback)
        rospy.Subscriber("/jamming_mode_active", Bool, self.mode_callback)
        rospy.Subscriber("/traffic_perception/center_path", MarkerArray, self.center_path_callback)

    # ---------------- Odom & Mode ----------------
    def kiss_odometry_callback(self, msg: Odometry):
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        q  = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion((q.x, q.y, q.z, q.w))
        self.odom_x, self.odom_y, self.odom_yaw = px, py, yaw

        t = msg.header.stamp.to_sec() if msg.header.stamp else rospy.get_time()
        if self._last_odom_t is not None:
            dt = max(1e-3, t - self._last_odom_t)
            if self._last_odom_x is not None:
                ds = math.hypot(px - self._last_odom_x, py - self._last_odom_y)
                self._est_speed = 0.7*self._est_speed + 0.3*(ds/dt)
        self._last_odom_t, self._last_odom_x, self._last_odom_y = t, px, py

        # 자동 활성화: odom이 0,0 근처로 리셋되는 패턴이면 on
        if self.auto_enable_on_zero_odom:
            if abs(px) < 0.3 and abs(py) < 0.3:
                if self._seen_nonzero_odom:
                    self.jamming_mode_active = True
            else:
                self._seen_nonzero_odom = True

    def mode_callback(self, msg: Bool):
        self.jamming_mode_active = msg.data
        #rospy.loginfo("[JZC] jamming_mode_active=%s", self.jamming_mode_active)

    # ---------------- Center Path ----------------
    def center_path_callback(self, array_msg: MarkerArray):
        # 모든 marker에서 points 수집 (x,y,z)
        frame = None
        pts_xyz = []
        for mk in array_msg.markers:
            if frame is None:
                frame = mk.header.frame_id or frame
            if mk.points:
                for p in mk.points:
                    pts_xyz.append((p.x, p.y, getattr(p, "z", 0.0)))

        rospy.loginfo_throttle(1.0, f"[JZC] Center path received: {len(pts_xyz)} points from {len(array_msg.markers)} markers, frame: {frame}")

        if len(pts_xyz) < 2:
            rospy.logwarn_throttle(1.0, f"[JZC] Insufficient path points: {len(pts_xyz)} < 2")
            return

        # 정확히 self.num_points개 사용: 많으면 균등 N개, 적으면 그대로
        n = len(pts_xyz)
        N = max(2, int(self.num_points))
        if n > N:
            step = float(n - 1) / float(N - 1)
            pts_xyz = [ pts_xyz[int(round(k * step))] for k in range(N) ]

        # 프레임이 로컬이 아니면 전역→로컬(SE2) 변환
        if not self.is_local_frame(frame):
            pts_local = [ self.world_to_local(x, y) for (x, y, _) in pts_xyz ]
        else:
            pts_local = [ (x, y) for (x, y, _) in pts_xyz ]

        # 뒤(x<-2m) / 과도거리(>~14m) 제거
        filtered = []
        for (x, y) in pts_local:
            if x > -2.0 and (x*x + y*y) < 200.0:
                filtered.append((x, y))
        if len(filtered) < 2:
            return

        self.pts_local = filtered
        # 누적거리
        self.cum_s = [0.0]
        for i in range(1, len(filtered)):
            x0,y0 = filtered[i-1]; x1,y1 = filtered[i]
            self.cum_s.append(self.cum_s[-1] + math.hypot(x1-x0, y1-y0))

        rospy.loginfo_throttle(1.0, f"[JZC] Path processed successfully: {len(self.pts_local)} local points, total distance: {self.cum_s[-1]:.2f}m")

    # ---------------- Frames ----------------
    def is_local_frame(self, frame_id):
        if frame_id is None: return True
        fid = frame_id.lower()
        return any(k in fid for k in ["base_link", "base", "lidar", "velodyne", "livox", "ego"])

    def world_to_local(self, gx, gy):
        x, y, yaw = self.odom_x, self.odom_y, self.odom_yaw
        c = math.cos(-yaw); s = math.sin(-yaw)
        dx = gx - x; dy = gy - y
        lx = c*dx - s*dy
        ly = s*dx + c*dy
        return (lx, ly)

    # ---------------- Pure Pursuit ----------------
    def pick_lookahead_point(self, v_est):
        if not self.pts_local or len(self.pts_local) < 2:
            return None
        Ld = clamp(self.base_lookahead + self.k_v_gain * v_est,
                   self.base_lookahead, self.max_lookahead)
        # 차량 원점에서 s >= Ld 첫 지점
        target_idx = None
        for i, s in enumerate(self.cum_s):
            if s >= Ld:
                target_idx = i; break
        if target_idx is None:
            target_idx = len(self.pts_local) - 1
        return self.pts_local[target_idx], Ld

    def compute_cmd(self):
        v = clamp(self.target_speed, 0.0, self.max_speed)
        # ROS publisher 호출 제거 - master에서 직접 호출할 때는 불필요
        # self.target_speed_pub.publish(Float32(v))

        rospy.loginfo_throttle(2.0, f"[JZC Debug] pts_local count: {len(self.pts_local) if self.pts_local else 0}, jamming_mode_active: {self.jamming_mode_active}")

        picked = self.pick_lookahead_point(self._est_speed)
        if picked is None:
            rospy.logwarn_throttle(2.0, "[JZC Debug] pick_lookahead_point returned None - no path available")
            return 0.0, 0.0
        (xt, yt), Ld = picked
        if Ld < 0.5: Ld = 0.5

        rospy.loginfo_throttle(2.0, f"[JZC Debug] Target point: ({xt:.2f}, {yt:.2f}), Lookahead: {Ld:.2f}m")

        # Pure-Pursuit (로컬): κ ≈ 2*yt / Ld^2 → yaw_rate = v * κ
        kappa   = 2.0 * yt / (Ld * Ld)
        yaw_rate = clamp(self.k_yaw_rate * v * kappa, -self.max_yaw_rate, self.max_yaw_rate)

        rospy.loginfo_throttle(2.0, f"[JZC Debug] Kappa: {kappa:.4f}, Yaw rate: {yaw_rate:.4f}")

        return v, yaw_rate

    # ---------------- Main loop ----------------
    def run(self):
        rate = rospy.Rate(20)  # 20Hz
        while not rospy.is_shutdown():
            cmd = Twist()
            if self.jamming_mode_active:
                v, w = self.compute_cmd()
                cmd.linear.x = v
                cmd.angular.z = w
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
            rate.sleep()

if __name__ == "__main__":
    try:
        node = JammingZoneController()
        node.run()
    except rospy.ROSInterruptException:
        pass