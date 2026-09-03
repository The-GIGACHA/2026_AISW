#!/home/inji2/.local/rospython/python3
# -*- coding: utf-8 -*-
"""
2026 국토부 AI/SW 모빌리티 경진대회용 UDP <-> ROS 게이트웨이 (팀 PC에서 실행)
- 대회 규정상 시뮬레이터와의 통신은 전부 UDP. 이 노드가 UDP를 ROS 토픽으로 변환해
  기존 ROS 기반 제어코드(control_test_mj.py 등)를 그대로 쓸 수 있게 함.

수신(UDP -> ROS):
  GPS   127.0.0.1:9281  NMEA(GPGGA)      -> /gps (morai_msgs/GPSMessage)
  IMU   127.0.0.1:9283  '#IMUData$'      -> /imu (sensor_msgs/Imu)
  CAM   127.0.0.1:9291/9293/9295 (JPEG 조각) -> /image_jpeg{,_left,_right}/compressed  (cam:=true 일 때)
  COLL  127.0.0.1:9092  CollisionData    -> 로그 출력(포맷 덤프)
  COMP  127.0.0.1:9087  CompetitionInfo  -> 로그 출력(포맷 덤프)
송신(ROS -> UDP):
  /ctrl_cmd (morai_msgs/CtrlCmd) -> '#MoraiCtrlCmd$' -> 시뮬 127.0.0.1:9093

사용:
  source ~/catkin_ws/devel/setup.bash   # morai_msgs
  rosrun 없이:  python3 ~/control_ws/src/erp_42/aisw_udp_bridge.py _cam:=false
파라미터(rosparam ~네임스페이스):
  ~sim_ip (기본 127.0.0.1)  대회 당일 시뮬 PC IP로 변경
  ~ctrl_port 9093 | ~gps_port 9281 | ~imu_port 9283 | ~cam(false) | ~dump(true)
  ~east_offset / ~north_offset : GPSMessage offset (NMEA에는 없어 파라미터로 주입.
      연습 시 ROS 모드 /gps 의 eastOffset/northOffset 값을 확인해 넣을 것)
  ~gear 4(D) | ~ctrl_mode 2(AutoMode)
* LiDAR(VLP16)는 velodyne 표준 패킷(포트 2368)이라 velodyne 드라이버 사용:
  roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="" port:=2368
"""
import socket, struct, threading, math
import rospy
from sensor_msgs.msg import Imu, CompressedImage
from morai_msgs.msg import GPSMessage, CtrlCmd, EgoVehicleStatus
from pyproj import Proj

def udp_sock(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.settimeout(1.0)
    s.bind(('0.0.0.0', port))
    return s

def nmea_deg(v, hemi):
    # ddmm.mmmm -> 십진도
    if not v: return 0.0
    f = float(v)
    d = int(f // 100)
    m = f - d * 100
    deg = d + m / 60.0
    if hemi in ('S', 'W'): deg = -deg
    return deg

class Bridge:
    def __init__(self):
        rospy.init_node('aisw_udp_bridge')
        gp = lambda n, d: rospy.get_param('~' + n, d)
        self.sim_ip   = gp('sim_ip', '127.0.0.1')
        self.ctrl_to  = (self.sim_ip, int(gp('ctrl_port', 9093)))
        self.eoff     = float(gp('east_offset', 302595.0))   # K-City 2025 mgeo local_origin (global_info.json)
        self.noff     = float(gp('north_offset', 4124145.0))
        self.gear     = int(gp('gear', 4))        # 1P 2R 3N 4D
        self.cmode    = int(gp('ctrl_mode', 2))   # 2=AutoMode
        self.dump     = bool(gp('dump', True))
        self.use_cam  = bool(gp('cam', False))

        self.pub_gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        # CompetitionInfo UDP 패킷 포맷 확정 전까지 GPS 미분으로 속도 추정해 /Competition_topic 대체 발행
        self.pub_comp = rospy.Publisher('/Competition_topic', EgoVehicleStatus, queue_size=1) if gp('pub_competition', True) else None
        self._utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self._prev_fix = None  # (t, ex, ny)
        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=1)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.cb_ctrl, queue_size=1)

        self.threads = []
        self.spawn(self.loop_gps, int(gp('gps_port', 9281)))
        self.spawn(self.loop_imu, int(gp('imu_port', 9283)))
        if self.dump:
            self.spawn(self.loop_dump, 9092, 'CollisionData')
            self.spawn(self.loop_dump, 9087, 'CompetitionInfo')
        if self.use_cam:
            for port, topic in ((9291, '/image_jpeg/compressed'),
                                (9293, '/image_jpeg_left/compressed'),
                                (9295, '/image_jpeg_right/compressed')):
                self.spawn(self.loop_cam, port, topic)
        # [2026_AISW] 제어 두절 워치독: /ctrl_cmd 0.6초 이상 끊기면 정지 패킷 송신 (노드 재시작 중 마지막 명령 래치 방지)
        self._last_ctrl_t = None
        rospy.Timer(rospy.Duration(0.1), self._watchdog)
        rospy.on_shutdown(self._send_stop_burst)
        rospy.loginfo('[aisw_udp_bridge] 시작 — 시뮬 %s, ctrl->%s', self.sim_ip, self.ctrl_to)

    def _stop_pkt(self):
        data = struct.pack('<BBB5f', self.cmode, self.gear, 1, 0.0, 0.0, 0.0, 0.6, 0.0)
        return b'#MoraiCtrlCmd$' + struct.pack('<i', len(data)) + b'\x00'*12 + data + b'\r\n'

    def _watchdog(self, _evt):
        if self._last_ctrl_t is not None and (rospy.Time.now() - self._last_ctrl_t).to_sec() > 0.6:
            try: self.tx.sendto(self._stop_pkt(), self.ctrl_to)
            except OSError: pass

    def _send_stop_burst(self):
        for _ in range(5):
            try: self.tx.sendto(self._stop_pkt(), self.ctrl_to)
            except OSError: pass

    def spawn(self, fn, *a):
        t = threading.Thread(target=fn, args=a, daemon=True)
        t.start(); self.threads.append(t)

    # ---------- ROS -> UDP : Ego Ctrl Cmd ----------
    def cb_ctrl(self, m):
        self._last_ctrl_t = rospy.Time.now()
        # '#MoraiCtrlCmd$' + int32 len(23) + aux12 + [mode u8, gear u8, cmdType u8, vel f, accval f, accel f, brake f, steer f] + \r\n
        data = struct.pack('<BBB5f', self.cmode, self.gear,
                           m.longlCmdType if m.longlCmdType else 1,
                           m.velocity, m.acceleration, m.accel, m.brake, -m.steering)  # [2026_AISW] MORAI 조향부호 반대 → 반전(실측: 우회전 명령에 차가 좌로 감)
        pkt = b'#MoraiCtrlCmd$' + struct.pack('<i', len(data)) + b'\x00'*12 + data + b'\r\n'
        try: self.tx.sendto(pkt, self.ctrl_to)
        except OSError: pass

    # ---------- GPS: NMEA GGA ----------
    def loop_gps(self, port):
        s = udp_sock(port)
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(8192)
            except socket.timeout: continue
            except OSError: break
            for line in raw.decode('ascii', 'ignore').splitlines():
                if 'GGA' not in line: continue
                f = line.split(',')
                if len(f) < 10: continue
                msg = GPSMessage()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'gps'
                try:
                    msg.latitude  = nmea_deg(f[2], f[3])
                    msg.longitude = nmea_deg(f[4], f[5])
                    msg.altitude  = float(f[9]) if f[9] else 0.0
                except ValueError: continue
                msg.eastOffset, msg.northOffset = self.eoff, self.noff
                msg.status = 1
                self.pub_gps.publish(msg)
                if self.pub_comp:
                    ux, uy = self._utm(msg.longitude, msg.latitude)
                    ex, ny = ux - self.eoff, uy - self.noff
                    t = msg.header.stamp.to_sec()
                    ego = EgoVehicleStatus(); ego.header.stamp = msg.header.stamp
                    ego.position.x, ego.position.y, ego.position.z = ex, ny, msg.altitude
                    if self._prev_fix is not None and t > self._prev_fix[0]:
                        dt = t - self._prev_fix[0]
                        if dt < 1.0:
                            raw_v = ((ex-self._prev_fix[1])**2 + (ny-self._prev_fix[2])**2) ** 0.5 / dt  # m/s
                            # [2026_AISW] EMA 저역필터 — 미분 노이즈가 PID D항 채터링(브레이크등 점멸) 유발 방지
                            self._v_ema = 0.25*raw_v + 0.75*getattr(self, '_v_ema', raw_v)
                            ego.velocity.x = self._v_ema
                    self._prev_fix = (t, ex, ny)
                    self.pub_comp.publish(ego)

    # ---------- IMU ----------
    def loop_imu(self, port):
        s = udp_sock(port)
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            if raw[0:9] != b'#IMUData$' or len(raw) < 113: continue
            d = struct.unpack('10d', raw[33:113])  # w x y z gx gy gz ax ay az
            m = Imu()
            m.header.stamp = rospy.Time.now(); m.header.frame_id = 'imu'
            m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z = d[0], d[1], d[2], d[3]
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = d[4], d[5], d[6]
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = d[7], d[8], d[9]
            self.pub_imu.publish(m)

    # ---------- Camera: JPEG 조각 (tail 'EI'가 마지막) ----------
    def loop_cam(self, port, topic):
        pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
        s = udp_sock(port)
        buf = b''
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65000)
            except socket.timeout: buf = b''; continue
            except OSError: break
            if len(raw) < 21: continue
            buf += raw[19:-2]
            if raw[-2:] == b'EI':
                m = CompressedImage()
                m.header.stamp = rospy.Time.now()
                m.format = 'jpeg'; m.data = buf
                pub.publish(m)
                buf = b''

    # ---------- 미해석 채널 덤프(포맷 확인용) ----------
    def loop_dump(self, port, name):
        s = udp_sock(port)
        last = 0
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            now = rospy.Time.now().to_sec()
            if now - last > 2.0:
                head = raw[:24]
                rospy.loginfo('[%s] %dB header=%r', name, len(raw), head)
                last = now

if __name__ == '__main__':
    Bridge()
    rospy.spin()
