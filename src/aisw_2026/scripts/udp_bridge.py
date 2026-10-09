#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2026 국토부 AI/SW 모빌리티 경진대회용 UDP <-> ROS 게이트웨이 (팀 PC에서 실행)
- 대회 규정상 시뮬레이터와의 통신은 전부 UDP. 이 노드가 UDP를 ROS 토픽으로 변환해
  ROS 기반 제어 노드들이 토픽으로 쓸 수 있게 한다.

수신(UDP -> ROS):
  GPS   127.0.0.1:9281  NMEA(GPGGA)      -> /gps (morai_msgs/GPSMessage)
  IMU   127.0.0.1:9283  '#IMUData$'      -> /imu (sensor_msgs/Imu)
  CAM   카메라 3대 (cam:=true 일 때만, JPEG 조각 → control/camera_packet.py 로 조립)
          전방 9291 -> /image_jpeg/compressed
          좌측 9293 -> /image_jpeg_left/compressed
          우측 9295 -> /image_jpeg_right/compressed
  COLL  127.0.0.1:9092  CollisionData    -> /CollisionData
  STAT  127.0.0.1:909   '#MoraiInfo$' Competition Vehicle Status (MORAI 기본 host 908 -> dest 909) -> /Competition_topic velocity.x (vel_x)
송신(ROS -> UDP):
  /ctrl_cmd (morai_msgs/CtrlCmd) -> '#MoraiCtrlCmd$' -> 시뮬 127.0.0.1:9093

사용:
  source <morai_msgs 가 있는 워크스페이스>/devel/setup.bash
  rosrun 없이:  python3 scripts/udp_bridge.py _cam:=false
파라미터(rosparam ~네임스페이스):
  ~sim_ip (기본 127.0.0.1)  대회 당일 시뮬 PC IP로 변경
  ~ctrl_port 9093 | ~gps_port 9281 | ~imu_port 9283 | ~status_port 909 | ~collision_port 9092 | ~dump(true)
  ~cam(false) | ~cam_front_port 9291 | ~cam_left_port 9293 | ~cam_right_port 9295
  ~east_offset / ~north_offset : GPSMessage offset (NMEA에는 없어 파라미터로 주입.
      연습 시 ROS 모드 /gps 의 eastOffset/northOffset 값을 확인해 넣을 것)
  ~gear 4(D) | ~ctrl_mode 2(AutoMode) | ~steer_scale 1/0.575 (MORAI 조향 이득 보상)
* LiDAR(VLP16)는 velodyne 표준 패킷(포트 2368)이라 velodyne 드라이버 사용:
  roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="" port:=2368
"""
import socket, struct, threading, math
from collections import deque
import rospy
from sensor_msgs.msg import Imu, CompressedImage
from morai_msgs.msg import GPSMessage, CtrlCmd, EgoVehicleStatus, CollisionData, ObjectStatus
from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from pyproj import Proj
from control.camera_packet import JpegAssembler

def udp_sock(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # LAN 수신 대비 버퍼 8MB 요청 (카메라 65KB datagram). 실제 상한은 net.core.rmem_max
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
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
        rospy.init_node('udp_bridge')
        gp = lambda n, d: rospy.get_param('~' + n, d)
        self.sim_ip   = gp('sim_ip', '127.0.0.1')
        self.ctrl_to  = (self.sim_ip, int(gp('ctrl_port', 9093)))
        self.eoff     = float(gp('east_offset', 302595.0))   # K-City 2025 mgeo local_origin (global_info.json)
        self.noff     = float(gp('north_offset', 4124145.0))
        self.gear     = int(gp('gear', 4))        # 1P 2R 3N 4D
        # MORAI 유효 조향각 = 0.575 x 명령 (실측: yaw rate = 0.57*v*tan(cmd)/wb, 명령 3~40deg 전 구간 비율 0.57~0.59, 지연 0.2s)
        # 보상 안 하면 조향 부족 → 급커브(R8~10m)에서 바깥으로 1~1.5m 밀림
        self.steer_scale = float(gp('steer_scale', 1.0 / 0.575))
        self.cmode    = int(gp('ctrl_mode', 2))   # 2=AutoMode
        self.dump     = bool(gp('dump', True))
        self.use_cam  = bool(gp('cam', False))

        self.pub_gps = rospy.Publisher('/gps', GPSMessage, queue_size=1)
        # /Competition_topic: 위치=GPS(규정상 Status에 pos 없음), 속도=Competition Vehicle Status vel_x
        # (Status 0.5초 이상 끊기면 GPS 0.5초 창 추정속도로 대체)
        self.pub_comp = rospy.Publisher('/Competition_topic', EgoVehicleStatus, queue_size=1) if gp('pub_competition', True) else None
        self._utm = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
        self._fix_hist = deque()  # (t, ex, ny) 최근 GPS 위치 — 속도 추정 창
        self._v_ema = 0.0
        self._status_vel = None  # (수신시각, vel_x [m/s])
        self.pub_imu = rospy.Publisher('/imu', Imu, queue_size=1)
        # 현재 MGeo 링크 ID (Status @114). 규정 속도예외 구간(A2256W000411~000153) 판정·링크↔인덱스 표 작성용
        self.pub_link = rospy.Publisher('/aisw/link_id', String, queue_size=1)
        self._last_link = None
        self._last_gps_pub = 0.0
        self._last_ego_xyz = (0.0, 0.0, 0.0)
        # CollisionData (UDP 9092) → /CollisionData (충돌 감시용)
        self.pub_coll = rospy.Publisher('/CollisionData', CollisionData, queue_size=1)
        self.pub_truth = rospy.Publisher('/aisw/debug/truth_xyz', PointStamped, queue_size=1)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self.cb_ctrl, queue_size=1)

        self.threads = []
        self.spawn(self.loop_gps, int(gp('gps_port', 9281)))
        self.spawn(self.loop_imu, int(gp('imu_port', 9283)))
        self.spawn(self.loop_status, int(gp('status_port', 909)))
        self.spawn(self.loop_collision, int(gp('collision_port', 9092)))
        if self.use_cam:
            for port, topic in ((int(gp('cam_front_port', 9291)), '/image_jpeg/compressed'),
                                (int(gp('cam_left_port', 9293)), '/image_jpeg_left/compressed'),
                                (int(gp('cam_right_port', 9295)), '/image_jpeg_right/compressed')):
                self.spawn(self.loop_cam, port, topic)
        # 제어 두절 워치독: /ctrl_cmd 0.6초 이상 끊기면 정지 패킷 송신 (노드 재시작 중 마지막 명령 래치 방지)
        self._last_ctrl_t = None
        rospy.Timer(rospy.Duration(0.1), self._watchdog)
        rospy.on_shutdown(self._send_stop_burst)
        rospy.loginfo('[udp_bridge] 시작 — 시뮬 %s, ctrl->%s', self.sim_ip, self.ctrl_to)

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
                           m.velocity, m.acceleration, m.accel, m.brake, m.steering * self.steer_scale)  # 반전 제거: 실측(25.S4.MolitComp03) 패킷 steering +0.3 → yaw +59° 좌회전, ROS 규약(+좌)과 동일
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
                # GPS 음영(제밍)구역: MORAI가 '0000.0000,N,00000.0000,E'(RMC는 A=유효)를 계속 송신
                # → (0,0)을 발행하면 자차가 수천km 밖으로 튀어 풀가속/최대조향 발생했음. 발행 중단 → 제어기 GPS두절 로직(크리프/정지) 동작
                if msg.latitude == 0.0 or msg.longitude == 0.0:
                    rospy.logwarn_throttle(2.0, '[udp_bridge] GPS 좌표 0 (음영구역) — /gps 발행 중단')
                    continue
                msg.eastOffset, msg.northOffset = self.eoff, self.noff
                msg.status = 1
                self.pub_gps.publish(msg)
                if self.pub_comp:
                    ux, uy = self._utm(msg.longitude, msg.latitude)
                    ex, ny = ux - self.eoff, uy - self.noff
                    t = msg.header.stamp.to_sec()
                    ego = EgoVehicleStatus(); ego.header.stamp = msg.header.stamp
                    ego.position.x, ego.position.y, ego.position.z = ex, ny, msg.altitude
                    # 속도 = 0.5초 창 변위/시간. NMEA 해상도(~0.2m)+동일 fix 반복(약 25%) 때문에
                    # 인접 fix 미분은 0~19m/s로 튐 → PID D항 채터링(accel/brake 반복) 원인이었음
                    hist = self._fix_hist
                    hist.append((t, ex, ny))
                    while len(hist) > 2 and t - hist[1][0] >= 0.5:
                        hist.popleft()
                    span = t - hist[0][0]
                    if span > 1.0:  # GPS 두절 후 재개: 창 초기화
                        hist.clear(); hist.append((t, ex, ny)); self._v_ema = 0.0
                    elif span >= 0.3:
                        raw_v = ((ex-hist[0][1])**2 + (ny-hist[0][2])**2) ** 0.5 / span  # m/s
                        self._v_ema = 0.5*raw_v + 0.5*self._v_ema
                    st = self._status_vel
                    ego.velocity.x = st[1] if st is not None and t - st[0] < 0.5 else self._v_ema
                    self.pub_comp.publish(ego)
                    self._last_gps_pub = t
                    self._last_ego_xyz = (ex, ny, msg.altitude)

    # ---------- Competition Vehicle Status ----------
    def loop_status(self, port):
        # '#MoraiInfo$'(11) + int32 len(152) + aux12 + data152 + '\r\n' = 181B
        # data: sec,nsec,ctrl_mode,gear,signed_vel,map_id,accel,brake,size3,overhang,wheelbase,rear_overhang(@0..49)
        #       pos3(@50, 규정상 0) rpy3(@62, deg) vel3(@74, km/h, vel_x만 제공) ang_vel3(@86) accel3(@98, 0) steer(@110) link_id(@114)
        try: s = udp_sock(port)
        except OSError as e:
            # 909는 1024 미만 특권포트 → sysctl net.ipv4.ip_unprivileged_port_start=908 필요
            rospy.logerr('[udp_bridge] Status 포트 %d bind 실패(%s) — GPS 추정속도로 대체. '
                         'sudo sysctl -w net.ipv4.ip_unprivileged_port_start=908', port, e)
            return
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            if raw[0:11] != b'#MoraiInfo$' or len(raw) < 27 + 114: continue
            # vel_x 단위 km/h (실측: GPS 1초 변위 속도 대비 정확히 3.6배) -> m/s
            self._status_vel = (rospy.Time.now().to_sec(), struct.unpack_from('<f', raw, 27 + 74)[0] / 3.6)
            # GPS 음영/두절 중에도 속도는 계속 발행 (추측항법이 실제 속도로 적분하도록)
            if self.pub_comp and self._status_vel[0] - self._last_gps_pub > 0.3:
                ego = EgoVehicleStatus(); ego.header.stamp = rospy.Time.now()
                ego.position.x, ego.position.y, ego.position.z = self._last_ego_xyz   # GPS 없음: 마지막 위치 (사용 안 함)
                ego.velocity.x = self._status_vel[1]
                self.pub_comp.publish(ego)
            # link_id: data @114 ~ 끝(152) 고정길이 문자열, NUL 패딩
            link = raw[27 + 114:27 + 152].split(b'\x00', 1)[0].decode('ascii', 'ignore').strip()
            if link:
                self.pub_link.publish(String(link))
                if link != self._last_link:
                    rospy.loginfo_throttle(1.0, '[udp_bridge] link_id=%s', link)
                    self._last_link = link

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

    # ---------- Camera: MORAI JPEG 조각 조립 (JpegAssembler) ----------
    def loop_cam(self, port, topic):
        pub = rospy.Publisher(topic, CompressedImage, queue_size=1)
        s = udp_sock(port)
        asm = JpegAssembler(); reported = 0
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65535)
            except socket.timeout: asm.reset(); continue
            except OSError: break
            frame = asm.feed(raw)
            if asm.dropped != reported:
                reported = asm.dropped
                rospy.logwarn_throttle(5.0, '[udp_bridge] %s 손실 프레임 누적 %d / 정상 %d', topic, asm.dropped, asm.frames)
            if frame is None: continue
            m = CompressedImage()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = topic.split('/')[1]
            m.format = 'jpeg'; m.data = frame[2]
            pub.publish(m)

    # ---------- 미해석 채널 덤프(포맷 확인용) ----------
    def loop_collision(self, port):
        # '#CollisionData$'(15) + int32 len + aux12 + data: sec,nsec(i4,i4) + 객체 type(u16, 0xffff=충돌 없음) ...
        s = udp_sock(port)
        while not rospy.is_shutdown():
            try: raw, _ = s.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            if raw[0:15] != b'#CollisionData$' or len(raw) < 31 + 12: continue
            b = raw[31:]
            m = CollisionData(); m.header.stamp = rospy.Time.now()
            otype = struct.unpack_from('<H', b, 8)[0]
            if otype != 0xFFFF:
                o = ObjectStatus(); o.type = otype
                o.position.x, o.position.y, o.position.z = struct.unpack_from('<3f', b, 12)
                m.collision_object.append(o)
                rospy.logwarn_throttle(1.0, '[udp_bridge] 충돌! type=%d', otype)
            self.pub_coll.publish(m)
            # 검증 전용: 패킷에 실린 자차 위치(음영에서도 나옴)를 추측항법 오차 평가용으로만 발행.
            # 제어에 쓰면 GPS 음영 미션을 우회하는 셈이라 주행 스택은 구독하지 않는다.
            if otype == 0xFFFF:
                pt = PointStamped(); pt.header.stamp = m.header.stamp; pt.header.frame_id = 'map'
                pt.point.x, pt.point.y, pt.point.z = struct.unpack_from('<3f', b, 12)
                self.pub_truth.publish(pt)

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
