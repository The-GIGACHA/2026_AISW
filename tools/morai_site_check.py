#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[2026_AISW] 대회 현장(시뮬 PC <-> 팀 PC LAN) 점검 도구. ROS 불필요, 제어 스택은 끄고 실행.

  1) 수신 점검 (차량 안 움직임)
     python3 morai_site_check.py
     python3 morai_site_check.py --scan          # 포트를 모를 때: 908~65535 전체에서 MORAI 패킷 탐색
  2) 왕복 지연 (차량 안 움직임: brake만 0.3/0.8 교대로 송신 → Status의 brake 에코 시간 측정)
     python3 morai_site_check.py --echo --sim-ip <시뮬PC IP>
  3) 조향 부호/이득·속도 단위 재측정 (차량이 S자로 약 30m 움직임)
     python3 morai_site_check.py --steer --sim-ip <시뮬PC IP>
  4) 카메라 스냅샷 저장 (렌더링 비교용, 차량 안 움직임)
     python3 morai_site_check.py --snapshot ~/site_cam/start_point

기본 포트는 aisw_udp_bridge.py와 동일: GPS 9281, IMU 9283, Status 909, Collision 9092,
Camera 9291/9293/9295, LiDAR 2368, Ctrl Cmd -> 시뮬 9093
"""
import argparse, collections, math, os, selectors, socket, struct, sys, threading, time
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
from morai_camera import JpegAssembler   # aisw_udp_bridge와 같은 조립 로직

PORTS = {9281: "GPS", 9283: "IMU", 909: "Status", 9092: "Collision",
         9291: "Camera front", 9293: "Camera left", 9295: "Camera right", 2368: "LiDAR"}
E0, N0 = 302595.0, 4124145.0   # K-City offset (aisw_udp_bridge와 동일)
VLP = {0x21: "HDL-32E", 0x22: "VLP-16", 0x28: "VLP-32C"}


def open_sock(port, bufsize=8 << 20):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, bufsize)
    s.bind(("0.0.0.0", port))
    s.setblocking(False)
    return s


def kind(raw):
    if raw.startswith(b"#MoraiInfo$"): return "Status"
    if raw.startswith(b"#IMUData$"): return "IMU"
    if raw.startswith(b"$GP"): return "GPS"
    if raw.startswith(b"#CollisionData$"): return "Collision"
    if raw.startswith(b"#MoraiObjInfo$"): return "ObjectInfo(규정외)"
    if raw.startswith(b"MOR"): return "Camera"
    if len(raw) == 1206: return "LiDAR"
    return "unknown"


def nmea_deg(v, h):
    f = float(v); d = int(f // 100); deg = d + (f - d * 100) / 60.0
    return -deg if h in "SW" else deg


def decode_status(raw):
    d = raw[27:]
    mode, gear = struct.unpack_from("<bb", d, 8)
    acc, brk = struct.unpack_from("<ff", d, 18)
    sx, sy = struct.unpack_from("<ff", d, 26); wb = struct.unpack_from("<f", d, 42)[0]
    yaw = struct.unpack_from("<f", d, 70)[0]; vx = struct.unpack_from("<f", d, 74)[0]
    return dict(mode=mode, gear=gear, accel=acc, brake=brk, size=(sx, sy), wheelbase=wb, yaw=yaw, vel_x=vx)


def imu_yaw(raw):
    w, x, y, z = struct.unpack_from("<4d", raw, 33)
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def passive(args):
    sel = selectors.DefaultSelector()
    ports = range(908, 65536) if args.scan else PORTS
    n_open = 0
    for p in ports:
        try: sel.register(open_sock(p, 1 << 16 if args.scan else 8 << 20), selectors.EVENT_READ, p); n_open += 1
        except OSError as e:
            if not args.scan: print("  [!] %d(%s) bind 실패: %s%s" % (p, PORTS[p], e, " → 이미 사용 중(velodyne 드라이버/aisw_udp_bridge 실행 중이면 끄고 재실행)" if e.errno == 98 else ""))
    print("수신 대기 %d개 포트, %.0f초..." % (n_open, args.duration))
    st = collections.defaultdict(lambda: dict(n=0, bytes=0, src=collections.Counter(), kinds=collections.Counter(), last=None, asm=JpegAssembler(), stamps=[], jpeg=None, az=[]))
    t0 = time.time()
    while time.time() - t0 < args.duration:
        for key, _ in sel.select(0.2):
            try: raw, addr = key.fileobj.recvfrom(65535)
            except OSError: continue
            s = st[key.data]; s["n"] += 1; s["bytes"] += len(raw); s["src"][addr[0]] += 1; k = kind(raw); s["kinds"][k] += 1; s["last"] = raw
            if k == "Camera":
                f = s["asm"].feed(raw)
                if f: s["stamps"].append(f[0] + f[1] * 1e-9); s["jpeg"] = f[2]
            if k == "LiDAR" and len(s["az"]) < 3000:
                s["az"].append(struct.unpack_from("<H", raw, 2)[0] / 100.0)
    dur = time.time() - t0
    print("\n%-6s %-20s %8s %9s  %-18s %s" % ("port", "종류", "pkt/s", "Mbps", "송신 IP", "내용"))
    for p in sorted(st):
        s = st[p]; k = s["kinds"].most_common(1)[0][0]; raw = s["last"]; info = ""
        try:
            if k == "GPS":
                ln = [l for l in raw.decode(errors="ignore").splitlines() if "GGA" in l or "RMC" in l][0].split(",")
                i = 2 if "GGA" in ln[0] else 3
                lat, lon = nmea_deg(ln[i], ln[i + 1]), nmea_deg(ln[i + 2], ln[i + 3])
                info = "lat %.6f lon %.6f%s" % (lat, lon, "  ← 좌표 0 (음영구역?)" if lat == 0 else "")
            elif k == "IMU":
                info = "yaw %.1f deg, acc_z %.2f (정지시 ~9.81)" % (imu_yaw(raw), struct.unpack_from("<d", raw, 105)[0])
            elif k == "Status":
                d = decode_status(raw)
                info = "ctrl_mode %d gear %d vel_x %.2f(km/h) yaw %.1f size %.3fx%.3f wb %.3f" % (d["mode"], d["gear"], d["vel_x"], d["yaw"], d["size"][0], d["size"][1], d["wheelbase"])
            elif k == "Camera":
                info = camera_info(s, dur, args.snapshot, p)
            elif k == "LiDAR":
                steps = [(b - a) % 360 for a, b in zip(s["az"], s["az"][1:]) if 0 < (b - a) % 360 < 10]
                rpm = (sum(steps) / len(steps)) * s["n"] / dur / 6 if steps else 0
                info = "model %s, ~%.0f rpm" % (VLP.get(raw[1205], hex(raw[1205])), rpm)
            elif k == "unknown":
                info = "len %d head %r" % (len(raw), raw[:16])
        except Exception as e:
            info = "디코딩 실패: %s" % e
        print("%-6d %-20s %8.1f %9.2f  %-18s %s" % (p, (PORTS.get(p, "") + " " + k).strip(), s["n"] / dur, s["bytes"] * 8 / dur / 1e6, ",".join(s["src"]), info))
    missing = [("%d(%s)" % (p, n)) for p, n in PORTS.items() if p not in st]
    if missing and not args.scan: print("\n[!] 수신 없음: %s  → MORAI Destination IP/Port, Connect, 방화벽 확인 (--scan으로 실제 포트 탐색)" % ", ".join(missing))
    total = sum(s["bytes"] for s in st.values()) * 8 / dur / 1e6
    print("\n총 수신 대역폭 %.1f Mbps" % total)


def camera_info(s, dur, snapshot_dir, port):
    """프레임률, 조립 실패, 헤더 타임스탬프 간격으로 본 누락 프레임, 이미지 크기"""
    asm, ts = s["asm"], s["stamps"]
    gaps = [b - a for a, b in zip(ts, ts[1:]) if b > a]
    # 프레임 타임스탬프 간격이 33~89ms로 흔들리므로(평균 50ms) 개별 간격이 아니라 전체 구간 기대 프레임 수와 비교
    period = sum(gaps) / len(gaps) if gaps else 0.05
    missing = max(0, round((ts[-1] - ts[0]) / 0.05) + 1 - len(ts)) if len(ts) > 1 else 0
    size = ""
    try:
        import cv2, numpy as np
        img = cv2.imdecode(np.frombuffer(s["jpeg"], np.uint8), cv2.IMREAD_COLOR) if s["jpeg"] else None
        size = "%dx%d" % (img.shape[1], img.shape[0]) if img is not None else "디코딩 실패!"
        if snapshot_dir and img is not None:
            os.makedirs(snapshot_dir, exist_ok=True)
            path = os.path.join(snapshot_dir, "cam_%d.jpg" % port)
            open(path, "wb").write(s["jpeg"]); size += " → " + path
    except ImportError:
        pass
    return "%.1f fps, 조립실패 %d, 누락(20Hz 기준) %d, 간격 평균 %.0f/최대 %.0f ms, %d KB %s" % (
        asm.frames / dur, asm.dropped, missing, period * 1000, max(gaps) * 1000 if gaps else 0, len(s["jpeg"] or b"") // 1024, size)


def ctrl_pkt(accel, brake, steer, mode=2, gear=4):
    data = struct.pack("<BBB5f", mode, gear, 1, 0.0, 0.0, accel, brake, steer)
    return b"#MoraiCtrlCmd$" + struct.pack("<i", len(data)) + b"\x00" * 12 + data + b"\r\n"


class Receiver(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.status, self.imu, self.gps, self.stop, self.errors, self.last_error = [], [], [], False, 0, None
        self.socks = {909: open_sock(909), 9283: open_sock(9283), 9281: open_sock(9281)}

    def run(self):
        sel = selectors.DefaultSelector()
        for p, s in self.socks.items(): sel.register(s, selectors.EVENT_READ, p)
        while not self.stop:
            for key, _ in sel.select(0.2):
                try: raw, _ = key.fileobj.recvfrom(65535)
                except OSError: continue
                t = time.time()
                try: self.handle(key.data, raw, t)
                except Exception as e: self.errors += 1; self.last_error = repr(e)

    def handle(self, port, raw, t):
        if port == 909 and raw.startswith(b"#MoraiInfo$"): self.status.append((t, decode_status(raw)))
        elif port == 9283 and raw.startswith(b"#IMUData$"): self.imu.append((t, imu_yaw(raw), struct.unpack_from("<d", raw, 81)[0]))
        elif port == 9281:
            for l in raw.decode(errors="ignore").splitlines():
                f = l.split(",")
                if "GGA" in f[0] and f[2]:
                    lat, lon = nmea_deg(f[2], f[3]), nmea_deg(f[4], f[5])
                    if lat: self.gps.append((t, lat, lon))


def echo(args):
    rx = Receiver(); rx.start(); tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); dst = (args.sim_ip, 9093)
    print("brake 0.3/0.8 교대 송신 → %s:9093, Status(909) brake 에코 측정 (차량 정지 유지)" % args.sim_ip)
    delays = []
    for k in range(10):
        target = 0.8 if k % 2 == 0 else 0.3
        t_send = time.time(); got = None
        while time.time() - t_send < 2.0:
            tx.sendto(ctrl_pkt(0.0, target, 0.0), dst)
            for t, d in reversed(rx.status):
                if t < t_send: break
                if abs(d["brake"] - target) < 0.01: got = t
            if got: break
            time.sleep(0.01)
        delays.append(got - t_send if got else None)
    for _ in range(5): tx.sendto(ctrl_pkt(0.0, 1.0, 0.0), dst); time.sleep(0.02)
    rx.stop = True
    ok = [d for d in delays if d is not None]
    if not rx.status: print("[!] Status 수신 없음 — 909 포트/MORAI Competition Vehicle Status 확인"); return
    iv = [b[0] - a[0] for a, b in zip(rx.status, rx.status[1:])]
    print("Status 수신 %.1f Hz (간격 최대 %.0f ms)" % (1 / (sum(iv) / len(iv)), max(iv) * 1000))
    if ok: print("명령→Status 반영 왕복지연: 중앙 %.0f ms, 최대 %.0f ms (%d/%d 성공, Status 주기 포함)" % (sorted(ok)[len(ok) // 2] * 1000, max(ok) * 1000, len(ok), len(delays)))
    else: print("[!] 에코 없음 — Ctrl Cmd 미수신(시뮬 PC Host IP/9093/Connect/Windows 방화벽) 또는 ctrl_mode 확인")


def steer(args):
    from pyproj import Proj
    utm = Proj(proj="utm", zone=52, ellps="WGS84")
    rx = Receiver(); rx.start(); tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); dst = (args.sim_ip, 9093)
    phases = [("accel", 4.0, 0.35, 0.0, 0.0), ("straight", 2.0, 0.2, 0.0, 0.0), ("left", 2.5, 0.2, 0.0, +0.15), ("right", 2.5, 0.2, 0.0, -0.15), ("brake", 3.0, 0.0, 1.0, 0.0)]
    marks = []
    print("주의: 차량이 S자로 약 30m 이동합니다 (3초 후 시작)"); time.sleep(3)
    for name, dur, a, b, s in phases:
        marks.append((name, time.time())); te = time.time() + dur
        while time.time() < te: tx.sendto(ctrl_pkt(a, b, s), dst); time.sleep(0.05)
    marks.append(("end", time.time())); rx.stop = True
    print("수신: Status %d, IMU %d, GPS(유효좌표) %d, 파싱오류 %d %s" % (len(rx.status), len(rx.imu), len(rx.gps), rx.errors, rx.last_error or ""))

    def win(arr, name):
        t0, t1 = [(m[1], n[1]) for m, n in zip(marks, marks[1:]) if m[0] == name][0]
        return [r for r in arr if t0 + 0.5 <= r[0] < t1]   # 조향 지연(0.2s)+과도응답 제외
    g = win(rx.gps, "straight"); st = win(rx.status, "straight")
    if len(g) > 5 and st:
        x0, y0 = utm(g[0][2], g[0][1]); x1, y1 = utm(g[-1][2], g[-1][1])
        v_gps = math.hypot(x1 - x0, y1 - y0) / (g[-1][0] - g[0][0]); v_st = sum(d["vel_x"] for _, d in st) / len(st)
        imu_s = win(rx.imu, "straight"); track = math.degrees(math.atan2(y1 - y0, x1 - x0))
        print("속도: GPS %.2f m/s, Status vel_x %.2f → 비율 %.2f (3.6이면 km/h, 브리지 /3.6 유지)" % (v_gps, v_st, v_st / max(v_gps, 1e-3)))
        if imu_s: print("IMU yaw %.1f vs GPS 진행방향 %.1f deg (차이 수 도 이내면 정상)" % (imu_s[-1][1], track))
    for name, cmd in (("left", 0.15), ("right", -0.15)):
        im = win(rx.imu, name); stt = win(rx.status, name)
        if not im or not stt: print("[!] %s 구간 데이터 부족" % name); continue
        wz = sum(r[2] for r in im) / len(im); v = sum(d["vel_x"] for _, d in stt) / len(stt) / 3.6
        eff = math.atan(wz * 3.0 / max(v, 0.1))
        print("조향 %+.2f rad: yaw rate %+.3f rad/s @ %.2f m/s → 유효조향 %+.3f rad, 이득 %.3f (부호 %s)" % (cmd, wz, v, eff, eff / cmd, "정상(+좌)" if eff * cmd > 0 else "반대!"))
    print("→ aisw_udp_bridge ~steer_scale = 1/이득 (현재 기본 1/0.575 = 1.739)")


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scan", action="store_true"); ap.add_argument("--echo", action="store_true"); ap.add_argument("--steer", action="store_true")
    ap.add_argument("--sim-ip", default="127.0.0.1"); ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--snapshot", metavar="DIR", help="수신 점검과 함께 카메라별 마지막 프레임을 DIR에 jpg로 저장")
    a = ap.parse_args()
    if a.echo: echo(a)
    elif a.steer: steer(a)
    else: passive(a)
