#!/usr/bin/env python3
"""연습용 복구 주행: 스택이 꺼진 상태에서 차를 전역경로로 되돌려 GPS 가 다시 잡힐 때까지(또는 목표 인덱스까지) 저속 주행.
위치 = CollisionData 패킷의 자차 위치(음영에서도 나옴), 방향 = Status yaw. 대회 스택과 무관한 연습 도구."""
import math, os, socket, struct, sys, time
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts'))
from control.path_utils import load_map_fields

rx, ry = [np.array(a) for a in load_map_fields()[:2]]
s_cum = np.concatenate([[0], np.cumsum(np.hypot(np.diff(rx), np.diff(ry)))])
STEER_SCALE = 1.0 / 0.575
TARGET_KPH = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
STOP_IDX = int(sys.argv[2]) if len(sys.argv) > 2 else -1


def sock(p):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(('0.0.0.0', p)); s.setblocking(False)
    return s


co, st, gp = sock(9092), sock(909), sock(9281)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def latest(s):
    d = None
    while True:
        try:
            d = s.recvfrom(65535)[0]
        except BlockingIOError:
            return d


def pkt(acc, brk, steer):
    d = struct.pack('<BBB5f', 2, 4, 1, 0.0, 0.0, acc, brk, steer * STEER_SCALE)
    return b'#MoraiCtrlCmd$' + struct.pack('<i', len(d)) + b'\x00' * 12 + d + b'\r\n'


pos = yaw = None; vel = 0.0; idx = None; gps_ok_since = None; t0 = time.time()
while time.time() - t0 < 180:
    c, s, g = latest(co), latest(st), latest(gp)
    if c: pos = struct.unpack_from('<2f', c, 31 + 12)
    if s:
        vel = struct.unpack_from('<f', s, 27 + 10)[0]
        yaw = math.radians(struct.unpack_from('<3f', s, 27 + 62)[2])
    if g:
        txt = g.decode('ascii', 'ignore')
        ok = 'GGA' in txt and '0000.0000,N' not in txt
        if ok: gps_ok_since = gps_ok_since or time.time()
        elif '0000.0000,N' in txt: gps_ok_since = None
    if pos is None or yaw is None:
        time.sleep(0.05); continue
    if idx is None:
        idx = int(np.argmin((rx - pos[0]) ** 2 + (ry - pos[1]) ** 2))
    else:
        lo = max(0, idx - 20); hi = min(len(rx), idx + 80)
        idx = lo + int(np.argmin((rx[lo:hi] - pos[0]) ** 2 + (ry[lo:hi] - pos[1]) ** 2))
    ld = 6.0
    j = min(int(np.searchsorted(s_cum, s_cum[idx] + ld)), len(rx) - 1)
    a = math.atan2(ry[j] - pos[1], rx[j] - pos[0]) - yaw
    a = (a + math.pi) % (2 * math.pi) - math.pi
    steer = max(-0.69, min(0.69, math.atan2(2 * 3.0 * math.sin(a), ld)))
    err = TARGET_KPH - vel
    acc, brk = (min(1.0, 0.15 * err + 0.3), 0.0) if err > 0 else (0.0, min(1.0, -0.1 * err))
    done = (STOP_IDX > 0 and idx >= STOP_IDX) if STOP_IDX > 0 else (gps_ok_since and time.time() - gps_ok_since > 1.5)
    if done:
        for _ in range(40):
            tx.sendto(pkt(0.0, 0.8, 0.0), ('127.0.0.1', 9093)); time.sleep(0.05)
        print('DONE idx=%d pos=(%.1f,%.1f) gps=%s' % (idx, pos[0], pos[1], bool(gps_ok_since)), flush=True)
        break
    tx.sendto(pkt(acc, brk, steer), ('127.0.0.1', 9093))
    if int((time.time() - t0) * 2) % 6 == 0:
        print('idx=%d v=%.1f steer=%.0f°' % (idx, vel, math.degrees(steer)), flush=True)
    time.sleep(0.066)
else:
    for _ in range(40):
        tx.sendto(pkt(0.0, 0.8, 0.0), ('127.0.0.1', 9093)); time.sleep(0.05)
    print('TIMEOUT idx=%s' % idx)
