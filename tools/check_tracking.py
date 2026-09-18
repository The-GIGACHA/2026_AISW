#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 경로추종 진단 — '빙글빙글 도는' 증상의 원인 구분용.

제어 스택이 떠 있는 상태에서 실행:
    python3 tools/check_tracking.py

CTE(횡편차)가 계속 커지면서 steer가 한쪽으로 포화되면 -> 조향 부호 반대
CTE가 처음부터 수십 m면                              -> 맵/오프셋 불일치
위치가 안 변하면                                      -> GPS 두절/고정
"""
import math, os, json, sys
import rospy
from morai_msgs.msg import CtrlCmd, GPSMessage
from pyproj import Proj

MAP = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'map', 'kcity_map.json')
E0, N0 = 302595.0, 4124145.0

def main():
    rospy.init_node('check_tracking', anonymous=True)
    d = json.load(open(MAP))
    ks = sorted(d.keys(), key=int)
    xs = [d[k]['x'] for k in ks]; ys = [d[k]['y'] for k in ks]
    P = Proj(proj='utm', zone=52, ellps='WGS84')
    st = {}
    rospy.Subscriber('/gps', GPSMessage, lambda m: st.__setitem__('p', P(m.longitude, m.latitude)))
    rospy.Subscriber('/ctrl_cmd', CtrlCmd, lambda m: st.__setitem__('c', m))

    # 토픽 연결까지 최대 15초 대기
    t0 = rospy.Time.now()
    while not rospy.is_shutdown() and ('p' not in st or 'c' not in st):
        if (rospy.Time.now() - t0).to_sec() > 15.0:
            print('[!] %s 수신 없음 — %s 확인' % (
                '/gps' if 'p' not in st else '/ctrl_cmd',
                '브리지(sim_ip/9281)' if 'p' not in st else '마스터'))
            return
        rospy.sleep(0.5)

    print('%-4s %-19s %8s %9s %8s' % ('#', '위치(x,y)', 'CTE[m]', 'steer[deg]', 'accel'))
    hist = []
    for i in range(12):
        rospy.sleep(1.0)
        ex, ny = st['p'][0] - E0, st['p'][1] - N0
        c = st['c']
        j = min(range(len(xs)), key=lambda q: math.hypot(ex - xs[q], ny - ys[q]))
        cte = math.hypot(ex - xs[j], ny - ys[j])
        deg = math.degrees(c.steering)
        hist.append((ex, ny, cte, deg))
        print('%-4d (%8.1f,%8.1f) %8.2f %9.1f %8.3f' % (i, ex, ny, cte, deg, c.accel))

    if len(hist) < 4:
        print('\n판정 불가 — 샘플 부족'); return
    cte0 = sum(h[2] for h in hist[:3]) / 3
    cte1 = sum(h[2] for h in hist[-3:]) / 3
    dgs  = [h[3] for h in hist]
    moved = math.hypot(hist[-1][0] - hist[0][0], hist[-1][1] - hist[0][1])
    same_sign = all(x > 5 for x in dgs[-5:]) or all(x < -5 for x in dgs[-5:])

    print('\n--- 판정 ---')
    print('이동거리 %.1f m, CTE %.2f -> %.2f m, 최근 steer %s' %
          (moved, cte0, cte1, '한쪽 포화' if same_sign else '정상 변동'))
    if moved < 2.0:
        print('=> 차량이 안 움직임. /ctrl_cmd 송신(sim_ip/9093) 확인')
    elif cte0 > 10.0:
        print('=> 시작부터 경로에서 멀다. 맵(kcity_map.json)·offset·시나리오 불일치 의심')
    elif cte1 > cte0 * 2 and cte1 > 2.0 and same_sign:
        print('=> 조향 부호 반대. steer_scale:=-1.7391 로 재시도 후 --steer 로 실측')
    elif cte1 > 2.0:
        print('=> 추종 불안정. steer_scale 크기 재측정 필요 (--steer)')
    else:
        print('=> 추종 정상')

if __name__ == '__main__':
    main()
