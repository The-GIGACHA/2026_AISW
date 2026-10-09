#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""회전교차로 진입 실험용 MORAI 시나리오 생성.

기준: scenarios/2026_practice_roundabout.json
- 기존 진입 생성 지점(#4: 링크 A2256W000144 → A2256W000102): NPC 생성 간격·속도 범위를 넓힘
- 생성 지점 2개 추가 (MORAI 가 확실히 받는 링크 점만 사용 — 범위 밖 pointIdx 는 Load 시 예외가 나고 네트워크가 멈춘다)
    #40 회전교차로 서쪽 A2256W000133 시작점(pointIdx 0) → #4 와 같은 목적지: 진입로 앞을 다른 간격으로 지나감
    #41 #4 와 같은 위치·목적지, 빠른 NPC 대역
  (회전교차로 링크 순서: 144 → 133(서) → 134(남, 자차 합류) → 135(동) → 102(북), 반시계)
MORAI 가 NPC 마다 범위 안에서 간격·속도를 뽑으므로 시간이 지나며 대수·속도가 계속 달라진다.

사용: gen_scenario.py <MORAI SaveFile/Scenario/R_KR_PR_K-city_2025 폴더>  → rb_entry_mix.json
      MORAI 에서 Load 한 번 (Load 후 Network Connect 확인)
"""
import copy
import json
import os
import sys

PKG = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
BASE = os.path.join(PKG, 'scenarios', '2026_practice_roundabout.json')
FEED_UID = 4

# (UNIQUEID, 시작 (링크, pointIdx), 위치, yaw[deg], 생성 간격 범위[s], 목표속도 범위[kph]) — 목적지는 #4 와 같음
EXTRA = [
    (40, ('A2256W000133', 0), (-115.3, 353.6), -132.5, (2.0, 14.0), (10.0, 40.0)),
    (41, None, None, None, (4.0, 18.0), (30.0, 50.0)),
]
FEED = dict(period=(1.5, 15.0), speed=(10.0, 50.0))


def xyz(p, x, y, z=None):
    p['x'], p['y'] = float(x), float(y)
    p['_x'], p['_y'] = '%.3f' % x, '%.3f' % y
    if z is not None:
        p['z'] = float(z)
        p['_z'] = '%.3f' % z


def configure(sp, period, speed):
    sp['minSpawnPeriod'], sp['maxSpawnPeriod'] = period
    sp['maximumSpawnVehicle'] = 999
    sp['DesiredVelocityType'] = 2
    sp['MinDesiredVelocity_Custom'], sp['MaxDesiredVelocity_Custom'] = speed
    sp['latBiasMode'] = 1
    sp['MinLatBias'], sp['MaxLatBias'] = -0.4, 0.4


def main():
    out_dir = sys.argv[1] if len(sys.argv) > 1 else '.'
    s = json.load(open(BASE))
    feed = next(sp for sp in s['spawnPointList'] if sp['UNIQUEID'] == FEED_UID)
    configure(feed, FEED['period'], FEED['speed'])
    for uid, start, pos, yaw, period, speed in EXTRA:
        sp = copy.deepcopy(feed)
        sp['UNIQUEID'] = uid
        if start is not None:
            xyz(sp['pos'], pos[0], pos[1], feed['pos']['z'])
            sp['rot']['yaw'] = '%.3f' % yaw
            sp['startLinkInfo'] = {'linkIdx': start[0], 'pointIdx': start[1]}
        configure(sp, period, speed)
        s['spawnPointList'].append(sp)
    path = os.path.join(out_dir, 'rb_entry_mix.json')
    json.dump(s, open(path, 'w'))
    print('저장: %s  (MORAI 에서 Load)' % path)


if __name__ == '__main__':
    main()
