#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""회전교차로 진입 실험: 자차를 회전교차로 앞으로 순간이동시키며 진입을 반복하고 결과를 기록한다.

필요: 판단·제어 스택 (roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1 obstacles:=morai)
      MORAI 시나리오 rb_entry_mix.json (gen_scenario.py), Multi Ego Setting = UDP 7604

한 회차
  1. 실험 조건을 /aisw/roundabout/trial 로 지정 (mode=explore) — master 가 회전교차로 구간 진입 때 읽는다
       stop_dist  정지 위치: 앞범퍼 ~ 정지선 [m]     3 ~ 5
       accept_gap 예측 최소 간격이 이 이상이면 출발 [m]  0 ~ 6
       v_entry    진입 속도 [kph]                      8 ~ 30
     mode=model: trial 없음 → 학습 모델(models/roundabout_entry.npz)로 판단 / mode=rule: 기본 규칙 / mode=eval: 둘 번갈아
  2. 자차를 경로 인덱스 1690~1712 (정지 위치 약 30~40 m 앞)로 순간이동, 초기 속도 0~25 kph
  3. 회전교차로를 빠져나갈 때까지 MORAI 정답 장애물(/aisw/obstacles)로 자차와 각 물체의 실제 최소 간격(차체 사각형)을 잰다
       대기 중(정지선 앞)과 출발 후를 나눠 기록, 앞/뒤 구분
결과: --out 에 한 줄씩 JSON
"""
import argparse
import json
import math
import os
import random
import socket
import struct
import sys
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CollisionData
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'scripts'))
from control.path_utils import NearestIndexer, get_section, load_map_fields, load_sections  # noqa: E402

EGO_X = (-0.79, 3.845)      # 후륜축 기준 차체 앞뒤 [m]
EGO_HW = 1.892 / 2.0
START_IDX = (1690, 1712)
EPISODE_TIMEOUT = 120.0


def rect(cx, cy, yaw, x0, x1, hw):
    c, s = math.cos(yaw), math.sin(yaw)
    pts = np.array([[x0, -hw], [x1, -hw], [x1, hw], [x0, hw]])
    return np.column_stack([cx + c * pts[:, 0] - s * pts[:, 1], cy + s * pts[:, 0] + c * pts[:, 1]])


def _overlap(a, b):
    for poly in (a, b):
        for i in range(4):
            e = poly[(i + 1) % 4] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() < pb.min() or pb.max() < pa.min():
                return False
    return True


def _pt_seg(p, a, b):
    ab = b - a
    t = np.clip(np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12), 0.0, 1.0)
    return np.linalg.norm(p - (a + t * ab))


def rect_gap(a, b):
    """볼록 사각형 사이 거리 [m] (겹치면 0)."""
    if _overlap(a, b):
        return 0.0
    return min(min(_pt_seg(p, q[i], q[(i + 1) % 4]) for p in pa for i in range(4))
               for pa, q in ((a, b), (b, a)))


def teleport(sock, addr, x, y, z, yaw_deg, v_kph):
    slot = struct.pack('<h3f3ffBB', 0, x, y, z, 0.0, 0.0, yaw_deg, v_kph, 4, 16)
    data = struct.pack('<ii', 1, 0) + slot + b'\x00' * 32 * 19
    pkt = b'#MultiEgoSetting$' + struct.pack('<i', len(data)) + b'\x00' * 12 + data + b'\r\n'
    for _ in range(3):
        sock.sendto(pkt, addr)
        time.sleep(0.1)


class Episodes:
    def __init__(self, a):
        self.a = a
        self.rng = random.Random(a.seed)
        rospy.init_node('rb_episodes', anonymous=True, disable_signals=True)
        self.rx, self.ry = [np.array(v) for v in load_map_fields()[:2]]
        self.indexer = NearestIndexer(self.rx, self.ry)
        cfg = get_section(load_sections(), 'roundabout', {})
        self.exit_idx = int(cfg['exit_idx'])
        self.pose = None
        self.pose_t = 0.0
        self.idx = None
        self.rb = {}
        self.rb_t = 0.0
        self.objs = []
        self.objs_t = 0.0
        self.coll = 0
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._pose, queue_size=1)
        rospy.Subscriber('/aisw/roundabout', String, self._rb, queue_size=1)
        rospy.Subscriber('/aisw/obstacles', Detection3DArray, self._obs, queue_size=1)
        rospy.Subscriber('/CollisionData', CollisionData, self._coll, queue_size=5)
        self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out = open(os.path.expanduser(a.out), 'a', buffering=1)

    def _pose(self, m):
        yaw = 2.0 * math.atan2(m.pose.orientation.z, m.pose.orientation.w)
        self.pose = (m.pose.position.x, m.pose.position.y, yaw)
        self.pose_t = time.time()
        self.idx = self.indexer.find(self.pose[0], self.pose[1])

    def _rb(self, m):
        self.rb = json.loads(m.data)
        self.rb_t = time.time()

    def _obs(self, m):
        self.objs = [(d.bbox.center.position.x, d.bbox.center.position.y, d.bbox.size.x, d.bbox.size.y,
                      2.0 * math.atan2(d.bbox.center.orientation.z, d.bbox.center.orientation.w), d.results[0].id)
                     for d in m.detections]
        self.objs_t = time.time()

    def _coll(self, m):
        if len(m.collision_object):
            self.coll += 1

    def gaps(self):
        """현재 자차–물체 최소 간격 [(gap, 앞/뒤, id)] (자차 기준 좌표)."""
        ego = rect(0.0, 0.0, 0.0, EGO_X[0], EGO_X[1], EGO_HW)
        out = []
        for x, y, L, W, yaw, oid in self.objs:
            box = rect(x, y, yaw, -L / 2.0, L / 2.0, W / 2.0)
            out.append((rect_gap(ego, box), 'front' if x > 1.5 else 'rear' if x < -0.5 else 'side', oid))
        return out

    def trial(self, mode):
        if mode == 'explore':
            return dict(stop_dist=round(self.rng.uniform(3.0, 5.0), 2), accept_gap=round(self.rng.uniform(0.0, 6.0), 2),
                        v_entry_kph=round(self.rng.uniform(8.0, 30.0), 1))
        if mode == 'rule':
            return dict(stop_dist=4.0, accept_gap=4.0, v_entry_kph=15.0)
        return None   # model

    def run(self):
        t_end = time.time() + self.a.hours * 3600
        n = 0
        while time.time() < t_end and not rospy.is_shutdown():
            if self.pose is None or time.time() - self.pose_t > 10.0:
                print('[rb_episodes] /aisw/ego_pose 없음 — 스택 확인', flush=True)
                if self.pose is not None:
                    sys.exit(2)
                time.sleep(1.0)
                continue
            n += 1
            mode = self.a.mode if self.a.mode != 'eval' else ('model' if n % 2 else 'rule')
            trial = self.trial(mode)
            if trial is None:
                if rospy.has_param('/aisw/roundabout/trial'):
                    rospy.delete_param('/aisw/roundabout/trial')
            else:
                rospy.set_param('/aisw/roundabout/trial', trial)
            i0 = self.rng.randint(*START_IDX)
            v0 = round(self.rng.uniform(0.0, 25.0), 1)
            yaw = math.degrees(math.atan2(self.ry[i0 + 1] - self.ry[i0], self.rx[i0 + 1] - self.rx[i0]))
            self.coll = 0
            teleport(self.tx, (self.a.sim_ip, self.a.ego_port), self.rx[i0], self.ry[i0], 28.9, yaw, v0)
            ep = dict(episode=n, mode=mode, trial=trial, start_idx=i0, v0_kph=v0, t0=time.time())
            t_tp = time.time()
            while time.time() - t_tp < 10.0 and not (self.idx is not None and abs(self.idx - i0) < 8
                                                       and self.pose_t > t_tp + 0.3):
                time.sleep(0.05)
            if self.idx is None or abs(self.idx - i0) >= 8:
                ep['teleport_fail'] = True
                self.out.write(json.dumps(ep) + '\n')
                continue
            self.rb, self.rb_t = {}, 0.0      # 이전 회차 상태가 섞이지 않게
            self.measure(ep)
            self.out.write(json.dumps(ep) + '\n')
            d = ep.get('decision') or {}
            print('[rb_episodes] #%d %s %s → %s 대기 %.1fs 통과 %.1fs 최소간격 대기 %.2f / 출발후 %.2f(%s)%s' % (
                n, mode, trial or '', d.get('source', '-'), ep.get('wait_s', float('nan')), ep.get('pass_s', float('nan')),
                ep['min_gap_wait'], ep['min_gap_go'], ep.get('min_gap_go_side', '-'),
                ' 충돌!' if ep['collision'] else (' 시간초과' if ep.get('timeout') else '')), flush=True)
        print('[rb_episodes] 종료: %d 회' % n, flush=True)

    def measure(self, ep):
        min_wait, min_go, side_go = 99.0, 99.0, None
        t_wait = t_go = t_done = None
        decision = None
        while not rospy.is_shutdown():
            time.sleep(0.04)
            now = time.time()
            st = self.rb.get('state') if self.rb_t > ep['t0'] + 0.5 else None
            if st == 'WAIT' and t_wait is None:
                t_wait = now
            if st == 'GO' and t_go is None:
                t_go = now
                decision = self.rb.get('decision')
            if now - self.objs_t < 0.5:
                for g, side, oid in self.gaps():
                    if t_go is None:
                        min_wait = min(min_wait, g)
                    elif g < min_go:
                        min_go, side_go = g, side
            if self.idx is not None and self.idx >= self.exit_idx + 15 and t_go is not None:
                t_done = now
                break
            if now - ep['t0'] > EPISODE_TIMEOUT:
                ep['timeout'] = True
                break
        ep.update(min_gap_wait=round(min_wait, 3), min_gap_go=round(min_go, 3), min_gap_go_side=side_go,
                  decision=decision, collision=bool(min(min_wait, min_go) <= 0.05 or self.coll > 0),
                  coll_packets=self.coll)
        if decision and 'wait_s' in decision:
            ep['wait_s'] = round(decision['wait_s'], 2)     # 정지 후 출발까지 (판단 모듈 기록)
        elif t_wait is not None and t_go is not None:
            ep['wait_s'] = round(t_go - t_wait, 2)
        if t_go is not None and t_done is not None:
            ep['pass_s'] = round(t_done - t_go, 2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--hours', type=float, default=1.0)
    ap.add_argument('--mode', choices=['explore', 'model', 'rule', 'eval'], default='explore')
    ap.add_argument('--out', required=True)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--sim_ip', default='127.0.0.1')
    ap.add_argument('--ego_port', type=int, default=7604)
    Episodes(ap.parse_args()).run()


if __name__ == '__main__':
    main()
