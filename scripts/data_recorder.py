#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[2026_AISW] 주행 데이터 기록 노드 — 판단/제어 AI 학습용 원천 로그.

주기마다 한 행씩 CSV 로 남긴다 (기본 15 Hz = 제어 주기).
  - 자차 상태: 지역좌표 x/y, yaw, 속도, 전역경로 인덱스, GPS 신선도
  - 명령: /ctrl_cmd 의 accel/brake/steering
  - 판단 상태: planner_mode, 재밍 모드, 앞차 상대속도, 합류 정지 플래그
  - Lattice 후보: 선택 행 + 끝점 행별 누적 비용 9개 (inf 는 -1)
  - 장애물: 개수, 최근접 거리(차량 좌표계)
  - 이벤트: 충돌 객체 수 (상승 에지는 학습 단계에서 계산)
  - [AI 구간] master 융합 위치(/aisw/ego_pose, 음영 중엔 추측항법), yaw rate, MGeo link_id,
    주행 모드(/aisw/drive_mode), 현재 미션, LiDAR 2D 스캔 72칸(/aisw/lidar_scan)
    → tools/train_policy.py 학습 입력, tools/link_index_table.py 링크↔인덱스 표

ROS bag 대신 CSV 를 쓰는 이유: 학습 스크립트(pandas)에서 바로 읽고, 주행 중
대용량 토픽(LiDAR/카메라)을 같이 저장하지 않아 디스크/CPU 부담이 없다.

실행: roslaunch aisw_2026 aisw_midterm.launch record:=true
출력: ~/aisw_logs/run_YYYYmmdd_HHMMSS.csv  (~log_dir 로 변경)
"""
import csv
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from morai_msgs.msg import CollisionData, CtrlCmd, EgoVehicleStatus, GPSMessage
from nav_msgs.msg import Path
from pyproj import Proj
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Float32, Float32MultiArray, String, UInt8
from tf.transformations import euler_from_quaternion
from vision_msgs.msg import Detection3DArray

from aisw_common import DEFAULT_MAP, NearestIndexer, load_map_fields
from ai.features import SCAN_BINS

N_CANDIDATES = 9   # lattice_planner_v2 Parameter.dd_sampling_num

COLUMNS = (
    ['t', 'x', 'y', 'yaw_deg', 'vel', 'global_idx', 'gps_age',
     'cmd_accel', 'cmd_brake', 'cmd_steer',
     'planner_mode', 'jamming', 'nearest_vrel', 'merge_stop',
     'local_path_len', 'cand_best']
    + ['cand_cost_%d' % i for i in range(N_CANDIDATES)]
    + ['n_obstacles', 'nearest_obs_dist', 'n_collision',
       'yaw_rate', 'pose_dr', 'link_id', 'drive_mode', 'mission']
    + ['scan_%d' % i for i in range(SCAN_BINS)]
)


class DataRecorder:
    def __init__(self):
        rospy.init_node('aisw_data_recorder')
        self.rate_hz = rospy.get_param('~rate', 15.0)
        log_dir = os.path.expanduser(rospy.get_param('~log_dir', '~/aisw_logs'))
        os.makedirs(log_dir, exist_ok=True)
        self.path = os.path.join(log_dir, time.strftime('run_%Y%m%d_%H%M%S.csv'))

        rx, ry = load_map_fields(rospy.get_param('~map_file', DEFAULT_MAP))[:2]
        self.indexer = NearestIndexer(rx, ry)
        self.proj = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        nan = float('nan')
        self.state = dict.fromkeys(COLUMNS, nan)
        self.state.update(planner_mode=0, jamming=0, merge_stop=0, n_obstacles=0, n_collision=0,
                          local_path_len=0, pose_dr=0, link_id='', drive_mode='', mission='')
        self.last_gps_t = None
        self.last_pose_t = None

        rospy.Subscriber('/gps', GPSMessage, self._gps)
        rospy.Subscriber('/imu', Imu, self._imu)
        rospy.Subscriber('/Competition_topic', EgoVehicleStatus, self._ego)
        rospy.Subscriber('/ctrl_cmd', CtrlCmd, self._cmd)
        rospy.Subscriber('/planner_mode', UInt8, self._set('planner_mode', int))
        rospy.Subscriber('/jamming_mode_active', Bool, self._set('jamming', int))
        rospy.Subscriber('/nearest_vrel', Float32, self._set('nearest_vrel', float))
        rospy.Subscriber('/merge_stop_flag', UInt8, self._set('merge_stop', int))
        rospy.Subscriber('/local_path', Path, self._local_path)
        rospy.Subscriber('/lattice_candidates', Float32MultiArray, self._candidates)
        rospy.Subscriber('/tracked_objects_3d', Detection3DArray, self._objects)
        rospy.Subscriber('/CollisionData', CollisionData, self._collision)
        rospy.Subscriber('/aisw/ego_pose', PoseStamped, self._ego_pose)
        rospy.Subscriber('/aisw/link_id', String, self._set('link_id', str))
        rospy.Subscriber('/aisw/drive_mode', String, self._set('drive_mode', str))
        rospy.Subscriber('/aisw/mission', String, self._set('mission', str))
        rospy.Subscriber('/aisw/lidar_scan', LaserScan, self._scan)

    # ── 콜백: 최신값만 덮어쓴다. 기록 주기는 run() 이 정한다. ──
    def _set(self, key, cast):
        def callback(msg):
            self.state[key] = cast(msg.data)
        return callback

    def _gps(self, msg):
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            return   # 재밍 구간: 브리지가 발행을 멈추지만 혹시 0 이 오면 무시
        self.last_gps_t = rospy.get_time()
        if self._pose_fresh():
            return   # master 융합 위치가 있으면 그쪽을 쓴다 (같은 값, 음영 중에도 연속)
        ux, uy = self.proj(msg.longitude, msg.latitude)
        self.state['x'] = ux - msg.eastOffset
        self.state['y'] = uy - msg.northOffset

    def _pose_fresh(self):
        return self.last_pose_t is not None and rospy.get_time() - self.last_pose_t < 0.3

    def _ego_pose(self, msg):
        self.last_pose_t = rospy.get_time()
        self.state['x'] = msg.pose.position.x
        self.state['y'] = msg.pose.position.y
        self.state['pose_dr'] = int(msg.pose.position.z > 0.5)

    def _scan(self, msg):
        if len(msg.ranges) == SCAN_BINS:
            for i, r in enumerate(msg.ranges):
                self.state['scan_%d' % i] = round(r, 2)

    def _imu(self, msg):
        q = msg.orientation
        self.state['yaw_deg'] = math.degrees(euler_from_quaternion((q.x, q.y, q.z, q.w))[2])
        self.state['yaw_rate'] = msg.angular_velocity.z

    def _ego(self, msg):
        self.state['vel'] = msg.velocity.x

    def _cmd(self, msg):
        self.state['cmd_accel'] = msg.accel
        self.state['cmd_brake'] = msg.brake
        self.state['cmd_steer'] = msg.steering

    def _local_path(self, msg):
        self.state['local_path_len'] = len(msg.poses)

    def _candidates(self, msg):
        data = list(msg.data)
        if not data:
            return
        self.state['cand_best'] = int(data[0])
        costs = data[1:1 + N_CANDIDATES]
        for i in range(N_CANDIDATES):
            self.state['cand_cost_%d' % i] = costs[i] if i < len(costs) else float('nan')

    def _objects(self, msg):
        dets = msg.detections or []
        self.state['n_obstacles'] = len(dets)
        self.state['nearest_obs_dist'] = min(
            (math.hypot(d.bbox.center.position.x, d.bbox.center.position.y) for d in dets),
            default=float('nan'))

    def _collision(self, msg):
        self.state['n_collision'] = len(getattr(msg, 'collision_object', []) or [])

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        rospy.loginfo('[data_recorder] 기록 시작: %s', self.path)
        with open(self.path, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=COLUMNS)
            writer.writeheader()
            rows = 0
            while not rospy.is_shutdown():
                now = rospy.get_time()
                self.state['t'] = now
                if self.last_gps_t is not None:
                    self.state['gps_age'] = now - self.last_gps_t
                if self.last_gps_t is not None or self._pose_fresh():
                    self.state['global_idx'] = self.indexer.find(self.state['x'], self.state['y'])
                writer.writerow(self.state)
                rows += 1
                if rows % int(self.rate_hz * 5) == 0:
                    f.flush()   # 강제 종료돼도 최근 5초 이전 데이터는 남도록
                try:
                    rate.sleep()
                except rospy.ROSInterruptException:
                    break
        rospy.loginfo('[data_recorder] %d행 저장: %s', rows, self.path)


if __name__ == '__main__':
    DataRecorder().run()
