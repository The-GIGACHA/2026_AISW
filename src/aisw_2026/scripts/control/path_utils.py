#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""공용 유틸 — 전역경로(PATH) 로딩, 최근접 인덱스 탐색, 3차 스플라인, 구간 설정.

rospy 에 의존하지 않으므로(ros_sections 제외) tools/ 의 오프라인 스크립트에서도 그대로 쓸 수 있다.
"""
import json
import os

import numpy as np
import yaml
from scipy.interpolate import CubicSpline

PKG_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
DEFAULT_MAP = os.path.join(PKG_DIR, 'map', 'kcity_map.json')
DEFAULT_SECTIONS = os.path.join(PKG_DIR, 'config', 'kcity_sections.yaml')
SECTIONS_PARAM = '/aisw/sections_file'   # launch 에서 덮어쓸 수 있는 전역 파라미터
# 주행 로그 폴더: 환경변수 AISW_LOG_DIR 로 변경 가능 (기본: 사용자 홈의 aisw_logs)
LOG_DIR = os.path.expanduser(os.environ.get('AISW_LOG_DIR', os.path.join('~', 'aisw_logs')))
SCENARIO_DIR = os.path.join(PKG_DIR, 'scenarios')


def load_map_fields(json_file=DEFAULT_MAP):
    """전역경로 json → (rx, ry, ryaw, rk, rvel[kph], rmission, rgear) 리스트 튜플.

    속도(velocity)는 kph 그대로 — PATH 가 m/s 로 바꾼다.
    """
    with open(json_file, 'r') as f:
        data = json.load(f)
    keys = sorted(data.keys(), key=lambda k: int(k))
    return tuple([data[k][field] for k in keys]
                 for field in ('x', 'y', 'yaw', 'curvature', 'velocity', 'mission', 'gear'))


class PATH:
    """경로 점 목록. cv 는 kph 로 받아 m/s 로 저장한다."""

    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = [v / 3.6 for v in cv]
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)


def load_ref_path(json_file=DEFAULT_MAP):
    """전역경로 json → PATH."""
    return PATH(*load_map_fields(json_file))


class CubicSpline2D:
    """(x, y) 점열을 누적거리 s 로 매개화한 자연 3차 스플라인."""

    def __init__(self, x, y):
        x = np.asarray(x, np.float64)
        y = np.asarray(y, np.float64)
        ds = np.hypot(np.diff(x), np.diff(y))
        self.s = np.empty(x.shape[0], np.float64)
        self.s[0] = 0.0
        self.s[1:] = np.cumsum(ds)
        self.sx = CubicSpline(self.s, x, bc_type='natural')
        self.sy = CubicSpline(self.s, y, bc_type='natural')

    def calc_position_vec(self, s_array):
        return self.sx(s_array), self.sy(s_array)

    def calc_yaw_vec(self, s_array):
        """[deg]"""
        dx = self.sx(s_array, 1)
        dy = self.sy(s_array, 1)
        return np.rad2deg(np.arctan2(dy, dx))

    def calc_curvature_vec(self, s_array):
        dx, ddx = self.sx(s_array, 1), self.sx(s_array, 2)
        dy, ddy = self.sy(s_array, 1), self.sy(s_array, 2)
        return (ddy * dx - ddx * dy) / np.power(dx * dx + dy * dy, 1.5)

    def calc_position(self, s):
        x, y = self.calc_position_vec(np.array([s], np.float64))
        return float(x[0]), float(y[0])

    def calc_yaw(self, s):
        return float(self.calc_yaw_vec(np.array([s], np.float64))[0])


def nearest_index_global(xs, ys, x, y):
    """경로 전체에서 (x, y) 최근접 인덱스. 벡터화 버전."""
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    return int(np.argmin((xs - x) ** 2 + (ys - y) ** 2))


class NearestIndexer:
    """직전 인덱스 주변 창에서만 최근접점을 찾는 전역경로 인덱서.

    K-City 전역경로는 시작점과 끝점이 같은 닫힌 루프라, 전체 argmin 은 결승 지점에서
    인덱스를 1 로 되감는다 → 종료 직전에 시작부 구간 설정이 다시 걸린다.
    창 탐색은 인덱스를 진행 방향으로만 이어가므로 끝까지 4391 을 유지한다.
    (경로가 공간상 겹치는 지점도 있으나, 좌측 6.4 m 오프셋 주행까지는 전체 argmin 도 튀지 않았다.)
    창 안의 최근접점이 max_jump 보다 멀거나 창 끝에 걸리면(창 밖이 더 가까울 수 있음)
    전체 탐색으로 재초기화한다.
    """

    def __init__(self, xs, ys, back_m=10.0, fwd_m=40.0, max_jump=5.0):
        self.xy = np.column_stack([np.asarray(xs, np.float64), np.asarray(ys, np.float64)])
        self.n = len(self.xy)
        spacing = float(np.mean(np.hypot(*np.diff(self.xy, axis=0).T))) if self.n > 1 else 1.0
        spacing = max(spacing, 1e-3)
        self.back = max(1, int(back_m / spacing))
        self.fwd = max(1, int(fwd_m / spacing))
        self.max_jump2 = max_jump * max_jump
        self.last = None

    def reset(self):
        self.last = None

    def find(self, x, y):
        p = np.array([x, y])
        if self.last is not None:
            lo = max(0, self.last - self.back)
            hi = min(self.n, self.last + self.fwd + 1)
            d2 = np.sum((self.xy[lo:hi] - p) ** 2, axis=1)
            k = int(np.argmin(d2))
            at_edge = (k == 0 and lo > 0) or (k == hi - lo - 1 and hi < self.n)
            if d2[k] <= self.max_jump2 and not at_edge:
                self.last = lo + k
                return self.last
        self.last = int(np.argmin(np.sum((self.xy - p) ** 2, axis=1)))
        return self.last


def load_sections(path=DEFAULT_SECTIONS):
    """구간 설정 yaml 로드. 파일이 없으면 모든 구간 비활성(빈 설정)을 돌려준다."""
    if not path or not os.path.exists(path):
        return {}
    with open(path, 'r') as f:
        return yaml.safe_load(f) or {}


def get_section(sections, dotted_key, default=None):
    """'lattice.merge_zones' 같은 점 표기 키로 중첩 값을 꺼낸다."""
    node = sections
    for key in dotted_key.split('.'):
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return default if node is None else node


def ros_sections():
    """rospy 노드에서 쓰는 헬퍼: SECTIONS_PARAM 이 가리키는 설정 파일을 로드한다."""
    import rospy
    path = rospy.get_param(SECTIONS_PARAM, DEFAULT_SECTIONS)
    sections = load_sections(path)
    rospy.loginfo('[path_utils] 구간 설정: %s (%s)', path,
                  'OK' if sections else '없음 — 모든 구간 비활성')
    return sections
