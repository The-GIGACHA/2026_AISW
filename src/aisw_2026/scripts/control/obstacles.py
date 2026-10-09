# -*- coding: utf-8 -*-
"""/aisw/obstacles (Detection3DArray, 자차 기준) → 계산용 배열."""
import math
import struct

import numpy as np


def detections_to_objects(detections, yaw_rad):
    """/aisw/obstacles 의 detections → (N,7) [x, y, vx, vy, 길이, 폭, 방향] 자차 기준.
    속도는 source_cloud.data 의 map 속도(vx, vy)를 자차 축으로 돌린 절대속도, 방향은 bbox 방향 [rad]."""
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    out = []
    for d in detections:
        vx = vy = 0.0
        if len(d.source_cloud.data) >= 12:
            _, vx, vy = struct.unpack('fff', bytes(d.source_cloud.data[:12]))
        q = d.bbox.center.orientation
        out.append((d.bbox.center.position.x, d.bbox.center.position.y,
                    c * vx + s * vy, -s * vx + c * vy, d.bbox.size.x, d.bbox.size.y,
                    2.0 * math.atan2(q.z, q.w) if (q.z or q.w) else 0.0))
    return np.array(out, np.float64).reshape(-1, 7)


def objects_to_world(objects, x, y, yaw_rad):
    """자차 기준 (N,7) → map 기준 (N,7) [x, y, vx, vy, 길이, 폭, 방향]."""
    if objects is None or len(objects) == 0:
        return np.zeros((0, 7))
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    w = objects.copy()
    w[:, 0] = x + c * objects[:, 0] - s * objects[:, 1]
    w[:, 1] = y + s * objects[:, 0] + c * objects[:, 1]
    w[:, 2] = c * objects[:, 2] - s * objects[:, 3]
    w[:, 3] = s * objects[:, 2] + c * objects[:, 3]
    w[:, 6] = objects[:, 6] + yaw_rad
    return w
