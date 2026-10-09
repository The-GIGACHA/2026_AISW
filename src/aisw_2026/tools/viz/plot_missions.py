#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""미션 위치 확인용 그림 (ROS 불필요).

전역경로 위에 config/kcity_sections.yaml 의 missions / ai_zones 와,
시나리오 json 의 장애물·보행자·NPC·음영 박스를 함께 그린다.

  python3 tools/plot_missions.py [--scene <시나리오.json>] [--out <파일.png>]   (기본: scenarios/ 샘플, $AISW_LOG_DIR)
"""
import argparse
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'scripts'))
from control.path_utils import DEFAULT_MAP, DEFAULT_SECTIONS, LOG_DIR, SCENARIO_DIR, load_map_fields, load_sections, get_section  # noqa: E402

COLORS = {'checkpoint': 'limegreen', 'lane_keep': 'olive', 'obstacle': 'red', 'intersection': 'orange',
          'roundabout': 'dodgerblue', 'merge': 'magenta', 'speed_limit': 'c', 'gps_shaded': 'dimgray'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scene', default=os.path.join(SCENARIO_DIR, '2026_molit_comp_sample_scene.json'))
    ap.add_argument('--sections', default=DEFAULT_SECTIONS)
    ap.add_argument('--out', default=os.path.join(LOG_DIR, 'missions.png'))
    args = ap.parse_args()

    rx, ry = load_map_fields(DEFAULT_MAP)[:2]
    sec = load_sections(args.sections)
    fig, ax = plt.subplots(figsize=(14, 22))
    ax.plot(rx, ry, '-', c='lightgray', lw=3, zorder=1)
    for i in range(0, len(rx), 100):
        ax.annotate(str(i), (rx[i], ry[i]), fontsize=7, color='steelblue')

    for z in get_section(sec, 'ai_zones', []):
        a, b = int(z['enter']), int(z['end'])
        ax.plot(rx[a:b + 1], ry[a:b + 1], '-', c='purple', lw=14, alpha=0.25, zorder=2)
    for k, m in enumerate(get_section(sec, 'missions', [])):
        a, b = int(m['start']), int(m['end'])
        a, b = (max(0, a - 4), b + 4) if a == b else (a, b)
        c = COLORS.get(m.get('type'), 'k')
        ax.plot(rx[a:b + 1], ry[a:b + 1], '-', c=c, lw=5, zorder=3)
        ax.annotate('%s [%d-%d]%s' % (m['name'], m['start'], m['end'], '?' if m.get('source') == 'estimate' else ''),
                    (rx[a], ry[a]), xytext=(12, 8 - 14 * (k % 3)), textcoords='offset points',
                    fontsize=9, color=c, weight='bold', zorder=5)

    if os.path.exists(args.scene):
        d = json.load(open(args.scene))
        f = lambda p: (float(p['x']), float(p['y']))  # noqa: E731
        for v in d.get('vehicleList', []):
            ax.plot(*f(v['initPosition']['pos']), 's', c='red', ms=5)
        for o in d.get('objectList', []):
            ax.plot(*f(o['pos']), 'X', c='k', ms=12)
        for p in d.get('pedestrianList', []):
            ax.plot(*f(p['pos']), 'o', c='green', ms=9)
        for sp in d.get('spawnPointList', []):
            ax.plot(*f(sp['pos']), '^', c='m', ms=9)
        for a in d.get('shadedAreaList', []):
            # 시나리오 박스: 긴 변(size.x)이 ENU yaw+90° 방향일 때 경로와 정렬됨
            cx, cy = f(a['pos'])
            th = math.radians(float(a['rot']['yaw']) + 90.0)
            L, W = float(a['size']['x']), float(a['size']['y'])
            ux, uy, vx, vy = math.cos(th), math.sin(th), -math.sin(th), math.cos(th)
            pts = [(cx + sx * L / 2 * ux + sy * W / 2 * vx, cy + sx * L / 2 * uy + sy * W / 2 * vy)
                   for sx, sy in ((1, 1), (1, -1), (-1, -1), (-1, 1), (1, 1))]
            ax.plot(*zip(*pts), '--', c='k')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.3)
    ax.set_title('K-City 2026 missions (? = estimate, purple = AI zone)')   # 한글 폰트 없는 PC 대비 영문
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    fig.savefig(args.out, dpi=80, bbox_inches='tight')
    print('저장:', args.out)


if __name__ == '__main__':
    main()
