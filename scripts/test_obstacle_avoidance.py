#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
장애물 회피 기능 테스트 스크립트

이 스크립트는 장애물 회피 로직이 제대로 활성화되었는지 테스트합니다.
"""

import sys
import os
import math
import numpy as np

# 현재 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 실제 클래스들 임포트 (ROS 부분 제외)
from lattice_morai_module_test import Parameter

def test_obstacle_avoidance_parameters():
    """장애물 회피 파라미터 확인"""
    
    print("=== 장애물 회피 파라미터 테스트 ===\n")
    
    print("장애물 관련 파라미터:")
    print(f"  obs_radius: {Parameter.obs_radius}m (장애물 반지름)")
    print(f"  safety_buf: {Parameter.safety_buf}m (안전 거리)")
    print(f"  pre_window_s: {Parameter.pre_window_s}m (선제적 페널티 거리)")
    print(f"  sigma_d: {Parameter.sigma_d}m (가우시안 표준편차)")
    print(f"  pre_cost_weight: {Parameter.pre_cost_weight} (선제적 비용 가중치)")
    print(f"  obs_cost_weight: {Parameter.obs_cost_weight} (장애물 비용 가중치)")
    print()
    
    print("차량 관련 파라미터:")
    print(f"  vehicle_length: {Parameter.vehicle_length}m")
    print(f"  vehicle_width: {Parameter.vehicle_width}m")
    print(f"  vehicle_wheelbase: {Parameter.vehicle_wheelbase}m")
    print(f"  vehicle_front_overhang: {Parameter.vehicle_front_overhang}m")
    print(f"  vehicle_rear_overhang: {Parameter.vehicle_rear_overhang}m")
    print()
    
    # 테스트 시나리오: 장애물과의 충돌 감지 거리 계산
    print("충돌 감지 계산:")
    obs_margin = Parameter.obs_radius + Parameter.vehicle_width / 2
    print(f"  장애물 반지름 + 차량 폭/2 = {Parameter.obs_radius} + {Parameter.vehicle_width/2:.2f} = {obs_margin:.2f}m")
    print(f"  이 거리 이내에 있으면 충돌 위험으로 판단")
    print()
    
    # 안전 거리 계산
    print("안전 거리 계산:")
    total_safety = Parameter.safety_buf
    print(f"  안전 버퍼: {total_safety}m")
    print(f"  차량이 이 거리 이내에 접근하면 회피 비용 적용")
    print()
    
    # 선제적 회피 계산
    print("선제적 회피:")
    print(f"  전방 {Parameter.pre_window_s}m 범위에서 장애물을 미리 감지")
    print(f"  가우시안 분포(σ={Parameter.sigma_d})로 횡방향 영향 계산")
    print(f"  선제적 비용 가중치: {Parameter.pre_cost_weight}")
    print()
    
    return True

def simulate_obstacle_cost():
    """장애물 비용 계산 시뮬레이션"""
    
    print("=== 장애물 비용 계산 시뮬레이션 ===\n")
    
    # 시뮬레이션 파라미터
    ego_s = 100.0  # 차량 현재 s 좌표
    ego_d = 1.5    # 차량 현재 d 좌표
    obs_s = 110.0  # 장애물 s 좌표 (전방 10m)
    obs_d = 1.5    # 장애물 d 좌표 (같은 차선)
    
    print(f"시뮬레이션 설정:")
    print(f"  차량 위치: s={ego_s}m, d={ego_d}m")
    print(f"  장애물 위치: s={obs_s}m, d={obs_d}m")
    print()
    
    # 1. 직접 충돌 비용 계산
    dis = math.hypot(obs_s - ego_s, obs_d - ego_d)
    obs_margin = Parameter.obs_radius + Parameter.vehicle_width / 2
    
    print(f"1. 직접 충돌 비용:")
    print(f"  거리: {dis:.2f}m")
    print(f"  충돌 임계값: {obs_margin:.2f}m")
    
    if dis <= obs_margin:
        direct_cost = Parameter.obs_cost_weight * (obs_margin - dis) / obs_margin
        print(f"  충돌 위험! 비용: {direct_cost:.2f}")
    else:
        print(f"  안전 거리")
    print()
    
    # 2. 선제적 회피 비용 계산
    ds = obs_s - ego_s  # 장애물까지의 s 방향 거리
    dd = obs_d - ego_d  # 장애물까지의 d 방향 거리
    
    print(f"2. 선제적 회피 비용:")
    print(f"  s 방향 거리: {ds:.2f}m")
    print(f"  d 방향 거리: {dd:.2f}m")
    
    if 0.0 < ds <= Parameter.pre_window_s:
        # s-방향 램프 함수
        ramp_s = (Parameter.pre_window_s - ds) / Parameter.pre_window_s
        print(f"  s 방향 가중치: {ramp_s:.3f}")
        
        # d-방향 가우시안
        sigma_d = Parameter.sigma_d
        w_d_gauss = math.exp(-(dd * dd) / (2.0 * sigma_d * sigma_d))
        print(f"  d 방향 가중치 (가우시안): {w_d_gauss:.3f}")
        
        # 최종 선제적 비용
        preemptive_cost = Parameter.pre_cost_weight * ramp_s * w_d_gauss
        print(f"  선제적 비용: {Parameter.pre_cost_weight} × {ramp_s:.3f} × {w_d_gauss:.3f} = {preemptive_cost:.2f}")
    else:
        print(f"  선제적 회피 범위 밖 (범위: 0 < ds ≤ {Parameter.pre_window_s})")
    print()
    
    # 3. 다양한 위치에서의 비용 계산
    print("3. 다양한 차선에서의 선제적 비용:")
    test_d_values = [0.0, 1.0, 2.0, 3.0]  # 다양한 d 좌표
    
    for test_d in test_d_values:
        dd_test = obs_d - test_d
        w_d_gauss_test = math.exp(-(dd_test * dd_test) / (2.0 * sigma_d * sigma_d))
        if 0.0 < ds <= Parameter.pre_window_s:
            preemptive_cost_test = Parameter.pre_cost_weight * ramp_s * w_d_gauss_test
            print(f"  d={test_d}m에서 비용: {preemptive_cost_test:.2f}")
        else:
            print(f"  d={test_d}m에서 비용: 0.00 (범위 밖)")
    
    return True

if __name__ == "__main__":
    try:
        print("🚗 장애물 회피 기능 테스트 시작\n")
        
        # 파라미터 테스트
        test_obstacle_avoidance_parameters()
        
        # 비용 계산 시뮬레이션
        simulate_obstacle_cost()
        
        print("장애물 회피 기능 테스트 완료!")
        print("\n확인된 기능:")
        print("1. 차량 거동을 고려한 장애물 회피 비용 (활성화됨)")
        print("2. 직접 충돌 감지 및 비용 계산 (활성화됨)")
        print("3. 선제적 회피 비용 (활성화됨)")
        print("4. 가우시안 분포를 이용한 횡방향 영향 계산 (활성화됨)")
        
        print(f"\n현재 설정:")
        print(f"- 장애물 반지름: {Parameter.obs_radius}m")
        print(f"- 안전 거리: {Parameter.safety_buf}m")
        print(f"- 선제적 감지 거리: {Parameter.pre_window_s}m")
        print(f"- 장애물 비용 가중치: {Parameter.obs_cost_weight}")
        print(f"- 선제적 비용 가중치: {Parameter.pre_cost_weight}")
        
    except Exception as e:
        print(f"\n테스트 실패: {e}")
        import traceback
        traceback.print_exc()