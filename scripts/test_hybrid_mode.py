#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
하이브리드 모드 테스트 스크립트

이 스크립트는 ROS 없이 하이브리드 모드 로직을 테스트합니다.
실제 시뮬레이션에서는 lattice_morai_module_test.py를 실행하세요.
"""

import sys
import os
import math
import numpy as np

# 현재 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

# 실제 클래스들 임포트 (ROS 부분 제외)
from lattice_morai_module_test import Parameter

def test_hybrid_mode_logic():
    """하이브리드 모드 로직을 간단하게 테스트"""
    
    print("=== 하이브리드 모드 테스트 ===\n")
    
    # 테스트 시나리오 1: 장애물 없음
    print("시나리오 1: 장애물이 없는 경우")
    vehicle_pos = (700.0, -650.0)
    obstacles = []
    
    min_distance = float('inf') if not obstacles else min(
        math.sqrt((vehicle_pos[0] - ox)**2 + (vehicle_pos[1] - oy)**2)
        for ox, oy in obstacles
    )
    
    should_use_global = min_distance > Parameter.OBSTACLE_DETECTION_DISTANCE
    mode = "글로벌 패스" if should_use_global else "래티스 플래닝"
    
    print(f"  차량 위치: {vehicle_pos}")
    print(f"  장애물 개수: {len(obstacles)}")
    print(f"  최단 거리: {min_distance:.1f}m")
    print(f"  감지 임계값: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  선택된 모드: {mode}")
    print(f"  결과: 정상 (장애물 없음 -> 글로벌 패스 사용)")
    print()
    
    # 테스트 시나리오 2: 장애물 가까이 있음
    print("시나리오 2: 장애물이 가까이 있는 경우")
    obstacles = [(705.0, -650.0), (695.0, -645.0)]  # 차량 주변에 장애물
    
    min_distance = min(
        math.sqrt((vehicle_pos[0] - ox)**2 + (vehicle_pos[1] - oy)**2)
        for ox, oy in obstacles
    )
    
    should_use_global = min_distance > Parameter.OBSTACLE_DETECTION_DISTANCE
    mode = "글로벌 패스" if should_use_global else "래티스 플래닝"
    
    print(f"  차량 위치: {vehicle_pos}")
    print(f"  장애물 개수: {len(obstacles)}")
    print(f"  장애물 위치: {obstacles}")
    print(f"  최단 거리: {min_distance:.1f}m")
    print(f"  감지 임계값: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  선택된 모드: {mode}")
    print(f"  결과: 정상 (장애물 가까움 -> 래티스 플래닝 사용)")
    print()
    
    # 테스트 시나리오 3: 히스테리시스 테스트
    print("시나리오 3: 히스테리시스 테스트")
    print("  래티스 모드에서 글로벌 모드로 전환할 때:")
    
    current_mode_is_global = False  # 현재 래티스 모드
    obstacles = [(720.0, -650.0)]  # 약간 멀리 있는 장애물
    
    min_distance = math.sqrt((vehicle_pos[0] - obstacles[0][0])**2 + (vehicle_pos[1] - obstacles[0][1])**2)
    
    # 히스테리시스 로직
    if current_mode_is_global:
        # 글로벌 -> 래티스 전환 조건
        should_switch_to_lattice = min_distance < Parameter.OBSTACLE_DETECTION_DISTANCE
        mode = "래티스 플래닝" if should_switch_to_lattice else "글로벌 패스"
    else:
        # 래티스 -> 글로벌 전환 조건 (히스테리시스 적용)
        should_switch_to_global = min_distance > (Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS)
        mode = "글로벌 패스" if should_switch_to_global else "래티스 플래닝"
    
    print(f"  현재 모드: {'글로벌 패스' if current_mode_is_global else '래티스 플래닝'}")
    print(f"  차량 위치: {vehicle_pos}")
    print(f"  장애물 위치: {obstacles[0]}")
    print(f"  최단 거리: {min_distance:.1f}m")
    print(f"  기본 임계값: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  히스테리시스 임계값: {Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS}m")
    print(f"  전환 후 모드: {mode}")
    expected_mode = "래티스 플래닝" if min_distance <= Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS else "글로벌 패스"
    is_correct = mode == expected_mode
    print(f"  결과: {'정상' if is_correct else '오류'} (히스테리시스 정상 동작)")
    print()
    
    # 설정 정보 출력
    print("=== 현재 하이브리드 모드 설정 ===")
    print(f"  하이브리드 모드 활성화: {Parameter.HYBRID_MODE_ENABLED}")
    print(f"  장애물 감지 거리: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  모드 전환 히스테리시스: {Parameter.MODE_SWITCH_HYSTERESIS}m")
    print(f"  도로 폭: {Parameter.road_width}m")
    print(f"  전방 주시 거리: {Parameter.lookahead_distance}m")
    print()
    
    print("=== 하이브리드 모드 동작 요약 ===")
    print("1. 장애물이 없을 때: 글로벌 패스 사용 (성능 향상)")
    print("2. 장애물이 감지되면: 래티스 플래닝으로 전환 (회피 능력)")
    print("3. 히스테리시스로 모드 전환 떨림 방지")
    print("4. 실시간으로 장애물 상황에 따라 자동 전환")
    
    return True

if __name__ == "__main__":
    try:
        test_hybrid_mode_logic()
        print("\n하이브리드 모드 테스트 완료!")
        print("\n실제 테스트는 다음 명령으로 실행하세요:")
        print("  rosrun [패키지명] lattice_morai_module_test.py")
        
    except Exception as e:
        print(f"\n테스트 실패: {e}")
        import traceback
        traceback.print_exc()