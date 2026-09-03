#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
회피 후 복귀 안정화 모드 테스트 스크립트

장애물 회피 후 급격한 글로벌 패스 전환을 방지하는 안정화 로직을 테스트합니다.
"""

import sys
import os
import math
import time

# 현재 디렉토리를 sys.path에 추가
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from lattice_morai_module_test import Parameter

def test_recovery_mode_parameters():
    """회피 후 복귀 안정화 파라미터 확인"""
    
    print("=== 회피 후 복귀 안정화 파라미터 ===\n")
    
    print("기본 하이브리드 모드 설정:")
    print(f"  HYBRID_MODE_ENABLED: {Parameter.HYBRID_MODE_ENABLED}")
    print(f"  OBSTACLE_DETECTION_DISTANCE: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  MODE_SWITCH_HYSTERESIS: {Parameter.MODE_SWITCH_HYSTERESIS}m (증가됨)")
    print()
    
    print("회피 후 복귀 안정화 설정:")
    print(f"  AVOIDANCE_RECOVERY_TIME: {Parameter.AVOIDANCE_RECOVERY_TIME}초")
    print(f"  AVOIDANCE_RECOVERY_DISTANCE: {Parameter.AVOIDANCE_RECOVERY_DISTANCE}m")
    print()
    
    print("개선된 전환 로직:")
    print(f"1. 장애물 감지 거리: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"2. 회피 모드 진입 시 복귀 안정화 모드 활성화")
    print(f"3. 복귀 조건: 시간({Parameter.AVOIDANCE_RECOVERY_TIME}초) AND 거리({Parameter.AVOIDANCE_RECOVERY_DISTANCE}m) 모두 충족")
    print(f"4. 히스테리시스 적용: {Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS}m 이상 멀어져야 글로벌 전환")
    print()
    
    return True

def simulate_avoidance_scenario():
    """장애물 회피 시나리오 시뮬레이션"""
    
    print("=== 장애물 회피 시나리오 시뮬레이션 ===\n")
    
    # 시나리오 설정
    scenarios = [
        {
            "name": "시나리오 1: 정상적인 회피 및 복귀",
            "vehicle_path": [
                (100, 0),    # 시작점 (글로벌 모드)
                (110, 0),    # 장애물 접근 (래티스 전환)
                (120, 1),    # 회피 중
                (130, 0.5),  # 회피 완료, 복귀 중
                (140, 0),    # 복귀 완료
                (150, 0),    # 안정화 후 글로벌 전환 가능
            ],
            "obstacles": [(115, 0)],  # 장애물 위치
        },
        {
            "name": "시나리오 2: 연속 장애물",
            "vehicle_path": [
                (100, 0),    # 시작점
                (110, 0),    # 첫 번째 장애물 접근
                (120, 1),    # 첫 번째 회피
                (130, 1),    # 두 번째 장애물 접근
                (140, 2),    # 두 번째 회피
                (150, 1),    # 복귀 시작
                (160, 0),    # 복귀 완료
            ],
            "obstacles": [(115, 0), (135, 1)],
        }
    ]
    
    for scenario in scenarios:
        print(f"{scenario['name']}")
        print(f"장애물 위치: {scenario['obstacles']}")
        print("경로 진행:")
        
        # 초기 상태
        use_global = True
        is_in_recovery = False
        last_avoidance_time = 0
        current_time = 0
        
        for i, (vx, vy) in enumerate(scenario['vehicle_path']):
            current_time = i * 1.0  # 1초씩 증가
            
            # 장애물과의 최단 거리 계산
            min_distance = float('inf')
            for ox, oy in scenario['obstacles']:
                dist = math.sqrt((vx - ox)**2 + (vy - oy)**2)
                min_distance = min(min_distance, dist)
            
            # 모드 전환 로직 시뮬레이션
            obstacles_nearby = min_distance < Parameter.OBSTACLE_DETECTION_DISTANCE
            
            # 회피 복귀 모드 시간 체크
            recovery_time_passed = (current_time - last_avoidance_time) >= Parameter.AVOIDANCE_RECOVERY_TIME if last_avoidance_time > 0 else False
            
            if use_global and obstacles_nearby:
                # 글로벌 -> 래티스 전환
                use_global = False
                is_in_recovery = True
                last_avoidance_time = current_time
                status = "래티스 전환 (회피 시작)"
                
            elif not use_global and not obstacles_nearby:
                # 래티스 -> 글로벌 전환 검토
                if not is_in_recovery:
                    if min_distance > (Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS):
                        use_global = True
                        status = "글로벌 전환"
                    else:
                        status = f"래티스 유지 (히스테리시스, 거리: {min_distance:.1f}m)"
                else:
                    if recovery_time_passed:
                        is_in_recovery = False
                        if min_distance > (Parameter.OBSTACLE_DETECTION_DISTANCE + Parameter.MODE_SWITCH_HYSTERESIS):
                            use_global = True
                            status = "글로벌 전환 (복귀 완료)"
                        else:
                            status = "복귀 모드 종료, 히스테리시스 대기"
                    else:
                        remaining_time = Parameter.AVOIDANCE_RECOVERY_TIME - (current_time - last_avoidance_time)
                        status = f"복귀 모드 ({remaining_time:.1f}초 남음)"
            else:
                # 상태 유지
                if use_global:
                    status = "글로벌 유지"
                else:
                    if is_in_recovery:
                        remaining_time = max(0, Parameter.AVOIDANCE_RECOVERY_TIME - (current_time - last_avoidance_time))
                        status = f"래티스 유지 (복귀모드 {remaining_time:.1f}초)"
                    else:
                        status = "래티스 유지"
            
            mode = "글로벌" if use_global else "래티스"
            print(f"  {current_time:.0f}초: 위치({vx}, {vy}), 거리: {min_distance:.1f}m, 모드: {mode}, {status}")
        
        print()
    
    return True

def test_parameter_tuning():
    """파라미터 튜닝 가이드"""
    
    print("=== 파라미터 튜닝 가이드 ===\n")
    
    print("🔧 현재 설정값:")
    print(f"  장애물 감지 거리: {Parameter.OBSTACLE_DETECTION_DISTANCE}m")
    print(f"  모드 전환 히스테리시스: {Parameter.MODE_SWITCH_HYSTERESIS}m")
    print(f"  회피 후 복귀 시간: {Parameter.AVOIDANCE_RECOVERY_TIME}초")
    print(f"  회피 후 복귀 거리: {Parameter.AVOIDANCE_RECOVERY_DISTANCE}m")
    print()
    
    print("📝 튜닝 가이드:")
    print("1. 전환이 너무 빈번한 경우:")
    print(f"   → MODE_SWITCH_HYSTERESIS를 증가 (현재: {Parameter.MODE_SWITCH_HYSTERESIS}m → 추천: 7-10m)")
    print()
    
    print("2. 회피 후 복귀가 너무 급작스러운 경우:")
    print(f"   → AVOIDANCE_RECOVERY_TIME을 증가 (현재: {Parameter.AVOIDANCE_RECOVERY_TIME}초 → 추천: 4-6초)")
    print(f"   → AVOIDANCE_RECOVERY_DISTANCE을 증가 (현재: {Parameter.AVOIDANCE_RECOVERY_DISTANCE}m → 추천: 15-20m)")
    print()
    
    print("3. 장애물 감지가 늦은 경우:")
    print(f"   → OBSTACLE_DETECTION_DISTANCE를 증가 (현재: {Parameter.OBSTACLE_DETECTION_DISTANCE}m → 추천: 20-25m)")
    print()
    
    print("4. 성능 우선 시:")
    print(f"   → AVOIDANCE_RECOVERY_TIME을 감소 (현재: {Parameter.AVOIDANCE_RECOVERY_TIME}초 → 최소: 2초)")
    print(f"   → MODE_SWITCH_HYSTERESIS를 감소 (현재: {Parameter.MODE_SWITCH_HYSTERESIS}m → 최소: 3m)")
    print()
    
    return True

if __name__ == "__main__":
    try:
        print("🔄 회피 후 복귀 안정화 모드 테스트 시작\n")
        
        # 파라미터 확인
        test_recovery_mode_parameters()
        
        # 시나리오 시뮬레이션
        simulate_avoidance_scenario()
        
        # 튜닝 가이드
        test_parameter_tuning()
        
        print("회피 후 복귀 안정화 모드 테스트 완료!")
        print("\n개선 효과:")
        print("1. 회피 후 급격한 글로벌 전환 방지")
        print("2. 안정화 시간/거리 기반 점진적 복귀") 
        print("3. 히스테리시스 증가로 모드 전환 떨림 감소")
        print("4. 시각화에 복귀 모드 상태 표시")
        print("5. 로그에 상세한 전환 이유 및 시간 표시")
        
    except Exception as e:
        print(f"\n테스트 실패: {e}")
        import traceback
        traceback.print_exc()