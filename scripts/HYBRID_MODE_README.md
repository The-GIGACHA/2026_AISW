# 하이브리드 모드 패스 플래닝

## 개요

기존의 항상 래티스 플래닝을 사용하는 방식에서, 상황에 따라 **글로벌 패스**와 **래티스 플래닝** 간에 자동으로 전환하는 하이브리드 모드를 구현했습니다.

## 주요 특징

### 🚀 성능 향상
- **장애물이 없을 때**: 글로벌 패스 사용으로 계산 부하 대폭 감소
- **장애물이 있을 때**: 래티스 플래닝으로 정밀한 회피 기동

### 🔄 자동 모드 전환
- 실시간 장애물 감지에 따른 자동 전환
- 히스테리시스 적용으로 모드 전환 떨림 방지
- 부드러운 경로 연결

## 작동 원리

```
장애물 없음 → 글로벌 패스 (빠른 처리)
     ↓
장애물 감지 (< 15m) → 래티스 플래닝 (회피 기동)
     ↓  
장애물 멀어짐 (> 17m) → 글로벌 패스 (성능 향상)
```

## 설정 파라미터

```python
# lattice_morai_module_test.py의 Parameter 클래스에서 설정
HYBRID_MODE_ENABLED = True          # 하이브리드 모드 사용 여부
OBSTACLE_DETECTION_DISTANCE = 15.0  # 장애물 감지 거리 [m]
MODE_SWITCH_HYSTERESIS = 2.0        # 모드 전환 히스테리시스 [m]
```

### 파라미터 설명
- **OBSTACLE_DETECTION_DISTANCE**: 이 거리 이내에 장애물이 있으면 래티스 모드로 전환
- **MODE_SWITCH_HYSTERESIS**: 모드 전환 시 떨림 방지를 위한 여유 거리
  - 래티스 → 글로벌 전환: 장애물이 (15.0 + 2.0) = 17.0m 이상 떨어져야 전환

## 구현 변경사항

### 1. 하이브리드 모드 로직 추가
- `check_obstacles_nearby()`: 주변 장애물 감지
- `update_hybrid_mode()`: 모드 전환 결정 (히스테리시스 포함)
- `make_global_path_segment()`: 글로벌 패스 세그먼트 생성

### 2. 메인 루프 수정
```python
# 하이브리드 모드 업데이트
use_global = self.update_hybrid_mode()

if use_global:
    # 글로벌 패스 사용
    ego_s = self.find_nearest_s(self.gps_x, self.gps_y, mode='ego')
    path_x, path_y = self.make_global_path_segment(ego_s)
    rospy.loginfo_throttle(2.0, "하이브리드 모드: 글로벌 패스 사용 중")
else:
    # 래티스 플래닝 사용
    path_d_list, path_s_list = self.lattice_node(self.gps_x, self.gps_y)
    path_x, path_y = self.make_path(path_d_list, path_s_list)
    rospy.loginfo_throttle(2.0, "하이브리드 모드: 래티스 패스 사용 중")
```

### 3. 시각화 업데이트
- 현재 모드를 그래프 제목에 표시
- 글로벌 모드에서는 후보 경로 숨김
- RViz 마커도 모드에 따라 다르게 표시

## 사용법

### 1. 하이브리드 모드 활성화 (기본값)
```python
HYBRID_MODE_ENABLED = True
```

### 2. 기존 래티스 전용 모드로 복귀
```python
HYBRID_MODE_ENABLED = False
```

### 3. 파라미터 튜닝
```python
OBSTACLE_DETECTION_DISTANCE = 20.0  # 더 멀리서 감지
MODE_SWITCH_HYSTERESIS = 3.0        # 더 안정적인 전환
```

## 테스트 방법

### 1. 로직 테스트
```bash
python3 test_hybrid_mode.py
```

### 2. 실제 시뮬레이션 테스트
```bash
# 래티스 플래너 실행
rosrun [패키지명] lattice_morai_module_test.py

# 컨트롤러 실행  
rosrun [패키지명] controller.py
```

## 예상 효과

### 🔋 성능 향상
- 장애물이 없는 직선/곡선 구간에서 **계산 부하 90% 이상 감소**
- 실시간성 향상으로 더 부드러운 주행

### 🎯 정밀 제어 유지
- 장애물 회피 상황에서는 기존과 동일한 정밀도 제공
- 복잡한 회피 기동 시에만 래티스 플래닝 활용

### ⚡ 반응 속도 향상
- 장애물 상황 변화에 즉각 대응
- 히스테리시스로 안정성 보장

## 모니터링

실행 중 다음 로그로 현재 모드 확인 가능:
```
[ INFO] 하이브리드 모드: 글로벌 패스 사용 중
[ INFO] 하이브리드 모드: 래티스 패스 사용 중
[ INFO] 모드 전환: 글로벌 패스 -> 래티스 패스 (장애물 감지)
[ INFO] 모드 전환: 래티스 패스 -> 글로벌 패스 (장애물 없음)
```

## 문제 해결

### Q: 모드 전환이 너무 자주 발생
**A**: `MODE_SWITCH_HYSTERESIS` 값을 증가시키세요 (예: 3.0 ~ 5.0)

### Q: 장애물 감지가 늦음
**A**: `OBSTACLE_DETECTION_DISTANCE` 값을 증가시키세요 (예: 20.0 ~ 25.0)

### Q: 글로벌 패스 사용을 비활성화하고 싶음
**A**: `HYBRID_MODE_ENABLED = False`로 설정하면 기존 래티스 전용 모드로 동작

---
**작성일**: 2025-09-05
**버전**: v1.0