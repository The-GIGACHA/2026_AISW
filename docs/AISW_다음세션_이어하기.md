# 2026_AISW 다음 세션 이어하기 (2026-09-03 밤 중단 시점)

## 오늘 상태 한 줄
조향 부호 반대(근본버그) 해결로 **경로추종 완벽(이탈 0.2m 실측)**. 남은 건 **시작점 리셋 안정화 → 박스 회피 실증 → 완주 검증**뿐. 코드는 전부 GitHub(The-GIGACHA/2026_AISW) 반영됨.

## 해결한 것 (커밋 순, 전부 실측 기반)
1. 곡률 속도 프로파일 (급커브 이탈 방지)
2. accel/brake 채터링 제거 (속도 EMA + 코스팅 데드밴드) — 브레이크등 점멸 해결
3. 위빙 제거 (거리기반 룩어헤드 L=0.6v+3.0, 조향 EMA+변화율제한) + 제어두절 워치독
4. 폐루프 경계 1점 경로 퇴화 → 도로이탈 버그 (랩어라운드)
5. GPS 음영/두절 대응 (0.7~3초 크리프, 3초+ 정지)
6. **★조향 부호 반대 = 모든 이탈의 진짜 근본원인**. 게이트웨이에서 `-m.steering` 반전. 화면 실측 확인(좌명령→좌회전). cross-track은 이론부호(-) 확정
7. LiDAR 유령장애물 필터 (전방0.5~25m·측면±4m만) — 도로변 가로등/신호등/표지판 오탐 제거

## 검증된 사실
- 조향 잡힌 후 idx185~259 구간 이탈 0.2m 완벽 주행 (실측)
- 회피 로직(9×16 래티스 DP)은 원본 그대로, 정상. 내가 안 건드림
- 아까 "회피 실패(24m 폭주)"는 조향부호+LiDAR오탐 겹친 것. 둘 다 수정됨 → 재검증 필요

## 다음 세션 첫 할 일
1. **시뮬 껐다 켜기**(맵 R_KR_PR_K-city_2025 + Ioniq5) → 확실한 시작점. 시나리오 Load가 시작점 복귀를 자꾸 실패해서 재시작이 제일 깨끗
2. 시나리오 Load `2026_molit_comp_sample_scene` → P → **F5 Load `2026_molit_comp_full_set`**(센서 재부착 필수)
3. 스택 실행: `roslaunch aisw_2026 aisw_midterm.launch` (환경: `export PATH=~/.local/rospython:$PATH` 먼저 — setcap 우회)
4. 출발 후 **박스 구간(idx440~460) 회피** 관찰. 유령장애물 제거됐으니 이번엔 진짜 박스만 피하면 됨
5. 완주되면 중간점검 영상 녹화 (완주 감지: `scratchpad/lapwatch.py`)

## 관측 도구 (scratchpad)
- `lapwatch.py` — GPS 기반 완주/이탈 감시 (Monitor로 실행)
- `pathlog.py` / `logrec.py` — 경로추종 CSV 로거 (오프라인 분석용)

## 주의/한계
- ROS 노드는 반드시 `~/.local/rospython/python3` (셔뱅 박아둠). 시스템 python3.8은 setcap 때문에 ROS lib 로드 실패
- 시나리오 리로드 시 센서 탈락 → F5 재로드 필수. ego만 리셋은 I키(이 버전선 위치복귀 불안정)
- Competition Vehicle Status UDP 파서 미완 → 속도는 GPS미분+EMA 임시 (게이트웨이)
- 신호등 인지·음영구간 인지·주차 미션은 미구현 (본선 과제)
- 회피 재검증 후 여전히 과하면: lattice `road_width`(3.2)·`lateral_cost_weight`(3.0)·d_list 범위 튜닝
