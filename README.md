# 2026_AISW

2026 국토부 AI·S/W 모빌리티 경진대회 (AI융합자율주행부문) — 자율주행 제어 스택

- 시뮬레이터: MORAI 25.S4.MolitComp03 / 맵 R_KR_PR_K-city_2025 / 차량 2023_Hyundai_Ioniq5
- 통신: 전부 UDP (대회 규정) — `aisw_udp_bridge.py`가 ROS 토픽으로 변환
- 실행: `roslaunch aisw_2026 aisw_midterm.launch`
- 상세 가이드: `docs/AISW_MORAI_세팅_가이드.md` / 진행 상황·다음 할 일: `docs/AISW_다음세션_이어하기.md`
- `legacy/erp_42/`: 2026_AISW 패키지 분리 이전의 초기 MORAI 제어 코드 (Pure Pursuit + 회피, 초기 UDP 브리지)

구성: master_v2(상황판단+제어) / lattice_planner_v2(회피) / aisw_udp_bridge(UDP↔ROS) / aisw_lidar_obstacles(VLP16 직파싱 장애물)
(2025_HL_MORAI_FINAL-ROUND 스택 기반, 공식 전역경로 맵 + 인덱스 파라미터화 적용)
