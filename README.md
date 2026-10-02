# 2026_AISW

2026 국토부 KATRI 대학생 AI/SW 모빌리티 경진대회 (AI융합자율주행 부문) — 자율주행 제어 스택

| 항목 | 값 |
|---|---|
| 시뮬레이터 | MORAI 25.S4.MolitComp03 |
| 맵 / 차량 | R_KR_PR_K-city_2025 / 2023_Hyundai_Ioniq5 |
| 통신 | 전부 UDP (대회 규정 2-8) — `aisw_udp_bridge.py` 가 ROS 토픽으로 변환 |
| 주행 방식 | 룰베이스 기본 + **GPS 음영 구역 / 회전교차로만 AI(학습 정책)** |
| 규정 | 규정집 v1.1 (2026.09.08) |

---

## 빠른 시작

### 1. 처음 한 번만
```bash
# Python 의존성 (Ubuntu 20.04 시스템 numpy 1.17 과 호환되는 버전)
pip3 install --user --no-deps numba==0.53.1 llvmlite==0.36.0

# catkin 워크스페이스에 패키지 연결 (morai_msgs 가 있는 워크스페이스 위에 올림)
mkdir -p ~/aisw_ws/src && ln -s ~/2026_AISW ~/aisw_ws/src/aisw_2026
source /opt/ros/noetic/setup.bash && source ~/taeho_ws/devel/setup.bash
cd ~/aisw_ws && catkin_make

# Competition Vehicle Status 포트 909 는 특권 포트 → 일반 사용자 bind 허용
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=908
```
> ⚠️ `pip3 install numba` 를 그냥 하면 최신 numpy 가 깔려 시스템 scipy 가 깨진다. 위 버전 고정 명령을 쓸 것.

### 2. MORAI 설정
- 시나리오: `2026_molit_comp_sample_scene.json` → `SaveFile/Scenario/R_KR_PR_K-city_2025/`
- 센서: `2026_molit_comp_full_set_2.json` → `SaveFile/Sensor/25.S4.MolitComp03/` 에서 Load
- Network Settings > Ego-0: Cmd Control(UDP, cmd type 1, ctrl mode 2), Competition Vehicle Status, CollisionData

| 데이터 | 방향 | 포트 |
|---|---|---|
| GPS (NMEA) | 시뮬 → 팀 PC | 9281 |
| IMU | 시뮬 → 팀 PC | 9283 |
| Competition Vehicle Status | 시뮬 → 팀 PC | 909 (host 908) |
| CollisionData | 시뮬 → 팀 PC | 9092 |
| LiDAR VLP16 | 시뮬 → 팀 PC | 2368 |
| Camera Front/Left/Right | 시뮬 → 팀 PC | 9291 / 9293 / 9295 |
| Ego Ctrl Cmd | 팀 PC → 시뮬 | 9093 |

같은 PC 에서 돌리면 목적지 IP 는 모두 `127.0.0.1`.

### 3. 실행
```bash
source /opt/ros/noetic/setup.bash
source ~/taeho_ws/devel/setup.bash
source ~/aisw_ws/devel/setup.bash
roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1
```

| 인자 | 기본값 | 설명 |
|---|---|---|
| `sim_ip` | `192.168.0.1` | 제어 명령을 보낼 시뮬 PC IP (같은 PC 면 `127.0.0.1`) |
| `lidar_obstacles` | `true` | VLP16 직접 파싱 → 장애물/스캔. velodyne 드라이버를 따로 띄울 때만 `false` (포트 2368 충돌) |
| `lidar_x` | `0.58` | 후륜축 → 라이다 거리 [m] (공식 센서 파일 기준) |
| `ai_enable` | `true` | AI 구간에서 학습 정책 사용. `false` 면 룰 폴백만 |
| `model_dir` | `models/` | 학습 정책 위치 (`shaded.npz`, `roundabout.npz`) |
| `record` | `false` | 주행 데이터 CSV 기록 (`~/aisw_logs`) |
| `cam` | `false` | 카메라 3대 → `/image_jpeg{,_left,_right}/compressed` |
| `stop_on_blocked` | `false` | 회피 후보가 모두 막히면 정지 |

### 4. 동작 확인
```bash
rostopic hz /gps /imu /Competition_topic   # 시뮬 데이터 수신
rostopic hz /local_path /ctrl_cmd          # 플래너·제어 출력
rostopic echo /aisw/drive_mode             # RULE / AI_shaded:... / AI_roundabout:...
rostopic echo /aisw/mission                # 현재 미션 이름
```
RViz: Fixed Frame `map`, `/aisw/mission_markers` 추가 → 미션 위치 표시.

---

## 구조

```
MORAI ──UDP──▶ aisw_udp_bridge ──/gps /imu /Competition_topic /aisw/link_id──┐
  ▲                                                                           ▼
  │      aisw_lidar_obstacles ──/tracked_objects_3d, /aisw/lidar_scan──▶ lattice_planner ──/local_path──▶ master_controller_node
  │                                                                                                          │
  └──────────────────────────────────────────── /ctrl_cmd ◀─────────────────────────────────────────────────┘
                                  (룰 구간: controller / AI 구간: ai.zone_controller)
```

| 노드 | 파일 | 역할 |
|---|---|---|
| `aisw_udp_bridge` | `scripts/aisw_udp_bridge.py` | UDP ↔ ROS 변환, 제어 두절 워치독, 조향 이득 보상, link_id 발행 |
| `aisw_lidar_obstacles` | `scripts/aisw_lidar_obstacles.py` | VLP16 패킷 → 클러스터 장애물 + 72칸 2D 스캔 |
| `lattice_planner` | `scripts/lattice_planner_v2.py` | 장애물 회피 로컬 경로, 끼어들기 감시 |
| `master_controller_node` | `scripts/master_v2.py` | 상황 판단(신호/카팔로잉/AI 구간), 추측항법, 제어 |
| ㄴ `Morai_Control_Node` | `scripts/controller.py` | Pure Pursuit + 곡률 기반 속도 (룰 구간) |
| ㄴ `AIZoneController` | `scripts/ai/zone_controller.py` | AI 구간 제어 |
| `aisw_mission_markers` | `scripts/mission_markers.py` | 미션 위치 RViz 마커 |
| `aisw_data_recorder` | `scripts/data_recorder.py` | 학습용 주행 로그 (`record:=true`) |

### 주요 토픽
| 토픽 | 타입 | 발행 |
|---|---|---|
| `/gps`, `/imu`, `/Competition_topic` | GPSMessage / Imu / EgoVehicleStatus | bridge |
| `/aisw/link_id` | String | bridge (Status 의 MGeo 링크 ID) |
| `/tracked_objects_3d` | Detection3DArray | lidar_obstacles |
| `/aisw/lidar_scan` | LaserScan (72칸, 5°) | lidar_obstacles |
| `/local_path`, `/planner_mode`, `/nearest_vrel`, `/merge_stop_flag` | | lattice_planner |
| `/ctrl_cmd` | CtrlCmd | master (controller / AI) |
| `/jamming_mode_active` | Bool | master — True 면 AI 구간(controller·lattice 대기) |
| `/aisw/drive_mode`, `/aisw/mission` | String | master |
| `/aisw/ego_pose` | PoseStamped | master — GPS/추측항법 융합 위치 (z=1 이면 추측항법) |
| `/aisw/mission_markers` | MarkerArray (latch) | mission_markers |

> 신호등(`/traffic_light_*`) 인지 노드는 아직 없다 — 신호 교차로 처리는 인지 노드 추가 후 동작.

---

## 미션 위치 (`config/kcity_sections.yaml`)

전역경로 `map/kcity_map.json` (4392점, 0.5 m 간격, 2184.6 m 순환). 대회 배포 전역경로
(`2026_molit_comp_global_path.txt`)와 형상이 같음을 확인(최대 편차 0.00 m).

| 미션 | 인덱스 | 근거 |
|---|---|---|
| 출발 5% 지점 | 220 | 규정 (109 m) |
| S자 커브 | 140~290 | 경로 곡률 |
| 정적 장애물 | 620~700 (중심 661) | 시나리오 좌표 |
| 보행자 | 895~955 (중심 925) | 시나리오 좌표 |
| 도심 우회전 / 좌회전 (신호 교차로 후보) | 1100~1180 / 1220~1290 | 추정 |
| **회전교차로** | **1765~1865** | 곡률 우-좌-우 패턴 + NPC 순환 |
| 고주로 합류 (좌측 감시) | 2140~2260 | 추정 |
| 속도 제한 예외 (A2256W000411~000153) | 2140~3600 | 추정 |
| **GPS 음영** | **3780~3974** | 시나리오 음영 박스 |
| 완주 | 4391 | 규정 (제한 15분) |

- 확인용 그림: `python3 tools/plot_missions.py` → `~/aisw_logs/missions.png`
- 인덱스 찾기: `python3 tools/find_index.py <x> <y>`
- **추정 항목 확정 방법**: `record:=true` 로 한 바퀴 → `python3 tools/link_index_table.py` (link_id ↔ 인덱스 표, 속도 예외 구간 자동 계산)
- 비어 있음: 신호등 정지선(`stop_lines`, `traffic_zones`), 체크포인트 좌표

---

## AI 구간 (GPS 음영 / 회전교차로)

`ai_zones` 안에서만 `master_v2` 가 제어권을 가져가 `scripts/ai/zone_controller.py` 로 주행하고,
나머지 구간은 기존 룰베이스(controller + lattice_planner)가 그대로 동작한다.

```
우선순위  1. LiDAR 안전 감독   경로 통로(차폭+0.6 m)가 막히면 앞범퍼 3 m 앞 정지 속도로 제한 — 항상 적용
          2. 학습 정책(MLP)    입력: 경로 미리보기 8점 + 속도 + yaw rate + LiDAR 72칸 → 조향, 목표속도
                               룰 조향과 20° 넘게 다르면 거부
          3. 룰 폴백           Pure Pursuit. 음영 구역은 막히면 ±1.8 / ±3.6 m 평행 회피 (음영 구역은 차로 준수 미적용)
```

- **추측항법**: GPS 가 끊기면 Status 속도 × IMU yaw 로 위치 적분 (올해 GPS/IMU 노이즈 없음, 규정 2-6-2)
- **진입 조건**: 인덱스가 `[enter, end]` 안이거나, **어디서든 GPS 가 0.5초 넘게 끊기면** shaded 모드
- **탈출**: 구간 끝을 지나고 GPS 가 복구되면 룰베이스로 복귀
- 모델이 없으면 자동으로 룰 폴백 (현재 저장소에는 학습 모델 없음)

### 학습 루프
```bash
# 1) 데이터 수집 — 같은 구간을 다양한 속도/위치/NPC 상황으로 여러 번
roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1 record:=true

# 2) 학습 (numpy 전용, torch 불필요) → models/<mode>.npz
python3 tools/train_policy.py --mode shaded
python3 tools/train_policy.py --mode roundabout

# 3) 재실행하면 자동 로드. 끄려면 ai_enable:=false
```
정책은 시연 주행을 모방하므로 시연 품질이 상한이다. 회전교차로에서 양보를 배우려면
NPC 가 있을 때 멈췄다 진입하는 주행이 로그에 충분히 있어야 한다.

### 오프라인 검증 결과 (차량 운동 모델 시뮬레이션, 2026-10-02)
| 시나리오 | 결과 |
|---|---|
| 음영 구역 직진 / 시작 1 m 이탈 | 최대 횡오차 0.25 m / 1.02 m → 끝 0.2 m |
| 속도 2% 오차 시 추측항법 | 위치 오차 2.3 m (구역 통과 후) |
| 음영 구역 장애물 1개 / 2개 | 회피 성공, 최소 여유 0.75 m |
| 회전교차로 경로 위 정지 물체 | 앞범퍼 1.24 m 앞 정지 |
| 학습 정책(합성 로그) | 검증 조향 RMSE 0.19° (평균 예측 2.42°), 구역 완주 |

---

## 도구 (`tools/`)
| 파일 | 용도 |
|---|---|
| `plot_missions.py` | 미션 위치 + 시나리오 객체 그림 |
| `link_index_table.py` | 로그 → link_id ↔ 인덱스 표 |
| `train_policy.py` | AI 구간 정책 학습 |
| `find_index.py` | 좌표 → 전역경로 인덱스 |
| `check_tracking.py` | 경로 추종 진단 |
| `morai_site_check.py` | 현장 점검 (UDP 수신, 조향 이득 측정) |

## 알려진 이슈 / TODO
- 신호등 인지 노드 없음 → 신호 교차로 미션 미대응
- 체크포인트, 신호 정지선, 속도 예외 구간 정확한 인덱스 미확정 (위 확정 방법 참고)
- `real_scripts/` 는 ROS 모드용 별도 스택 (현재 UDP 대회 환경에서는 미사용)

(2025_HL_MORAI_FINAL-ROUND 스택 기반)
