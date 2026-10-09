# 26aisw_control_ws

2026 국토부 KATRI 대학생 AI/SW 모빌리티 경진대회 (AI융합자율주행 부문) — 판단·제어 워크스페이스

| 항목 | 값 |
|---|---|
| 시뮬레이터 | MORAI 25.S4.MolitComp03 |
| 맵 / 차량 | R_KR_PR_K-city_2025 / 2023_Hyundai_Ioniq5 |
| 통신 | 전부 UDP (대회 규정 2-8) — `udp_bridge.py` 가 ROS 토픽으로 변환 |
| 장애물 | 시뮬 개발: MORAI Object Info (`obstacles:=morai`, 기본) / 본선: `26aisw_lidar_ws` 클러스터 `/Clustered_cloud` (`obstacles:=lidar`) |

---

## 설치 (새 PC, 처음 한 번)

```bash
sudo apt install -y libpcap-dev nlohmann-json3-dev ros-noetic-vision-msgs ros-noetic-pcl-ros
pip3 install --user pyproj

# 1) LiDAR 워크스페이스 (morai_msgs 포함)
cd ~/26aisw_lidar_ws && catkin_make -DCMAKE_BUILD_TYPE=Release
#    PCL 을 못 찾으면: catkin_make -DPCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl

# 2) 판단·제어 워크스페이스 (LiDAR 워크스페이스 위에 빌드)
source ~/26aisw_lidar_ws/devel/setup.bash
cd ~/26aisw_control_ws && catkin_make
```

매 터미널: `source ~/26aisw_control_ws/devel/setup.bash` (LiDAR 워크스페이스까지 함께 잡힌다)

---

## 1. 기본 순서 (터미널 2개)

```bash
# 터미널 1 — LiDAR 인지/측위
roslaunch LiDAR_perception main.launch

# 터미널 2 — 판단·제어
roslaunch aisw_2026 aisw_midterm.launch sim_ip:=<시뮬 PC IP> obstacles:=lidar
```
같은 PC 에서 시뮬을 돌리면 `sim_ip:=127.0.0.1`.

**시뮬 개발 (LiDAR 쪽 장애물 속도가 오기 전까지):** MORAI Network Settings 에서 Object Info 를 UDP 7505 로 보내게 하고
```bash
roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1          # obstacles:=morai (기본)
```
Object Info 는 대회 허용 입력이 아니므로 본선은 반드시 `obstacles:=lidar`.

### 튜닝 / 부분 실행

```bash
# 브리지만 (GPS/IMU 토픽 확인용)
rosrun aisw_2026 udp_bridge.py _sim_ip:=192.168.0.1

# 포트 수신 점검 — 판단·제어 스택 끄고 실행
python3 ~/26aisw_control_ws/src/aisw_2026/tools/test/morai_site_check.py --duration 8

# 포트를 모를 때 전체 탐색
python3 ~/26aisw_control_ws/src/aisw_2026/tools/test/morai_site_check.py --scan

# 조향 이득 재측정 (차량이 S자로 약 30m 움직임)
python3 ~/26aisw_control_ws/src/aisw_2026/tools/test/morai_site_check.py --steer --sim-ip 192.168.0.1
```

---

## 2. 기본 구조

### 런치 옵션

**aisw_midterm.launch** (판단·제어)

| 옵션 | 기본값 | 용도 |
|---|---|---|
| `sim_ip` | 192.168.0.1 | 시뮬 PC IP — 현장에서 반드시 지정 |
| `obstacles` | morai | 장애물 출처: morai(Object Info, 시뮬 개발용) / lidar(LiDAR 클러스터, 본선) |
| `cam` | false | 카메라 3대 → `/image_jpeg*/compressed` 발행 |
| `steer_scale` | 1.7391 | 조향 이득 보상 (= 1/0.575) |
| `ai_enable` | true | 회전교차로 AI 속도 보조 (`models/roundabout.npz`, 없으면 룰만) |
| `planner` | lattice | 경로계획기: lattice / frenet |
| `record` | false | 주행 기록 CSV (`~/aisw_logs`, 환경변수 `AISW_LOG_DIR` 로 변경) |
| `viz` | false | 미션 위치 RViz 마커 |

**main.launch** (LiDAR, `26aisw_lidar_ws`)

| 옵션 | 기본값 | 용도 |
|---|---|---|
| `lidar` | vlp16 | 32e 면 실차 HDL-32E |
| `multi` | false | 멀티 LiDAR 캘리브레이션 |
| `lpm` | false | true 면 ndt_localizer 제외 |
| `local_test` | false | 측위 테스트 모드 |

### 상태 확인

```bash
rostopic hz /gps /imu                    # 브리지 살아있나
rostopic hz /aisw/obstacles              # 장애물 (morai_objects 또는 lidar_clusters)
rostopic hz /Clustered_cloud             # obstacles:=lidar 일 때 LiDAR 클러스터 수신
rostopic info /ctrl_cmd                  # master 가 제어 명령 내보내나
rostopic echo -n1 /Competition_topic     # 자차 속도
rostopic echo /aisw/drive_mode           # RULE / AI_roundabout:... / AI_shaded:...
rosnode list | grep -E 'bridge|lidar_clusters|morai_objects|master|lattice'
```

### 포트

| 포트 | 센서 | 방향 | 받는 곳 |
|---|---|---|---|
| 9281 / 9283 | GPS / IMU | 수신 | udp_bridge |
| 909 / 9092 | Status / Collision | 수신 | udp_bridge |
| 9291 / 9293 / 9295 | Camera 전방/좌측/우측 | 수신 | udp_bridge (`cam:=true` 일 때) |
| 2368 | LiDAR | 수신 | velodyne 드라이버 (LiDAR 워크스페이스) |
| 7505 | Object Info (시뮬 개발용) | 수신 | morai_objects (`obstacles:=morai` 일 때) |
| 9093 | Ctrl Cmd | 송신 | udp_bridge |

---

## 3. 코드 구조

```
26aisw_control_ws/
  src/aisw_2026/
    launch/aisw_midterm.launch
    config/kcity_sections.yaml   전역경로 인덱스 구간 (미션, AI 구간, 래티스 구간, 속도표)
    map/kcity_map.json           K-City 전역경로 (4392점, 0.5 m 간격)
    models/                      회전교차로 AI 모델 (roundabout.npz)
    scripts/                     ← 본선 노드 (런치에서 실행)
      master.py                  메인 판단·제어: 구간 판단, 추측항법, 속도 상한, AI 구간 직접 제어
      lattice_planner.py         로컬 경로 (장애물 회피, 합류 감시, 앞차 상대속도)
      frenet_planner.py          대체 경로계획기 (planner:=frenet)
      udp_bridge.py              UDP ↔ ROS
      lidar_clusters.py          /Clustered_cloud → /aisw/obstacles (본선, 속도 없음)
      morai_objects.py           MORAI Object Info → /aisw/obstacles (시뮬 개발용, NPC 속도 포함)
      control/                   ← 노드가 불러 쓰는 부품
        normal_drive.py          일반 구간 경로 추종·속도 제어
        roundabout_shaded.py     회전교차로·GPS 음영 구간 제어
        traffic_light.py         신호등 판단
        gps_jamming.py           GPS 음영 중 위치 추정 (추측항법)
        ai_input.py / ai_model.py  회전교차로 AI 입력 / 모델
        path_utils.py            전역경로·인덱스·스플라인·구간 설정
        frenet_core.py           Frenet 계산부
        camera_packet.py         카메라 UDP 패킷 조립
    tools/                       ← 본선 외 (기록·시각화·점검·학습)
      log/   data_recorder.py, link_index_table.py
      viz/   mission_markers.py, viz_node.py, plot_missions.py
      test/  morai_site_check.py, SITE_CHECKLIST.md, check_tracking.py, find_index.py, practice/
      train/ train_policy.py
```

### 데이터 흐름

```
MORAI ──UDP──▶ udp_bridge ──/gps /imu /Competition_topic──▶ master, lattice_planner, LiDAR main
MORAI ──UDP 2368──▶ [LiDAR ws] velodyne → patchwork++ → main ──/Clustered_cloud──▶ lidar_clusters ──/aisw/obstacles──┐
MORAI ──UDP 7505 (Object Info, 시뮬 개발용)──────────────────────────────────▶ morai_objects ──/aisw/obstacles──┤
                                                                                                                     ▼
                                               lattice_planner ──/local_path──▶ master ──/ctrl_cmd──▶ udp_bridge ──UDP 9093──▶ MORAI
                                               (일반 구간: control/normal_drive · AI 구간: control/roundabout_shaded)
```

### 주요 토픽

| 토픽 | 타입 | 발행 |
|---|---|---|
| `/gps`, `/imu`, `/Competition_topic`, `/CollisionData` | GPSMessage / Imu / EgoVehicleStatus / CollisionData | udp_bridge |
| `/Clustered_cloud` | PointCloud2 (map) | LiDAR ws main (받는 토픽) |
| `/aisw/obstacles` | Detection3DArray (자차 기준, `source_cloud.data` = [상대속도, vx, vy, 확정]) | lidar_clusters(속도 0) 또는 morai_objects |
| `/local_path`, `/planner_mode`, `/nearest_vrel`, `/merge_stop_flag` | | lattice_planner |
| `/ctrl_cmd` | CtrlCmd | master |
| `/jamming_mode_active` | Bool | master — True 면 AI 구간 (normal_drive·플래너 대기) |
| `/aisw/ego_pose` | PoseStamped | master — GPS/추측항법 위치 (z=1 이면 추측항법) |
| `/aisw/drive_mode`, `/aisw/mission` | String | master |
| `/aisw/ai_label` | Float32MultiArray [전문가 속도, AI 속도] | master — 회전교차로 DAgger 기록용 |

---

## 4. 미션 구간 (`config/kcity_sections.yaml`)

| 미션 | 인덱스 |
|---|---|
| 출발 5% 지점 | 220 |
| S자 커브 | 140~290 |
| 정적 장애물 | 620~700 |
| 보행자 | 895~955 |
| 도심 우회전 / 좌회전 | 1100~1180 / 1220~1290 |
| **회전교차로** (AI 속도 보조) | **1765~1865** |
| 고주로 합류 (좌측 감시) | 2140~2260 |
| 속도 제한 예외 | 2140~3600 |
| **GPS 음영** (추측항법, 룰) | **3780~4005** |
| 완주 | 4391 |

인덱스 찾기: `python3 src/aisw_2026/tools/test/find_index.py <x> <y>`

---

## 5. 회전교차로 AI (속도 보조 + DAgger)

```
조향: 항상 룰 (Pure Pursuit)
속도: min(룰 속도, AI 속도)   — AI 는 양보·진입 대기처럼 속도를 낮추는 쪽으로만
안전: /aisw/obstacles 가 경로 통로를 막으면 제동거리 기반 상한 (항상 우선)
입력: 경로 미리보기 8점 + 속도 + yaw rate + 가까운 NPC 6대 (자차 기준 위치·속도·크기)
```

GPS 음영 구간은 AI 없이 룰(추측항법 + 통로 감시 + 평행 회피)만 쓴다.

### 학습 루프

```bash
# 1) 기록 — 회전교차로를 다양한 NPC 상황으로 여러 번
roslaunch aisw_2026 aisw_midterm.launch sim_ip:=127.0.0.1 record:=true
# 2) 학습 → models/roundabout.npz
python3 ~/26aisw_control_ws/src/aisw_2026/tools/train/train_policy.py
# 3) 다시 주행(AI 보조 + 기록) → 2) 반복
```
매 주기 전문가(룰 + 안전 감독) 속도가 `label_v` 로 기록되므로, AI 가 속도를 낮춘 상황에서도 정답이 남는다(DAgger).
모델 입력 형식이 바뀌면(`control/ai_input.py` 의 `FEATURE_VERSION`) 이전 모델은 로드되지 않고 룰로 주행한다.
