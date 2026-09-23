# 2026 국토부 AI/SW 모빌리티 경진대회 — MORAI 환경/센서 세팅 (2026-09-03 적용 완료)

대상 설치본: `~/MoraiLauncher_Lin_AISW` (심 25.S4.MolitComp03, 맵 R_KR_PR_K-city_2025 설치 확인됨)
※ 배달로봇용 구런처(`~/MoraiLauncher_Lin`)는 건드리지 않음.

## 1. 적용된 파일 (백업: 같은 폴더 `*.bak_260903`)
| 파일 | 내용 |
|---|---|
| `SaveFile/Sensor/25.S4.MolitComp03/2026_molit_comp_full_set.json` | 규정 전체 센서 프리셋 (심 Sensor Edit → Load 로 불러오기 가능) |
| `SaveFile/Sensor/25.S4.MolitComp03/SensorInfo_2023_Hyundai_Ioniq5.json` | 자동 로드본 — Ioniq5 선택 시 바로 적용 |
| `SaveFile/Network/25.S4.MolitComp03/NetworkInfo_2023_Hyundai_Ioniq5.json` | 허용 네트워크만 ON, 전부 UDP |

## 2. 센서 구성 (규정 §2-6 대조 — 전 항목 준수)
| 센서 | 위치 x,y,z (m) | 각도 r,p,y (°) | 스펙 | 출력 | UDP 수신포트(팀 PC) |
|---|---|---|---|---|---|
| CAM_Front (고정) | 1.90, 0.00, 1.20 | 0, 2, 0 | 1280×720, FOV 90 | 20Hz (≤30 규정) | 9291 |
| CAM_Left (고정) | 1.15, 0.65, 1.20 | 0, 10, 70 | 640×480, FOV 130 | 20Hz | 9293 |
| CAM_Right (고정) | 1.15, −0.65, 1.20 | 0, 10, 290 | 640×480, FOV 130 | 20Hz | 9295 |
| VLP16 | 1.50, 0.00, 1.80 | 0, 0, 0 | Intensity 타입, 10Hz(권장, ≤15) | 10Hz | 2368 (velodyne 표준) |
| GPS | 0.00, 0.00, 1.60 | — | NMEA(GPGGA) | 30Hz (규정 최대) | 9281 |
| IMU | 0.00, 0.00, 0.50 | — | 쿼터니언+각속도+가속도 | 50Hz (규정 최대) | 9283 |
- 4번째 자유 카메라는 미장착(규정상 선택). 필요 시 Sensor Edit에서 추가 후 같은 파일로 저장.
- GPS/IMU를 차량 원점(후륜축 중심) 위에 둬서 레버암 보정 불필요.
- GT(Ground Truth) 계열/바운딩박스 시각화 전부 꺼짐 (규정 §2-6-2).

## 3. 네트워크 (규정 §2-8 허용 7종만 — 전부 UDP)
| 채널 | 상태 | 포트 |
|---|---|---|
| Ego Ctrl Cmd (수신) | ON, AutoMode, 종방향 type 1(Throttle) | 시뮬 9093 으로 송신 |
| Collision Data | ON | → 9092 |
| Competition Vehicle Status | ON | → 9087 (기존 908/909는 리눅스 특권포트라 변경) |
| GPS / IMU / Camera / LiDAR | ON | 위 표 |
| EgoVehicleStatus·ObjectInfo·신호등제어(TLCtrl)·SensorSync·SL·EventCmd | **전부 OFF** (허용 목록 외 — 사용 시 실격) |

⚠ **ObjectInfo OFF ⇒ `/Object_topic` 이 안 나옴.** `control_test_mj.py`의 장애물 회피는 대회 환경에선 동작하지 않음 → **LiDAR(VLP16) 기반 장애물 검출로 전환 필요** (연습 시에만 Network 설정에서 ObjectInfo를 켜서 로직 검증 가능).

## 4. 실행 방법 (로컬 연습, 全 UDP)
```bash
# T1: 시뮬 실행 (~/MoraiLauncher_Lin_AISW) → Ioniq5 + R_KR_PR_K-city_2025 선택
#     Sensor Edit에서 "2026_molit_comp_full_set" Load 확인, Network Setting 확인
# T2: roscore
# T3: UDP↔ROS 게이트웨이 (GPS/IMU→토픽, /ctrl_cmd→UDP)
source ~/catkin_ws/devel/setup.bash
python3 ~/control_ws/src/erp_42/aisw_udp_bridge.py            # _cam:=true 면 카메라도 토픽으로
# T4: LiDAR (velodyne 표준 패킷)
roslaunch velodyne_pointcloud VLP16_points.launch device_ip:="" port:=2368
# T5: 제어코드 (예: control_test_mj.py — 아래 §6 파라미터 수정 후)
```

## 5. 대회 당일 체크리스트
1. 시뮬 PC(윈도우11)와 팀 PC를 LAN 연결 → 센서 6개 + CollisionData + CompetitionInfo의 `destinationIP`를 **팀 PC IP**로, EgoCtrl은 게이트웨이 `_sim_ip:=<시뮬PC IP>` 로 변경 (심 Network/Sensor UI에서 IP만 수정).
2. 팀 PC 방화벽에서 UDP 2368, 9281, 9283, 9291/9293/9295, 9092, 9087 인바운드 허용.
3. 게이트웨이 `~east_offset/~north_offset` 파라미터: 연습 중 ROS 모드 `/gps`의 eastOffset/northOffset 값(맵 원점 오프셋)을 미리 기록해 넣기 — NMEA에는 이 값이 없음.
4. 15분 제한·체크포인트·GPS 음영구간 규정 숙지 (음영구간 = GPS 끊김 → IMU/LiDAR 항법 대비).

## 6. Ioniq5 제어 파라미터 (control_test 코드 수정 필요 — 아직 미적용)
| 항목 | ERP42(현재 코드) | Ioniq5(규정 §2-5) |
|---|---|---|
| WHEEL_BASE | 0.8 | **3.0** |
| 최대 조향각(바퀴) | — | **40°** (최소회전반경 5.87m) |
| 차체 | — | 길이 4.635 × 폭 1.892 m |
| 종방향 제어 | — | **type 1 (accel/brake 0~1)** 만 허용 |

## 7. 검증 필요(첫 구동 때 확인)
- Ego Ctrl UDP 패킷(`#MoraiCtrlCmd$`+23B) 수신 여부: 차가 안 움직이면 심 Network Setting의 Ego Ctrl 포트/모드(AutoMode)와 게이트웨이 `_ctrl_port` 대조.
- 게이트웨이 로그에 `[CompetitionInfo] ...B header=...` 덤프가 뜨면 알려주기 → 파서 마저 작성 예정.
- LiDAR Intensity 드롭다운이 "Intensity"인지 심 UI에서 육안 확인(저장값 0 = Intensity, DLL enum으로 확인함).

## 8. 중간점검 (2026 AI융합자율주행부문_중간점검 가이드.pdf 기준 — 자가검증 2026-09-03 전 항목 ✅)
**제출 마감: 9/4(금) 23:59, yhpark@morai.ai, 메일 제목에 학교명/팀명 필수**
1. 개발 현황 리포트 (양식 파일 활용)
2. 주행 영상 (파일명 영문: `팀명_.확장자` — 화면 녹화 프로그램 사용)
3. 센서 파일 (영문: `팀명_sensor.json`) → `2026_molit_comp_full_set.json`을 팀명으로 복사해 제출

실행 순서(가이드 P.9~13):
1. 심 25.S4.MolitComp03 → 맵 **R_KR_PR_K-city_2025** + 2023_Hyundai_Ioniq5, 날씨 **Sunny** → START
2. Edit > Scenario > Load Scenario > `2026_molit_comp_sample_scene.json` > Load (배치 완료됨: SaveFile/Scenario/R_KR_PR_K-city_2025/) → 로드 후 **P키로 Parking 기어**
3. F5 > Load > `2026_molit_comp_full_set` 선택 (고정캠 3 + VLP16 + GPS + IMU, BBOX 토글 전부 해제 상태)
4. 네트워크: 허용 3종(EgoCtrlCmd/Collision/CompetitionStatus)만 ON 상태로 저장돼 있음
5. Viewport: 좌상단 톱니바퀴 → **Show Performance / Show Vehicle Cluster and Map Info 활성화**
6. 알고리즘(UDP) 실행 → 주행 → 녹화
- 전역경로: `~/Downloads/2026_molit_comp_global_path.txt` (Link ID, [e,n,u] 좌표)
- 중간점검은 난이도 조정용, 대회 결과에 영향 없음

## 9. 제어 스택 `~/control_ws/src/2026_AISW` (구 2025_HL_MORAI_FINAL-ROUND, 2026-09-03 이식·수정)
ROS 패키지명은 숫자 시작 불가라 **`aisw_2026`** (폴더명은 2026_AISW).
```bash
source /opt/ros/noetic/setup.bash && source ~/catkin_ws/devel/setup.bash && source ~/control_ws/devel/setup.bash
roslaunch aisw_2026 aisw_midterm.launch          # 게이트웨이+라이다장애물+master_v2+lattice 일괄 실행
```
적용한 수정 (원본 대비):
1. **맵 교체**: 공식 `2026_molit_comp_global_path.txt` → `map/kcity_map.json` 재생성 (4,392pt·0.5m 간격·폐루프 2,185m, 시작점=시나리오 ego 위치 일치, 최소회전반경 7.9m>Ioniq5 한계 5.87m). 옛 맵은 `kcity_map.json.hl_old`
2. **옛 상암맵 인덱스 무력화**: lattice 구간상수 19개(9999999), controller 1633 풀브레이크(`~obstacle_stop_index`, 기본 -1), master 제밍구역(`~jamming_zone_start/end`)·신호등 구간(`~traffic_zones` [[s,e],...])·강제green 전부 파라미터화(기본 꺼짐) — 새 맵에서 오발동 방지
3. **콜백 레이스 수정**: controller 구독자 등록을 __init__ 끝으로 이동 (proj_UTM 등 미초기화 크래시)
4. **EventCmd 옵셔널화**: `/Service_MoraiEventCmd` 2초 대기 후 없으면 비활성 (대회 네트워크엔 없음 — 이전엔 무한 블로킹)
5. **accel/brake 0~1 클램프** (규정 longlCmdType 1 범위)
6. **신규 노드**: `aisw_lidar_obstacles.py` — VLP16 UDP(2368) 직접 파싱→클러스터링→`/tracked_objects_3d` (velodyne 드라이버 불필요, 정적 장애물용·속도추정 0)
7. 게이트웨이: `/Competition_topic`을 GPS 미분 속도로 대체 발행 (`~pub_competition`, 실제 패킷 포맷 확정 시 교체 예정), GPS 오프셋 기본값 = K-City 2025 mgeo (302595, 4124145)
8. `real_scripts/` 실행권한 제거 (rosrun 중복 방지, 내용 보존)

스모크 테스트 통과: 시나리오 시작점 가짜 GPS/IMU 주입 → `/local_path` 시작점 일치, `/ctrl_cmd` 15Hz, accel>0 출발 확인.

### ⚠ 시스템 참고
`/usr/bin/python3.8`에 `cap_net_bind_service`가 걸려 있어 ROS .so 로드 실패(LD_LIBRARY_PATH 무시) → `~/.local/rospython/python3`(cap 없는 복사본)을 PATH 앞에 추가해 우회(bashrc 반영). 근본 해결: `sudo setcap -r /usr/bin/python3.8` (908/909 특권포트를 9086/9087로 옮겨서 cap 이제 불필요)

### 남은 한계 (중간점검 전 확인)
- 신호등 인지 없음(`/traffic_light_*` 퍼블리셔 부재) → 신호 unknown=통과. 신호등 구간·정지선 매핑은 실주행 후 `~traffic_zones` 파라미터로 설정
- 장애물 속도 추정 없음 → 동적 NPC는 정적으로 취급(정지/회피는 됨, 예측 회피는 안 됨)
- `/Competition_topic` 속도는 GPS 미분(지연 약간) — 실제 UDP 패킷 포맷 잡히면 게이트웨이에 정식 파서 추가
