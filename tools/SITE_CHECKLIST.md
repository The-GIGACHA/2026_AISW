# AISW 현장 점검표

2026 국토부 AI·SW 모빌리티 경진대회 · AI융합자율주행부문

- **일시:** 2026.09.18 (금) 10:00–17:00
- **장소:** 노벨빌딩 7층 모라이 연구소
- **구성:** 시뮬 PC(Windows, 모라이 제공) ↔ 팀 PC(Ubuntu 20.04) LAN 직결. 시뮬레이터는 시뮬 PC, 자율주행 스택만 팀 PC에서 실행

명령어의 `<팀PC_IP>`, `<시뮬PC_IP>`는 현장 안내에 맞춰 바꿔서 입력하세요. 두 PC는 같은 /24 대역이어야 합니다.

> LAN을 연결하면 팀 PC의 인터넷이 끊깁니다(이더넷 포트 1개). 이 파일은 오프라인에서도 볼 수 있게 PC에 저장해 둔 것입니다.

---

## 핵심 요약

- **인터페이스 구성은 그대로 사용 가능.** 제어 스택 입력은 규정 허용 7종(Ego Ctrl Cmd, CollisionData, Competition Vehicle Status, GPS, IMU, Camera, LiDAR)과 같습니다. Object Info는 들어오지만 사용하지 않습니다.
- **보정값은 현장 PC에서 재확인.** 조향 이득 0.575, 조향 부호(+좌), vel_x km/h 단위는 로컬 Linux MORAI에서 측정한 값입니다.
- **차량 위치를 옮기면 Ego-0 재연결.** Ctrl Cmd, Status, Collision 설정이 초기화됩니다. 센서 설정은 유지됩니다.
- **카메라 인지 로직은 아직 없음.** 신호등 인식이 없어 빨간불에 서지 않습니다. 현장에서는 카메라 전송 품질과 렌더링 비교 자료만 확보합니다. 샘플 시나리오 전체 녹화는 로컬에서 합니다.

---

## 현장 점검 순서

제어 스택(roslaunch)과 LiDAR launch는 9번 전까지 끈 상태로 진행합니다. 점검 도구가 같은 포트를 씁니다. 10번(카메라)은 스택을 끈 상태에서 합니다.

### 1. LAN 연결 · 링크 속도 확인

- [ ] 완료

평소 링크가 **100 Mb/s**로 잡혀 있었습니다. MORAI 카메라는 프레임마다 65 KB 고정 크기 조각을 보내서 3대만 약 **62 Mbps**, LiDAR 포함 약 70 Mbps입니다. **100 Mb/s로는 부족합니다.**

```bash
cat /sys/class/net/enp3s0/speed
```

**통과:** `1000`. `100`이면 LAN선 교체(Cat5e 이상) 또는 스위치/포트 변경.

### 2. 팀 PC 고정 IP 설정

- [ ] 완료

```bash
sudo nmcli con add type ethernet ifname enp3s0 con-name morai-lan \
  ipv4.method manual ipv4.addresses <팀PC_IP>/24 autoconnect no
sudo nmcli con up morai-lan
ping -c 3 <시뮬PC_IP>
# 복귀: sudo nmcli con up "유선 연결 1"
```

**통과:** ping 응답. Windows는 ping을 막아둘 수 있으니 실패해도 5번 결과로 판단.

### 3. 방화벽 · UDP 수신 버퍼

- [ ] 완료

ufw 서비스가 켜져 있습니다. 로컬 통신엔 영향이 없었지만 다른 PC에서 오는 UDP는 막힐 수 있습니다.

```bash
sudo ufw status
sudo ufw allow from <시뮬PC_IP>        # 또는 점검 동안만: sudo ufw disable
sudo sysctl -w net.core.rmem_max=26214400
sysctl net.ipv4.ip_unprivileged_port_start   # 908이어야 함 (909 포트 수신용, 설정 완료)
```

### 4. 시뮬 PC · MORAI Network Settings

- [ ] 완료

- 센서, Status, Collision: **Destination IP = 팀 PC**
- Ego Ctrl Cmd: **Host IP = 시뮬 PC**
- 포트는 아래 [포트 표](#포트-표) 그대로. 항목마다 **Connect**
- Windows 방화벽이 UDP 9093 인바운드를 막지 않는지 모라이 측에 확인 요청

### 5. 수신 점검 (차량 안 움직임)

- [ ] 완료

```bash
cd <저장소>/tools
python3 morai_site_check.py
python3 morai_site_check.py --scan     # 수신 없는 채널이 있으면: 실제 도착 포트 탐색
```

**통과:** 8개 채널 모두 수신, 송신 IP = `<시뮬PC_IP>`, 수치가 [로컬 기준값](#로컬-기준값)과 비슷. 카메라는 3대 모두 **20 fps, 조립실패 0, 누락 0**.

### 6. 왕복 지연 측정 (차량 안 움직임)

- [ ] 완료

brake 0.3/0.8만 번갈아 보내고 Status에 반영되는 시간을 잽니다.

```bash
python3 morai_site_check.py --echo --sim-ip <시뮬PC_IP>
```

**통과:** 10/10 성공, 중앙값이 로컬 50 ms에서 크게 벗어나지 않음.

### 7. 조향 부호·이득, 속도 단위 재측정 (차량 S자 약 30m 이동)

- [ ] 완료

GPS가 정상 수신되는 넓은 직선 구간에서 실행하세요. 이 모드는 로컬에서 아직 끝까지 검증하지 못했습니다.

```bash
python3 morai_site_check.py --steer --sim-ip <시뮬PC_IP>
```

- 부호 **정상(+좌)**. "반대!"면 조향 반전 문제
- 이득 ≈ **0.575**. 다르면 9번 실행 시 `steer_scale:=<1/이득>` 추가 (기본 1.7391)
- 속도 비율 ≈ **3.6**. vel_x가 km/h (게이트웨이 /3.6 유지)

### 8. 차량을 시작점으로 · Ego-0 재연결

- [ ] 완료

시작점: 전역경로 idx 0, 좌표 (-131.7, -428.3), 방향 61°. 옮긴 뒤 **Ctrl Cmd, Status, Collision 다시 Connect**.

### 9. 자율주행 1분 주행

- [ ] 완료

카메라까지 켠 전체 부하 상태로 주행합니다.

```bash
source <aisw 워크스페이스>/devel/setup.bash
roslaunch aisw_2026 aisw_midterm.launch python:=/usr/bin/python3 sim_ip:=<시뮬PC_IP> cam:=true
# 조향 이득이 다르게 측정됐으면: ... steer_scale:=<1/측정이득>

# 다른 터미널: 주행 중 카메라 1~2분 확인
rostopic hz /image_jpeg/compressed /image_jpeg_left/compressed /image_jpeg_right/compressed
```

**통과:** idx 150·245 부근 R10 급커브 2곳을 경로 이탈 0.3 m 이내로 통과, accel/brake 떨림 없음. 카메라 3개 토픽 약 20 Hz 유지, 게이트웨이 로그에 "손실 프레임" 경고 없음.

### 10. 카메라 비교 자료 (약 10분, 스택 끈 상태)

- [ ] 완료

1. **MORAI 버전·시나리오가 로컬과 같은지 확인** (로컬: 25.S4.MolitComp03, K-City, 샘플 시나리오). 다르면 현장 데이터를 더 모을지 그 자리에서 판단.
2. **센서 설정 화면 사진 촬영** → 아래 [카메라 설정](#카메라-설정) 표와 비교.
3. **같은 위치 스냅샷 2~3곳** (시작점, 신호등 앞 등). 로컬에서 같은 위치 이미지를 찍어 밝기·그림자·화질 비교. 스냅샷은 Ego-0 연결 없이도 됩니다.

```bash
python3 morai_site_check.py --snapshot ~/site_cam/start_point
python3 morai_site_check.py --snapshot ~/site_cam/traffic_light_1
```

---

## 포트 표

게이트웨이(`aisw_udp_bridge.py`) 기본값 기준. 시뮬 PC MORAI 설정을 여기에 맞춥니다.

| 인터페이스 | 방향 | Host (시뮬 PC) | Destination | 팀 PC에서 받는 곳 |
|---|---|---|---|---|
| Ego Ctrl Cmd | 팀 → 시뮬 | `<시뮬PC_IP>:9093` | 임의 (예: 9094) | 게이트웨이가 송신 · cmd type 1, ctrl mode 2 |
| Competition Vehicle Status | 시뮬 → 팀 | `908` | `<팀PC_IP>:909` | 게이트웨이 → `/Competition_topic` 속도 |
| CollisionData | 시뮬 → 팀 | `9091` | `<팀PC_IP>:9092` | 게이트웨이 로그만 |
| GPS | 시뮬 → 팀 | `9280` | `<팀PC_IP>:9281` | `/gps` |
| IMU | 시뮬 → 팀 | `9282` | `<팀PC_IP>:9283` | `/imu` |
| Camera 전 / 좌 / 우 | 시뮬 → 팀 | `9290 / 9292 / 9294` | `<팀PC_IP>:9291 / 9293 / 9295` | `/image_jpeg*/compressed` (`cam:=true`) |
| 3D LiDAR (VLP-16) | 시뮬 → 팀 | `2369` | `<팀PC_IP>:2368` | velodyne 드라이버 → `/velodyne_points` |

---

## 카메라 설정

대회 설정 파일(`2026_molit_comp_cam_set.json`) 기준, 로컬 MORAI와 동일. 모두 20 Hz, JPEG 압축률 90.

| 카메라 | 위치 x, y, z (m) | pitch, yaw | 해상도 | FOV | 토픽 |
|---|---|---|---|---|---|
| 전방 | 1.90, 0, 1.20 | 2°, 0° | 1280×720 | 90° | `/image_jpeg/compressed` |
| 좌측 | 1.15, 0.65, 1.20 | 10°, 70° | 640×480 | 130° | `/image_jpeg_left/compressed` |
| 우측 | 1.15, -0.65, 1.20 | 10°, 290° | 640×480 | 130° | `/image_jpeg_right/compressed` |

> 설정 파일의 `focalLengthpixel: 320`은 1280×720 · FOV 90°와 맞지 않고, FOV가 가로/세로 기준인지에 따라 초점거리가 640 또는 360으로 달라집니다. **내부 파라미터는 현장이 아니라 로컬 이미지로 확정**하세요.

---

## 로컬 기준값

2026-09-17, 같은 PC에서 MORAI 25.S4.MolitComp03(Linux) + K-City + Ioniq5로 측정. 현장 결과와 나란히 비교하세요.

| 항목 | 로컬 측정값 | 현장에서 다르면 |
|---|---|---|
| Status 수신 | 38–45 Hz, 181 B `#MoraiInfo$` | 확인: 길이·헤더가 다르면 속도 파싱 불가 |
| GPS / IMU | 25 Hz GGA+RMC / 40 Hz 115 B | 확인: 낮으면 네트워크·부하 확인 |
| Camera | 20 Hz × 3, 조립실패 0, 누락 0, 간격 평균 50 / 최대 ≈ 89 ms, 합계 ≈ 62 Mbps | 확인: 누락 시 링크 속도·rmem_max |
| LiDAR | VLP-16, 675 pkt/s, ≈ 540 rpm | 확인: 모델이 다르면 `lidar:=` 변경 |
| 명령 → Status 왕복 지연 | 중앙 50 ms, 최대 66 ms | 확인: 크게 늘면 제어 응답 저하 |
| 조향 부호 · 이득 | + = 좌회전, 유효 = 명령 × 0.575, 지연 0.2 s | **중요:** steer_scale 재설정 |
| Status vel_x 단위 | km/h (GPS 대비 3.6배) | **중요:** 틀리면 속도 3.6배 오차 |
| GPS 음영구역 | 좌표 0 송신 → 발행 중단 → 3초 뒤 정지 | 의도된 동작 |
| 주행 성능 | 직선 이탈 ≤ 0.2 m, R10 커브 ≤ 0.28 m, 5.5 m/s | 확인: 크게 다르면 조향 이득부터 |

---

## 문제 생기면

**특정 채널 수신 없음**
MORAI에서 Destination IP가 팀 PC인지, Connect를 눌렀는지 확인 → `sudo ufw status` → `morai_site_check.py --scan`으로 실제 도착 포트 확인.

**점검 도구에서 "bind 실패 [Errno 98]"**
게이트웨이나 velodyne 드라이버가 같은 포트를 쓰는 중입니다. roslaunch를 모두 끄고 다시 실행하세요.

**909 포트 "Permission denied"**
1024 미만 포트 권한 설정이 빠졌습니다.

```bash
sudo sysctl -w net.ipv4.ip_unprivileged_port_start=908
```

**명령을 보내는데 차가 안 움직임**
- `sim_ip:=`를 시뮬 PC IP로 줬는지
- MORAI Ctrl Cmd Host IP가 시뮬 PC IP, 포트 9093, Connect 상태인지
- 차량을 옮긴 뒤 Ego-0 재연결을 했는지
- Windows 방화벽 UDP 9093 인바운드
- `--echo`가 "에코 없음"이면 명령 자체가 안 닿는 상태

**차가 경로를 벗어나 한쪽으로 계속 돎**
조향 부호 문제. `--steer` 결과가 "반대!"면 게이트웨이 `cb_ctrl`의 부호를 확인하세요. 로컬에서는 반전 없음(+좌)이 맞았습니다.

**급커브에서 바깥으로 밀림 / 직선에서 좌우로 흔들림**
조향 이득 불일치. `--steer`로 측정한 이득으로 실행하세요. 밀리면 실제 이득이 더 작은 것, 흔들리면 더 큰 것입니다.

```bash
roslaunch aisw_2026 aisw_midterm.launch python:=/usr/bin/python3 sim_ip:=<시뮬PC_IP> steer_scale:=<1/측정이득>
```

**속도가 너무 느림 / 빠름, accel·brake 떨림**
vel_x 단위 확인(`--steer`의 속도 비율 3.6). Status가 끊기면 게이트웨이가 GPS 추정속도로 대체하므로 Status 수신(909)도 확인하세요.

**카메라 프레임 누락 / "손실 프레임" 경고**
프레임 하나가 65 KB 조각이라 LAN에서 IP 조각 하나만 잃어도 프레임 전체가 사라집니다. 링크 1000 Mb/s 확인 → `sudo sysctl -w net.core.rmem_max=26214400` → 게이트웨이 재시작. 게이트웨이는 깨진 프레임을 버리고 발행하지 않습니다.

**GPS 좌표 0 / 주행 중 멈춤**
GPS 음영(제밍)구역입니다. 현재는 좌표 0이 들어오면 /gps 발행을 멈추고 3초 뒤 정지하도록 설정해 두었습니다(의도된 동작).

---

## 챙길 것 · 출발 전

- [ ] 개발 PC + **전원선**, 키보드·마우스 세트
- [ ] LAN선 여분 (Cat5e 이상). 링크 100 Mb/s면 카메라 대역폭 부족
- [ ] 권장: USB 이더넷 어댑터. 인터넷과 시뮬 LAN을 동시에 유지
- [ ] 변경사항이 이 PC에만 있고 **커밋되지 않음**: 게이트웨이, controller, launch, 점검 도구. 출발 전 별도 브랜치에 커밋 권장

> **inji2 PC와 설정이 다릅니다.** 조향 반전 제거, 909 특권 포트, python 경로 인자 등. main에 바로 합치기 전에 팀과 맞추세요.

---

기준 코드: 저장소 `2026_AISW` (패키지 aisw_2026) · 점검 도구: `tools/morai_site_check.py` · 카메라 조립: `scripts/morai_camera.py`
