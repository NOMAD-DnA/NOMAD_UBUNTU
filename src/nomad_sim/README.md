# NOMAD MVSim 맵·센서 패키지

현재 ROS 2 Jazzy + MVSim 1.4.0에서 쓰는 패키지입니다. 설치·빌드 순서는 워크스페이스 최상위 `README.md`를 참고하세요.

```bash
~/nomad_ws/run_forest.sh
```

종료는 실행 터미널 Ctrl+C. 두 MVSim을 동시에 띄우지 마세요(포트23700). 환경 설정은 `~/nomad_ws/.nomad/env.sh`; 기본 ROS domain42·LOCALHOST, 다운스트림은 `use_sim_time:=true`입니다.

## 핵심 파일

- `launch/forest.launch.py` → 공유 `launch/elevation.launch.py`: 현재 실행 연결. 공유 launch는 예전 이름이어도 현재 실행에 필요합니다.
- `worlds/forest.world.xml`: 현재 차량·월드와 센서 include.
- `config/sensors.yaml`, `sensors/`: 카메라·LiDAR 설정과 임시 장착 위치.
- `scripts/generate_forest_map.py`, `scripts/forest_models.py`: 맵·모델 생성기.
- `assets/forest/layout.json`, `forest_overview.png`, 높이·흙길 PNG: 제작 좌표와 배치도. 주행 알고리즘에 자동 전달되는 지도는 아닙니다.
- `assets/models/`, `assets/skybox/`: 로컬 모델·텍스처·출처/라이선스.
- `scripts/oak_bridge.py`, `scripts/lidar_bridge.py`: 원본 센서의 ROS 변환.
- `tests/`, `patches/`: 읽기 전용 검사와 MVSim 종료/먼 나무 높이 제외 로컬 패치 기록.

## 맵·차량

70×50m, 폭3m의 굽은 흙길입니다. 출발→첫 갈림길(언덕/평지)→두 번째 갈림길(돌/평지)→목적지 근처 막다른 벽. 벽에서 돌아와 미방문 언덕을 선택하는 시나리오용 배치이며 자동 복귀 기능은 아닙니다.

언덕은 초반0→2.4→0m, 중앙선 1m 평균 최대 상승 약19.9°/하강16.6°. 돌은 (3,1)에서 길 중간을 막고 뒤로 약9.9m 길이 이어져 (7.945125,9.527875)에 한 번 합류합니다. 도로가 되돌아 겹치는 구간은 제거했습니다. 나무145개 중 도로 가까운20개만 물리 충돌을 유지하며 먼125개는 렌더링만 남습니다. 풀157개, 돌4개와 벽을 유지합니다. 길 밖 이동과 장애물 측면 우회도 물리적으로 가능합니다.

현재 차량은 upstream Jackal 차동구동4바퀴 + `twist_ideal`. 자체 Ackermann 차량, 모터 출력/토크 한도·페이로드 등판 실패, 서보 LiDAR·IMU, VINS, traversability·자율 탐색/복귀는 아직 구현하지 않았습니다.

## 센서·토픽

- 임시 OAK-D Lite FF 유사 RGB/깊이·좌우 mono:640×480, 목표30Hz, baseline75mm. 실제 보정·장착은 미확정.
- YDLIDAR G2 12m형 근사 2D LiDAR:360°·500점·10Hz·0.12–12m. `raytrace_3d=true`는 경사 자세에서 3D 지형과 교차하는 2D 평면 스캔이며 3D LiDAR가 아닙니다. 순차 회전 스캔 왜곡은 재현하지 않습니다.
- `/scan`, `/oak/{rgb,depth}/image_raw`, `/oak/{left,right}/image_rect`와 각 `camera_info`.
- `/cmd_vel`, `/odom`, `/base_pose_ground_truth`, `/clock`, `/tf`, `/tf_static`. ground truth는 SLAM/VIO 결과가 아닙니다.

## 참고

MVSim 창의 CPU%는 센서 대기까지 포함한 처리 경과시간 지표로 OS 전체 CPU 사용률과 다릅니다. 하늘 텍스처는 월드에 포함돼 있습니다.

## 맵 재생성

생성기 상단의 `WAYPOINTS`, `HILL_TURN`, `route_paths()`, `HILL_PROFILE`을 수정합니다. 아래 명령은 수동 편집한 생성 산출물을 덮어쓰므로 먼저 백업하세요. 센서 설정·차량 제어기는 맵 생성만으로 바뀌지 않습니다.

```bash
cd ~/nomad_ws
source .nomad/env.sh
python3 -B src/nomad_sim/scripts/generate_forest_map.py --output-dir src/nomad_sim
./.nomad/build.sh
```

새 맵은 기존 실행을 Ctrl+C로 종료한 뒤 다시 실행해야 반영됩니다.
