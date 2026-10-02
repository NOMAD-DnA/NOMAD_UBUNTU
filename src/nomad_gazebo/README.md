# NOMAD Gazebo 패키지

이 패키지는 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic에서 이전 NOMAD 맵의 길 관계를 3D 지형으로 재현한다. 기본 실행은 워크스페이스 최상단의 `./run_forest.sh`다. Gazebo가 물리·센서를 계산하고 `ros_gz_bridge`가 ROS 2에 연결한다. MVSim 프로세스는 실행하지 않는다.

## 맵

`assets/authoring/`의 배치 입력과 `scripts/generate_world.py`로 약 70×50m Gazebo 월드를 생성한다. 이전 NOMAD의 갈림길·경로 좌표는 유지하되 이전 높이 PNG는 참고용으로 보존하고 현재 지면은 생성기의 고지대 프로필로 만든다. 돌 차단·벽·동굴·줄 형태 경계는 현재 월드에서 제외했다.

지면에는 Poly Haven CC0 흙·낙엽 사진 텍스처를 혼합하며 도로 마스크를 표시하지 않는다. 뒤쪽 고지대 약 4m, 왼쪽 상승 구간 약 7.8m, 오른쪽 약 22m이며 전체 지면에 최대 ±32.5cm 굴곡을 더한다. 출발 접지 구역만 평탄하며 시각/충돌 지면은 동일하다. 나무 363그루 중 가까운 103그루만 충돌체다. 작은 돌 260개와 가지는 `roadside_details.obj` 시각 장식이다. 성긴 풀은 `meadow_grass.obj`, 두 번째 갈림길 왼쪽 중앙의 높은 풀 벽은 `path_grass.obj`로 묶는다. 풀 벽은 높이 2.04~2.89m, 폭 7.5m, 두께 1.3m이며 길에 수직인 방향에서 반시계로 0.3rad 회전한다. 모든 풀은 충돌이 없지만 카메라·렌더 기반 센서에 보일 수 있다. 현재 식생은 주행 불가 영역을 완전히 보장하지 않는다. `./run_forest.sh vegetation:=False`는 모든 식생을 숨기는 비교용이다.

## 차량

`models/nomad_vehicle/model.sdf`는 임시 블록형 Ackermann 차량이다. 차체, 4개 바퀴, 2개 앞 조향 링크와 총 6개 동적 관절이 있다. wheelbase 0.72m, track 0.60m, 바퀴 반경 0.15m, 총 질량 약 34.6kg은 임시값이다. `urdf/nomad_vehicle.urdf`는 같은 링크·관절과 센서 TF를 ROS에 제공한다. 실제 차량 외형·질량·서스펜션은 아직 아니다.

AckermannSteering 플러그인은 속도 명령 기반이다. 관절 `<effort>`만으로 모터 토크가 부족해 오르막에서 멈추는 현상이 검증되는 것은 아니다. 페이로드/토크 실험에는 제한된 토크를 실제 바퀴 관절에 가하는 제어기와 경사 시험이 필요하다.

## ROS 연결

`launch/forest.launch.py`가 Gazebo, `ros_gz_bridge`, `robot_state_publisher`, 카메라·LiDAR ROS 후처리, `/cmd_vel` watchdog을 실행한다. 최종 팀 인터페이스는 `/oak/rgb/image_raw`, `/oak/depth/image_raw`, `/oak/left/image_rect`, `/oak/right/image_rect`, 해당 `/camera_info`, `/oak/imu`, `/scan`, `/odom`, `/joint_states`, `/tf`, `/clock`, `/cmd_vel`이다. 카메라는 640×480·30Hz, IMU는 200Hz, 2D LiDAR는 360°·500점·10Hz 임시 목표다. 모두 시뮬레이션 시간 기준이다.

수동 주행용 `scripts/teleop.py`는 별도 Qt 조종 창을 연다. `W/S`는 속도 목표를 단계적으로 변경하고, `A/D`는 누르는 동안만 조향한다. 키를 떼면 회전 명령이 부드럽게 0으로 돌아오며, 창 포커스를 잃거나 `Space`를 누르면 정지한다. 조종기는 `/cmd_vel`을 20Hz로 발행하고 `command_watchdog.py`가 끊긴 명령을 감시한다.

`/oak/imu`는 Gazebo의 3축 각속도·3축 가속도를 직접 ROS Imu 메시지로 브리지한다. 카메라 마운트와 같은 위치, 차량 축과 같은 방향인 `oak_imu_frame`은 임시 가정이다. 실제 OAK-D Lite FF의 IMU 탑재 여부·칩 축·장착 위치를 확인해 보정해야 한다. VINS와 동기화·추정 성공까지 이 패키지의 정적 생성만으로 보장되지 않는다.

`/odom`과 `odom → base_link`는 Gazebo의 3D ground truth다. Ackermann 플러그인의 평면 wheel_odom/wheel_tf는 별도이며 ROS로 브리지하지 않는다. VINS/SLAM이 TF를 발행할 때 중복을 피해야 한다. RGBD depth는 좌우 스테레오 매칭 결과가 아니라 별도 렌더다.

## 설정 수정

반복할 변경은 생성 파일이 아닌 입력에 넣는다. `models/nomad_vehicle/model.sdf`, `urdf/nomad_vehicle.urdf`, `worlds/*.sdf`, `assets/geometry/*`는 생성 결과다.

```bash
cd ~/nomad_ws/src/nomad_gazebo
python3 scripts/generate_vehicle.py --output-dir . --sensor-config config/sensors.yaml
python3 scripts/generate_world.py --output-dir .
cd ~/nomad_ws
./.nomad/build.sh
./run_forest.sh
```

`generate_world.py --map-package ~/nomad_ws/src/nomad_sim`은 MVSim 맵 데이터를 다시 가져올 때만 사용한다. 맵을 의도하지 않게 덮어쓰지 않는다. 빌드·실행/다른 터미널의 환경 설정은 최상위 [README](../../README.md)를 따른다. 정적 월드 검사는 `python3 tests/test_world.py`다. 이전 보관본의 실행 기록은 현재 식생 배치와 IMU가 다른 상태의 기록이므로 현재 성능 검증으로 간주하지 않는다.

## 전체 숲 밀도 2.5배 (2026-10-01)

방금 만든 국소 숲 영역과 이전 줄 형태의 경계는 기본 월드에서 제거했다. 기존 전체 맵 나무 145그루를 기준으로 2.5배를 올림해 363그루로 늘린다. 특정 영역을 막기 위한 줄/울타리/차단 배치는 만들지 않는다. 기존 145그루 위치는 유지하고 추가 218그루는 맵 전체에 불규칙하게 분포시킨다. 새 나무는 기존 경로 중심에서 최소 2.3m, 다른 줄기 중심에서 최소 1.1m 떨어뜨린다. 도로 색은 없다.

입력은 `assets/authoring/forest_density.json`이다. multiplier를 바꾸면 재생성할 수 있다. `assets/authoring/layout.json`은 145그루 원본으로 보존하고 `assets/authoring/forest_layout.json`은 밀도 증가 후 생성된 배치를 기록한다. 생성기는 항상 원본을 읽으므로 재생성할 때 2.5배가 누적되지 않는다. 가까운 나무만 기존 충돌 정책을 따르고 먼 나무는 묶인 시각 메시다. `vegetation:=False`에서는 전체 나무를 숨긴다. 전체 지형, 경사, 센서 설정은 바꾸지 않는다.

## 출발점 오른쪽 풀 한 포기 제거 (2026-10-01)

사용자 요청으로 낮은 풀 중심 `(-30.654947, -18.369343)` 한 포기만 제거했다.
`assets/authoring/path_grass.json`의 `background_exclusions`에 반경 0.1m의
생성 제외 항목을 기록한다. 모든 난수 배치 생성 후 제외하므로 다른 풀 위치는
변하지 않는다. 낮은 풀은 300개에서 299개이며 나무·높은 풀·지형·차량은 유지한다.
통과 영역 costmap 예외는 사용하지 않는다. 기존 Gazebo 세션은 재시작해야 반영된다.

시작 방향은 `assets/authoring/spawn_pose.json`의 `yaw_rad`에서 설정한다.
현재 값은 0.8267518515rad(약 47.37°)로, 요청 당시 차량 방향에서 왼쪽으로 90°
회전한 값이다. 시작 위치 `(-30, -18, 0.03)`은 유지하며 재실행부터 적용된다.
