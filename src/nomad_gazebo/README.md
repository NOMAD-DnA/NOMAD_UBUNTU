# NOMAD Gazebo 패키지

이 패키지는 Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic에서 이전 NOMAD 맵의 길 관계를 3D 지형으로 재현한다. 기본 실행은 워크스페이스 최상단의 `./run_forest.sh`다. Gazebo가 물리·센서를 계산하고 `ros_gz_bridge`가 ROS 2에 연결한다. MVSim 프로세스는 실행하지 않는다.

## 맵

현재 `src/nomad_sim/assets/forest`의 높이맵과 길·돌·벽·식생 배치를 `assets/authoring/`으로 가져와 `scripts/generate_world.py`로 Gazebo SDF·지형 메시를 만들었다. 약 70×50m, 폭 약 3m의 굽은 길이다. 첫 갈림길의 평지와 오르막, 평지 쪽에서 돌로 막혔으나 뒤까지 계속 이어지는 연결 길, 목적지 근처의 막다른 벽, 되돌아가 오르막을 택하는 관계를 그대로 유지한다. 첫 언덕은 0→2.4→0m로 오르내린다.

지면 시각 텍스처만 논문의 숲 장면을 참고해 마른 흙 팔레트로 바꿨다. 시각 메시와 충돌 메시의 높이는 같으며, 길 색은 마찰이나 traversability 비용이 아니다. 기본 `forest.sdf`에는 현재 맵의 나무 145그루가 보이고 가까운 20그루만 충돌체다. 풀 157개와 먼 나무 125그루는 시각용이다. `./run_forest.sh vegetation:=False`는 무식생 비교용이다.

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
