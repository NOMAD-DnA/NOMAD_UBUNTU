# 실물 인지 코어 실행

`real_perception`은 2026-10-30의 `semantic_ground_validation`에서 계산과 ROS 연결에 필요한 파일을 이식한 별도 실행 노드다. Pi 5의 융합 위치를 사용하고 Planning이 받는 네 지도와 상태를 발행한다. 웹 서버, 실험 라이브러리, 재생기, 결과 CSV/NPZ/PLY/PNG 저장은 포함하지 않는다. 기존 시뮬레이션 인지의 계산과 실행 기본값은 유지한다.

## 포함 파일

| 경로 | 역할 |
|---|---|
| `nomad_perception/real/node.py`, `publisher_lock.py` | 센서 수신, 정지 초기화, 고장 처리, 중복 실행 제한 |
| `real/map_core.py`, `depth_core.py`, `depth_filter.py`, `ros_cloud.py` | 원래 LiDAR bin 기하와 점별 시각, deskew, 깊이 필터 |
| `real/semantic_validation.py` | ZED 라벨과 깊이의 시각 및 가림 검사, 지면점과 의미 격자 |
| `real/ground_slope_live.py`, `worker_io.py`, `ground_reference/` | 별도 작업 프로세스의 기존 경사 계산과 최신 상태 전달 |
| `real/pi5_pose.py`, `vio_health.py` | Pi 5 위치 및 상태 검사, 실제 고정 TF를 통한 raw IMU pose 변환 |
| `real/capture_timing.py`, `planning_transport.py` | 정수 촬영 시각 보존, QoS, 최신성 및 READY 유효기간 |
| `real/planning_costs.py`, `cost_output.py`, `observation_support.py` | 별도 경사/의미 비용, 반복 촬영 관측지원, Planning 메시지 |
| `real/oak_basalt_bridge.py`, `oak_basalt_calibration.py`, `camera_intrinsics_override.py`, `stereo_rectification.py` | 선택 실행하는 OAK 실시간 입력과 실제 장치 보정 정보 |
| `config/real/` | 선택 보정값, Pi 5 및 의미 설정, G2B 설정, 원본/이식 파일 SHA-256 목록 |
| `launch/real_perception.launch.py`, `scripts/run_real_perception.sh` | 화면 없는 실물 실행 |
| `.nomad/real_sensors/ydlidar_ros2_driver/`, `.nomad/build_real_lidar.sh` | 점별 시각을 제공하는 기존 LiDAR 드라이버의 실행 소스와 별도 빌드 |
| `test/real/` | 계산, 시각, QoS, 좌표 변환과 설치 실행 파일의 합성 DDS 시험 |

전체 파일 목록과 변경 전후 해시는 `config/real/import_manifest.json`에 있다. 패키지 상대 import와 설치 리소스를 사용한다. 원래 실험 폴더, 기존 venv, Basalt 실행 파일 경로를 참조하지 않는다. 경사 평면 계산은 유지하며 파일 기록과 화면 helper만 제거했다. 최신 상태 JSON은 임시 폴더에서 작업 프로세스와 ROS 노드 사이에 전달하며 정상 종료 시 지운다. 일반 ROS/launch 콘솔 로그는 ROS의 기본 동작을 따른다.

## 입력 계약

| 입력 | 형식 및 조건 |
|---|---|
| `/scan` + `/point_cloud` | 같은 header 시각의 LaserScan과 구형 `sensor_msgs/PointCloud`, `g2_laser`. PointCloud의 `stamps` 채널은 각 점의 스캔 시작 이후 초 단위 SDK offset, `intensities`도 보존 |
| `/oak/depth/points` | `sensor_msgs/PointCloud2`, `oak_cam_a_optical_frame`, m 단위 실제 XYZ, 원래 촬영 시각 |
| 의미 설정의 labels/depth/camera_info | ZED 왼쪽 rectified optical 기준. labels는 정수 클래스 ID, depth는 optical Z(m). 실제 모델의 클래스표와 영상 정렬 확인 필요 |
| `/nomad/localization/odometry` | Pi 5 `nav_msgs/Odometry`, `odom` / `base_link`, 시각과 자세의 유효성 및 최대 0.6초 나이 검사 |
| `/nomad/localization/status` | Pi 5 `ModuleStatus.READY`, 유효기간 이내 |
| `/tf_static` | 실제 장착에서 측정한 `base_link <- oak_imu_frame` 고정 변환. 이 노드는 TF를 발행하지 않음 |
| `camera_metadata_file` | 현재 OAK 입력이 만든 metadata.json. `calibration_provenance.T_raw_imu_cameras[0]`와 `T_raw_imu_cam_a`, 선택한 K/D와 일치하는 실제 보정 정보 |

`T_odom_rawIMU(t) = T_odom_base_link(t) @ T_base_link_rawIMU`를 계산한다. 여기에 raw IMU에서 센서까지의 보정을 연결한다. `world`를 `odom`으로 이름만 바꾸거나 시뮬레이션 차체 치수를 사용하지 않는다. Pi 5는 `odom -> base_link`의 단일 소유자다.

초기화는 유효한 Pi 5 pose의 최근 2초 정지 판정 이후 5초 정지를 유지해야 시작한다. 위치나 상태가 만료되면 출력 READY가 해제된다. 시계 되돌림, 고정 TF 변경, 계산 작업 오류는 재시작이 필요한 실패다. 같은 호스트에서 동일 domain/출력 토픽으로 `real_perception` 두 개가 뜨지 않도록 파일 잠금을 사용한다. 다른 인지 발행자가 발견되어도 READY를 해제한다.

## 계산과 출력

선택 프로파일은 12cm 셀, 평면 반경 18cm, 최근 3초 점 누적, 전방 3m이다. ZED가 지면으로 분류한 실제 LiDAR/OAK 점으로 경사를 계산한다. 경사 비용은 0도 이상 20도 미만에서 `4*theta`, 20도 이상 100이며 `slope_cost`는 이를 100으로 나눈 값이다. `terrain_cost`는 클래스 비용/100으로 별도 유지한다. 두 특징의 미측정은 NaN이다. OccupancyGrid의 미관측은 -1, 일반 비용은 0~79, 차단은 100이다.

관측지원은 같은 셀에서 서로 다른 matched ZED 촬영 시각만 센다. 최근 3초의 3회와 0.5초 간격에서 최대 강도이며 같은 프레임의 많은 점이나 두 센서를 여러 관측으로 세지 않는다. 마지막 관측 뒤 10초 유지, 30초까지 선형 감소한다. 이 값은 Planning의 관측지원이며 경사 차단 판정을 반복 확정 정책으로 바꾼 것이 아니다.

| 토픽 | 형식 | QoS |
|---|---|---|
| `/nomad/navigation_costmap` | OccupancyGrid | reliable / transient_local / depth 1 |
| `/nomad/navigation_local_costmap` | OccupancyGrid | 동일 |
| `/nomad/planning_cost_layers` | PlanningCostLayers | 동일 |
| `/nomad/camera_observation_support` | OccupancyGrid | 동일 |
| `/nomad/perception/status` | ModuleStatus | reliable / volatile / depth 10 |

네 지도는 촬영 시각을 유지한다. cost layers의 내장 grid와 전역 지도 및 support는 같은 header와 격자를 사용한다. 과거 snapshot의 반복 전송으로 READY를 연장하지 않는다. 계산 및 전송 상태 진단은 `/nomad/semantic_validation/*`에만 발행하며 파일 결과는 저장하지 않는다.

## 설치와 실행

검증 환경은 Ubuntu 24.04, ROS 2 Jazzy, Python 3.12이다. Pi 5 또는 Jetson의 Humble/ARM 환경과 장치 동작은 별도 검증 대상이다. 기본 ROS 의존성 외에 NumPy, SciPy, OpenCV, PyYAML, nomad_interfaces가 필요하다. 직접 OAK를 여는 선택 실행은 DepthAI 3 API를 사용한다. 원본 장치 환경의 SDK 버전은 3.11.0이며 해당 Python 환경에서 ROS rclpy도 import할 수 있어야 한다. 이식하면서 원래 venv를 자동 참조하지 않는다. ZED SDK/ROS 드라이버와 SegFormer, Pi 5 융합 노드는 외부에서 실행한다.

저장소 루트에서 설치한 ROS 환경을 source한 뒤:

```bash
colcon build --base-paths src/nomad_interfaces src/nomad_logging src/nomad_vehicle_model src/nomad_perception \
  --packages-select nomad_interfaces nomad_logging nomad_vehicle_model nomad_perception
source install/local_setup.bash
```

LiDAR 실행 소스는 기본 시뮬레이션 빌드에 추가 의존성을 만들지 않도록 `.nomad/real_sensors`에 둔다. YDLidar SDK를 설치한 후 아래 명령으로 별도 빌드한다. 시스템의 CMake가 SDK를 찾으면 환경변수는 생략한다. 확인한 SDK는 1.2.20이다.

```bash
YDLIDAR_SDK_ROOT=/path/to/sdk_install ./.nomad/build_real_lidar.sh
source install/local_setup.bash
```

완성한 의미 설정은 설치 파일을 직접 수정하지 말고 별도 파일로 지정한다. 기본 `semantic.real.json`은 선택 보정값을 보관하지만 실제 클래스 ID와 시각, 가림 허용값이 미정이다. 세 확인 플래그도 false다. 따라서 그대로 실행하면 오류로 중단한다. 확인되지 않은 값을 시험용 숫자로 채워 실물 READY를 만들지 않는다.

현재 OAK 입력을 이미 실행 중이라면 해당 실행의 metadata.json과 완성한 의미 설정을 전달한다.

```bash
./scripts/run_real_perception.sh \
  semantic_config:=/path/to/verified_semantic.json \
  camera_metadata_file:=/path/to/current_oak/metadata.json
```

OAK를 이 launch가 단독으로 열 때만 `start_oak_capture:=true`를 사용한다. 보정과 실행 상태는 자동 임시 폴더에 두고 종료 시 정리한다. 다른 OAK 입력 발행자와 동시에 실행하지 않는다.

```bash
./scripts/run_real_perception.sh \
  semantic_config:=/path/to/verified_semantic.json start_oak_capture:=true
```

LiDAR는 별도 터미널에서 실제 포트를 지정한다. 이 명령은 장치를 구동하므로 준비된 실물 실행에서만 사용한다.

```bash
source install/local_setup.bash
ros2 run ydlidar_ros2_driver ydlidar_ros2_driver_node --ros-args \
  --params-file "$(ros2 pkg prefix nomad_perception)/share/nomad_perception/config/real/g2b.yaml" \
  -p port:=/dev/ydlidar
```

기본 domain은 42, discovery는 SUBNET이다. `NOMAD_ROS_DOMAIN_ID`, `NOMAD_DISCOVERY_RANGE`로 바꾸며 Pi 5와 Planning 호스트도 맞춘다. scan, lidar_points, oak_depth 토픽과 세 설정 파일은 launch 인자로 바꿀 수 있다. 이 실행은 Planning, Control, Gazebo나 로컬 VIO를 시작하지 않는다.

기존 `nomad_bringup`의 모듈 실행과 연결할 때는 설치된 `modules_real_perception.yaml`을 `modules_file`로 명시한다. 이 프로파일은 perception/vio/control을 모두 external로 두므로 내장 발행자를 추가하지 않는다. 실제 Planning/Control의 차량 프로파일과 구동 연결을 완성한 실행 명령은 아니다. `forest.launch.py`, 일반 GUI, 기존 시뮬레이션 지도 실행을 실물 지도와 함께 띄우지 않는다.

## 검사

```bash
source install/local_setup.bash
ROS_DOMAIN_ID=199 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q src/nomad_perception/test/real
ros2 launch nomad_perception real_perception.launch.py --show-args
ros2 run nomad_perception real_oak_capture --help
```

합성 DDS 시험은 매번 별도 토픽 namespace를 사용하고 센서나 구동기를 열지 않는다. 설치된 실행 파일에서 촬영 시각, 비영점 Pi 5 원점과 고정 장착 변환, 10도 평면의 0.4 경사 비용, 반복 관측지원, 상태 악화 시 READY 해제, 정상 종료를 검사한다. 성공은 실물 K/D, 장착 TF, 보드 간 공통 시계, 실제 모델 클래스나 경사 정확도의 검증을 뜻하지 않는다.
