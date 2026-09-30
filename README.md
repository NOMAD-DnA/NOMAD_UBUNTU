# NOMAD Gazebo 시뮬레이션 환경

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic에서 NOMAD의 야지 주행을 시험하는 워크스페이스입니다. 기본 실행은 `nomad_gazebo` 패키지입니다. 이전 MVSim 소스와 맵 제작 자료는 `src/mvsim`, `src/nomad_sim`에 보존했지만 기본 빌드·실행에는 사용하지 않습니다.

## 준비와 빌드

ROS 2 Jazzy와 Gazebo Harmonic을 설치한 뒤 ROS-Gazebo 브리지를 준비합니다. 처음 설치하는 팀원은 아래 명령을 사용하세요.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep python3-pyqt5 ros-jazzy-ros-gz ros-jazzy-robot-state-publisher
git clone https://github.com/NOMAD-DnA/NOMAD_UBUNTU.git ~/nomad_ws
cd ~/nomad_ws
rosdep install --from-paths src/nomad_gazebo --ignore-src -r -y --rosdistro jazzy
./.nomad/build.sh
```

이미 `~/nomad_ws`가 있으면 복제로 덮어쓰지 말고 기존 파일을 먼저 확인하세요. `build/`, `install/`, `log/`는 로컬 생성물이며 Git에 올리지 않습니다.

## 실행

```bash
cd ~/nomad_ws
./run_forest.sh
```

Gazebo 창이 열립니다. 종료는 실행 터미널에서 Ctrl+C입니다. 기본값은 현재 NOMAD 맵의 나무·풀을 포함한 숲 버전입니다. 식생 렌더링 부하를 분리하고 싶을 때만 `./run_forest.sh vegetation:=False`를 사용하세요. `headless:=True`는 GUI 없는 별도 시험용입니다. 같은 ROS domain/Gazebo partition에 시뮬레이터를 중복 실행하지 마세요.

다른 터미널에서 토픽을 보려면 그 터미널에서도 환경을 읽어야 합니다.

```bash
source ~/nomad_ws/.nomad/env.sh
ros2 topic list -t
```

매번 경로를 입력하기 싫다면 개인 `~/.bashrc`에 아래 함수를 추가할 수 있습니다. 설치 스크립트가 개인 설정을 자동 변경하지는 않습니다.

```bash
nomad() { source "$HOME/nomad_ws/.nomad/env.sh"; }
```

시뮬레이터 시작 터미널에서는 `source`가 필요하지 않습니다. `run_forest.sh`가 자체적으로 읽습니다. 기본 `ROS_DOMAIN_ID=42`, 발견 범위는 `LOCALHOST`입니다. 팀 PC 간 ROS 통신은 별도 네트워크 설정이 필요합니다. ROS 소비 노드에서는 `use_sim_time=true`로 설정하세요.

차량 수동 주행은 Gazebo 창의 키가 아니라 별도 터미널에서 조종 창을 엽니다.

```bash
source ~/nomad_ws/.nomad/env.sh
ros2 run nomad_gazebo teleop.py
```

조종 창을 클릭한 상태에서 `W`/`S`를 한 번 누를 때마다 목표 속도가 0.1 m/s씩 바뀝니다(전진 최대 1.0 m/s, 후진 최대 0.6 m/s). `A`/`D`는 **누르는 동안만** 좌우로 조향하고, 손을 떼면 중앙으로 부드럽게 복귀합니다. `Space`는 즉시 정지, `Q`는 조종 창 종료입니다. 조종 창이 포커스를 잃으면 안전을 위해 속도와 조향 명령을 0으로 되돌립니다. 차량 주행 화면은 Gazebo 창에 보이므로 두 창을 나란히 놓고 조종하면 편합니다.

RViz로 ROS 토픽을 보고 싶다면 또 다른 터미널에서 `source ~/nomad_ws/.nomad/env.sh` 후 `rviz2`를 실행하세요. VINS-Fusion은 이 저장소에 포함되지 않은 별도 워크스페이스입니다. VINS를 사용하는 팀원은 Gazebo의 `/oak/left/image_rect`, `/oak/right/image_rect`, `/oak/imu`를 자신의 설정 파일에 연결해야 합니다.

## 맵·차량·토픽

맵의 길 관계는 이전 NOMAD 맵과 같습니다. 첫 갈림길의 평지·오르막, 평지 쪽의 돌로 막힌 연결 길, 목적지 가까운 막다른 길, 돌아와 오르막을 선택하는 구조를 유지했습니다. `src/nomad_sim/assets/forest`의 현재 높이맵과 배치를 Gazebo용 3D 충돌 메시로 옮겼고, 지면 색상만 논문 숲 장면을 참고해 마른 흙 톤으로 표현했습니다. 시각 텍스처 색은 마찰계수나 traversability 점수가 아닙니다.

차량은 차체·바퀴·앞 조향 링크로 구성한 **임시 Ackermann 4륜 모델**입니다. 질량·치수·마찰은 실측값이 아닙니다. 현재 속도 명령형 Ackermann 플러그인은 모터 토크 상한을 검증하는 구동기가 아닙니다. 페이로드에 따라 경사에서 실제로 멈추는 시험은 토크 제한 구동기와 별도 검증이 필요합니다.

주요 ROS 토픽은 `/cmd_vel`(Twist), `/odom`(Gazebo 3D ground truth), `/joint_states`, `/tf`, `/clock`, `/oak/rgb/image_raw`, `/oak/left/image_rect`, `/oak/right/image_rect`, `/oak/depth/image_raw`, `/oak/imu`(6축 IMU), `/scan`입니다. 카메라 640×480·30Hz, IMU 200Hz, LiDAR 360°·500점·10Hz는 모두 **시뮬레이션 시간 기준 임시값**입니다. 실제 OAK-D Lite FF와 YDLIDAR G2의 측정 보정값으로 바꿔야 합니다. OAK-D Lite 초기 Kickstarter 제품에는 IMU가 없을 수 있으니 실제 장비를 확인하세요.

Gazebo는 자체 Transport로 센서·차량을 계산하고 `ros_gz_bridge`가 ROS 2 토픽으로 변환합니다. `gazebo_sensor_adapter.py`와 `lidar_bridge.py`는 변환된 ROS 센서 메시지를 팀 토픽 형식으로 정리합니다. 센서 위치는 `config/sensors.yaml`과 생성된 차량 SDF/URDF를 함께 확인하세요. 카메라·LiDAR·IMU 장착값을 바꾼 뒤에는 `scripts/generate_vehicle.py`로 모델을 재생성하고 빌드해야 합니다.

## 폴더 구조

- `src/nomad_gazebo/worlds/`, `models/`, `urdf/`: Gazebo 월드·물리 차량·TF 차량 모델
- `src/nomad_gazebo/assets/authoring/`, `assets/geometry/`: 원본 맵 배치/높이/흙길 입력과 Gazebo 지형 메시
- `src/nomad_gazebo/config/`, `launch/`, `scripts/`: 센서 값, 브리지, 실행, 제작 코드
- `src/nomad_sim/`, `src/mvsim/`: 이전 MVSim 구현과 맵의 원본 데이터; 기본 실행에는 사용하지 않음
- `.nomad/`: 환경·빌드 스크립트

팀 AI가 코드 작업을 할 때는 [AGENTS.md](AGENTS.md)에서 토픽·TF 소유권과 생성 파일의 경계를 먼저 확인하세요. Gazebo 패키지의 세부 사항은 [패키지 README](src/nomad_gazebo/README.md)에 있습니다.
