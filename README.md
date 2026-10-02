# NOMAD Gazebo 시뮬레이션 환경

## 모듈 분리 구조 (2026-10-02)

현재 자율주행은 인지 / VIO / Path Planning / Control로 분리했다. 이전 단일 planning 패키지 설명보다 이 절과 각 모듈 문서를 우선한다.

- [인지 입력·출력·교체](src/nomad_perception/README.md)
- [VIO 입력·출력·TF 소유권](src/nomad_vio/README.md)
- [Path Planning: GPP·LPP·후진 복구·명령 선택](src/nomad_path_planning/README.md)
- [Control: 속도·조향 명령 실행·실측 피드백](src/nomad_control/README.md)
- [공통 메시지](src/nomad_interfaces/README.md) / [전체 실행·통합 가이드](src/nomad_bringup/README.md)

토픽명은 `src/nomad_bringup/config/topics.yaml`, 교체 공급자는 `modules.yaml`, 공통 차량 기하는 `vehicle.yaml`에서 관리한다. YAML 수정 후 관련 노드를 재시작한다. 메시지 타입 변환에는 어댑터가 필요하다.

Planning은 `/nomad/planning/drive_command`의 `nomad_interfaces/msg/DriveCommand`로 목표 속도(m/s, 음수 후진)와 전륜 중심 조향각(rad, 양수 왼쪽), 유효기간과 정지 요청을 보낸다. Control은 `/nomad/control/vehicle_state`로 실측 피드백을 보낸다. 경로·지도·도착·후진 선택은 Planning에만 있다. 기본 Control의 `/cmd_vel`은 Gazebo용 내부 출력이다.

기본 VIO는 ground truth 어댑터이며 실제 VIO 추정기가 아니다. `vio: external`이면 `tf_owner: vio`를 함께 설정하고 Gazebo ground-truth 동적 TF를 끈다. 센서 정적 TF는 유지한다. 이미 실행 중인 사용자 시뮬레이션을 자동 종료/초기화하지 않는다.

빌드: `source .nomad/env.sh` 후 `colcon build --base-paths src --packages-up-to nomad_bringup nomad_gazebo --symlink-install`. 전체 실행은 기존 `./run_autonomy.sh`. 자세한 단독 실행 명령은 통합 가이드를 따른다.


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

시뮬레이터 시작 터미널에서는 `source`가 필요하지 않습니다. `run_forest.sh`가 자체적으로 읽습니다. 기본 `ROS_DOMAIN_ID=42`, 발견 범위는 `LOCALHOST`입니다. 시뮬레이션 실행 시 Gazebo 통신은 `nomad_gazebo_<hostname>_42` partition과 `127.0.0.1`로 분리합니다. 팀 PC 간 ROS 통신은 별도 네트워크 설정이 필요합니다. ROS 소비 노드에서는 `use_sim_time=true`로 설정하세요.

차량 수동 주행은 Gazebo 창의 키가 아니라 별도 터미널에서 조종 창을 엽니다.

```bash
~/nomad_ws/run_teleop.sh
```

조종 창을 클릭한 상태에서 `W`/`S`를 한 번 누를 때마다 목표 속도가 0.1 m/s씩 바뀝니다(전진 최대 1.0 m/s, 후진 최대 0.6 m/s). `A`/`D`는 **누르는 동안만** 좌우로 조향하고, 손을 떼면 중앙으로 부드럽게 복귀합니다. `Space`는 즉시 정지, `Q`는 조종 창 종료입니다. 조종 창이 포커스를 잃으면 안전을 위해 속도와 조향 명령을 0으로 되돌립니다. 차량 주행 화면은 Gazebo 창에 보이므로 두 창을 나란히 놓고 조종하면 편합니다.

RViz로 ROS 토픽을 보고 싶다면 또 다른 터미널에서 `source ~/nomad_ws/.nomad/env.sh` 후 `rviz2`를 실행하세요. VINS-Fusion은 이 저장소에 포함되지 않은 별도 워크스페이스입니다. VINS를 사용하는 팀원은 Gazebo의 `/oak/left/image_rect`, `/oak/right/image_rect`, `/oak/imu`를 자신의 설정 파일에 연결해야 합니다.

## 맵·차량·토픽

약 70×50m 맵에 흙·낙엽 바닥, 굴곡진 지면, 뒤쪽 약 4m 고지대를 구성했습니다. 왼쪽 상승 구간은 약 7.8m로 가파르고 오른쪽은 약 22m로 완만합니다. 전체 지면에 최대 ±32.5cm 굴곡을 더했으며 출발 접지 구역만 평탄합니다. 시각 메시와 충돌 메시의 지면 높이는 같습니다.

이전 NOMAD의 갈림길·경로 좌표는 배치 기준으로 보존하지만 도로 색은 표시하지 않습니다. 이전 돌 차단·벽·동굴·줄 형태 경계는 현재 월드에 없으며, 경로 밖을 완전히 막는 맵은 아닙니다. 나무 363그루 중 길 근처 103그루만 충돌체가 있고 나머지는 시각용입니다. 작은 돌 260개와 떨어진 가지, 풀도 시각용입니다. 두 번째 갈림길 왼쪽에는 아래 설명한 높은 풀 벽이 있지만 물리적인 차단은 아닙니다.

바닥은 Poly Haven CC0 흙·낙엽 사진 텍스처를 혼합했습니다. 원본과 출처는 `src/nomad_gazebo/assets/materials/`에 포함되어 실행 시 다운로드가 필요하지 않습니다. 재질 색은 마찰계수나 traversability 점수가 아닙니다.

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

## 전체 숲 밀도 2.5배 (2026-10-01)

방금 만든 국소 숲 영역과 이전 줄 형태의 경계는 기본 월드에서 제거했다. 기존 전체 맵 나무 145그루를 기준으로 2.5배를 올림해 363그루로 늘린다. 특정 영역을 막기 위한 줄/울타리/차단 배치는 만들지 않는다. 기존 145그루 위치는 유지하고 추가 218그루는 맵 전체에 불규칙하게 분포시킨다. 새 나무는 기존 경로 중심에서 최소 2.3m, 다른 줄기 중심에서 최소 1.1m 떨어뜨린다. 도로 색은 없다.

입력은 `assets/authoring/forest_density.json`이다. multiplier를 바꾸면 재생성할 수 있다. `assets/authoring/layout.json`은 145그루 원본으로 보존하고 `assets/authoring/forest_layout.json`은 밀도 증가 후 생성된 배치를 기록한다. 생성기는 항상 원본을 읽으므로 재생성할 때 2.5배가 누적되지 않는다. 가까운 나무만 기존 충돌 정책을 따르고 먼 나무는 묶인 시각 메시다. `vegetation:=False`에서는 전체 나무를 숨긴다. 전체 지형, 경사, 센서 설정은 바꾸지 않는다.

## 경로 위의 키 큰 풀

두 번째 갈림길 왼쪽 연결 경로(`rock_branch`)의 중앙에 높이 2.04~2.89m 풀을 벽처럼 모았다. 길에 수직인 방향에서 위에서 볼 때 반시계 방향으로 0.3rad 더 돌리고 양끝을 늘려 폭 7.5m, 두께 1.3m로 배치한다. 오른쪽 경로에는 높은 풀을 두지 않는다. 별도로 맵 전체에는 높이 0.2~0.45m 풀을 약 3.4m 간격으로 듬성듬성 배치한다. 두 풀 영역은 각각 하나의 시각 메시로 묶고 충돌체는 넣지 않아 차량이 통과할 수 있다. 카메라 시야가 가려질 수 있으며 GPU LiDAR에서도 보일 수 있다. 식생 렌더링 부하는 추가된다.

높이·간격은 `src/nomad_gazebo/assets/authoring/path_grass.json`에서 설정한다. 생성기로 재생성하고 빌드한 뒤 Gazebo를 다시 실행하면 적용된다. `vegetation:=False`에서는 이 풀도 숨긴다.

## 야지 계획 비용 (2026-10-01)

`nomad_path_planning`의 기본 지형 모드는 `terrain.mode: geometry`다.
깊이 영상과 촬영 시각 TF로 관측 표면의 기울기·평면 잔차 거칠기·높이 불연속을
평가하고, 기준 초과 영역을 LiDAR 장애물과 함께 costmap에 차단 비용 100으로
추가한다. `/nomad/terrain_costmap`은 팽창 전 기하 비용/위험 영역이며,
`/nomad/terrain_labels`는 이전 colour 모드에서만 발행한다.
기본 기준 약 20°/RMS 4cm/잔차 범위 18cm는 실측 차량 한계가 아닌 초기값이다.
설정과 관측 누락·식생·낙차 등의 한계는 `src/nomad_path_planning/README.md`를 따른다.
월드·차량·센서 생성 입력은 이 변경으로 수정하지 않는다.

낮은 풀은 정렬 RGB-D의 색상·지면 대비 높이·반복 관측으로 구분해 통과 가능으로
처리한다(`terrain.grass_enabled`). 초기 휴리스틱으로서 카메라 밖/높은 풀은
구분하지 못한다. 센서에서 풀을 숨기지 않으며 여유 반경은 0.70m 그대로다.
관측 조건과 한계는 [계획 README](src/nomad_path_planning/README.md#통과-가능한-낮은-풀-인식)를 따른다.

## 출발점 오른쪽 풀 한 포기 제거 (2026-10-01)

사용자 요청으로 낮은 풀 중심 `(-30.654947, -18.369343)` 한 포기만 제거했다.
`assets/authoring/path_grass.json`의 `background_exclusions`에 반경 0.1m의
생성 제외 항목을 기록한다. 모든 난수 배치 생성 후 제외하므로 다른 풀 위치는
변하지 않는다. 낮은 풀은 300개에서 299개이며 나무·높은 풀·지형·차량은 유지한다.
통과 영역 costmap 예외는 사용하지 않는다. 기존 Gazebo 세션은 재시작해야 반영된다.

시작 방향은 `assets/authoring/spawn_pose.json`의 `yaw_rad`에서 설정한다.
현재 값은 0.8267518515rad(약 47.37°)로, 요청 당시 차량 방향에서 왼쪽으로 90°
회전한 값이다. 시작 위치 `(-30, -18, 0.03)`은 유지하며 재실행부터 적용된다.

계획기는 깊이 관측에서 차량 접지 기준과 이어지는 지면을 확인하여 일치하는
LiDAR 지면 반사를 장애물에서 제외한다(`terrain.ground_filter_enabled`).
내리막 지면 누락을 줄이기 위해 깊이는 2픽셀 간격으로 사용한다. 누적 LiDAR
높이는 작은 XY 구간별 최소/최대 범위를 보존해 기록 개수 초과로 폐기하지 않는다.
확인된 지면 높이는 최대 10초 보관해 카메라 가장자리 밖의 LiDAR 지면 반사도
제외한다. 신선한 깊이 입력이 있어야 사용하며 새 물체가 관측되면 옛 지면 기록을
폐기한다. 처음부터 관측하지 못한 지면은 제외 대상이 아니다.
최신 기본 `terrain.objects_only: true`에서는 연결 지면의 경사·거칠기 위험 비용을
끄고 지면 위 12cm 이상 돌출 물체를 깊이 장애물로 판정한다. 기준 지면이 없는
부분은 미확인으로 유지하며 LiDAR 장애물과 팽창은 유지한다.
현재는 사용자 요청으로 `terrain.slope_enabled: false`, `tilt_guard_enabled: false`로
경사 비용·경사 차단·지면 후보 경사 제한·차량/스캔 기울기 정지를 비활성화했다.
고체 장애물 판정과 3D TF 보정은 유지한다. 계획 노드 재시작 후 적용된다.
관측과 허용 오차 조건은
[계획 README](src/nomad_path_planning/README.md#깊이-기반-지면-반사-구분)에 있다.

2026-10-02부터 `navigation.lidar_enabled: false`로 자율주행의 LiDAR 판정을 끈다.
기존 LiDAR+카메라 코스트맵과 기울기 계산은 유지하고, GPP/LPP/제어/복구는 별도의
`/nomad/navigation_costmap`, `/nomad/navigation_local_costmap`을 사용한다.
주행 지도는 카메라로 물체를 판정하고 LiDAR 장애물/팽창만 제외한다. 기존 LiDAR
자유 공간 관측은 보존해 차체 아래 카메라 사각지대 때문에 출발이 막히지 않게 한다.
물체 여유 반경과 미관측 차단은 유지하며 반사 뒤의 미관측 영역은 열지 않는다.
계획 노드 재시작부터 반영되며 자세한 설정은
[계획 README](src/nomad_path_planning/README.md#자율주행-lidar-제외-2026-10-02)에 있다.

깊이 계산 중 위치·센서 수신이 밀리지 않도록 센서 수신/health와 지도 계산을
분리했다. 새 깊이 입력은 0.6초, 완료된 지역 지도는 기존 2초 유효 기간으로
별도 검사하며 입력 중단과 계산 정지 모두 정지 조건으로 유지한다.

곡면에서도 지면을 연결하도록 시작점은 가까운 관측의 낮은 표면 띠로 선택한다.
전진 LPP는 목표가 1.5m보다 가까우면 목표 거리까지만 후보를 생성해, 목표 뒤의
차단·미관측 영역 때문에 도착 전부터 멈추지 않도록 한다.
