# VIO 모듈 — nomad_vio

> 인터페이스 버전 1 · 2026-10-02 · ROS 2 Jazzy

VIO 담당자는 연속적인 차량 3D 위치·자세·속도와 추정 상태를 제공한다. 인지는 관측을 지도에 투영하고, Planning은 현재 위치에서 경로를 생성하는 데 이 출력을 쓴다.

## 1. 현재 구현과 교체 경계

```mermaid
flowchart LR
    G[Gazebo ground truth /odom] --> S[기본 SimVIO 어댑터]
    C[외부 스테레오 / IMU] --> V[팀 VIO 구현]
    S -->|기본 구현 선택| O[Odometry + ModuleStatus]
    V -->|외부 구현 선택| O
    O --> P[인지 / Planning]
```

기본 실행 파일 `sim_vio`, 노드 `nomad_vio`는 **실제 VIO 알고리즘이 아니다.** Gazebo `/odom`을 검증하고 공통 출력으로 전달하는 대체 모듈이다. 기존 OpenVINS 등의 코드를 수정하거나 VIO 추정 성능을 검증한 작업은 아니다.

## 2. 입력

| 구현 | 설정 키 / 기본 토픽 | 타입 | 의미 |
|---|---|---|---|
| 현재 기본 | `sensors.odometry` / `/odom` | `nav_msgs/msg/Odometry` | Gazebo의 3D ground truth, 기본 50Hz 시뮬레이션 시간 |
| 외부 VIO용 제공 센서 | `sensors.left_image`, `right_image` / `/oak/{left,right}/image_rect` | `sensor_msgs/msg/Image` | 스테레오 mono8, 기본 640×480 |
| 외부 VIO용 제공 센서 | `sensors.left_info`, `right_info` / `/oak/{left,right}/camera_info` | `sensor_msgs/msg/CameraInfo` | 스테레오 내부 파라미터 |
| 외부 VIO용 제공 센서 | `sensors.imu` / `/oak/imu` | `sensor_msgs/msg/Imu` | 가속도 m/s², 각속도 rad/s, 기본 200Hz 시뮬레이션 시간 |
| 인프라 | `/tf_static`, `/clock` | TF / Clock | 센서 외부 파라미터 및 시간 |

기본 SimVIO는 카메라·IMU를 구독하지 않는다. 외부 구현이 사용할 센서 토픽을 표로 정의한 것이다. 입력 센서는 Best Effort 수신을 권장하며, 현재 SimVIO의 odom 구독은 Sensor Data QoS다. 영상·IMU 동기화와 카메라→차량 변환은 VIO 담당자의 책임이다.

## 3. 출력 계약

| 설정 키 | 기본 토픽 | 타입 | QoS / 발행 |
|---|---|---|---|
| `localization.odometry` | `/nomad/localization/odometry` | `nav_msgs/msg/Odometry` | Reliable, Volatile, depth 10; 유효 입력마다 전달 |
| `localization.status` | `/nomad/localization/status` | `nomad_interfaces/msg/ModuleStatus` | Reliable, Volatile, depth 10; 기본 wall 10Hz, lease 0.6초 |

`Odometry` 필드:

| 필드 | 정의 |
|---|---|
| `header.stamp` | **측정/추정 시각**의 ROS time. 재발행 시 현재 시각으로 바꾸지 않는다 |
| `header.frame_id` | `odom`(공통 planning_frame) |
| `child_frame_id` | `base_link`(공통 base_frame) |
| `pose.pose.position` | odom 좌표계에서 차량 base_link 원점의 x,y,z [m] |
| `pose.pose.orientation` | base_link 자세 quaternion x,y,z,w; 정규화 필요 |
| `twist.twist` | child_frame_id 좌표계 속도 [m/s], 각속도 [rad/s] |
| `pose.covariance`, `twist.covariance` | 해당 추정 불확실성. SimVIO는 입력을 보존하며 새 추정치를 만들지 않는다 |

차량 +X 전방, +Y 왼쪽, +Z 위다. 현재 base_link는 차량 중심 기준이며 뒷축보다 0.36m 앞에 있다. 카메라/IMU 자체 pose를 base_link pose라고 그대로 발행하면 안 된다. Planning은 현재 전역 x,y와 yaw를 쓰지만 인지는 3D 변환을 필요로 한다.

Planning과 기본 SimVIO는 frame, 유한 좌표, quaternion 정규화, 최신 stamp를 검사한다. 기본 허용 odom 나이는 0.6 ROS초 / 3 wall초다. 같은 timestamp 반복 발행으로 측정의 유효기간을 연장하지 않는다. 위치가 갑자기 크게 뛰거나 시뮬레이션 시계가 뒤로 가면 Planning 입력은 fault로 유지되며 재시작이 필요하다.

## 4. TF 소유권

**odom → base_link는 한 공급자만 발행한다.**

| 모드 | modules.yaml | odom → base_link 공급자 |
|---|---|---|
| 현재 기본 | `vio: builtin`, `tf_owner: gazebo` | Gazebo ground-truth TF bridge; SimVIO는 TF를 발행하지 않음 |
| 외부 VIO | `vio: external`, `tf_owner: vio` | 외부 VIO 또는 그 어댑터가 동적 TF 발행 |

`nomad_bringup forest.launch.py`는 외부 VIO 모드에서 Gazebo의 ground-truth TF bridge만 끈다. 센서 고정 TF와 관절 TF는 기존 robot_state_publisher가 유지한다. 이미 실행 중인 Gazebo에 Planning만 붙이는 경우 이 설정으로 기존 bridge가 자동 변경되지는 않는다. 사용자가 Gazebo 실행 시 `publish_ground_truth_tf:=false`를 적용해야 한다.

외부 VIO가 map 프레임을 쓰면 모든 소비자와 지도·목표 프레임을 함께 맞추거나 어댑터에서 odom 계약으로 변환한다. 토픽 이름 변경은 좌표 변환 기능이 아니다. 전역 loop closure의 위치 점프를 연속 odom으로 그대로 전달하지 않는다.

## 5. 실행·교체

```bash
cd ~/nomad_ws
source .nomad/env.sh
ros2 launch nomad_bringup vio.launch.py
```

외부 구현 연결 순서:

1. `modules.yaml`에서 `vio: external`, `tf_owner: vio`를 설정한다.
2. [topics.yaml](../../../nomad_bringup/config/topics.yaml)의 localization 출력 이름을 팀 토픽으로 맞춘다.
3. 팀 VIO가 위 Odometry와 ModuleStatus를 발행하고, 관측 시각 TF를 공급한다. 타입이 다르면 경계 어댑터에서 변환한다.
4. 초기화·추적 실패·영상/IMU 중단 시 WAITING 또는 FAULT를 알리고, 이전 정상 자세에 새 timestamp를 붙여 재활용하지 않는다.
5. 공통 설정으로 관련 노드를 재시작하고 frame/stamp/QoS/TF 중복을 점검한다.

공통 [메시지 명세](../../../nomad_interfaces/README.md), [통합 방법](../../../nomad_bringup/README.md)을 함께 따른다.

## GNSS·VIO 융합 — 선택 실행

`nomad_vio`의 실행/차량 기준점 어댑터와 `nomad_localization`의 C++ Ceres 백엔드를 사용한다. 기존 `sim_vio` GT 어댑터와 기본 실행은 유지한다.

```bash
cd ~/nomad_ws
source .nomad/env.sh
colcon build --base-paths src --packages-up-to nomad_vio --symlink-install
source install/local_setup.bash
# 별도 터미널: 동적 TF는 융합 어댑터만 발행
./run_forest.sh publish_ground_truth_tf:=false
# VIO를 직접 실행했다면: Basalt /basalt/odom 또는 OpenVINS /ov_msckf/poseimu 수신
ros2 launch nomad_vio fusion.launch.py method:=basalt
# VIO까지 실행: 아래 팀용 설치를 마쳤다면 NOMAD 환경만 source
ros2 launch nomad_vio fusion.launch.py method:=basalt manage_vio:=true
# OpenVINS는 method:=openvins
```

두 융합 명령 중 하나만 실행한다. Basalt/OpenVINS 소스는 아래 고정 버전 설치 스크립트로 같은 워크스페이스에 가져온다. 관리 실행은 `vio_config_directory`로 설정 폴더를 바꿀 수 있다. 실물은 `use_sim_time:=false`를 지정한다. GPS 오류노드는 별도로 수동 실행한다.

```bash
# 오류 GPS를 시험할 때만 GPS.md 명령으로 오류노드를 먼저 실행
ros2 launch nomad_vio fusion.launch.py method:=basalt   gps_topic:=/experiment/gps/fix quality_topic:=/experiment/gps/quality
# 공통 차량 위치/상태 확인
ros2 topic echo /nomad/localization/odometry --once
ros2 topic echo /nomad/localization/status --once
# 팀 자율주행 연결: GT 공급자 대신 외부 공급자를 선택
ros2 launch nomad_bringup autonomy.launch.py   modules_file:=$HOME/nomad_ws/src/nomad_bringup/config/modules_gnss_vio.yaml
```

설정 위치:
- `nomad_localization/config/fusion.yaml`: ENU datum, GNSS 안테나 lever arm, 가중치·윈도·실패 판단. datum은 선택 월드와 반드시 동일해야 한다.
- `nomad_vio/config/fusion_adapter.yaml`: IMU→차량 중심 변환, 공통 frame·TF. 현재 Gazebo IMU `(0.46,0,0.29)m`·차량 축과 평행 기준. 회전 장착은 별도 외부파라미터 변환이 필요하다.
- `nomad_bringup/config/modules_gnss_vio.yaml`: `vio: external`, `tf_owner: vio`. 기본 modules.yaml은 GT 그대로다.

입력은 GNSS 위치/공분산/품질과 VIO pose다. 원시 IMU를 외부에서 재융합하지 않는다. 10초·5Hz 포즈 윈도, 2Hz solve, 최대10회 반복. `/fusion/odometry`는 IMU 기준, 어댑터의 `/nomad/localization/odometry`는 base_link 기준이다. `/fusion/status`의 자세 유효성을 같은 측정 시각에 확인한 뒤 공통 출력/TF를 발행한다. GNSS-only의 보류된 자세는 자율주행에 전달하지 않고 WAITING으로 알린다. 속도는 포즈 차분값이고 공분산은 미보정 placeholder이며 보정 점프가 생길 수 있다. 연속 local odom 분리나 Planning 재시작 정책은 추가 검증 대상이다.

자동 VIO 재시작은 기본 꺼져 있다. `manage_vio:=true auto_restart:=true`일 때만 관리 프로세스 재시작을 요청한다. `stop_before_restart:=true`는 `/fusion/recovery_stop`을 내보내고 `/fusion/restart_ready` 정지 확인을 기다린다. **팀 Control의 정지·확인 연결은 아직 없으므로 자동 재시작을 기본 자율주행에 활성화하지 않는다.** 이 토픽을 rosbag/GT로 대신 확인하지 않는다. 내부 VIO 초기화 성공이나 임의 실패 복구를 보장하지 않는다.

GNSS 품질이 없으면 unknown으로 사용하며 비FIX 가중치를 적용한다. 초기 정렬은 이동 구간에서 GPS와 VIO의 yaw/translation을 추정한다. GNSS가 yaw를 측정하는 것이 아니다. 실제 센서의 시간·원점·높이·장착 오프셋·불확실성은 실물에서 조정해야 한다.

## 팀용 Basalt·OpenVINS 설치 / 단독 실행

Ubuntu 24.04 + ROS2 Jazzy가 설치된 PC 기준. Git에는 고정 소스 버전·Jazzy 호환 패치·NOMAD 센서 설정을 보관하고, 큰 원본 소스/빌드 결과는 설치 시 가져온다. 다른 PC는 인터넷이 필요하다.

```bash
cd ~/nomad_ws
./.nomad/install_vio.sh                 # 소스 다운로드·의존성 설치·두 VIO/융합기 빌드
source .nomad/env.sh
ros2 launch nomad_vio estimator.launch.py method:=basalt    # Basalt만
ros2 launch nomad_vio estimator.launch.py method:=openvins  # OpenVINS만
# 둘 중 하나만 실행. 시뮬레이션/rosbag은 별도 실행.
# VIO + GNSS 융합까지 실행할 때는 estimator 대신 아래 한 명령
ros2 launch nomad_vio fusion.launch.py method:=basalt manage_vio:=true
# OpenVINS + 융합은 method:=openvins
```

- 재빌드만: `./.nomad/install_vio.sh --skip-deps`. 소스만 준비: `--sources-only`.
- 소스: `src/third_party/vio/`(Git 제외). `.nomad/vio/sources.json` 고정 commit, `patches/` 호환 수정. 기존 다른 SHA 소스는 덮어쓰지 않고 중단한다.
- 설정: `src/nomad_vio/config/vio/basalt/`, `config/vio/openvins/nomad_gazebo/`. 설치 후 패키지 share의 복사본을 읽으므로 설정 변경 후 재빌드한다. `vio_config_directory`로 외부 설정을 지정할 수도 있다.
- 저장된 값은 Gazebo 스테레오·IMU 보정값이다. 실제 OAK/IMU에 그대로 사용하지 않는다. Pi5 실시간 성능과 새 PC 설치는 별도 검증 필요하다.
- Basalt ROS 포트는 upstream 유지보수가 중단된 고정 버전이며 NOMAD Jazzy/정상 종료 패치를 포함한다. upstream 라이선스는 원 소스에 유지한다(OpenVINS GPLv3).

초기 실행 순서: Gazebo → VIO/융합 실행 → 잠시 정지해 IMU 초기화 → `./run_teleop.sh`로 2m 이상 수동 정렬 주행 → 정지·teleop 종료 → localization/status READY 확인 → 자율주행 실행. GPS는 정지 yaw를 제공하지 않으므로 초기 이동 없이 융합 READY를 기다리면 출발할 수 없다. OpenVINS는 시작 ZUPT 동안 feature update 전까지 pose 출력이 없을 수 있으므로 첫 포즈를 기다린 뒤에만 이동시키는 시험 조건을 쓰지 않는다.

rosbag 재생은 `use_sim_time=true`에 맞춰 `ros2 bag play <bag> --clock`으로 실행한다. GPS가 없는 기존 bag은 `estimator.launch.py`로 VIO만 실행하며, GNSS 융합에는 같은 시각의 GPS 입력이 필요하다.
