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
2. [topics.yaml](../nomad_bringup/config/topics.yaml)의 localization 출력 이름을 팀 토픽으로 맞춘다.
3. 팀 VIO가 위 Odometry와 ModuleStatus를 발행하고, 관측 시각 TF를 공급한다. 타입이 다르면 경계 어댑터에서 변환한다.
4. 초기화·추적 실패·영상/IMU 중단 시 WAITING 또는 FAULT를 알리고, 이전 정상 자세에 새 timestamp를 붙여 재활용하지 않는다.
5. 공통 설정으로 관련 노드를 재시작하고 frame/stamp/QoS/TF 중복을 점검한다.

공통 [메시지 명세](../nomad_interfaces/README.md), [통합 방법](../nomad_bringup/README.md)을 함께 따른다.
