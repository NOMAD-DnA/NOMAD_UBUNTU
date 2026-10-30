# NOMAD 실차 PC 운영 GUI

PC에서 전방 영상과 주행 그리드맵을 받아 표시하고, 지도 셀을 선택한 뒤 버튼을 눌러 목표점을 보낸다. 차량 구동 노드는 PC에서 직접 실행하지 않으며, 보드 시작 버튼은 각 보드의 SSH 실행 스크립트를 호출한다.

## ROS 토픽 계약

| 방향 | 기본 토픽 | 타입 | 조건 |
|---|---|---|---|
| 수신 | `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | Jetson 통합 OAK 취득기 발행. `rgb8`, `bgr8`, `mono8` 지원, SensorData best effort QoS |
| 예비 수신 | `/perception/semantic/input_image` | `sensor_msgs/msg/Image` | 같은 통합 실행본의 추론 영상. 주 영상이 2초 동안 없을 때 표시 |
| 수신 | `/nomad/navigation_costmap` | `nav_msgs/msg/OccupancyGrid` | `odom` 프레임, 회전 없는 origin, reliable + transient local QoS |
| 수신(선택) | `/nomad/localization/odometry` | `nav_msgs/msg/Odometry` | `odom` 프레임 차량 위치 표시 |
| 발행 | `/goal_pose` | `geometry_msgs/msg/PoseStamped` | 버튼 클릭 시 1회, `odom` 프레임, 선택 셀 중심 XY, 방향 quaternion `w=1` |

지도값 `-1`(미관측) 또는 `80` 이상(차단)은 목표 선택을 막는다. 지도 클릭은 발행하지 않으며, 목표 토픽을 구독하는 노드가 연결된 뒤 발행 버튼을 따로 눌러야 한다. 발행 표시는 플래너의 목표 수락이나 차량 주행을 보장하지 않는다.

토픽 이름은 CLI 옵션 `--image`, `--image-fallback`, `--grid`, `--odom`, `--goal`로 바꿀 수 있다. 기본값은 `nomad_bringup/config/topics.yaml`과 일치한다. PC와 Jetson은 동일한 `ROS_DOMAIN_ID`를 사용해야 한다.

## PC 빌드·실행

Ubuntu 24.04/ROS 2 Jazzy 기준. `python3-pyqt5`를 설치한 뒤 워크스페이스 루트에서:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --packages-select nomad_operator_gui --symlink-install
ROS_DOMAIN_ID=<Jetson과_같은_번호> ./scripts/run_real_operator_gui.sh
```

Jetson이 아직 데이터를 발행하지 않으면 화면에는 수신 대기 상태가 표시된다. 시험에는 실제 차량 명령 없이 영상/지도 토픽만 발행하면 된다.

## 보드 원격 시작 버튼

`~/.config/nomad/boards.env`에 `NOMAD_PI5_HOST`, `NOMAD_PI5_SCRIPT`, `NOMAD_PI4_HOST`, `NOMAD_PI4_SCRIPT`, `NOMAD_SSH_KEY`를 설정한다. 스크립트 경로는 **각 보드에서의 절대 경로**다. GUI는 이 파일을 실행할 때 읽는다. Jetson 주소·계정·스크립트 경로는 PC 설정의 환경변수 `NOMAD_JETSON_HOST`, `NOMAD_JETSON_USER`, `NOMAD_JETSON_SCRIPT`로 지정한다. GUI 화면은 이 내부 연결값을 입력받지 않는다. 현재 Jetson 경로는 `/home/nvidia/nomad_semantic_20261030/scripts/run_jetson_autonomy.sh`다. GUI는 실행 ID를 생성해 팀 배포본의 `start --run-id`와 `stop --run-id`를 직접 호출한다.

‘세 보드 자율주행 기능 시작’은 확인 대화상자 후 Pi 5 → Jetson → Pi 4 순으로 호출한다. Jetson의 영상·위치·지도 상태가 `READY`가 될 때까지 최대 130초 기다린 뒤 Pi 4를 시작한다. Pi 4에는 `NOMAD_ENABLE_ACTUATION=1`을 넘기므로 실제 차량 구동이 활성화될 수 있다. 앞 단계가 실패하면 이번 클릭에서 시작한 보드만 역순으로 중지한다. ‘이번 실행 중지’는 실행 ID가 일치하는 프로세스만 역순으로 종료한다. 실행 중에는 GUI 창 닫기를 막는다. SSH는 `BatchMode=yes`로 호출하므로 공개키 인증이 필요하다. 기존 수동 실행은 GUI가 소유하지 않는다.

현재 PC 설정은 같은 네트워크의 `user-desktop.local`(Pi 5), `ubuntu.local`(Jetson), `lpp.local`(Pi 4)을 사용한다. 세 보드 모두 GUI 전용 SSH 키로 `status=STOPPED` 조회를 확인했다. 버튼과 GUI 표시를 확인했으며, 세 보드 통합 시작은 아직 시험하지 않았다.

Jetson 현재 통합 배포본은 OAK를 여는 `semantic_perception_rgbd.pyz` 하나에서 `/oak/rgb/image_raw`(640×480 `bgr8`), `/perception/semantic/input_image`(640×360 `bgr8`), 의미 마스크, 깊이, 양안 영상과 IMU를 발행한다. 별도 `oak_rgb_publisher.pyz`는 GUI 시작 버튼에서 실행하지 않는다. GUI는 주 토픽이 없으면 추론 영상을 표시한다. 실제 라이브 지도·영상 동시 수신은 아직 검증하지 않았다.
