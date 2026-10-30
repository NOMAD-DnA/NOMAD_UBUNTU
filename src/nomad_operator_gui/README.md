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

`~/.config/nomad/boards.env`에 `NOMAD_PI5_HOST`, `NOMAD_PI5_SCRIPT`, `NOMAD_PI4_HOST`, `NOMAD_PI4_SCRIPT`, `NOMAD_SSH_KEY`를 설정한다. 스크립트 경로는 **각 보드에서의 절대 경로**다. GUI는 이 파일을 실행할 때 읽는다. Jetson 주소·계정·스크립트 경로는 PC 설정의 환경변수 `NOMAD_JETSON_HOST`, `NOMAD_JETSON_USER`, `NOMAD_JETSON_SCRIPT`로 지정한다. GUI 화면은 이 내부 연결값을 입력받지 않는다. 현재 Jetson 경로는 `/home/nvidia/nomad_semantic_20261030/scripts/run_jetson_autonomy_local.sh`다. GUI는 실행 ID를 생성해 팀 배포본의 `start --run-id`와 `stop --run-id`를 직접 호출한다.

‘세 보드 자율주행 기능 시작’은 확인 대화상자 후 Pi 5 → Jetson → Pi 4 순으로 호출한다. Jetson의 영상·위치·지도 상태가 `READY`가 될 때까지 최대 130초 기다린 뒤 Pi 4를 시작한다. Pi 4에는 `NOMAD_ENABLE_ACTUATION=1`을 넘기므로 실제 차량 구동이 활성화될 수 있다. 앞 단계가 실패하면 이번 클릭에서 시작한 보드만 역순으로 중지한다. ‘이번 실행 중지’는 실행 ID가 일치하는 프로세스만 역순으로 종료한다. 실행 중에는 GUI 창 닫기를 막는다. SSH는 `BatchMode=yes`로 호출하므로 공개키 인증이 필요하다. 기존 수동 실행은 GUI가 소유하지 않는다.

현재 PC 설정은 같은 네트워크의 `user-desktop.local`(Pi 5), `ubuntu.local`(Jetson), `lpp.local`(Pi 4)을 사용한다. 기본 SSH 공개키 인증을 사용한다. 최신 배포는 세 보드 모두 로컬 Humble 실행 경로를 사용한다. 2026-10-31 GUI 통합 시작은 실제 실행했으나 Jetson의 양안 입력 최신성 검사에서 시간 초과되어 전체 루프를 완료하지 못했다.

Jetson 현재 통합 배포본은 OAK를 여는 `semantic_perception_rgbd_local.pyz` 하나에서 `/oak/rgb/image_raw`(640×480 `bgr8`), `/perception/semantic/input_image`(640×360 `bgr8`), 의미 마스크, 깊이, 양안 영상과 IMU를 발행한다. 별도 `oak_rgb_publisher.pyz`는 GUI 시작 버튼에서 실행하지 않는다. GUI는 주 토픽이 없으면 추론 영상을 표시한다. 실제 라이브 지도·영상 동시 수신은 아직 검증하지 않았다.

## 시도별 로그 기록

기본 기록은 꺼져 있다. GUI의 **새 시도 로그 기록**에서 기록할 파트를 선택하면
`src/nomad_operator_gui/logs/YYYYMMDD_HHMMSS_마이크로초_ID/`를 새로 만든다.
같은 초에 다시 시작해도 이전 기록을 덮어쓰지 않는다. 기록 버튼 한 번이 하나의 시도이며,
각 주행 시도 전에 눌러 기록한다. 목표 버튼마다 자동으로 새 폴더를 만들지는 않는다.

- `events.jsonl`: UTC 시각, 파트, 이벤트 종류, 상세 데이터가 있는 한 줄 JSON 기록.
- `manifest.json`: 시도 ID, 시작/종료 시각, 선택 파트, ROS domain/토픽, 보드 실행 ID,
  기록 수, 종료 이유. `recording`으로 남아 있으면 정상 종료가 확인되지 않은 기록이다.
- 기록 대상: GUI 보드 실행 진행/SSH 응답/실패/중지, 목표 요청·발행, 영상·지도·목표
  구독 연결 상태 변화, 같은 domain에서 실제 수신한 `/rosout` 메시지.
- ROS 로그는 노드 이름으로 위치추정/인지/플래닝/제어/시뮬레이션을 분류하고,
  분류되지 않은 노드는 실행 관리로 기록한다. GUI 화면 표시와 저장 파트 선택은 독립이다.
- 영상·지도 원본이나 ROS bag은 저장하지 않는다. 원격 프로그램의 전체 stdout 파일을
  내려받는 기능은 아니며, `/rosout`에 없는 native SDK 출력은 SSH 응답에 포함된 경우만 기록된다.
- **기록 종료**, 이번 보드 실행 전체 중지, 시작 실패 후 모든 보드 정리 완료, GUI 정상 종료에
  flush와 종료 메타데이터 저장을 수행한다. 보드가 남아 있으면 계속 기록한다.
  1초마다 buffered 파일을 flush한다. 디스크 오류는 GUI에 표시하며 보드 제어와 독립이다.
- **로그 폴더 열기**로 기록을 볼 수 있다. 로그는 Git에서 제외한다.

권장 순서: 기록 시작 → 보드 시작 → 목표 전송 → 이번 실행 중지. 기록 버튼 자체는
보드를 실행하거나 모터 명령을 보내지 않는다. 설치본을 직접 실행하는 경우
`NOMAD_OPERATOR_GUI_LOG_DIR`로 저장 위치를 지정할 수 있으며 wrapper는 GUI 소스 폴더로 지정한다.


## 2026-10-31 로컬 실행 최종 설정

`config/boards.env.example`을 PC의 `~/.config/nomad/boards.env`로 복사한 뒤 실제 호스트 이름과 설치 경로를 맞춘다. 암호는 이 파일에 넣지 않는다. Pi 5는 `run_pi5_autonomy_local.sh`, Jetson은 `run_jetson_autonomy_local.sh`, Pi 4는 `run_pi4_autonomy.sh`를 호출한다. 이 예시는 PC의 SSH 실행 설정이며 보드의 ROS 설치·배포를 대신하지 않는다.

현재 보드 배포는 의미분할 목표 상한 2Hz, 양안 입력 10Hz다. Pi 5의 `~/nomad_gnss_pi5/config/localization.env`에서 `NOMAD_PI5_LOCALIZATION_MODE=vio_only`를 선택한 실내 시험 모드이며, 첫 유효 VIO 위치를 로컬 원점으로 사용한다. GNSS 융합으로 복귀하려면 보드 설정을 `gnss_vio`로 바꾸고 실행을 재시작한다. 모드는 GUI가 임의로 전환하지 않는다. 새 실행에서는 목표를 다시 선택한다.

최종 적용 내역과 성공/미해결 로그는 [실내 VIO GUI 실행 기록](../../docs/(최종최종)NOMAD_실내_VIO_GUI_실행_20261031.md)을 참고한다. 의미분할 마지막 누적 처리율은 약 1.49Hz이며, 지도·플래닝까지 전체 루프 완료를 확인한 상태는 아니다.
