# NOMAD 실차 제어와 Qt GUI

Pi의 최신 실차 제어 소스와 설정을 포함합니다. `controller_node`가 LPP 명령을 VESC 속도·서보 명령으로 변환하고, 상태 감시와 정지·재출발을 처리합니다. Qt 수동 GUI, 장치 없는 모의 화면, 수신 전용 시험 기록기도 포함합니다.

이 패키지는 기존 Gazebo `/cmd_vel` 경로에 자동 연결되지 않습니다. 실차 제어기는 `use_sim_time=false`를 사용합니다. 기존 `.nomad/build.sh`와 `run_forest.sh`의 시뮬레이터 실행은 변경하지 않았습니다. 실차에서 확인한 환경은 ROS 2 Humble이며, 이 저장소의 Jazzy 시뮬레이션 검증과 구분합니다.

## 인터페이스

| 방향 | 토픽 | 타입·단위 |
|---|---|---|
| LPP → 제어기 | `/nomad/planning/drive_command` | `nomad_interfaces/msg/DriveCommand`: m/s, rad, STOP, stamp, valid_for |
| VESC → 제어기 | `/nomad_bench/sensors/core` | `vesc_msgs/msg/VescStateStamped`: ERPM, A, V, 오류 |
| 제어기 → VESC | `/nomad_bench/commands/motor/speed` | `std_msgs/msg/Float64`, ERPM |
| 제어기 → VESC | `/nomad_bench/commands/servo/position` | `std_msgs/msg/Float64`, 0~1 |
| 제어기 → VESC | `/nomad_bench/commands/motor/current` | `std_msgs/msg/Float64`, 정지 시 0 A |
| 제어기 → LPP | `/nomad/control/vehicle_state` | `nomad_interfaces/msg/VehicleState` |
| 제어기 → GUI·로그 | `/nomad_vesc_trial/status_json` | `std_msgs/msg/String`, JSON 상태 |

`DriveCommand`는 `frame_id=base_link`, 증가하는 ROS stamp, `0 < valid_for <= 0.5초`를 사용합니다. 같은 stamp의 이동 명령으로 유효기간을 연장하지 않습니다. 수신 후 0.35초 이상 갱신되지 않아도 출력을 해제합니다. 현재 실행은 전진·정지만 허용합니다.

## 현재 차량 설정

- 설정 순서: `commissioning.yaml` → `planning.yaml` → `planning_headless.yaml` → `nomad_vehicle_ready.yaml`.
- m/s → ERPM 계수: `3721.882617894008`.
- 양수 자율주행 목표: `max(1000 ERPM, 환산 요청 ERPM)`. 0/STOP은 0 A 출력 해제입니다. 과속 감속 중에는 최소 출력 적용을 해제합니다.
- LPP 입력 상한은 0.25 m/s입니다. 이 상한의 환산값도 1000보다 낮으므로, 현재 유효한 양수 요청은 정상 상태에서 모두 1000 ERPM으로 올라갑니다. 입력 m/s를 그대로 추종하는 설정은 아닙니다.
- 조향 입력 ±0.6108652382 rad(±35°) → 서보 0~1, 중앙 0.5. 실제 바퀴 각도 피드백은 없습니다.
- VESC 내부 속도 PID를 사용합니다. C++은 ERPM 목표와 과속 감시를 담당합니다. 과속 감시 기준은 1200 ERPM이며 50 ERPM 여유를 넘는 초과분에 따라 목표를 낮춥니다.
- 전류 기준(`commissioning.yaml`): **모터 전류 55 A**(`motor_current_limit_a`), **배터리 입력 전류 30 A**(`current_limit_a`), 즉시 잠금 60 A(`overcurrent_hard_a`), 지속 0.3초(`overcurrent_sustain_s`), 반복 10초에 3회(`overcurrent_spike_limit`, `overcurrent_spike_window_s`). 저속 출발은 모터 전류만 크고(30~35 A) 입력 전류는 2~3 A라 두 기준을 나눴습니다.
- 정지는 능동 브레이크·경사로 정차 유지가 아닙니다(0 A 관성 정지).
- `VehicleState.speed_valid`와 `steering_valid`는 현재 false입니다. 값 0을 실측 정지·직진으로 해석하지 마세요. ERPM·전류·전압·오류는 별도로 제공합니다. IMU 경사·전복 보호는 현재 활성화하지 않습니다.

### 정지·잠금 정책 (headless 자율주행)

| 구분 | 경우 | 동작 |
|---|---|---|
| 치명 | VESC 고장 코드, 전압 0 V 이하, 과속 2500 ERPM 초과, 바퀴 걸림(명령 중 50 ERPM 미만 2초), 다른 프로그램의 VESC 명령 토픽 발행, 운영자 STOP, 과전류(모터 55 A·입력 30 A 초과가 0.3초 지속 / 60 A 1회 / 10초에 3회), 시작 전 Idle에서 본 치명 고장 | 0 A 후 **잠김**. RESET(차가 0.5초 이상 정지) 필요 |
| 비치명 | 짧은 전류 피크, 제어 주기 0.2초 이상 지연, ROS 시계 역행(기준 재설정), 값이 숫자가 아닌 상태값 1회, LPP 명령 끊김·만료·STOP, 모터 상태 끊김, 드라이버 일시 사라짐 | 0 A 후 **대기**. 새 유효 입력이 오면 RESET 없이 재출발 |

- 수동 GUI 세션은 예전처럼 모두 잠급니다.
- 잠금·대기 사유는 하나로 묶지 않고 값과 함께 나옵니다. 예: `raw current spike: motor 33.0 A (limit 55), input 2.1 A (limit 30); output released`, `raw overspeed: 2600 ERPM > 2500 ERPM`, `raw VESC fault code 7`.
- `status_json`에 `planning_supervisor_state`(IDLE·ACTIVE·SOFT_STOPPED·RESUME_PENDING·BLOCKED)와 `motor_current_limit_a`, `input_current_limit_a`, `overcurrent_hard_a`가 있습니다.
- 통합 GUI 플래닝 개발 창의 **'제어기 복구'** 버튼(`feat/gui`)이 Pi4 제어기의 `~/reset`만 부릅니다(구동 명령 없음). 출력 허용 상태면 복구 직후 다시 출발합니다.

현재 설정은 이 README와 YAML을 기준으로 확인하세요. 날짜가 붙은 기존 `.ko.txt` 문서는 당시 시험·배포 기록이며 최신 실행 설정과 다를 수 있습니다.

## VESC 설정 (2026-11-12, HFI)

센서리스 VESC가 정지 상태에서 회전자 위치를 못 잡아 책 높이 턱에서 바퀴가 떨기만 했습니다. VESC Tool 단위시험에서 `foc_sensor_mode`를 **HFI**로 바꾸자(Start 8 V, Run 4 V, Max 8 V, Samples 16, Sensorless ERPM HFI 2000) 턱을 넘었습니다. `s_pid_min_erpm`은 900→50으로 바꿨습니다(단독으로는 효과 없음). 전류 한계(±60 A, 배터리 250 A, 회생 −200 A)는 아직 그대로입니다.

변경 전·후 설정 파일, 시험 경과, 되돌리는 방법: [`vesc_config_20261112/README.md`](vesc_config_20261112/README.md). VESC Tool의 모터 설정 마법사와 'Read Default Motor Configuration'은 쓰지 마세요.

## 코드 구조

- `src/controller_main.cpp`: 실행 진입점. Ctrl+C·SIGTERM 때 0 A를 보낸 뒤 종료합니다.
- `src/controller_node.cpp`: ROS 입출력과 자율주행 연결. `nomad_controller_core` 정적 라이브러리로 한 번만 빌드되며, 실행 파일과 ROS 시험이 같은 코드를 씁니다.
- `include/nomad_control_cpp/controller_node.hpp`: `ControllerNode::create()`, `prepareShutdown()`.
- `src/raw_trial.cpp`: 원시 ERPM·서보 상태머신, 상태값 검사(사유 분리), 과전류 규칙.
- `include/nomad_control_cpp/planning_supervisor.hpp`: LPP 출력 소유 상태머신(ROS 없음, 단위시험 `test/test_planning_supervisor.cpp`).
- `include/nomad_control_cpp/control_limits.hpp`: 명령 신선도, 미래 stamp 허용, LPP lease, 1850 ERPM 상한 등 공용 안전 상수.
- `include/nomad_control_cpp/planning_command.hpp`: LPP 명령 검사. ERPM 상한은 `commissioning.motor_max_erpm`을 따릅니다.
- `planning.headless=true`는 `mode=commission`에서만 허용합니다(다른 모드는 시작 거부).

## ROS 빌드와 자율주행 실행

공통 메시지 소스는 이 저장소의 `src/nomad_interfaces`에 있습니다. `vesc_msgs`와 VESC 드라이버는 포함하지 않으므로 팀이 사용하는 VESC 워크스페이스를 먼저 빌드하고 source해야 합니다. 이 패키지의 실행기는 두 번째 드라이버를 시작하지 않습니다.

```bash
source /opt/ros/humble/setup.bash
source ~/vesc_ws/install/setup.bash
cd ~/nomad_ws
colcon build --packages-up-to nomad_control_cpp \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DNOMAD_ENABLE_PLANNING=ON
source install/setup.bash
ros2 launch nomad_control_cpp lpp.launch.xml
```

Pi4 운영 작업공간(`~/nomad_cpp_ws`)은 루트에 예전 백업 폴더가 있으므로 `colcon build --base-paths src --packages-up-to nomad_control_cpp --cmake-args -DNOMAD_ENABLE_PLANNING=ON`으로 빌드합니다. 백업 폴더(`release_backups/`, `commission_session/release_backup_*`)에는 `COLCON_IGNORE`가 있습니다. 배포 때마다 `release_backups/feat_control_<커밋>_pre_<시각>/`에 이전 소스·설치본과 `RESTORE.txt`, `BUILD_NOTE.txt`를 남깁니다.

Jazzy 환경에서는 먼저 해당 ROS 환경을 source하세요. LPP·제어기·드라이버의 ROS domain과 네트워크 설정을 맞춰야 합니다. `nomad_vehicle_ready.yaml`은 기존 NOMAD 차량의 준비 설정으로, 다른 차량의 VESC 설정을 검증하는 도구가 아닙니다.

## Qt GUI

```bash
sudo apt install build-essential cmake qt6-base-dev openssh-client curl util-linux
bash src/nomad_control_cpp/desktop/start-linux.sh --simulate
```

실차 GUI는 `linux/settings.sh`에서 Pi 접속 정보를 설정한 뒤 `desktop/start-linux.sh`로 엽니다. Pi의 기존 `commission_session` 서비스 스크립트, 빌드 확인 파일, VESC 워크스페이스의 `inspect_vesc.py`가 준비되어 있어야 합니다. 저장소 복제만으로 이 별도 Pi 배포 환경이 설치되지는 않습니다. GUI에서 시작할 때까지 모터 구동 명령을 보내지 않으며, headless 자율주행과 수동 조종은 동시에 출력하지 않습니다.

## 시험 기록

제어기 로그에는 LPP 요청·목표·송신 ERPM, 추정 ERPM, 전류, 상태와 정지 사유가 남습니다. 추가 기록기는 ROS 토픽만 구독하며 VESC USB를 열거나 구동 명령을 발행하지 않습니다.

```bash
# 실행 중인 제어기와 같은 domain/localhost 설정을 사용하세요.
NOMAD_WORKSPACE="$PWD" ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 \
  bash src/nomad_control_cpp/scripts/start_recording.sh
```

기록 위치는 `$NOMAD_WORKSPACE/log/diagnostics/`입니다. JSONL·CSV·상태 전환과 기존 설정 파일의 스냅샷을 저장합니다. 설정 스냅샷은 최신 USB 재조회가 아니며, Windows VESC Tool의 직접 USB 명령·설정 변경은 자동 기록되지 않습니다. VESC 설정을 바꾼 시험은 변경 전후 Motor Configuration을 별도로 내보내 보관하세요. GUI의 기존 `/diagnostics/start_recording.sh` 설치와 별도 domain의 통합 시험 기록은 각각 구분합니다.

## 장치 없는 코드 검사

```bash
cmake -S src/nomad_control_cpp -B build-control-offline \
  -DNOMAD_BUILD_ROS2=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-control-offline -j2
ctest --test-dir build-control-offline --output-on-failure
python3 src/nomad_control_cpp/scripts/test_record_control_trials.py
```

이 검사는 실제 VESC·Pi 연동 시험을 대신하지 않습니다.

ROS 환경에서는 `colcon test`로 16개 시험이 돕니다(ROS 통합시험은 약 62초, 제한 150초). 알려진 실패: `rpm_running_floor_ros`의 100 ERPM 경계 기대값 2건은 의도적으로 예전 값을 유지해 실패하고, `controller_ros_integration`의 'stale IMU' 받침대 시험은 타이밍 때문에 가끔 실패합니다(실차 경로 무관).

## 작업 내역 (2026-11-11 ~ 11-12, BAE)

모두 Pi4 `~/nomad_cpp_ws`에 반영·빌드했습니다(설치본 e173e9f, 제어기 실행 파일 sha256 `f03e70a3…`).

| 커밋 | 내용 |
|---|---|
| 4a16650 | headless 플래닝은 commission 모드에서만 허용(다른 모드에서 null 역참조로 죽던 조합 차단) |
| e48d541 | 안전 상수를 `control_limits.hpp`로, LPP guard가 `motor_max_erpm`을 따름 |
| 2cd3580 | 정지 경로를 `stopAndRelease()` 하나로 |
| 2b7b5a5 | 제어 노드를 라이브러리로 한 번만 컴파일 |
| 1ef60a2 | 손댄 `.cpp`에 `using namespace std` |
| 3dbbba3 | LPP 출력 소유를 `PlanningSupervisor` 상태머신으로 |
| 9ffea82 | 시험 include 정리 |
| ea5e603 | 상태값 이상 사유 분리, 치명 고장만 잠금 |
| 95bc3e2 | 리뷰 수정(시계 역행·드라이버 준비·Idle 중 치명 고장·피크 즉시 해제) |
| cc4ef3e | 모터 55 A / 입력 30 A 분리, 즉시 잠금 60 A |
| e173e9f | ROS 시험 기대값을 현재 정책에 맞춤, 시험 제한시간 150초 |
| 8d5a409 | VESC HFI 설정 파일과 정리 문서 |

통합 GUI(`feat/gui` ddfb8c3, a1c24af): 플래닝 창 '제어기 복구' 버튼, 복구 결과 전용 표시, 복구 중 창 닫기 보호.

### 남은 일

1. VESC 전류 한계 정리(권장): 모터 ±50 A(VESC가 먼저 제한), 배터리 30 A(제어기 입력 기준과 일치), 회생 0 A.
2. Pi4 `commission_session/ensure_gui.sh`의 VESC 점검 기대값(30 A·회생 0·PID 50)을 새 VESC 값에 맞추기. 지금은 수동 GUI 경로가 이 점검에서 막힙니다. 자율주행 경로(`run_planning_output.sh`)에는 VESC 설정 점검이 없습니다.
3. 리뷰에서 나온 개선 후보(미수정): 전류 피크 직후 재출발 대기 없음, RESET 때 과전류 기록 미초기화, 과속 감속 명령이 VESC PID 최소값 아래로 내려감, 운영자 STOP 후 복구하면 곧바로 출발.
4. 플래너 쪽 합의: 플래너가 1000 ERPM 하한을 모름, 종료 시 드라이버·제어기 동시 종료, `vesc_commission.yaml` 서보 범위 중복.
5. HFI 적용 후 통합시험(ERPM 모드)에서 턱 넘기·바퀴 걸림 감시·모터 발열 확인. VIO duty 모드는 계속 사용 금지.
