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
- 정상 LPP STOP이나 명령 만료는 출력 해제 후 새 유효 명령을 기다립니다. 장치 상태가 정상이면 GUI·RESET 없이 재개합니다. 운영자 STOP·하드웨어/드라이버/제어 루프 오류는 잠금과 RESET 경로를 유지합니다.
- telemetry 0.30초, 제어 루프 공백 0.20초, 정체 2초 등의 기존 감시가 있습니다. 정지는 능동 브레이크·경사로 정차 유지가 아닙니다.
- `VehicleState.speed_valid`와 `steering_valid`는 현재 false입니다. 값 0을 실측 정지·직진으로 해석하지 마세요. ERPM·전류·전압·오류는 별도로 제공합니다. IMU 경사·전복 보호는 현재 활성화하지 않습니다.

현재 설정은 이 README와 YAML을 기준으로 확인하세요. 날짜가 붙은 기존 `.ko.txt` 문서는 당시 시험·배포 기록이며 최신 실행 설정과 다를 수 있습니다.

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
