# Pi4 C++ planning

`feat/plan`의 **Hybrid A* + RPP** 구성에 해당하는 새 C++17 패키지다. 기존 Python·위치추정·인지·제어기를 수정하거나 자동 실행하지 않는다. 기본 설정은 **경로 확인만 수행하고 DriveCommand는 STOP**을 보낸다.

## Pi4 실행

```bash
~/nomad_cpp_ws/src/nomad_planning_cpp/scripts/run_planning.sh
```

ROS domain 기본값은 59다. 기존 지도·위치와 READY 상태가 들어오면 목표점에 대해 경로를 계산한다. 현재 profile은 별도 카메라 관측 지원 토픽을 구독하거나 기다리지 않는다.

```bash
source /opt/ros/humble/setup.bash
source ~/nomad_cpp_ws/install/setup.bash
export ROS_DOMAIN_ID=59
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
  '{header: {stamp: now, frame_id: odom}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
```

위 좌표는 입력 형식 예시다. 실제 목표는 GUI에서 현재 지도에 찍는다. 모든 좌표는 `odom` 기준이다.

```bash
~/nomad_cpp_ws/src/nomad_planning_cpp/scripts/stop_planning.sh
```

종료 스크립트는 본 패키지 프로세스만 종료한다. 센서와 기존 제어기는 유지한다.

## 입출력

| 토픽 | 타입 | 의미 |
|---|---|---|
| `/goal_pose` | PoseStamped | 최종 목표, odom·신선한 ROS 시각 |
| `/nomad/localization/odometry` | Odometry | odom/base_link 현재 위치·방향 |
| `/nomad/localization/status` | ModuleStatus | 위치 유효성 |
| `/nomad/perception/status` | ModuleStatus | 인지·지도 유효성 |
| `/nomad/planning_cost_layers` | PlanningCostLayers | 코스트맵·경사비용·지면비용 |
| `/nomad/navigation_local_costmap` | OccupancyGrid | 지역 충돌·미관측 검사 |
| `/nomad/camera_observation_support` | OccupancyGrid | 현재 profile은 미구독. 별도 camera guard를 켤 때만 global grid와 시각·형상을 맞춰 사용 |
| `/nomad/control/vehicle_state` | VehicleState | 실제 속도·조향 피드백. 명령 echo는 유효한 측정 아님 |
| `/nomad/global_path`, `/nomad/local_path` | Path | 전역 참조·RPP 예측 경로; 원본 위치 시각 보존 |
| `/nomad/goal` | PoseStamped | 수락한 최종 목표 |
| `/nomad/planning/status` | ModuleStatus | 경로 계산/대기·출력 차단 이유 |
| `/nomad/planning/drive_command` | DriveCommand | 기본값 STOP, base_link·m/s·등가조향 rad |

grid의 `-1`은 미관측, `>=80`은 차단이다. 부가 비용은 0~1이며 NaN은 근거 부족이다. 미관측·결측을 무료 통과 영역으로 만들지 않는다. 알고리즘 코어는 원점 회전을 지원하지만, 현재 관측·중간목표 정책은 축정렬 격자만 수용한다. 회전된 격자는 STOP으로 처리한다.

2026-11-09 사용자 선택으로 두 실행 프로필의 `rpp.unknown_speed`를 `0.10`m/s로 설정했다. 차량 기준점의 **로컬 지도 셀이 미관측일 때만** 종전 비용 기반 감속값(출력 프로필에서 `0.25/4.5 ≈ 0.0556`m/s)을 이 상한으로 대체한다. 최소속도 강제가 아니므로 목표 접근·곡률·관측 제동거리 제한은 더 낮은 속도나 STOP을 유지한다. 알려진 셀의 비용 감속, 장애물 임계값 80, 풋프린트, GPP 비용/탐색 및 제어기의 출발 보조는 변경하지 않는다. 코어/ROS 기본값 `0`은 기존 비용 기반 감속을 보존하며, 양수 값은 `forward_speed` 이하여야 한다. 이 값은 명목 명령값이며 실차 속도 보정 결과가 아니다.

현재 설정은 기존 코스트맵을 판단 기준으로 사용한다. `corridor.observation_guard=false`, `corridor.observation_turn_weight=0`으로 별도의 카메라 support 제약만 제거했다. `observed_guard.enabled=true`는 유지하므로 지도상 미관측 구간의 known-prefix, 차량 외곽 예측 충돌·제동거리 검사는 계속 적용한다. 시작 전방 영역의 초기 주행가능 처리는 지도 발행단이 기존 맵에 반영하며, 플래너에서 새 맵이나 마스크를 만들지 않는다.

## 설정

`~/nomad_cpp_ws/src/nomad_planning_cpp/config/planning_real.yaml`

위 경로는 편집용 원본이다. 실행 스크립트는 `~/nomad_cpp_ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config/planning_real.yaml`을 읽으므로 원본 수정 후 패키지를 다시 빌드해야 반영된다.

현재 크기·축간거리·차체 기준점은 기존 시뮬레이션의 값이며 **실차 측정값이 아니다**. `geometry_confirmed: false`, `commissioning_confirmed: false`, `plan_only: true`를 유지한다. 실제 차량 제어 연결은 실차 계수·실측 피드백을 확인하고 별도로 승인받아야 한다. 명시 arm 서비스도 기본 profile에서는 거부된다. 프로세스 재기동·목표 취소·시각 역행 시 자동 arm하지 않는다.

상태·센서 시각이 오래됐거나 프레임·격자가 어긋나면 STOP/빈 경로를 발행한다. 새 지도는 재계산 전까지 이전 주행 명령에 사용하지 않는다. 탐색은 별도 worker에서 수행하고 독립 타이머가 정지를 유지한다. 무입력 대기는 낮은 주기로 처리한다.

Upstream의 탐색 상한 20초/200000회 대신 보드 preview profile은 0.5초/50000회, 중간 목표 탐색 0.25초/20000회로 제한한다. 결과가 없으면 대기하며 직선 경로로 대체하지 않는다. 시간 제한으로 찾지 못한 것을 경로 부재의 증명으로 표현하지 않는다.

## GUI용 SSH 스트림

```bash
~/nomad_cpp_ws/src/nomad_planning_cpp/scripts/run_planning.sh --stdio
```

stdin은 JSON 한 줄씩 받는다.

```json
{"cmd":"goal","frame":"odom","x":2.0,"y":0.0,"yaw":0.0}
{"cmd":"cancel"}
{"cmd":"status"}
{"cmd":"quit"}
```

`goal`은 도착 시각을 ROS stamp로 사용한다. stdout은 `type=ack` 또는 `type=planning` JSON이다. planning에는 `t`(발행시각), `stamp_ns`(원본 위치 측정시각), `frame`, `state`, `detail`, `goal`, `path`, `local_path`, `target`, 후보 속도·조향 및 차단 상태가 있다. 경로 배열 원소는 `[x,y,yaw]`다. 입력 종료(EOF)는 목표 취소·disarm 후 노드를 종료한다. 별도 GUI가 기존 운영자의 planner를 중복 시작하면 flock가 거절한다. stdio로 arm은 제공하지 않는다.

## 빌드

Pi4에서 공식 프로젝트 `nomad_interfaces` 전체가 필요하다. 과거 제어기용 DriveCommand/VehicleState만 있는 설치로는 ModuleStatus/PlanningCostLayers가 없어 빌드되지 않는다.

```bash
source /opt/ros/humble/setup.bash
cd ~/nomad_cpp_ws
colcon build --packages-select nomad_interfaces nomad_planning_cpp \
  --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
```

ROS 없이 알고리즘 검증만 하는 경우:

```bash
cmake -S . -B /tmp/nomad_planning_core -DNOMAD_BUILD_ROS=OFF -DBUILD_TESTING=ON
cmake --build /tmp/nomad_planning_core -j2
ctest --test-dir /tmp/nomad_planning_core --output-on-failure
```

이 문서의 실행 명령은 사용 방법이며, 그 자체가 실차 주행 성공 기록은 아니다. 실제 빌드·합성/실수신 결과는 프로젝트 작업 로그에 따로 남긴다.
# LPP pause diagnostics (2026-11-09)

During an active goal, `[LPP_PAUSE]` records the exact candidate failure reason,
GPP/LPP state, path validity, result timestamp, odometry/local-map ages and path
point counts. Identical reasons are suppressed until the reason/state/goal
changes or a candidate recovers. `[LPP_CANDIDATE_RECOVERED]` is candidate recovery,
not actuator permission. Startup without a goal, cancellation and arrival do not
produce failure records. Arm/disarm policy and planning algorithms are unchanged.

The timer emits these records through the ROS logger even without GUI stdio.
`scripts/run_planning_output.sh` already redirects stderr to
`log/planning_output_<date>_<pid>/planner.log`; GUI and existing read-only log
collectors can inspect the same file. Historical missing reasons cannot be
reconstructed by this change.

## 인지 상태 정지 원인 로그 (2026-11-10)

`PERCEPTION_NOT_READY`인 `[LPP_PAUSE]`에는 `perception_status={...}` JSON을
추가한다. `failure.category`로 세 경우를 구분한다.

| category | 의미 |
|---|---|
| `MISSING` | 상태 메시지 미수신 (`NO_MESSAGE`) |
| `STATE_NOT_READY` | 상태가 READY가 아님. `state`, `state_name`, `detail` 확인 |
| `TIMING_INVALID` | 수신 지연·원본 시각 만료·유효기간 만료 또는 잘못된 시각/유효기간 |

`failure.cause`는 기존 검사 순서에서 먼저 실패한 조건이며, `failures`에는
동시에 실패한 조건을 모두 `|`로 연결해 남긴다. 수신 지연은 `RECEIVE_TIMEOUT`,
원본 시각 만료는 `SOURCE_TIMEOUT`, 메시지 유효기간 만료는 `LEASE_EXPIRED`다.
`STAMP_INVALID`, `STAMP_IN_FUTURE`, `VALID_FOR_INVALID`, `RECEIVE_TIME_INVALID`도
따로 기록한다. 상태와 시간이 함께 잘못되면 주원인만으로 다른 실패를 배제하지 않는다.

원본 `stamp_ns`, 판정 시각 `evaluated_ros`, 수신 후 경과 `receive_age_s`,
원본 시각부터 경과 `source_age_s`, `valid_for_s`, 적용된 `timeout_s`와
`future_tolerance_s`, 인지 모듈의 `detail`을 기록한다. detail은 JSON 이스케이프하고
512바이트까지만 표시하며 잘렸으면 `detail_truncated=true`다. 미수신의 age는 -1이다.

`origin=current_input`은 현재 타이머 입력 검사에서 실패한 경우다.
`origin=worker_result`는 현재 상태는 회복했지만 이전 worker의 실패 결과가 남은 경우다.
이때 `failure`는 실패 당시 증거를 보존하며 `current`는 현재 상태를 별도로 보여준다.
시각이 계속 바뀐다는 이유로 반복 기록하지 않고 실패 조건·상태·origin이 달라질 때
새 줄을 기록한다. 따라서 로그 줄 수는 제어기의 실제 정지 전환 횟수와 다르다.

기존 `PERCEPTION_NOT_READY` 문자열, ModuleStatus/DriveCommand, STOP 조건,
유효기간·주기·QoS·arm 및 경로 계산은 변경하지 않는다. 다음 실행의 기존
`planner.log`에 기록되며 실행 중인 프로세스는 자동 재시작하지 않는다.
기존 로그에는 이 증거가 없어 세 경우의 과거 빈도를 복원할 수 없다.

`status_diagnostics_test`는 세 범주·세부 원인·경계값·동시 실패·worker 증거 보존·
중복 로그 억제를 ROS 없이 검사한다. `tests/test_status_diagnostics_ros.py`는
격리된 localhost ROS domain에서 `plan_only=true`인 별도 플래너에 합성 입력을 넣고
실제 로그와 STOP 출력을 검증한다. 차량 제어기·드라이버는 실행하지 않는다.

## LPP 차체 검사 크기 (2026-11-09)

사용자 지정 검사 영역은 플래닝 기준점에서 **앞 0.05m·뒤 0.35m·좌우 각 0.10m**이다. `observed_guard.half_length=0.20`, `half_width=0.10`, `center_offset=-0.15`으로 길이 0.40m × 폭 0.20m 사각형을 전방축 반대 방향으로 0.15m 이동한다. 현재 preview 기준점은 OAK IMU와 일치하며, 위치추정/TF/차량 운동학 기준점을 옮기지 않는다. 관측거리·최종 궤적 충돌 검사·고정 초기 footprint 예외에 같은 오프셋을 적용하고 회전 시에도 기준점과 함께 회전한다. margin은 기존 0.10m를 유지한다. 두 실행 YAML, ROS 노드 기본값, C++ 계산층 기본값에 동일하게 적용하며, 시작 로그의 `LPP footprint`에서 앞뒤좌우 실제 적용값을 확인할 수 있다. 계산 helper의 선택적 마지막 인자 `center_offset`은 기본 0으로 기존 대칭 호출을 보존한다. 이 변경은 실측 보정 완료를 의미하지 않는다.


## 2026-11-10 출력 실행의 위치 자동 기록

RPM / VIO-duty 출력 런처는 해당 실행 폴더를 `NOMAD_PLANNING_LOG_DIR`로 플래너에 전달한다.
`planner.log`, `controller.log`, `driver.log`와 같은 폴더의 `telemetry.jsonl`에 위치를 자동 기록한다.
PC 분석 GUI는 실행 로그를 가져올 때 이 파일을 함께 복사한다. PC 연결/기록 버튼은 필요 없다.
일반 preview 런처와 직접 노드 실행은 기존처럼 자동 파일을 만들지 않으며,
명시적 시험에서는 `trajectory_log_file` ROS 파라미터로 저장할 새 파일 경로를 지정할 수 있다.

별도 스레드가 최신 수신 Odometry를 최대 10Hz 샘플링한다. 동일 메시지 객체를 반복 기록하지 않는다.
위치는 원본 x/y/z이며 정수 `t_ns`는 메시지 촬영/추정 시각, `sample_ns`는 기록 샘플 시각이다.
프레임과 원본 quaternion을 보존하고 yaw는 기존 planning_pose의 표시 방향을 사용한다.
입력이 없거나 유효한 위치/자세를 만들 수 없으면 좌표를 만들어 채우지 않는다.
이는 위치추정 궤적이며 실제 차체 정지의 독립적 측정이 아니다. GPP 경로/지도 자동 저장은 이 변경에 포함하지 않는다.

파일 I/O·직렬화는 제어 타이머 밖에서 수행하며 별도 이력 큐가 없어 저장 지연이 메모리 누적으로 이어지지 않는다.
기존 파일/심볼릭 링크를 덮어쓰지 않고 1초마다 fdatasync한다. 정상 종료에는 closed 레코드와 최종 sync를 남긴다.
강제 종료·전원 단절 때 마지막 기록의 손실 가능성은 남는다. 실행당 64MiB 한도 또는 저장 실패 시
`[TRAJECTORY_LOG_ERROR]`를 남기고 좌표 기록만 중단한다. 플래닝·출력 허용·정지 판정은 변경하지 않는다.
PC 분석 GUI는 이 오류를 경고하며 JSONL 마지막 줄이 불완전하면 그 줄을 제외한다.


## 2026-11-11 corner clearance profile

`planning_output.yaml` opts into `footprint_planning` and
`corridor.corner_aware`. Other profiles retain their previous defaults.
The existing obstacle weight 16 / radius 4 m and cluster term remain unchanged.
`cost.clearance_weight=24` / `cost.clearance_radius=0.45` adds a soft near-wall
cost for all blocked cells (>=80), including adjacent unknown transit cells.
It does not relabel unknown cells, inflate the blocking threshold, or make a
narrow but passable corridor impassable. Distance is a conservative distance
from the planning reference to the occupied cell square, not body clearance.

Waypoint ranking balances continuous goal bearing, progress and preferred
clearance (`corridor.preferred_clearance=0.45`, weight 4), replacing the dominant
10-degree bearing bands in this opt-in profile. Hybrid A* and retained routes
check the execution footprint at interpolated poses, including rotation and
rear offset. A candidate must also admit a footprint-safe initial RPP command;
RPP tries alternate lookaheads before failing. Persistent motion/observation
rejection or latest-map execution rejection requests reselection after 1 s.
Feedback is scoped to the current goal/session; safe paths retain their existing
publication behavior. STOP, freshness, arm intent and fixed initial map-edge
exceptions remain enforced.

The configured guard stays at half_length 0.10, half_width 0.05, offset -0.075 m.
This is the existing user-selected guard, not a measured full vehicle envelope.
The soft 0.45 m preference is separate from that geometry. Physical success
still requires a real course trial and the existing calibration constraints.

`corner_regressions_test` covers blocked classes, unknown transit, waypoint
clearance, alternate RPP lookahead, new local obstacles, retention, rejection
feedback, cancel, stale input and initial map-edge departure.
`corner_scenarios` runs the actual Navigator plus rear-steer kinematics and the
execution footprint validator in closed loop at 10 cm and 12.5 cm map resolution.
It covers mirrored right-angle turns, straight/narrow passages, a gentle curve
and open diagonal turns. This is a deterministic model test, not Gazebo physics
or replay of the real sensor logs.

### Ordered RPP lookahead candidates (2026-11-13)

Optional `rpp.lookahead_candidates` is a ROS double array, for example
`[0.8, 1.0, 0.6, 1.2, 1.5]`. A nonempty array is tried in exactly that order;
the first candidate passing the existing steering, collision and braking checks
is returned immediately. It is not a cost ranking. It accepts 1–32 unique finite
distances in [0.1, 3.0] metres. An absent/empty array preserves the existing
`rpp.lookahead` and `rpp.adaptive_lookahead` behavior, including legacy sorting.
The GUI displays that effective order and can add/remove/edit candidates.
Saving applies on the next Jetson planner start, not to a running planner.
No speed, footprint, GPP cost, current saved settings or default candidate
values are changed by adding this optional parameter.
