# NOMAD 공통 메시지 명세

> 계약 버전 1 · 2026-10-02 · 정의 파일은 [msg/](msg/)에 있다.

메시지 타입은 팀 경계에서 고정한다. **토픽명은 topics.yaml로 바꾸고, 타입이 다르면 어댑터로 변환한다.** 런타임에 타입 문자열만 교체하는 기능은 제공하지 않는다. `nomad_interfaces`는 Python/C++ 양쪽에서 쓸 ROS 2 인터페이스를 빌드한다.

## 기본 흐름

| 발행 → 구독 | 메시지 | 필수 내용 |
|---|---|---|
| 인지 → Planning | `nav_msgs/msg/OccupancyGrid` × 2 | 전역/지역 통행 비용 지도 |
| VIO → 인지·Planning | `nav_msgs/msg/Odometry` | 차량 자세·속도·측정 시각 |
| 인지/VIO → Planning | `nomad_interfaces/msg/ModuleStatus` | READY 여부·유효기간 |
| Planning → Control | `nomad_interfaces/msg/DriveCommand` | 목표 속도·조향각·만료·정지 |
| Control → Planning | `nomad_interfaces/msg/VehicleState` | 실측 속도·조향각·ready/fault |
| Planning/Control → 모니터링 | `nomad_interfaces/msg/ModuleStatus` | 모듈 상태 |

지도는 Reliable/Transient Local/depth 1. 나머지 모듈 경계 데이터는 Reliable/Volatile이며 DriveCommand depth 1, 나머지 depth 10이다. 외부 노드도 호환되는 QoS로 발행해야 한다. 특히 Best Effort publisher는 Reliable subscriber와 연결되지 않는다.

## 공통 좌표·시간

- 거리 m, 속도 m/s, 각도 rad, 각속도 rad/s. 차량 +X 전방, +Y 왼쪽, +Z 위.
- 기본 전역 기준 `odom`, 차량 기준 `base_link`. 공통 vehicle.yaml의 planning_frame/base_frame으로 설정한다.
- 경로와 VIO pose는 base_link 원점. 현재 차량 중심 기준이며 뒷축보다 0.36m 앞에 있다.
- 명령/피드백의 속도는 차량 종방향, 조향각은 가상 전륜 중심 각도다. 현재 평면 차량 모델에서 종방향 속도는 중심과 뒷축에 공통이고 횡방향 속도는 다를 수 있다.
- stamp는 ROS time이다. 시뮬레이터는 use_sim_time=true와 /clock을 사용한다. 센서 측정 시각과 메시지 조립 시각의 차이를 각 메시지에서 지킨다.
- `valid_for`는 header.stamp로부터의 유효기간이다. 수신 시각부터 새로 세지 않는다. ROS 시계가 멈춘 상황을 위해 wall-clock timeout도 별도로 적용한다.
- `/tf`, `/tf_static`, `/clock`과 ROS parameter/event 인프라는 topics.yaml의 대상이 아니다.

## DriveCommand.msg — Planning → Control

```text
std_msgs/Header header
float64 target_speed
float64 target_steering_angle
builtin_interfaces/Duration valid_for
bool stop_requested
```

| 필드 | 정의 |
|---|---|
| header | 생성 ROS 시각, frame_id=base_link |
| target_speed | 뒷축 중심 종방향 m/s; 양수 전진, 음수 후진 |
| target_steering_angle | 가상 전륜 중심 조향각 rad; 양수 왼쪽, 후진 시에도 같은 부호 |
| valid_for | 기본 0.3초; 기본 Control은 0초 초과·0.5초 이하만 허용 |
| stop_requested | true이면 속도·조향 목표보다 우선하는 정지 요청 |

기본 Planning은 wall 10Hz로 발행한다. Control은 유한값·frame·시각·한계를 확인하고 만료되면 정지한다. 같은 timestamp의 이동 명령을 다시 보내 lease를 갱신하면 안 된다. 같은 timestamp의 정지는 이동을 덮어쓸 수 있다. 기본 Control은 wall 0.35초 무수신에도 정지한다. 이 메시지는 PWM이나 yaw rate가 아니다.

```yaml
header: {stamp: {sec: 10, nanosec: 0}, frame_id: base_link}
target_speed: -0.2
target_steering_angle: 0.15
valid_for: {sec: 0, nanosec: 300000000}
stop_requested: false
```

위 stamp는 설명용이며 그대로 현재 시스템에 발행하면 오래된 명령으로 거부될 수 있다. 실시간 발행자는 현재 ROS 시각을 넣어야 한다.

## VehicleState.msg — Control → Planning

```text
std_msgs/Header header
float64 speed
float64 steering_angle
bool speed_valid
bool steering_valid
bool ready
bool fault
string detail
```

header는 **상태 조립 시각**, frame_id=base_link다. speed/steering_angle은 실측이며 단위·부호는 DriveCommand와 같다. valid는 해당 측정이 최신이고 수치가 정상이라는 의미다. ready는 실행 준비, fault는 고장/복구 필요 상태다. valid=false인 숫자는 유효 측정으로 사용하지 않는다.

기본 Control은 wall 20Hz로 발행하며 내부 측정의 나이를 0.6 ROS초/3 wall초로 제한한다. Planning은 조립된 피드백 메시지도 0.6 ROS초/0.6 wall초 이내인지 다시 검사한다. 외부 Control도 내부 측정을 검증해야 한다. 새 header만 붙인 오래된 측정이나 명령 echo는 계약 위반이다.

```yaml
header: {stamp: {sec: 10, nanosec: 0}, frame_id: base_link}
speed: -0.18
steering_angle: 0.14
speed_valid: true
steering_valid: true
ready: true
fault: false
detail: "measured wheel speed and steering"
```

## ModuleStatus.msg — 공통 상태

```text
uint8 WAITING=0
uint8 READY=1
uint8 STOPPED=2
uint8 FAULT=3
std_msgs/Header header
uint8 state
builtin_interfaces/Duration valid_for
string detail
```

| 상태 | 의미 |
|---|---|
| WAITING | 초기화 중이거나 유효 데이터 부족 |
| READY | 해당 모듈 출력이 자신의 계약을 충족함 |
| STOPPED | 계획 실행이 정지/보류 중 |
| FAULT | 재초기화·복구가 필요한 오류 |

인지/VIO/Planning frame은 planning_frame, Control frame은 base_frame이다. 기본 lease는 0.6초다. Planning은 인지와 VIO의 READY가 모두 최신이어야 실행한다. Control의 ModuleStatus는 진단용이며 Planning의 실행 허용은 VehicleState로 판단한다. Planning READY는 완전한 차량 안전 인증을 뜻하지 않는다.

## PlannedMotion.msg — Planning 내부

```text
nav_msgs/Path path
float64 target_speed
float64 target_steering_angle
bool active
```

LPP/후진 복구에서 선택한 궤적과 생성에 사용한 명령을 한 메시지로 보낸다. `path.header`는 생성 시각/전역 좌표계, 각 pose는 base_link 자세다. `active`는 복구의 실행 소유권이다. 복구 active=true + 빈/한 점 경로는 정지 유지, active=false는 일반 LPP에 반환이다. Control은 이 메시지를 구독하지 않는다.

## 타입을 변경해야 할 때

기존 팀 모듈이 다른 타입을 쓰면 공통 계약 앞에 얇은 어댑터를 추가한다. 예를 들어 AckermannDriveStamped의 speed와 steering_angle을 DriveCommand로 옮길 수 있지만, 유효기간·정지 의미와 측정 피드백은 별도로 정의해 보완해야 한다. 필드 이름만 비슷하다는 이유로 단위나 frame을 추측해서 연결하지 않는다.

실제 필드는 다음 명령으로 확인한다.

```bash
source ~/nomad_ws/.nomad/env.sh
ros2 interface show nomad_interfaces/msg/DriveCommand
ros2 interface show nomad_interfaces/msg/VehicleState
ros2 interface show nomad_interfaces/msg/ModuleStatus
```


## GnssQuality.msg — GNSS 수신 품질

NavSatFix와 동일한 측정 stamp/안테나 frame으로 위치와 연결한다. solution_type은 UNKNOWN/NO_FIX/SINGLE/DGNSS/RTK_FLOAT/RTK_FIXED. satellites_used는 사용 위성 수, pdop는 무차원, correction_age_s는 수신기 보고 보정 나이(로컬 RTCM 수신 나이가 아님). valid_fields에 해당 비트가 없는 값은 사용하지 않는다. 미제공 pdop/보정 나이는 NaN, 위성수는 유효비트로 구분한다. 위치 공분산은 NavSatFix ENU m²에 둔다.

F9P 실물에서는 NAV-PVT의 fixType/gnssFixOK/diffSoln/carrSoln, numSV/pDOP 및 NMEA GGA/GNS diffAge를 정규화한다. 제품 펌웨어·드라이버 지원과 출력 활성화 확인이 필요하다. 시뮬레이션 품질은 설정한 시험값이며 위성/RTK 계산 결과가 아니다. 실제 주입 오차나 이벤트 이름은 품질 메시지에 넣지 않는다.


## Opt-in multi-factor GPP experiment (2026-10-19)

`/nomad/planning_cost_layers` (`nomad_interfaces/msg/PlanningCostLayers`) is an
atomic global snapshot: the embedded OccupancyGrid provides frame, timestamp,
resolution, origin and hard blocking (>=80). Row-major float arrays contain
`slope_cost=min(slope_deg/20,1)^2` and semantic vote-weighted `terrain_cost/79`.
Both are normalized [0,1]; NaN means not measured. Values use global observation
memory. A laser free ray or bootstrap assumption does not invent terrain/slope.
QoS: reliable, transient-local, depth 1. Bundled geometry avoids cross-topic races.

`gpp.cost_mode=multi` uses `sum ds*(1+wO*pO+wS*pS+wT*pT)` and ignores the old
blended soft occupancy costs, retaining >=80 blocking. The proximity feature is
`max(0,1-d/r)^2`, from confirmed 100-cell centres, not body clearance. Missing
features use configurable prior 1; unknown occupancy additionally costs a fixed
4.5 surcharge. These priors are assumptions, not observations. Default remains
`legacy`. The experiment freezes radius 2 m, length weight 1 and these priors;
searches obstacle/slope/terrain weights. There is no density/cluster cost.

Multi-mode D* uses the existing bounded strategy adapter; it does not use the
legacy node's A* fallback. This execution difference must be reported separately
from cost improvements. LPP, vehicle limits and perception blocking are unchanged.

### Camera observation support (opt-in planning consumer)
`/nomad/camera_observation_support`: nav_msgs/OccupancyGrid, odom, identical geometry/capture stamp to PlanningCostLayers, transient-local depth1. Values0..100 describe recent repeated direct RGBD cell sampling, NOT occupancy/free-volume probability; zero means no sufficient recent support. Computed over2sim seconds with3capture/.5sec span saturation and2sec age decay. Planning requires matching snapshots when enabled; original occupancy remains authoritative.

## LppProgress.msg — 선택적 LPP → GPP 우회 협력 (2026-10-21)
`/nomad/planning/lpp_progress`, Reliable/Volatile/depth1, 권장10Hz.
`header`는 상태 ROS시각/planning_frame, `reference_stamp`는 참조 전역 Path의 원본stamp,
`state`는 TRACKING=0/AVOIDING=1/BLOCKED=2, `request_replan`은 재계획 요청,
`rejoin_index`는 고정된 전방 합류 인덱스, `motion`은 local_motion에도 보내는 동일한 PlannedMotion이다.
기존 타입은 변경하지 않으며 Control은 이 메시지를 구독하지 않는다.
0.5 ROS초/1 wall초 상태 lease, 입력 검증 실패 시 GPP 유지 불가. 같은 stamp 재전송은 lease 갱신 불가.
현재 observed_corridor GPP만 소비하며 새 LPP의 상태 발행이 있어야 동작한다.
자세한 계약: `nomad_path_planning/docs/LPP_GPP_우회협력.md`.
