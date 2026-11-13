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
| 제어기 → VESC | `/nomad_bench/commands/motor/current` | `std_msgs/msg/Float64`, 정지 시 0 A(정지 브레이크·유지를 끈 경우, 또는 VESC 고장 코드로 잠긴 경우) |
| 제어기 → VESC | `/nomad_bench/commands/motor/brake` | `std_msgs/msg/Float64`, A. 정지 중 핸드브레이크를 걸지 않은 동안(\|ERPM\| > `stop_handbrake_below_erpm`, 상태값 끊김, 유지 포기) |
| 제어기 → VESC | `/nomad_bench/commands/motor/handbrake` | `std_msgs/msg/Float64`, A. 정지 중 정상 상태값이 그 이하일 때 걸고 다음 구동 명령까지 유지. **패치한 VESC 드라이버 필요** |
| 제어기 → LPP | `/nomad/control/vehicle_state` | `nomad_interfaces/msg/VehicleState` |
| 제어기 → GUI·로그 | `/nomad_vesc_trial/status_json` | `std_msgs/msg/String`, JSON 상태 |

`DriveCommand`는 `frame_id=base_link`, 증가하는 ROS stamp, `0 < valid_for <= 0.5초`를 사용합니다. 같은 stamp의 이동 명령으로 유효기간을 연장하지 않습니다. 수신 후 0.35초 이상 갱신되지 않아도 출력을 해제합니다. 현재 실행은 전진·정지만 허용합니다.

## 현재 차량 설정

- 설정 순서: `commissioning.yaml` → `planning.yaml` → `planning_headless.yaml` → `nomad_vehicle_ready.yaml`.
- m/s → ERPM 계수: `3721.882617894008`.
- 양수 자율주행 목표: `max(1000 ERPM, 환산 요청 ERPM)`. LPP STOP·명령 끊김은 0.3초 동안 속도를 200 ERPM까지 줄이고, 측정 속도가 200 이하가 되면(최대 0.6초 더) 핸드브레이크로 넘어갑니다(11-13, 아래 '감속 정지'). 주행 중 속도 0 명령은 감속 없이 정지 출력입니다(아래 '정지 출력'). 과속 감속 중에는 최소 출력 적용을 해제합니다.
- LPP 입력 상한은 0.25 m/s입니다. 이 상한의 환산값도 1000보다 낮으므로, 현재 유효한 양수 요청은 정상 상태에서 모두 1000 ERPM으로 올라갑니다. 입력 m/s를 그대로 추종하는 설정은 아닙니다.
- 조향 입력 ±0.6108652382 rad(±35°) → 서보 0~1, 중앙 0.5. 실제 바퀴 각도 피드백은 없습니다.
- VESC 내부 속도 PID를 사용합니다. C++은 ERPM 목표와 과속 감시를 담당합니다. 과속 감시 기준은 1200 ERPM이며 50 ERPM 여유를 넘는 초과분에 따라 목표를 낮춥니다.
- 전류 기준(`commissioning.yaml`): **모터 전류 55 A**(`motor_current_limit_a`), **배터리 입력 전류 30 A**(`current_limit_a`), 즉시 잠금 60 A(`overcurrent_hard_a`), 지속 0.3초(`overcurrent_sustain_s`), 반복 10초에 3회(`overcurrent_spike_limit`, `overcurrent_spike_window_s`). 저속 출발은 모터 전류만 크고(30~35 A) 입력 전류는 2~3 A라 두 기준을 나눴습니다.
- 정지 출력은 VESC 브레이크 20 A → 거의 멈추면 핸드브레이크 유지입니다(2026-11-12). 유지 전류는 정지 직전 안정 주행의 모터 전류(구동 부하)에 맞춰 12.5~30 A입니다(11-13, 후보값). VESC가 고장 코드를 보내 잠기면 0 A로 풉니다(11-13). 경사 정차가 보장되지는 않습니다. 아래 '정지 출력'을 보세요.
- ESC MOSFET 온도 보호: 65 ℃ 경고, 80 ℃ 이상이면 자율주행 출력 정지, 70 ℃ 미만에서 재출발 허용. 아래 'ESC 온도 보호'를 보세요.
- `VehicleState.speed_valid`와 `steering_valid`는 현재 false입니다. 값 0을 실측 정지·직진으로 해석하지 마세요. ERPM·전류·전압·오류는 별도로 제공합니다. IMU 경사·전복 보호는 현재 활성화하지 않습니다.

### 정지·잠금 정책 (headless 자율주행)

| 구분 | 경우 | 동작 |
|---|---|---|
| 치명 | VESC 고장 코드, 전압 0 V 이하, 과속 2500 ERPM 초과, 바퀴 걸림(명령 중 50 ERPM 미만 2초), 다른 프로그램의 VESC 명령 토픽 발행, 운영자 STOP, 과전류(모터 55 A·입력 30 A 초과가 0.3초 지속 / 60 A 1회 / 10초에 3회), 시작 전 Idle에서 본 치명 고장 | 정지 출력 후 **잠김**(VESC 고장 코드로 잠기면 0 A). RESET(차가 0.5초 이상 정지) 필요. 핸드브레이크 중에는 ERPM이 0으로 고정돼 미끄러져도 이 조건을 통과하므로 복구 전 눈으로 정지를 확인 |
| 비치명 | 짧은 전류 피크, 제어 주기 0.2초 이상 지연, ROS 시계 역행(기준 재설정), 값이 숫자가 아닌 상태값 1회, LPP 명령 끊김·만료·STOP, 모터 상태 끊김, 드라이버 일시 사라짐, ESC 80 ℃ 이상(70 ℃ 미만까지) | 정지 출력 후 **대기**. 새 유효 입력이 오면 RESET 없이 재출발. LPP STOP과 명령 끊김·만료(플래너 발행자 1개 아님 포함)만 감속 뒤 정지 출력(아래 '감속 정지'), 나머지는 즉시 |

- 수동 GUI 세션은 예전처럼 모두 잠급니다.
- 잠금·대기 사유는 하나로 묶지 않고 값과 함께 나옵니다. 예: `raw current spike: motor 33.0 A (limit 55), input 2.1 A (limit 30); output released`, `raw overspeed: 2600 ERPM > 2500 ERPM`, `raw VESC fault code 7`.
- 지금 정지 출력 상태: `status_json`의 `stop_output`(`none` 주행 중이거나 아직 없음 · `coast` 0 A · `brake` · `hold` 핸드브레이크), `stop_output_a`(보내는 전류), `stop_hold_s`(유지 시간), `erpm_valid`(핸드브레이크 중 false: 이때 ERPM 0은 측정값이 아님), `vesc_fault`(VESC 고장 코드로 잠긴 상태, 출력권이 있으면 0 A로 풂. 0 A 해제가 시작될 때 `[STOP_OUTPUT]` 경고가 한 번 나옴), `stop_holds_abandoned`(노드 시작 뒤 유지를 포기한 누적 횟수), `stop_hold_basis`·`stop_hold_load_a`(유지 전류를 정한 근거와 그때 쓴 구동 부하 전류, 아래 '유지 전류'), `planning_wait_reason`(SOFT_STOPPED·RESUME_PENDING일 때 대기 사유, 그 밖에는 빈 문자열). `control_trace` 로그 줄에는 `stop=`, `stop_a=`, `erpm_valid=`, `[PLANNING_WAIT]`에는 `brake_pub_sub=`, `handbrake_pub_sub=`(발행자,구독자)가 있습니다.
- `status_json`에 `planning_supervisor_state`(IDLE·ACTIVE·SOFT_STOPPED·RESUME_PENDING·BLOCKED)와 `motor_current_limit_a`, `input_current_limit_a`, `overcurrent_hard_a`, `esc_temp_c`, `esc_temp_state`, `stop_brake_current_a`, `stop_handbrake_current_a`, `stop_handbrake_below_erpm`, `stop_handbrake_min_current_a`, `stop_handbrake_load_gain`, `stop_handbrake_rolling_current_a`, `stop_handbrake_margin_a`, `planning_stop_ramp_s`, `stop_ramp_active`가 있습니다(정지 전류는 실제로 켜진 값, 꺼져 있으면 0. 부하 기반 유지를 끄면 `stop_handbrake_min_current_a`가 0).
- 통합 GUI 플래닝 개발 창의 **'제어기 복구'** 버튼(`feat/gui`)이 Pi4 제어기의 `~/reset`만 부릅니다(구동 명령 없음). 출력 허용 상태면 복구 직후 다시 출발합니다. 핸드브레이크 중에는 차가 미끄러져도 정지로 보이므로 누르기 전에 눈으로 확인하세요.

### 정지 출력 (2026-11-12, 브레이크·핸드브레이크)

예전 0 A 정지에서는 LPP가 오르막에서 STOP을 보내면 차가 뒤로 굴러 −2500 ERPM 과속으로 잠겼습니다(11-12 야지 시험 4회). 그래서 Pi4에서 정지 출력을 바꿨고, 이 저장소에 옮겼습니다.

| 파라미터(`commissioning.yaml`) | 값 | 의미 |
|---|---|---|
| `stop_brake_current_a` | 20.0 | \|ERPM\|이 아래 기준보다 크면 VESC 브레이크 전류(0~30 A, 0이면 끔) |
| `stop_handbrake_current_a` | 30.0 | 기준 이하이면 VESC 핸드브레이크(고정 전류로 회전자 잠금, 0~30 A, 0이면 끔). 부하 기반 유지를 켜면 최대값 |
| `stop_handbrake_min_current_a` | 12.5 | 부하 기반 유지의 최소값(0이면 끔 = 항상 위 값) |
| `stop_handbrake_load_gain` | 1.45 | 구름저항을 뺀 부하 전류 1 A당 유지 전류(0~5) |
| `stop_handbrake_rolling_current_a` | 6.0 | 평지 주행 전류(구름저항, 0~30 A) |
| `stop_handbrake_margin_a` | 8.0 | 더하는 여유(0~30 A) |
| `stop_handbrake_below_erpm` | 200.0 | 브레이크 → 핸드브레이크 전환 기준(0~2000 ERPM) |
| `motor_handbrake_topic` | `/nomad_bench/commands/motor/handbrake` | 핸드브레이크 토픽 |

- 정지 출력은 LPP STOP·명령 만료·속도 0, 운영자 STOP, 치명 잠금, 종료를 포함한 모든 해제에 쓰이고(LPP STOP·명령 만료는 아래 '감속 정지' 뒤), 출력권을 가진 동안 20 ms마다 다시 보냅니다. 두 값이 0이면 예전처럼 0 A입니다. 브레이크만 0이면 핸드브레이크를 걸지 않은 동안 0 A, 핸드브레이크만 0이면 항상 브레이크입니다. 어느 설정이든 VESC 고장 코드로 잠기면 0 A입니다. 무엇을 낼지는 `include/nomad_control_cpp/stop_output.hpp`(ROS 없는 정책, 단위시험 `stop_output_policy`)가 정합니다.
- **핸드브레이크 진입과 유지(11-13):** 새로 받은 정상 상태값이 200 ERPM 이하이고 드라이버가 핸드브레이크 토픽을 구독할 때만 겁니다. 상태값이 끊기거나 오래되면 걸지 않고 브레이크를 보냅니다. 한 번 걸리면 다음 구동 명령(0이 아닌 속도)까지 유지합니다. 다만 건 지 0.1초가 지난 뒤 최근에 받은 상태값(검사에서 거부된 값 포함)이 400 ERPM(진입 기준의 2배와 기준+200 중 큰 값)을 넘거나 핸드브레이크 구독자가 사라지면, 유지가 실제로 안 되고 있는 것이므로 브레이크로 돌아갑니다(진짜 유지 중에는 0으로 읽힘). 그 뒤 구독자가 있고 정상 상태값이 다시 200 ERPM 이하이면 다시 겁니다.
- **VESC 고장 코드(11-13):** VESC가 고장 코드를 보내 잠기면 브레이크·핸드브레이크 대신 0 A로 풉니다(고장을 보고한 VESC에 전류를 다시 걸지 않기 위해, 대신 경사에서는 굴러갈 수 있음). 멈춘 상태에서 출력권을 가진 채 고장이 와도 잠깁니다. 과전류·과속·바퀴 걸림·전압 이상·운영자 STOP 같은 다른 치명 잠금은 브레이크·유지를 계속합니다. RESET이 받아들여지면 해제됩니다.
- 브레이크나 핸드브레이크를 켜면 그 토픽의 발행자가 제어기 하나뿐이고 구독자(드라이버)가 있어야 출발합니다. 패치 안 한 드라이버에서는 출발하지 않습니다. 이때 사유는 처음 출발 때 `raw start refused: need exclusive driver ...`, 재출발 대기 중에는 `VESC handbrake topic has no subscriber (patched vesc_driver required); waiting`(드라이버의 상태값·속도·전류 토픽은 보일 때. 브레이크 토픽만 구독자가 없으면 `VESC brake topic has no subscriber; waiting`, 드라이버가 안 보이는 등 그 밖의 경우는 `driver endpoints unavailable; waiting`)으로 나옵니다. 처음 출발 실패는 핸드브레이크를 직접 가리키지 않으므로 `ros2 topic info -v /nomad_bench/commands/motor/handbrake`로 구독자를 확인하세요. 드라이버 패치·설정: [`vesc_config_20261112/driver/README.md`](vesc_config_20261112/driver/README.md).
- **유지 전류 (11-13, 부하 기반):** 핸드브레이크를 걸 때마다 정지 직전 주행의 VESC `avg_iq`(모터 q축 전류)로 전류를 정합니다. 제어기가 이미 받는 VESC 상태값만 쓰며, IMU·지도 등 다른 파트의 경사 정보는 쓰지 않습니다. 주행 방향 기준 전류(`s·iq`, s는 속도 명령 부호)는 오르막에서 경사+구름저항, 내리막에서는 속도 PID가 제동하므로 구름저항−경사입니다. 그래서 구름저항을 뺀 부하 전류 `|s·iq − 구름저항|`은 오르막·내리막 모두 같고 평지에서는 0입니다. 유지 전류 = `clamp(배율 × 부하 전류 + 여유, 최소, 최대)`입니다. 지금 값이면 평지 12.5 A, 오르막 주행 16 A(약 7°) 22.5 A, 21 A(약 10°) 29.8 A, 14° 오르막(27 A) 이상 30 A입니다.
  - **안정 주행일 때만 씁니다(리뷰 반영).** 이 관계는 일정 속도일 때만 맞습니다. 출발 직후나 경사에 막 들어서 속도가 처질 때는 속도 PID(5.03은 넘길 때마다 적분 0에서 시작)가 아직 부하만큼 전류를 내지 못해 전류가 작게 읽히고, 작게 건 핸드브레이크는 미끄러져도 ERPM이 0이라 알 수 없습니다. 그래서 측정 ERPM이 명령의 ±25 % 안에 0.3초 이상 머문 채 멈췄을 때만, 마지막 1초의 안정 구간에서 5개 중앙값으로 거른 부하 전류 중 가장 큰 값을 씁니다.
  - **가속 전류는 부하로 치지 않습니다(11-13 저녁).** 모터 전류에는 부하에 더해 가속·감속분이 섞입니다. 평지 출발(초당 +2,000~5,000 ERPM에 15~25 A)과 오버슈트를 되돌릴 때(구름저항보다 낮은 0~4 A)는 경사처럼 읽혀 평지 정지를 22~30 A로 키웠습니다. 그래서 다음 샘플만 부하 근거로 씁니다.
    - **속도가 거의 일정한 샘플:** 앞뒤 0.2초 동안 속도 변화가 초당 600 ERPM 이하이고, 그 안의 최고·최저 차가 300 ERPM 이하인 것. 이때는 모터 전류가 곧 부하입니다. 출발 순간 측정 속도가 600 근처로 튀었다 떨어지는 것도 이 조건으로 거릅니다.
    - **하한값이 되는 샘플(출발 0.2초 뒤부터):** 전류가 구름저항보다 높은데 느려지는 샘플(경사·장애물에 걸림), 낮은데 빨라지는 샘플(내리막). 가속분이 부하를 오히려 작게 보이게 하므로, 낮게 잡은 관성분(초당 1 ERPM에 0.002 A)만 되더해 그대로 씁니다.
    - **멈추기 직전 0.1초:** 앞뒤 구간이 없어 뒤쪽 0.2초 구간으로 판정합니다.
    - **근거가 없는 짧은 출발**(계속 가속만 하다 멈춤): 예전처럼 출발 0.2초 뒤부터의 전류로 보수적으로 정합니다. 다만 마지막으로 전류를 정한 정지 뒤 합쳐서 3 전기회전(약 5 cm) 이하이고 0.28초보다 짧으면 직전 값을 씁니다.
    - **멈추는 순간 세게 감속하면서 전류가 구름저항보다 2 A 넘게 높으면**(방금 경사·장애물에 걸림), 최소한 출발 전류 기준으로 잡습니다.
    - **효과:** 11-13 실제 평지 정지 71건을 다시 돌려 보면 평균 22.8 → 18.1 A, 30 A 정지 26 → 16번, 정지 중 발열 합계 약 −34%입니다. 리뷰 모의(경사 진입·재출발·잡음·간격 흔들림 수천 건)에서 배포본보다 덜 잡으면서 필요한 값보다도 낮은 경우는, 앞바퀴가 경사에 닿은 지 수십 ms 안에 멈춰 전류에 아직 신호가 없는 경우뿐입니다. 이때 미끄러져도 앞바퀴가 경사 시작점까지 약 1 cm 돌아갑니다.
  - 그렇지 않으면: 전류를 새로 정한 마지막 유지(부하 기반·30 A) 뒤로 합쳐서 15 전기회전(이 차에서 약 24 cm) 이하로 움직였으면(짧은 출발·멈춘 언덕 출발) 직전 유지 전류를 쓰되, 그 주행 자체의 전류(안정 여부와 상관없이 가장 큰 부하 전류)가 요구하는 값이 더 크면 그 값으로 올립니다(11-12 21:42처럼 33.8 A로 버티다 멈춘 경우 30 A). 직전 값을 그대로 이어 쓴 유지는 거리를 다시 세지 않으므로, 짧은 출발이 이어져 합쳐서 24 cm를 넘으면 최대값입니다. 움직였거나 직전 유지가 없으면 최대값(30 A)을 씁니다. 이번 정지에서 유지가 높은 ERPM으로 포기됐으면 최대값입니다. 재출발 뒤 첫 상태값 3개(60 ms)는 쓰지 않습니다(`avg_iq`는 직전 읽기 이후 평균이라 직전 핸드브레이크·브레이크 전류가 섞임).
  - 근거는 `stop_hold_basis`(`load` 안정 주행 · `same_place` 제자리 · `unsettled` 안정 아님 · `unknown` 기록 없음 · `escalated` · `fixed`)와 `[STOP_OUTPUT] handbrake hold <A> basis= load_a= moved_erev=` 로그(유지 시작 때 한 번)로 남습니다.
  - 측정 근거(11-13 VESC Tool 핸드브레이크 칸): 14°에서 30 A는 바로 멈추고, 그보다 낮으면 조금 밀린 뒤 멈춤. 20°에서도 30 A로 멈춤. 14° 오르막 주행은 약 27 A였습니다. 14° 미만은 측정 없이 이은 값이고, 구름저항 6 A는 11-12 Pi4 주행 기록의 1000 ERPM 부근 중앙값(약 8 A)·하위 25 %(약 4 A) 사이로 잡은 후보값입니다.
  - 열: 핸드브레이크는 위상을 0으로 고정해 두 상에 전류가 흐르므로 모터 발열은 약 1.5 × 0.0112 Ω × I²(30 A 약 15 W, 12.5 A 약 2.6 W)입니다. 고정 30 A라면 평지 정지에서도 15 W가 계속 나므로 부하에 맞춥니다. 경사에서 오래 서 있으면 여전히 30 A이고, 제어기 온도 보호는 ESC만 봅니다. **11-13 18:33~18:51 실차:** 평지 정지가 대부분 30 A(출발마다 1500 ERPM 넘게 튀어 안정 주행이 안 됨)였고, LPP가 멈춰 있는 동안(한 번에 최대 56초) 30 A로 버텨 모터가 매우 뜨거워졌습니다. 시험하지 않을 때는 자율주행을 꺼 두세요. 출렁임은 `s_pid_kp` 0.010으로 줄였습니다(아래 '재출발과 VESC 속도 PID').
- 11-12 저녁 Pi4 시험: 브레이크만 걸었을 때 10 A(21:15)는 정지 중 최대 −1239 ERPM, 20 A(21:23)는 최대 −1792 ERPM으로 뒤로 밀렸습니다. 21:10 시험은 드라이버 `brake_max`가 0이라 브레이크가 0 A로 잘려 사실상 관성 정지였습니다(−1405 ERPM). 핸드브레이크 15 A를 더한 21:42 시험(VIO 위치)에서는 13번 정지 중 33.8 A로 오르던 중 멈춘 1번은 약 15 cm 밀렸고, 나머지 12번은 정지~재출발 사이 순변위가 약 4 cm 이내(대부분 1 cm 이내)였습니다(953 ERPM에서 멈춘 1번은 약 5 cm 앞으로 갔다가 되돌아옴). 다시 출발할 때 뒤로 밀리지는 않았지만, 전류가 오르는 1~2초 동안 거의 움직이지 않았습니다.

**한계 (다음 경사 시험 전에 개선 예정)**
- 핸드브레이크 중에는 VESC가 보고하는 ERPM·타코가 0으로 고정됩니다(펌웨어 5.03이 위상을 0으로 고정). 미끄러져도 ERPM이 0이라 과속 잠금(2500 ERPM), RESET·'제어기 복구'의 정지 확인, 재출발 전 역방향 확인이 모두 정지로 판단합니다. 21:42 시험에서 33.8 A로 오르던 중 정지한 한 번은 ERPM 0으로 기록됐지만 VIO로 약 10~15 cm 밀렸습니다. 11-12 설정 15 A의 최대 유지력은 바퀴에서 약 6.7 N(지금 최대 30 A는 약 13.4 N)이고, 11-12 오르막 주행 전류 약 16~34 A는 약 7~15 N입니다(환산 계수 3721.88 ERPM/(m/s) 미검증 기준).
- 브레이크 전류는 속도에 비례하는 감쇠라 경사에서 차를 세우지 못하고, 뒤로 밀리는 속도가 수백~1800 ERPM에서 더 커지지 않을 뿐입니다. 이미 뒤로 밀리거나 내리막에서 정지하면 200 ERPM 아래로 내려오지 않아 핸드브레이크가 걸리지 않을 수 있습니다(내리막 정지 미시험).
- 출발: kp·ki 0.004일 때는 처음 약 10 A에서 1초에 약 10 A씩 올라(모터 한계 50 A 기준, 60 A였던 11-12에는 12 A) 30 A 이상이 필요한 경사에서 출발이 늦고 2초 바퀴 걸림 잠금에 걸릴 수 있었습니다. 11-13 `s_pid_ki` 0.04 적용 뒤 VESC Tool 14° 시험에서는 약 0.15초에 전류가 오르기 시작했습니다(제어기 경로 실차 미확인).
- VESC 고장 코드로 잠기면 0 A라서 경사에서는 굴러갈 수 있습니다. 다른 치명 잠금 중에는 정지 출력을 계속 보냅니다.
- 제어기·드라이버가 꺼지면 VESC는 300 ms 뒤 `timeout_brake_current`(0 = 관성)로 풀립니다.
- ROS 시험: `mode=commission` headless 자율주행 경로는 고정 유지 설정(20 A / 15 A / 200 ERPM)으로 돌립니다(STOP 유지, 움직이는 중 STOP의 브레이크 → 유지, 유지 고정과 포기, 과속 잠금 중 브레이크, VESC 고장 0 A와 RESET 뒤 유지 복귀, 상태값 끊김 때 브레이크, 핸드브레이크 구독자 없음과 복귀). 같은 경로를 실제 설정(부하 기반 12.5~30 A)으로도 돌립니다(안정되지 않은 첫 주행 30 A, 오르막·내리막 22.5 A, 평지 12.5 A(재출발 첫 상태값의 직전 유지 전류는 무시), 가파른 오르막 30 A, 제자리 짧은 출발·멈춘 언덕 출발은 직전 값 유지). 감속 정지는 LPP STOP·명령 만료(브레이크 없이 핸드브레이크)·플래너 발행자 2개·막힌 차(감속 없음)·측정 속도가 늦을 때 0.6초 대기 뒤 브레이크·오르막 뒤 유지 전류 22.5 A·감속 중 새 명령(취소)과 그 뒤 평지 정지 12.5 A·감속 중 속도 0·운영자 STOP·VESC 고장·감속 중 잘못된 메시지를 가짜 VESC로 확인합니다(ctest `planning_stop_ramp_ros` = `controller_ros_tests --stop-ramp-only`, 전체 ROS 시험과 따로 돌려 150초 제한 여유를 유지). `mode=bench`는 브레이크 8 A·핸드브레이크 12 A로 브레이크 반복, 정지 유지, 멈춘 시험 중 VESC 고장 0 A와 RESET 뒤 유지 복귀를 봅니다. 수동 GUI 경로의 ROS 시험은 아직 0 A 설정입니다.
- `operation_environment`는 계속 `raised_bench`만 허용합니다.

### 감속 정지 (2026-11-13 밤)

LPP가 자주 멈췄다 가는데, 멈출 때마다 약 1000 ERPM에서 브레이크 20 A → 핸드브레이크까지 약 50 ms 만에 서서 차가 앞뒤로 흔들렸습니다(11-13 평지 기록 71번 정지 모두 LPP STOP 또는 명령 만료). 그래서 이 두 경우만 속도를 줄인 뒤 멈춥니다.

| 파라미터(`planning_headless.yaml`) | 값 | 의미 |
|---|---|---|
| `planning.stop_ramp_s` | 0.3 | 감속 시간(0~1초, 0이면 예전처럼 즉시 정지). headless·실측 속도 제어·자동 재출발 프로필에서만 허용 |

- **적용:** LPP `stop_requested`, LPP 명령 만료(유효시간 0.3초 또는 수신 0.35초 초과), 플래너 발행자가 1개가 아님(사라지거나 2개 이상. 2개면 감속 뒤 RESUME_PENDING으로 기다림).
- **동작:** 속도 명령을 `min(지금 명령, 측정 ERPM)`에서 `stop_handbrake_below_erpm`(200 ERPM)까지 0.3초 동안 일정하게 줄입니다(1000 ERPM이면 20 ms마다 약 53 ERPM, 1000 ERPM 하한은 적용 안 함). VESC 속도 PID가 명령보다 늦게 따라와 명령이 200에 닿을 때 차는 아직 약 300~500 ERPM이므로(리뷰 모델), 측정 속도가 200 이하가 될 때까지 명령 200을 유지합니다(평지·오르막은 보통 0.1~0.16초, 최대 0.6초 더). 측정 속도가 200 이하가 되는 순간(경사·장애물로 더 빨리 느려지면 그보다 일찍) 핸드브레이크로 바로 넘기고 예전과 같은 대기(SOFT_STOPPED)로 갑니다. 0.6초를 더 기다려도 200 아래로 안 내려오면 평소처럼 브레이크 20 A 뒤 200 이하에서 핸드브레이크입니다. 0.6초인 이유: 가파른 내리막(짐을 실으면 14°, 그 밖에는 17~20°)에서는 속도 PID가 30~50 A로 제동하는데 브레이크 20 A로는 못 버팁니다(운영자 STOP·고장 같은 즉시 정지와 0.3초인 1afcbb1은 이런 경사에서 여전히 브레이크 20 A로 멈춥니다). 리뷰 모의(내리막 180가지)에서 0.3초는 브레이크로 넘긴 경우 141가지·폭주 78가지였고, 0.6초는 둘 다 0이었습니다. 조향은 마지막 LPP 값을 유지합니다.
- **즉시 정지 그대로:** VESC 고장 코드, 과속, 과전류, 운영자 STOP, 상태값·드라이버 끊김, 제어 주기 지연, ESC 과열. 감속 중에 이런 일이 생겨도 그 순간 바로 멈춥니다. 감속 중이 아닐 때 온 잘못된 LPP 명령(숫자 아님·범위 밖)과 속도 0 명령도 예전처럼 즉시 정지 출력입니다. 이미 200 ERPM 이하(막혀서 밀고 있음·기어감·뒤로 밀림)면 감속 없이 바로 정지 출력입니다.
- **감속 중 LPP 메시지:** 새 이동 명령(속도 > 0, 플래너 발행자 1개)이면 감속을 취소하고 그대로 주행합니다(멈추지 않음, 1000 ERPM 하한 다시 적용). 속도 0 명령·STOP·잘못되거나 오래된 메시지는 감속을 끊지 않고 끝까지 마칩니다(이미 멈추는 중이라, 바로 세우면 처음의 뒤뚱거림이 다시 생김).
- **유지 전류:** 감속 시작 순간에 멈춘 것처럼 정합니다(감속 중 떨어지는 속도와 줄어드는 전류는 '안정 아님'이나 경사로 읽지 않음). 감속 시작 0.15초 뒤부터의 샘플은 값을 올리기만 합니다: 감속 중에 경사를 만나 전류가 오르거나, 내리막에서 속도 PID가 속도를 못 줄이고 크게 제동하는 경우(`stop_output.hpp` `beginStopRamp`). 명령이 200에 닿아 PID가 세게 제동하기 시작하면, 아직 판정 안 된 감속 샘플을 바로 앞 구간으로도 판정해 봅니다(뒤의 급제동 때문에 내리막 신호가 묻히지 않게). 이때 부하 근거가 되는 샘플이 2개 이상일 때만 그 결과를 쓰고, 나머지는 원래대로 앞뒤 구간으로 판정합니다(측정 속도 지연과 잡음 때문에 평지 샘플 하나가 내리막으로 읽히는 것을 막음. 리뷰 모의에서 평지 정지 평균 12.60 → 12.69 A). 감속 중 간 거리도 '제자리'·'짧게 기어감' 거리에 넣습니다(보수적: 0.16~0.3초짜리 짧은 평지 출발 뒤 정지는 20~30 A로 잡힐 수 있음). **한계(경사 경계):** 정지 직전에 넘은 경사 경계는 전류에 아직 안 보여 평지 값으로 잡힐 수 있습니다. 리뷰 모의(실제 코드, 경계 위치를 바꿔 가며 미끄러지는 경계 위치의 폭)에서 내리막 경계(앞으로 미끄러짐)는 12~20°에서 2.3~3.8 cm로 예전 즉시 정지(4.9~5.5 cm)와 1afcbb1(4.9~8.9 cm)보다 좁습니다(감속 0.3초 기준; 0.4~0.5초로 바꿔도 1afcbb1보다 넓어지지 않음). 오르막 경계(뒤로 경사 시작점까지 몇 cm 밀리고 섬)는 감속으로 더 가는 만큼 12°에서 8.9 cm, 14°에서 7.1 cm로 예전(약 5 cm)보다 넓습니다.
- **플래너 팀 전달:** 1000 ERPM(약 0.27 m/s)에서 STOP 뒤 핸드브레이크까지 약 0.4~0.46초(예전보다 약 0.35~0.4초 더)이고, 정지 거리가 약 4~6 cm 늘어납니다(감속과 200 유지 중 약 3.5~4 전기회전 ≈ 6 cm, 예전 브레이크 정지는 약 1~2 cm; 리뷰 모의 4.1~5.7 cm). 가파른 내리막에서는 200 유지가 최대 0.6초(STOP 뒤 약 0.9초)이고, 그래도 200 이하가 안 되면 브레이크 20 A로 넘어가며 이때는 시간 상한이 없습니다. 명령 만료로 멈출 때는 마지막 명령 뒤 유효시간 0.3초가 더해집니다.
- **기록:** `[STOP_RAMP] begin reason= cmd_erpm= measured_erpm= ramp_from_erpm= end_erpm= ramp_s= iq_a=`, 감속 중 매 주기 `[STOP_RAMP] t= cmd_erpm= measured_erpm= iq_a=`, 끝에 `[STOP_RAMP] end t= measured_erpm= state= reason=` 또는 `[STOP_RAMP] cancelled`. 바로 핸드브레이크로 넘기면(end의 `measured_erpm` 200 이하) 같은 주기에 `[STOP_OUTPUT] handbrake hold`가 `[STOP_RAMP] end`보다 먼저 찍히고, 그 뒤 `raw transition=STOPPED_OUTPUT ... raw_erpm=`(넘길 때 측정 속도)입니다. 0.6초를 기다린 뒤 넘기면 `[STOP_RAMP] end` → `raw transition` → (브레이크, 로그 없음) → `[STOP_OUTPUT] handbrake hold` 순서입니다. `status_json`: `stop_ramp_active`, `planning_stop_ramp_s`, 감속 중 `planning_start_reason` = `speed ramp before stop`.
- **실차 미확인:** VESC 속도 PID(kp 0.010·ki 0.04)가 감속을 얼마나 따라가는지, 200 ERPM 근처 저속 속도 제어(HFI)가 매끄러운지, 명령이 200에 닿은 뒤 0.6초 안에 측정 속도가 200 이하로 내려와 브레이크 없이 핸드브레이크로 넘어가는지는 실차 로그로 확인해야 합니다(11-13 22:52 Pi4 배포본 1afcbb1(`369e3700…`)은 이 대기가 0.3초이므로, 다시 배포하기 전 로그는 0.3초 기준). `[STOP_RAMP] end`의 `measured_erpm`이 200 이하면 바로 핸드브레이크, 200보다 크면 0.6초를 기다린 뒤 브레이크를 거친 것입니다. 감속 시간을 늘려도 PID 뒤처짐은 남으므로(0.5초에서도 명령 200일 때 약 260~330 ERPM) 흔들림 해결책이 아닙니다.

### 재출발과 VESC 속도 PID (2026-11-13)

- 핸드브레이크에서 속도 명령으로 넘어가면 VESC 속도 PID가 적분 0에서 다시 시작합니다(펌웨어 5.03). kp·ki 0.004에서는 1000 ERPM 오차에 약 10 A, 그 뒤 1초에 약 10 A씩만 늘어(모터 한계 50 A 기준) 경사 재출발이 1~2초 멈칫했습니다. 11-13 VESC Tool로 `s_pid_ki`를 0.04로 올렸고(적분이 1000 ERPM 오차에 1초 약 100 A, 14° 경사에서 명령 후 전류가 오르기까지 약 1.5초 → 약 0.15초), 평지 출발 때 1500 ERPM 넘게 튀는 출렁임 때문에 저녁에 `s_pid_kp`를 0.010으로 올렸습니다(처음 P 전류 25 A, 평지 첫 최고 약 1310 ERPM, 약 1.2초에 1000 근처 안정). 절차와 결과는 [`vesc_config_20261112/README.md`](vesc_config_20261112/README.md) 6절이고, 지금 VESC 설정 파일은 `vesc_config_20261112/VESC_설정파일/mcconf_20261113_PID_kp010.xml`이고, 실차(제어기 포함) 확인은 남았습니다.
- 제어기 전류로 출발 순간만 밀어 주는 '출발 보조'도 구현해 검토했지만, 속도 명령으로 넘기는 순간 같은 초기화가 일어나고(전류 20~30 A → 거의 0), 그동안 VESC 적분이 쌓이지 않아 무거운 경사에서는 오히려 더 밀릴 수 있어 넣지 않았습니다.
- **재출발 기록:** 정지 출력 뒤 다시 출발하면 `restart_trace_s`(기본 3초, 0이면 끔) 동안 매 제어 주기 `[RESTART_TRACE] t= cmd_erpm= measured_erpm= current_a= iq_a= telemetry_age_ms= state=` 줄을 남깁니다. 첫 줄은 직전 정지 출력(`from=hold` 등)과 유지 시간·전류를 적습니다. `iq_a`는 VESC `avg_iq`입니다(`current_a`는 정지 상태에서 작게 읽힘). 첫 줄과 출발 직후 1~2줄의 `iq_a`는 핸드브레이크 유지 전류가 섞인 값이라 구동 힘이 아닙니다. `status_json`의 `restart_trace_active`로 진행 여부를 볼 수 있습니다.

### ESC 온도 보호 (2026-11-12)

- `include/nomad_thermal/thermal_guard.hpp`: VESC MOSFET NTC 3개(`ntc_temp_mos1..3`) 중 유효한 최고값을 씁니다. 65 ℃ 경고, 80 ℃ 이상 정지, 70 ℃ 미만 재출발 허용. 온도값(0 이하·150 ℃ 초과·숫자 아님)이 처음부터 무효면 `unknown`이고 막지 않습니다. 80 ℃ 정지 뒤 값이 무효가 되면 정지는 유지되지만 상태는 `unknown`으로 보입니다.
- LPP 자율주행 중에만 동작합니다. headless 재출발 설정이면 대기(비치명), 아니면 잠김입니다. 수동 GUI 세션은 감시하지 않습니다.
- 70~80 ℃는 `derate`로 표시되지만 ERPM 제어기는 출력을 줄이지 않습니다(출력 감쇄는 VIO duty 제어기만).
- 11-12 야지 시험 최고 31.4 ℃. 온도 정지 중에도 정지 출력(핸드브레이크)은 계속 나갑니다.

현재 설정은 이 README와 YAML을 기준으로 확인하세요. 날짜가 붙은 기존 `.ko.txt` 문서는 당시 시험·배포 기록이며 최신 실행 설정과 다를 수 있습니다.

## VESC 설정 (2026-11-12, HFI)

센서리스 VESC가 정지 상태에서 회전자 위치를 못 잡아 책 높이 턱에서 바퀴가 떨기만 했습니다. VESC Tool 단위시험에서 `foc_sensor_mode`를 **HFI**로 바꾸자(Start 8 V, Run 4 V, Max 8 V, Samples 16, Sensorless ERPM HFI 2000) 턱을 넘었습니다. `s_pid_min_erpm`은 900→50으로 바꿨습니다(단독으로는 효과 없음). 전류 한계(±60 A, 배터리 250 A, 회생 −200 A)는 아직 그대로입니다.

변경 전·후 설정 파일, 시험 경과, 되돌리는 방법: [`vesc_config_20261112/README.md`](vesc_config_20261112/README.md). VESC Tool의 모터 설정 마법사와 'Read Default Motor Configuration'은 쓰지 마세요.

## 코드 구조

- `src/controller_main.cpp`: 실행 진입점. Ctrl+C·SIGTERM 때 정지 출력을 보낸 뒤 종료합니다.
- `src/controller_node.cpp`: ROS 입출력과 자율주행 연결. `nomad_controller_core` 정적 라이브러리로 한 번만 빌드되며, 실행 파일과 ROS 시험이 같은 코드를 씁니다.
- `include/nomad_control_cpp/controller_node.hpp`: `ControllerNode::create()`, `prepareShutdown()`.
- `src/raw_trial.cpp`: 원시 ERPM·서보 상태머신, 상태값 검사(사유 분리), 과전류 규칙.
- `include/nomad_control_cpp/planning_supervisor.hpp`: LPP 출력 소유 상태머신(ROS 없음, 단위시험 `test/test_planning_supervisor.cpp`).
- `include/nomad_control_cpp/control_limits.hpp`: 명령 신선도, 미래 stamp 허용, LPP lease, 1850 ERPM 상한 등 공용 안전 상수.
- `include/nomad_control_cpp/planning_command.hpp`: LPP 명령 검사. ERPM 상한은 `commissioning.motor_max_erpm`을 따릅니다.
- `include/nomad_thermal/thermal_guard.hpp`: ESC 온도 보호(헤더만, 단위시험 `test/thermal_guard_test.cpp`).
- `include/nomad_control_cpp/stop_output.hpp`: 정지 출력 정책(0 A·브레이크·핸드브레이크 선택, 유지 고정·포기, VESC 고장 0 A; 단위시험 `test/test_stop_output.cpp`).
- `planning.headless=true`는 `mode=commission`에서만 허용합니다(다른 모드는 시작 거부).

## ROS 빌드와 자율주행 실행

공통 메시지 소스는 이 저장소의 `src/nomad_interfaces`에 있습니다. `vesc_msgs`와 VESC 드라이버는 포함하지 않으므로 팀이 사용하는 VESC 워크스페이스를 먼저 빌드하고 source해야 합니다. 기본 `commissioning.yaml`은 핸드브레이크를 쓰므로 VESC 드라이버에 [`vesc_config_20261112/driver`](vesc_config_20261112/driver/README.md)의 패치를 적용하고, 드라이버 yaml에 `brake_max ≥ 20`, `handbrake_max ≥ 15`를 넣어야 합니다(패치가 없으면 출발하지 않음). 이 패키지의 실행기는 두 번째 드라이버를 시작하지 않습니다.

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

ROS 환경에서는 `colcon test`로 18개 시험이 돕니다(ROS 통합시험은 약 75~80초, 제한 150초, 자체 DDS domain 226). 알려진 실패: `rpm_running_floor_ros`의 100 ERPM 경계 기대값 2건은 의도적으로 예전 값을 유지해 실패하고, `controller_ros_integration`의 'stale IMU' 받침대 시험은 타이밍 때문에 가끔 실패합니다(실차 경로 무관).

## 작업 내역 (2026-11-11 ~ 11-13, BAE)

e173e9f까지는 저장소에서 Pi4 `~/nomad_cpp_ws`로 배포했습니다(실행 파일 sha256 `f03e70a3…`). 그 뒤 Pi4에서 직접 온도 보호(11-12 야지 시험 전)와 정지 출력(11-12 저녁)을 넣어 21:33에 빌드했고(sha256 `c4a86cf9…`, 백업 `log/stop_brake_20261112`, `log/handbrake_20261112`), 그 소스를 아래 11-13 커밋으로 옮겼습니다. 07668df까지는 Pi4 설치본과 동작이 같고 문구·`status_json` 정지 필드·스타일만 다릅니다. 11-13 17:24에 f6a662b와 수동 GUI VESC 점검 수정을 Pi4에 배포했습니다(실행 파일 sha256 `d65b65c9…`, Release 빌드 5분, Pi4 시험 17/18 통과·실패는 의도된 `rpm_running_floor_ros` 2건, 백업 `release_backups/feat_control_f6a662b_pre_20261113_172018`). 11-13 22:52에 472ad91·1afcbb1(감속 정지)을 Pi4에 배포했습니다(실행 파일 sha256 `369e3700…`, Release 빌드 6분 9초·경고 0, 사용자 요청으로 빠른 단위시험 15/15만 실행하고 ROS 시험은 PC에서만 확인, 백업 `release_backups/feat_control_1afcbb1_pre_20261113_224455`; 감속만 끄려면 설치본 `install/nomad_control_cpp/share/nomad_control_cpp/config/planning_headless.yaml`의 `planning.stop_ramp_s`를 0으로, 다시 빌드 불필요). Pi4의 실행 스크립트(`scripts/*.sh`, `desktop/`)는 Pi4 것을 그대로 두었습니다(저장소 판은 경로 일반화판, Pi4 미검증).

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
| 2da7b01 | (11-13) ESC MOSFET 온도 보호(65 경고 / 80 정지 / 70 미만 재출발) |
| 07668df | (11-13) 정지 출력: 브레이크 20 A → 200 ERPM 이하 핸드브레이크 15 A, `status_json` 정지 필드, 웹 GUI 안내 |
| 74059b1 | (11-13) 문서·안내서 갱신, VESC 드라이버 패치·설정 기록 |
| 7a12ec0 | (11-13) 정지 출력 1단계: `StopOutput` 정책(단위시험), 핸드브레이크 유지 고정·포기 조건, VESC 고장 코드 0 A, 정지 상태 표시(`stop_output`·`erpm_valid` 등), 핸드브레이크 구독자 없음 대기 사유, 실제 설정 자율주행 ROS 시험 |
| 0e9fed4 | (11-13) 재출발 기록 `[RESTART_TRACE]`(`restart_trace_s`), VESC 속도 PID 조정 시험 절차, 출발 보조 검토 결과 |
| 5124b81 | (11-13) 정지 출력 3단계: 핸드브레이크 유지 전류를 정지 직전 안정 주행의 `avg_iq`(구동 부하)에 맞춤(12.5~30 A, 안정 주행이 아니면 약 24 cm 이내 제자리면 직전 값 이상·아니면 30 A, `stop_hold_basis`, 상태값·파라미터 이름은 `load`), VESC Tool 측정(14°·20°에서 30 A로 정지) 기록 |
| 1a2cc4f | (11-13) VESC 속도 PID `s_pid_ki` 0.04 적용과 11-13 VESC 전류 한계(±50 A·배터리 30 A·회생 −30 A) 기록(문서만) |
| f6a662b | (11-13) 지금 VESC 설정 `mcconf_20261113_PID.xml` 추가 |
| 66a4ebb | (11-13) 수동 GUI VESC 점검 기대값을 11-13 VESC 설정에 맞춤, Pi4 배포(`d65b65c9…`) 기록 |
| b917e79 | (11-13) VESC `s_pid_kp` 0.010과 설정 파일 `mcconf_20261113_PID_kp010.xml`, kp 비교·실차 평지 결과(30 A 유지와 모터 발열) 기록 |
| 472ad91 | (11-13) 유지 전류 판정에서 가속 전류 제외: 속도 일정 샘플과 하한 샘플(경사·장애물에 걸려 느려지는 중)만 부하 근거, 끝 0.1초·간격 흔들림 처리, 짧은 기어가기 기준, 끝에서 부하 증가 시 보수적. 실제 평지 71정지 평균 22.8 → 18.1 A |
| 1afcbb1 | (11-13) 감속 정지: LPP STOP·명령 끊김만 0.3초 동안 200 ERPM까지 속도를 줄이고 측정 속도가 200 이하가 되면(최대 0.3초 더, 다음 커밋에서 0.6초) 핸드브레이크(`planning.stop_ramp_s`, `[STOP_RAMP]` 기록), 고장·운영자 STOP·과속은 즉시, 감속 중 새 이동 명령이면 취소, 유지 전류는 감속 시작 기준(감속 중 경사·내리막 신호는 올리기만) |
| (이 커밋) | (11-14) 2차 리뷰 반영: 200 ERPM 유지 최대 0.6초(가파른 내리막에서 브레이크 20 A로 넘기지 않게), 명령이 200에 닿을 때 감속 샘플을 앞 구간으로도 판정(근거 2개 이상일 때만, 내리막 경계 사각지대 12~20°에서 2.3~3.8 cm로 예전보다 좁게), 시험 보강(대기 시간·취소 뒤 새 기록), 로그 순서 문서, 472ad91·1afcbb1 Pi4 배포 기록(`369e3700…`) |

통합 GUI(`feat/gui` ddfb8c3, a1c24af): 플래닝 창 '제어기 복구' 버튼, 복구 결과 전용 표시, 복구 중 창 닫기 보호.

### 남은 일

1. VESC 전류 한계 정리: 11-13 VESC에 적용(모터 ±50 A, 배터리 30 A, 회생 −30 A, 같은 날 `s_pid_ki` 0.04·`s_pid_kp` 0.010). 정지 브레이크 때문에 `l_current_min`은 −30 A 이하로 둡니다(지금 −50 A). 설정 파일 `vesc_config_20261112/VESC_설정파일/mcconf_20261113_PID_kp010.xml`.
2. 수동 GUI VESC 점검(`ensure_gui.sh`): 11-13 새 VESC 값으로 맞춤(모터 ≤50 A·하한 −50~−30 A, 배터리 ≤30 A·회생 −30~0 A, PID 최소 50). 자율주행 경로(`run_planning_output.sh`)에는 VESC 설정 점검이 없습니다. Pi4 `~/vesc_ws/inspect_vesc.py --bench-current-a`는 VESC 전류 한계를 써 버리므로(회생 0 A) 쓰지 마세요.
3. 리뷰에서 나온 개선 후보(미수정): 전류 피크 직후 재출발 대기 없음, RESET 때 과전류 기록 미초기화, 과속 감속 명령이 VESC PID 최소값 아래로 내려감, 운영자 STOP 후 복구하면 곧바로 출발.
4. 플래너 쪽 합의: 플래너가 1000 ERPM 하한을 모름, 종료 시 드라이버·제어기 동시 종료, `vesc_commission.yaml` 서보 범위 중복, LPP STOP 뒤 감속 정지로 STOP→핸드브레이크 약 0.4~0.46초(예전보다 약 0.35~0.4초 더, 가파른 내리막은 200 유지 최대 0.6초 뒤 브레이크로 넘어가면 상한 없음)·거리 약 4~6 cm 증가(11-13 감속 정지).
5. HFI 적용 후 통합시험(ERPM 모드)에서 턱 넘기·바퀴 걸림 감시·모터 발열 확인. VIO duty 모드는 계속 사용 금지(Pi4의 `nomad_vio_duty_control`에도 11-12에 같은 정지 출력과 온도 보호가 들어갔지만 실차 미시험, 이 저장소 밖).
6. 정지 출력 개선(다음 경사 시험 전): 1단계(유지 고정·표시, VESC 고장 0 A, 실제 설정 ROS 시험, 대기 사유)는 11-13에 완료. 재출발은 VESC 속도 PID 조정(VESC Tool, `vesc_config_20261112` 6절, 11-13 `s_pid_ki` 0.04·`s_pid_kp` 0.010 적용, `mcconf_20261113_PID_kp010.xml`)과 `[RESTART_TRACE]` 기록으로 진행(출발 보조는 넣지 않음). 3단계(부하에 맞춘 유지 전류 12.5~30 A)는 11-13 구현, 실차 확인 남음(`[STOP_OUTPUT] handbrake hold` 로그로 경사별 전류와 미끄러짐을 같이 보고 구름저항 6 A·배율 1.45·여유 8 A를 맞춤. 7°·10° 경사의 오르막 전류와 미끄러지지 않는 핸드브레이크 값, 평지 주행 전류 측정이 있으면 좋음). 남은 것: 4단계 VIO로 미끄러짐 감지(그때 유지 중 RESET·재출발이 ERPM 0을 정지로 믿지 않게), 5단계 밀리는 중·내리막 정지 때 능동 정지 후 유지.
7. 데스크톱 Qt GUI 도움말(`desktop/main.cpp`)의 '실제 차량에서는 브레이크가 아닙니다' 문구는 Qt6 빌드 확인 뒤 고칩니다. 웹 GUI는 실제 정지 설정을 표시합니다.
