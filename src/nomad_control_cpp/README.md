# NOMAD 실차 제어와 Qt GUI

Pi의 최신 실차 제어 소스와 설정을 포함합니다. `controller_node`가 LPP 명령을 VESC 속도·서보 명령으로 변환하고, 상태 감시와 정지·재출발을 처리합니다. Qt 수동 GUI, 장치 없는 모의 화면, 수신 전용 시험 기록기도 포함합니다.

이 패키지는 기존 Gazebo `/cmd_vel` 경로에 자동 연결되지 않습니다. 실차 제어기는 `use_sim_time=false`를 사용합니다. 기존 `.nomad/build.sh`와 `run_forest.sh`의 시뮬레이터 실행은 변경하지 않았습니다. 실차에서 확인한 환경은 ROS 2 Humble이며, 이 저장소의 Jazzy 시뮬레이션 검증과 구분합니다.

## 인터페이스

| 방향 | 토픽 | 타입·단위 |
|---|---|---|
| LPP → 제어기 | `/nomad/planning/drive_command` | `nomad_interfaces/msg/DriveCommand`: m/s, rad, STOP, stamp, valid_for. reliable 구독(계약대로) |
| VESC → 제어기 | `/nomad_bench/sensors/core` | `vesc_msgs/msg/VescStateStamped`: ERPM, A, V, 오류 |
| 제어기 → VESC | `/nomad_bench/commands/motor/speed` | `std_msgs/msg/Float64`, ERPM |
| 제어기 → VESC | `/nomad_bench/commands/servo/position` | `std_msgs/msg/Float64`, 0~1 |
| 제어기 → VESC | `/nomad_bench/commands/motor/current` | `std_msgs/msg/Float64`, 정지 시 0 A(정지 브레이크·유지를 끈 경우, 또는 VESC 고장 코드로 잠긴 경우) |
| 제어기 → VESC | `/nomad_bench/commands/motor/brake` | `std_msgs/msg/Float64`, A. 정지 중 핸드브레이크를 걸지 않은 동안(\|ERPM\| > `stop_handbrake_below_erpm`, 상태값 끊김, 유지 포기) |
| 제어기 → VESC | `/nomad_bench/commands/motor/handbrake` | `std_msgs/msg/Float64`, A. 정지 중 정상 상태값이 그 이하일 때 걸고 다음 구동 명령까지 유지. **패치한 VESC 드라이버 필요** |
| 제어기 → LPP | `/nomad/control/vehicle_state` | `nomad_interfaces/msg/VehicleState`. 제어 주기(약 50 Hz, 11-17부터), reliable |
| 제어기 → GUI·로그 | `/nomad_vesc_trial/status_json` | `std_msgs/msg/String`, JSON 상태 |

`DriveCommand`는 `frame_id=base_link`, 증가하는 ROS stamp, `0 < valid_for <= 0.5초`를 사용합니다. 같은 stamp의 이동 명령으로 유효기간을 연장하지 않습니다. 수신 후 0.35초 이상 갱신되지 않아도 출력을 해제합니다. LPP 자율주행은 전진·정지만 허용합니다(후진은 GUI 수동 조종만, 아래 '수동 후진').

## 현재 차량 설정

- 설정 순서: `commissioning.yaml` → `planning.yaml` → `planning_headless.yaml` → `nomad_vehicle_ready.yaml`.
- m/s → ERPM 계수: `3721.882617894008`.
- 양수 자율주행 목표: `max(1000 ERPM, 환산 요청 ERPM)`. LPP STOP·명령 끊김은 0.3초 동안 속도를 200 ERPM까지 줄이고, 측정 속도가 200 이하가 되면(최대 0.6초 더) 핸드브레이크로 넘어갑니다(11-13, 아래 '감속 정지'). 주행 중 속도 0 명령은 감속 없이 정지 출력입니다(아래 '정지 출력'). 과속 감속 중에는 최소 출력 적용을 해제합니다.
- LPP 입력 상한은 0.25 m/s입니다. 이 상한의 환산값도 1000보다 낮으므로, 현재 유효한 양수 요청은 정상 상태에서 모두 1000 ERPM으로 올라갑니다. 입력 m/s를 그대로 추종하는 설정은 아닙니다.
- 조향 입력 ±0.6108652382 rad(±35°) → 서보 0~1, 중앙 0.5. 실제 바퀴 각도 피드백은 없습니다.
- VESC 내부 속도 PID를 사용합니다. C++은 ERPM 목표와 과속 감시를 담당합니다. 과속 감시 기준은 1200 ERPM이며 50 ERPM 여유를 넘는 초과분에 따라 목표를 낮춥니다.
- 전류 기준(`commissioning.yaml`): **모터 전류 55 A**(`motor_current_limit_a`), **배터리 입력 전류 30 A**(`current_limit_a`), 즉시 잠금 60 A(`overcurrent_hard_a`), 지속 0.3초(`overcurrent_sustain_s`), 반복 10초에 3회(`overcurrent_spike_limit`, `overcurrent_spike_window_s`). 저속 출발은 모터 전류만 크고(30~35 A) 입력 전류는 2~3 A라 두 기준을 나눴습니다.
- 정지 출력은 VESC 브레이크 20 A → 거의 멈추면 핸드브레이크 유지입니다(2026-11-12). 유지 전류는 정지 직전 안정 주행의 모터 전류(구동 부하)에 맞춰 12.5~30 A입니다(11-13, 후보값). VESC가 고장 코드를 보내 잠기면 0 A로 풉니다(11-13). 경사 정차가 보장되지는 않습니다. 아래 '정지 출력'을 보세요.
- GUI 수동 조종 후진(11-15, 팀원 키보드 조종용): `commissioning.manual_reverse_max_erpm` 800 ERPM까지. LPP 자율주행은 항상 전진만입니다. Pi4 드라이버 yaml `speed_min`도 −800으로 맞춰야 합니다(아래 '수동 후진').
- 서보 변화율: MANUAL 축(수동 조종과 LPP 자율주행 모두) 1.0/s(`commissioning.manual_servo_rate_s`, 11-15 팀 합의로 0.40에서 올림, 서보 0~1 끝에서 끝 약 1초·중앙에서 끝 약 0.5초).
- ESC MOSFET 온도 보호: 65 ℃ 경고, 80 ℃ 이상이면 자율주행 출력 정지·수동 조종 세션 종료(11-15부터), 70 ℃ 미만에서 재출발·새 세션 허용. 아래 'ESC 온도 보호'를 보세요.
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
- **동작:** 속도 명령을 `min(지금 명령, 측정 ERPM)`(11-20부터 '측정 ERPM'은 0.1초 안의 최근 3개 중 가장 큰 값, 아래 '감속 시작 판단')에서 `stop_handbrake_below_erpm`(200 ERPM)까지 0.3초 동안 일정하게 줄입니다(1000 ERPM이면 20 ms마다 약 53 ERPM, 1000 ERPM 하한은 적용 안 함). VESC 속도 PID가 명령보다 늦게 따라와 명령이 200에 닿을 때 차는 아직 약 300~500 ERPM이므로(리뷰 모델), 측정 속도가 200 이하가 될 때까지 명령 200을 유지합니다(평지·오르막은 보통 0.1~0.16초, 최대 0.6초 더). 측정 속도가 200 이하가 되는 순간(경사·장애물로 더 빨리 느려지면 그보다 일찍) 핸드브레이크로 바로 넘기고 예전과 같은 대기(SOFT_STOPPED)로 갑니다. 0.6초를 더 기다려도 200 아래로 안 내려오면 평소처럼 브레이크 20 A 뒤 200 이하에서 핸드브레이크입니다. 0.6초인 이유: 가파른 내리막(짐을 실으면 14°, 그 밖에는 17~20°)에서는 속도 PID가 30~50 A로 제동하는데 브레이크 20 A로는 못 버팁니다(운영자 STOP·고장 같은 즉시 정지와 0.3초인 1afcbb1은 이런 경사에서 여전히 브레이크 20 A로 멈춥니다). 리뷰 모의(내리막 180가지)에서 0.3초는 브레이크로 넘긴 경우 141가지·폭주 78가지였고, 0.6초는 둘 다 0이었습니다. 조향은 마지막 LPP 값을 유지합니다.
- **즉시 정지 그대로:** VESC 고장 코드, 과속, 과전류, 운영자 STOP, 상태값·드라이버 끊김, 제어 주기 지연, ESC 과열. 감속 중에 이런 일이 생겨도 그 순간 바로 멈춥니다. 감속 중이 아닐 때 온 잘못된 LPP 명령(숫자 아님·범위 밖)과 속도 0 명령도 예전처럼 즉시 정지 출력입니다. 이미 200 ERPM 이하(막혀서 밀고 있음·기어감·뒤로 밀림, 11-20부터 최근 측정값 3개가 모두)면 감속 없이 바로 정지 출력입니다.
- **감속 중 LPP 메시지:** 새 이동 명령(속도 > 0, 플래너 발행자 1개)이면 감속을 취소하고 그대로 주행합니다(멈추지 않음, 1000 ERPM 하한 다시 적용). 속도 0 명령·STOP·잘못되거나 오래된 메시지는 감속을 끊지 않고 끝까지 마칩니다(이미 멈추는 중이라, 바로 세우면 처음의 뒤뚱거림이 다시 생김).
- **유지 전류:** 감속 시작 순간에 멈춘 것처럼 정합니다(감속 중 떨어지는 속도와 줄어드는 전류는 '안정 아님'이나 경사로 읽지 않음). 감속 시작 0.15초 뒤부터의 샘플은 값을 올리기만 합니다: 감속 중에 경사를 만나 전류가 오르거나, 내리막에서 속도 PID가 속도를 못 줄이고 크게 제동하는 경우(`stop_output.hpp` `beginStopRamp`). 명령이 200에 닿아 PID가 세게 제동하기 시작하면, 아직 판정 안 된 감속 샘플을 바로 앞 구간으로도 판정해 봅니다(뒤의 급제동 때문에 내리막 신호가 묻히지 않게). 이때 부하 근거가 되는 샘플이 2개 이상일 때만 그 결과를 쓰고, 나머지는 원래대로 앞뒤 구간으로 판정합니다(측정 속도 지연과 잡음 때문에 평지 샘플 하나가 내리막으로 읽히는 것을 막음. 리뷰 모의에서 평지 정지 평균 12.60 → 12.69 A). 감속 중 간 거리도 '제자리'·'짧게 기어감' 거리에 넣습니다(보수적: 0.16~0.3초짜리 짧은 평지 출발 뒤 정지는 20~30 A로 잡힐 수 있음). **한계(경사 경계):** 정지 직전에 넘은 경사 경계는 전류에 아직 안 보여 평지 값으로 잡힐 수 있습니다. 리뷰 모의(실제 코드, 경계 위치를 바꿔 가며 미끄러지는 경계 위치의 폭)에서 내리막 경계(앞으로 미끄러짐)는 12~20°에서 2.3~3.8 cm로 예전 즉시 정지(4.9~5.5 cm)와 1afcbb1(4.9~8.9 cm)보다 좁습니다(감속 0.3초 기준; 0.4~0.5초로 바꿔도 1afcbb1보다 넓어지지 않음). 오르막 경계(뒤로 경사 시작점까지 몇 cm 밀리고 섬)는 감속으로 더 가는 만큼 12°에서 8.9 cm, 14°에서 7.1 cm로 예전(약 5 cm)보다 넓습니다.
- **플래너 팀 전달:** 1000 ERPM(약 0.27 m/s)에서 STOP 뒤 핸드브레이크까지 약 0.4~0.46초(예전보다 약 0.35~0.4초 더)이고, 정지 거리가 약 4~6 cm 늘어납니다(감속과 200 유지 중 약 3.5~4 전기회전 ≈ 6 cm, 예전 브레이크 정지는 약 1~2 cm; 리뷰 모의 4.1~5.7 cm). 가파른 내리막에서는 200 유지가 최대 0.6초(STOP 뒤 약 0.9초)이고, 그래도 200 이하가 안 되면 브레이크 20 A로 넘어가며 이때는 시간 상한이 없습니다. 명령 만료로 멈출 때는 마지막 명령 뒤 유효시간 0.3초가 더해집니다.
- **기록:** `[STOP_RAMP] begin reason= cmd_erpm= measured_erpm= ramp_from_erpm= end_erpm= ramp_s= iq_a=`, 끝에 `[STOP_RAMP] end t= measured_erpm= state= reason=` 또는 `[STOP_RAMP] cancelled`. 11-19부터 끝 줄에 요약(`wait_s= end_samples= end_above= erpm_last= iq_peak_a=`, 아래 'GUI가 읽는 로그 줄이기')이 붙고, 감속 중 매 주기 줄 `[STOP_RAMP] t= cmd_erpm= measured_erpm= iq_a=`는 `trace_every_tick: true`일 때만 찍힙니다. 바로 핸드브레이크로 넘기면(end의 `measured_erpm` 200 이하) 같은 주기에 `[STOP_OUTPUT] handbrake hold` → `raw transition=STOPPED_OUTPUT ... raw_erpm=`(넘길 때 측정 속도) → `[STOP_RAMP] end` 순서입니다. 0.6초를 기다린 뒤 넘기면 (브레이크, 로그 없음) `raw transition` → `[STOP_RAMP] end` → 뒤 주기에 `[STOP_OUTPUT] handbrake hold` 순서입니다. 11-19 전 배포본은 `[STOP_RAMP] end`가 `raw transition`보다 먼저였습니다. `status_json`: `stop_ramp_active`, `planning_stop_ramp_s`, 감속 중 `planning_start_reason` = `speed ramp before stop`.
- **실차 미확인:** VESC 속도 PID(kp 0.010·ki 0.04)가 감속을 얼마나 따라가는지, 200 ERPM 근처 저속 속도 제어(HFI)가 매끄러운지, 명령이 200에 닿은 뒤 0.6초 안에 측정 속도가 200 이하로 내려와 브레이크 없이 핸드브레이크로 넘어가는지는 실차 로그로 확인해야 합니다(11-13 22:52~11-15 배포본 1afcbb1(`369e3700…`)과 팀원 빌드(`915e9894…`)는 이 대기가 0.3초이므로 그때 로그는 0.3초 기준; 11-15 17:01 1d47cf0 배포부터 0.6초). `[STOP_RAMP] end`의 `measured_erpm`이 200 이하면 바로 핸드브레이크, 200보다 크면 0.6초를 기다린 뒤 브레이크를 거친 것입니다. 감속 시간을 늘려도 PID 뒤처짐은 남으므로(0.5초에서도 명령 200일 때 약 260~330 ERPM) 흔들림 해결책이 아닙니다.

### 재출발과 VESC 속도 PID (2026-11-13)

- 핸드브레이크에서 속도 명령으로 넘어가면 VESC 속도 PID가 적분 0에서 다시 시작합니다(펌웨어 5.03). kp·ki 0.004에서는 1000 ERPM 오차에 약 10 A, 그 뒤 1초에 약 10 A씩만 늘어(모터 한계 50 A 기준) 경사 재출발이 1~2초 멈칫했습니다. 11-13 VESC Tool로 `s_pid_ki`를 0.04로 올렸고(적분이 1000 ERPM 오차에 1초 약 100 A, 14° 경사에서 명령 후 전류가 오르기까지 약 1.5초 → 약 0.15초), 평지 출발 때 1500 ERPM 넘게 튀는 출렁임 때문에 저녁에 `s_pid_kp`를 0.010으로 올렸습니다(처음 P 전류 25 A, 평지 첫 최고 약 1310 ERPM, 약 1.2초에 1000 근처 안정). 절차와 결과는 [`vesc_config_20261112/README.md`](vesc_config_20261112/README.md) 6절이고, 지금 VESC 설정 파일은 `vesc_config_20261112/VESC_설정파일/mcconf_20261113_PID_kp010.xml`이고, 실차(제어기 포함) 확인은 남았습니다.
- 제어기 전류로 출발 순간만 밀어 주는 '출발 보조'도 구현해 검토했지만, 속도 명령으로 넘기는 순간 같은 초기화가 일어나고(전류 20~30 A → 거의 0), 그동안 VESC 적분이 쌓이지 않아 무거운 경사에서는 오히려 더 밀릴 수 있어 넣지 않았습니다.
- **재출발 기록:** 정지 출력 뒤 다시 출발하면 `restart_trace_s`(기본 3초, 0이면 끔) 동안 기록합니다. 11-19부터는 시작 줄과 끝 요약 줄 `[RESTART_TRACE] end t= reason= ticks= reach_s= cmd_erpm= erpm_min= erpm_max= current_max_a= iq_peak_a= iq_peak_t= telemetry_age_max_ms=`만 찍고, 매 제어 주기 줄 `[RESTART_TRACE] t= cmd_erpm= measured_erpm= current_a= iq_a= telemetry_age_ms= state=`는 `trace_every_tick: true`일 때만 찍습니다(아래 'GUI가 읽는 로그 줄이기'). 아래 '첫 줄과 출발 직후 1~2줄'은 매 주기 줄 이야기이고, 끝 요약의 전류 최대값은 이 줄들을 뺀 값입니다. 첫 줄은 직전 정지 출력(`from=hold` 등)과 유지 시간·전류를 적습니다. `iq_a`는 VESC `avg_iq`입니다(`current_a`는 정지 상태에서 작게 읽힘). 첫 줄과 출발 직후 1~2줄의 `iq_a`는 핸드브레이크 유지 전류가 섞인 값이라 구동 힘이 아닙니다. `status_json`의 `restart_trace_active`로 진행 여부를 볼 수 있습니다.

### 수동 후진 (2026-11-15, 팀원)

GUI 팀원이 노트북 '키보드 조종'(공유폴더 GUI `keyboard_drive.py`, 제어기 HTTP `/api/commissioning`을 SSH 터널로 사용)에서 후진을 쓰려고 11-15 14시 20분쯤 Pi4 소스(1afcbb1)를 직접 고쳐 빌드했습니다(실행 파일 sha256 `915e9894…`; Pi4 시계가 틀려 정확한 시각은 모름). 그 변경분만 04b3886 위에 그대로 얹은 것이 11cfb8b입니다(`raw_trial.cpp`·`controller_node.cpp`는 Pi4와 같고, Pi4에는 04b3886의 0.6초 대기·감속 끝 판정이 없어 아직 0.3초입니다). 리뷰에서 나온 문제는 ba0d3e5와 그다음 커밋에서 고쳤습니다.

| 설정 | 값 | 의미 |
|---|---|---|
| `commissioning.manual_reverse_max_erpm` (`commissioning.yaml`) | 800 | GUI 수동 조종(MANUAL) 후진 최대 ERPM 크기. 0이면 전진만(11-15 이전). `motor_max_erpm` 이하 |
| 드라이버 `speed_min` (Pi4 `commission_session/vesc_commission.yaml`, 사본 `vesc_config_20261112/driver/vesc_commission.yaml`) | −800 | 드라이버가 속도 명령을 이 값에서 자릅니다. 제어기 값과 같은 크기로 두세요 |

- **허용 경로:** 수동 조종 서비스(`/commissioning_control`, 웹 GUI·키보드 조종)의 MANUAL 갱신만 음수를 받습니다. LPP 자율주행, MOTOR 시험, 자율주행 재출발은 음수를 거부합니다. 한계를 넘는 음수(예: −800.1)는 예전처럼 잘못된 목표로 잠기고 RESET이 필요합니다. 제어기 자체 웹 페이지와 Qt GUI에는 후진 버튼이 없습니다.
- **방향 전환:** 운영자가 마지막 구동 방향(키를 뗀 뒤에도 기억)과 반대를 누르면, 바퀴가 아직 옛 방향으로 50 ERPM(`stationary_erpm`)보다 빨리 도는 동안 출력을 풀어 정지 출력(브레이크 20 A → 200 ERPM 이하 핸드브레이크)으로 세운 뒤 새 방향을 0부터 올립니다. 핸드브레이크가 걸리면 보고 속도가 0이 되므로 곧바로 출발합니다. 최대 0.5초(`kReversalWaitMaxS`)만 기다리고, 그래도 안 서면 새 방향을 0부터 구동해 속도 PID가 제동하게 합니다. 기다리는 동안 옛 방향 속도가 누른 뒤 가장 낮았던 값보다 100 ERPM(`kReversalLosingErpm`) 넘게 오르면(브레이크 20 A가 지는 가파른 경사) 0.5초를 채우지 않고 바로 넘깁니다(리뷰 모의: 이 조건이 없으면 17~20° 내리막 반대 키 정지 거리가 즉시 구동의 2~5배였고, 짐을 실으면 2500 ERPM 과속 잠금까지 갔음). 키보드에서 두 키를 겹쳐 누르는 동안은 목표가 0이라 대기 시간은 반대 키만 눌린 순간부터 셉니다. 대기 중에는 `status_json` `manual_reversal_wait`가 true이고, 로그에 `[REVERSAL] wait begin target_erpm= measured_erpm=`과 `[REVERSAL] wait end t= reason= measured_erpm= cmd_erpm=`(reason: `still` 멈춤·핸드브레이크, `brake losing`, `timeout`, `released` 키 뗌, `cancelled` 원래 방향 다시 누름, `stopped`)이 남습니다. 내리막에서 빨리 내려가는 차를 세울 때는 반대 키를 톡톡 누르지 말고 STOP이나 같은 방향 키를 쓰세요(반대 키를 뗄 때마다 대기가 처음부터 다시 시작되고 그동안은 브레이크 20 A뿐).
- **같은 방향 명령은 측정 속도와 상관없이 계속 구동:** 오르막 출발에서 뒤로 밀리거나 핸드브레이크를 푼 직후 보고 속도가 한 번 튀어도(±200~400 ERPM) 명령을 끊지 않습니다(1afcbb1과 같음). 11cfb8b와 11-15 17:01 전까지의 Pi4 설치본(팀원 빌드 `915e9894…`, 같은 `raw_trial.cpp`)은 측정 속도의 부호만으로 방향 전환을 판단해, 전진 명령 중 −50 ERPM보다 뒤로 밀리면 출력을 풀었습니다. 그래서 리뷰 모의에서 수동 조종 오르막 출발이 핸드브레이크 ↔ 출발을 반복해 오르지 못하거나, 가파른 경사에서는 브레이크 20 A만 걸린 채 운전자가 전진을 눌러도 뒤로 굴렀습니다(실차에서 본 것은 아님). 후진 한계를 0으로 해도 같았습니다(LPP 자율주행은 1000 ERPM 하한 때문에 영향 없음). 11-15 Pi4 로그의 후진 출발 2번에 끼어 있던 20 A 브레이크 한 틱도 이 때문입니다. **이 수정은 11-15 17:01에 Pi4에 배포했습니다(1d47cf0, `d2bbe876…`). 그 전 팀원 빌드로는 수동·키보드 조종 경사 출발을 하지 마세요.**
- **유지 전류·감시:** 후진 뒤 정지도 전진과 같은 규칙(구동 방향 기준 부하)으로 유지 전류를 정합니다(리뷰 모의: 7° 24~26 A, 3.5° 16~18.5 A). 과속(2500 ERPM)·과전류·바퀴 걸림 감시는 크기(절댓값)로 봅니다.
- **실차 확인:** 11-15 Pi4 수동 세션에서 평지 후진 −600 ERPM 도달. 미확인: 경사 후진, 후진 구름저항(`stop_handbrake_rolling_current_a` 6 A는 전진 약 1000 ERPM 기록의 중앙값 약 8 A와 하위 25 % 약 4 A 사이에서 고른 후보값), 방향 전환 대기 시간(`[REVERSAL]` 로그로 확인).
- **키보드 조종 GUI(팀원 담당, 리뷰 결과 전달 필요):** 창 포커스를 잃을 때 보내는 세션 닫기가 다른 요청과 HTTP 연결 하나를 같이 써서 자주 빠집니다(그동안 Wi-Fi 수동 시간 제한 2초까지 그 속도 유지). '연결 끊기'와 조종 창 닫기는 Pi4 제어기·드라이버를 꺼서 핸드브레이크가 풀립니다(VESC `timeout_brake_current` 0, 바퀴가 자유로워져 경사에서는 굴러감).

### ESC 온도 보호 (2026-11-12)

- `include/nomad_thermal/thermal_guard.hpp`: VESC MOSFET NTC 3개(`ntc_temp_mos1..3`) 중 유효한 최고값을 씁니다. 65 ℃ 경고, 80 ℃ 이상 정지, 70 ℃ 미만 재출발 허용. 온도값(0 이하·150 ℃ 초과·숫자 아님)이 처음부터 무효면 `unknown`이고 막지 않습니다. 80 ℃ 정지 뒤 값이 무효가 되면 정지는 유지되지만 상태는 `unknown`으로 보입니다.
- LPP 자율주행: headless 재출발 설정이면 대기(비치명), 아니면 잠김입니다.
- 수동 GUI·키보드 조종(11-15부터): 80 ℃ 이상이면 세션을 끝내고(정지 출력, 사유 `ESC over-temperature (>=80 C) in manual control`), 70 ℃ 미만이 될 때까지 새 START를 거부합니다(`START allowed below 70 C`). 키보드 조종으로 지상 주행을 오래 하게 되어 넣었습니다. ROS 시험 `runManualThermalStop`.
- 70~80 ℃는 `derate`로 표시되지만 ERPM 제어기는 출력을 줄이지 않습니다(출력 감쇄는 VIO duty 제어기만).
- 11-12 야지 시험 최고 31.4 ℃. 온도 정지 중에도 정지 출력(핸드브레이크)은 계속 나갑니다.

### 팀 GUI용 VESC 값 (2026-11-16)

VESC가 보내는 값 중 지금까지 제어기가 넘기지 않던 것을 GUI 표시용으로 내보냅니다. 어떤 제어 판단에도 쓰지 않습니다. 첫 상태값이 오기 전에는 `null`입니다.

- **`status_json` 추가 키**(키보드·수동 모드는 GUI가 HTTP `/api/status`로 받음):
  - `input_current_a`: 배터리(입력) 전류 A. 상 전류로 계산한 VESC 추정값입니다(75_300에는 직류 전류 측정 저항이 없음).
  - `input_power_w`: 배터리 전력 W(전압 × 입력 전류, 회생이면 음수).
  - `iq_a`·`id_a`: 토크·자속 전류 A(VESC `avg_iq`·`avg_id`). `iq_a`가 모터 힘을 가장 잘 보여 줍니다. HFI에서는 `id_a`가 0 근처입니다.
  - `duty`: −1~1. 절댓값이 VESC 최대 0.95에 가까우면 그 배터리 전압에서 낼 수 있는 최고 속도입니다.
  - `vd_v`·`vq_v`: d·q축 전압 V(진단용).
  - `esc_mos_temps_c`: MOSFET 온도 3개 각각(`esc_temp_c`는 그중 최고).
  - `charge_drawn_ah`·`charge_regen_ah`·`energy_drawn_wh`·`energy_regen_wh`: VESC를 켠 뒤 누적 사용량. 모터 전류가 1 A를 넘을 때만 세고, VESC를 다시 켜면 0입니다.
  - `tacho`·`tacho_abs`: VESC 타코미터(전기 1회전에 6, 방향 반영·절댓값 누적). HFI 추정 각도로 세므로 핸드브레이크 중에는 멈추고, 거리 환산(극수·기어비·바퀴)은 확인되지 않았습니다.
  - `vesc_controller_id`: VESC 앱 ID(지금 111).
  - `fault_name`: `fault_code`의 펌웨어 5.03 이름(`NONE`, `UNDER_VOLTAGE`, `ABS_OVER_CURRENT`, …, 상태값 전이면 `NO_TELEMETRY`).
  - `telemetry_age_s`: 마지막 VESC 상태값을 받은 뒤 지난 시간.
  - `estimated_speed_mps`·`estimated_speed_verified`: ERPM ÷ `planning.erpm_per_mps`(3721.88)로 계산한 바퀴 속도. 지면 속도가 아니고 환산값도 실측 검증 전이라 `estimated_speed_verified`는 지금 false입니다. 핸드브레이크 중에는 0으로 고정됩니다(`erpm_valid` false).
- **`vesc_trace` 로그 줄**(1초에 1번, `control_trace` 바로 뒤): 자율주행 모드에는 HTTP 상태가 없어 GUI가 제어기 로그를 읽으므로, 거기서 볼 값을 따로 남깁니다. `control_trace`는 GUI·로그 분석 도구가 읽는 형식이라 그대로 두고, 각 줄이 GUI 로그 읽기의 400자 제한 안에 들어가도록 나눴습니다.
  `vesc_trace voltage_v= input_current_a= power_w= iq_a= duty= fault_code= fault= servo_cmd= servo_target= hold_basis= telemetry_age_ms= hold_s=`(`hold_s`는 11-17 추가)
  `servo_cmd`는 실제로 보낸 서보 명령(0~1), `servo_target`은 변화율을 적용하기 전 목표입니다. 둘 다 명령값이며 조향 측정값이 아닙니다(서보 피드백 없음). `telemetry_age_ms`는 상태값이 없으면 −1입니다.
- 이미 있던 값: `raw_erpm`, `current_a`, `voltage_v`, `fault_code`, `esc_temp_c`·`esc_temp_state`, `erpm_valid`, `stop_output`·`stop_output_a`·`stop_hold_basis`, `commanded_servo_position`·`requested_servo_position`, `commanded_motor_erpm`·`target_motor_erpm`, `telemetry_fresh`, `driver_ready`. `control_trace`에는 `esc_temp_c`, `thermal`, `stop`, `stop_a`, `erpm_valid`가 이미 있습니다.
- VESC는 알지만 지금 드라이버가 넘기지 않는 값(드라이버 수정 필요): VESC 명령 타임아웃 상태(응답 마지막 바이트), 가동 시간(재부팅 감지), 펌웨어·하드웨어 이름. 드라이버는 `temp_fet`·`temp_motor`를 읽고도 메시지에 넣지 않아 늘 0입니다(모터 온도 센서는 원래 없음).

### 통신·제어 주기 기록과 정지 유지 경고 (2026-11-17)

11-16 야지에서 Jetson 플래너와 Pi4 제어기 사이 Wi-Fi가 끊겨 명령 만료 정지가 31번 났습니다. 그중 12번은 0.3초 안에 다시 이어진 짧은 끊김이었고, 나머지는 1~7초 끊김이었습니다. 이번 변경은 끊김을 숫자로 남기고, 차량 상태를 더 자주 보내는 것입니다. 멈추고 출발하는 판단과 정지 출력 전류는 바뀌지 않았습니다(잘못된 시각을 받은 조향 범위 서비스가 종료 대신 거절하는 것만 다름).

- **`link_trace` 로그 줄**(`vesc_trace` 다음): `link_trace n= gap_ms= age_min_ms= age_max_ms= expired= tick_max_ms=`. 1초 창마다 계산하지만, 줄은 볼 것이 있는 창(만료, 40 ms 넘는 제어 주기, 명령이 오는 중 0.2초 넘는 공백)에만 찍고 그 밖에는 10초에 1번 찍습니다. 명령이 아예 없는 창(플래너 꺼짐, 또는 끊김이 이어지는 중)은 매초 찍지 않습니다. 끊김의 시작은 `expired=1`로, 전체 길이는 다음 명령이 온 창의 `gap_ms`로 남습니다. GUI가 이 로그를 1초에 약 1 KiB만 읽기 때문입니다. 모든 창은 `status_json`에 있습니다.
  - `n`: 그 1초 동안 받은 LPP 명령 수(플래너는 약 20 Hz).
  - `gap_ms`: 명령이 오지 않은 가장 긴 시간. 1초 창이 닫힐 때까지 이어지는 끊김도 포함합니다. 그래서 몇 초 끊기면 매초 커지다가, 다음 명령이 오면 전체 길이가 찍힙니다. 명령을 한 번도 안 받았으면 0입니다.
  - `age_min_ms`·`age_max_ms`: 명령을 받은 순간 이 노드의 ROS 시각에서 보낸 쪽 stamp를 뺀 값입니다. 네트워크 지연과 보드 간 시계 차이가 섞여 있고, 음수면 보낸 쪽 시계가 앞선 것입니다. 명령이 없으면 `na`입니다.
  - `expired`: LPP로 달리던 중 입력이 끊기거나 무효가 되어 감속 정지·대기가 시작된 횟수.
  - `tick_max_ms`: 20 ms 제어 주기 사이 가장 긴 간격. 크면 네트워크가 아니라 CPU 부하로 이 프로세스가 늦게 돈 것입니다.
- **`status_json` 키**(마지막으로 닫힌 1초 창, 첫 창 전에는 `null`): `link_window_s`, `link_commands`, `link_cmd_max_gap_ms`, `link_cmd_age_min_ms`, `link_cmd_age_max_ms`, `link_expiries`, `control_tick_max_ms`.
- **LPP 명령 구독은 reliable 그대로 둡니다.**
  - best_effort로 바꾸면 Wi-Fi에서 잃은 명령을 다시 보내느라 새 명령이 밀리는 일은 없어집니다.
  - 하지만 플래너가 STOP을 한 번만 보내면 그 STOP을 영영 잃을 수 있습니다. 정지를 잠그는 설정에서는 유효기간 검사도 이 경우를 막지 못합니다.
  - 팀 인터페이스 계약(`nomad_interfaces/README.md`)도 Reliable입니다.
  - 그래서 플래너가 STOP을 매 주기 반복해 보내기로 합의한 뒤에 바꿉니다(11-17 검토).
- **`/nomad/control/vehicle_state`를 10 Hz → 제어 주기(약 50 Hz)로 올렸습니다.**
  - Jetson 플래너는 이 값이 0.3초 넘게 늦으면 출력을 막습니다(`VEHICLE_STATE_STALE`). 10 Hz에서는 연달아 3개만 잃어도 막혔고, 50 Hz에서는 약 15개를 잃어야 막힙니다.
  - 발행은 reliable 그대로 둡니다. 플래너 구독이 reliable이어도 연결되게 하려는 것입니다. 플래너 구독을 best_effort로 바꾸는 것은 플래너 담당 결정입니다.
- **정지 유지 경고**(전류는 바꾸지 않음):
  - `status_json`에 `stop_hold_heat_w`(지금 유지 전류의 모터 구리 손실 추정, 1.5 × 0.0112 Ω × I², 30 A에서 약 15 W)와 `stop_hold_long`(25 A 이상을 10초 이상 유지)을 넣었습니다.
  - 25 A 이상 유지가 10초가 되면 `[HOLD_LONG]` 경고 줄이 나오고, 이어지는 동안 30초마다 다시 나옵니다.
  - `vesc_trace` 끝에 `hold_s=`(지금 유지가 이어진 초)를 붙였습니다.
  - 모터 온도는 재지 않습니다. 정지 유지 중에는 ERPM이 0으로 고정돼 미끄러져도 알 수 없습니다. 그래서 전류를 자동으로 낮추지 않습니다(사용자 결정).
- **입력 시각 처리:**
  - 메시지·서비스로 들어온 시각은 `wire_stamp.hpp`를 거쳐서만 바꿉니다.
    - 서비스 요청 시각은 `stampSeconds()`로 바꾸고, 음수 초나 10억 이상 나노초는 거절합니다.
    - LPP 명령 검사, VESC 상태값, IMU에는 예전과 같은 계산(`rawStampSeconds()`)을 그대로 씁니다. 동작을 바꾸지 않기 위해서입니다(예: 나노초가 잘못된 STOP도 지금처럼 명령을 지움).
  - 예전에는 `~/steering_test_range`에 음수 시각이 오면 `rclcpp::Time`이 예외를 던졌습니다. 콜백 예외는 곧 제어기 종료이므로(`controller_main.cpp`), 이제는 거절만 합니다.
  - 규칙: 메시지·서비스 입력으로 예외를 던지지 않습니다(`rclcpp::Time(stamp)`, JSON `at()`, `stod` 등 금지).
- 시험: 단위 시험 `field_diagnostics`(시각, 1초 창, 정지 유지 경고). ROS 시험은 새 상태 키, 잘못된 시각 거절 뒤에도 제어기가 살아 있는지, 차량 상태 50 Hz, 만료 횟수를 확인합니다.

### 감속 정지 끝 판정·끊김 뒤 재출발·미끄러짐 표시 (2026-11-17, 2차 묶음)

사용자가 항목마다 결정했습니다. 한 것은 1·2·4a·5a, 보류는 3(환산값 실측)·4b(자동 대응)·5b(출력 단계 떼어 내기)입니다.

- **1. 감속 정지 끝 판정:**
  - 감속 정지는 측정 ERPM이 200 이하인 샘플이 **최근 3개 중 2개**(최근 3개의 중앙값이 200 이하) 나와야 끝나고 핸드브레이크로 넘어갑니다(`RawTrialController::kStopRampEndLow`/`kStopRampEndWindow`). 0.6초 상한은 그대로입니다.
  - 11-17에는 '연속 3개'였습니다. 그런데 11-18 지하주차장 경사 주행에서 ERPM이 200 근처에서 오르내리며(444, 122, 348, 338 …) 감속 정지가 중앙값 0.63초로 길어졌고, 2번은 상한(약 0.9초)까지 갔습니다. 그래서 11-18에 바꿨습니다.
  - 11-16 기록에서 끝까지 간 감속 정지 95번 중 57번이, 센서리스 ERPM이 한 샘플만 300 넘게 떨어진 순간 끝났습니다. 그때 차는 VIO 기준 중앙값 0.14 m/s로 아직 움직이고 있었고, 핸드브레이크가 걸리며 덜컹했습니다.
  - 11-17 이전(샘플 하나로 끝냄)보다 보통 1샘플(약 20 ms) 늦어집니다. 한 샘플만 떨어지면 끝나지 않고, 200 근처에서 오르내리면 둘째 낮은 값에서 끝납니다.
- **2. 끊김 뒤 재출발 대기(`planning.link_resume_stable_s`, 0.3 s):**
  - LPP 입력이 끊기거나 무효여서 멈춘 경우에는 새 명령이 이 시간만큼 끊김 없이(명령 사이 0.15초 이하) **도착한** 뒤에 출발합니다. 유효기간이 긴 명령 하나로는 출발하지 않습니다(검토 11-17).
  - 운영자 STOP·RESET, 플래너 STOP(11-20부터는 지우지 않음), 대기에서 새로 시작할 때는 이 대기를 지웁니다.
  - 플래너 STOP으로 멈춘 경우는 지금처럼 다음 명령에 바로 출발합니다.
  - 기다리는 동안 대기 사유는 `LPP link recovering: waiting for 0.3 s of continuous commands`이고, 상태값 `planning_link_pause`가 true입니다.
  - 배포 설정(`measured_speed_control`)은 원래 출발 대기 장치를 거치지 않아, 명령 하나만 와도 바로 출발했습니다. 11-16에는 섰다가 0.1~0.44초 만에 다시 출발한 경우가 6번 있었습니다.
  - 0으로 두면 예전처럼 바로 출발합니다.
- **4a. 미끄러짐·막힘 표시(`slip.enabled`, `slip_monitor.hpp`):**
  - 융합 위치(`/nomad/localization/odometry`, 플래너와 같은 sensor-data QoS)와 위치 상태(`/nomad/localization/status`, READY)를 받아 차체 속도를 구합니다. 차체 속도는 0.5초 위치 변화로 계산합니다.
  - 헛돎·막힘(전진 못 함): 전진 명령 500 ERPM 이상, 평균 iq 15 A 이상인데 차체가 0.03 m/s 미만인 상태가 1초 계속될 때입니다.
    - 측정 ERPM은 조건에 넣지 않습니다. 11-16 헛돎 때 ERPM이 0~1000 사이를 오가서, 평균 ERPM 조건이 있으면 재현 자료에서 한 번도 걸리지 않았습니다.
    - 대신 사건 줄에 평균 ERPM을 남겨서, 헛돎(ERPM 높음)과 막힌 채 밀기(ERPM 0 근처)를 구분할 수 있게 합니다.
    - 우면산 재현 자료에서 오르막 주행 3개에 7번 잡혔고, 내리막에서는 0번이었습니다.
  - 뒤로 밀림: 전진 구동 중이거나 정지 유지 중에, 직전 전진 방향 기준으로 0.05 m/s 넘게 뒤로 가는 상태가 0.3초 계속될 때입니다. 밀린 거리도 냅니다.
    - '전진 방향'은 차체가 바퀴와 같이 움직일 때만 잡습니다(바퀴 속도의 절반 이상, 이전 방향과 60° 안). 전진 명령 중에 미끄러지는 방향을 앞으로 잘못 잡지 않기 위해서입니다.
  - **표시만 하고 출력은 바꾸지 않습니다**(자동 대응 4b는 보류).
  - `[SLIP] … start`와 `[SLIP] … end` 로그 줄을 남기고, 상태값은 `slip_state`(off/ok/stuck/backslide), `slip_off_reason`, `slip_state_s`, `body_speed_mps`, `body_along_mps`, `slip_events`, `slip_last_backslide_m`입니다.
  - 감지를 끄는 경우는 세 가지이고, 이유는 `slip_off_reason`에 나옵니다.
    - 위치가 0.3초 넘게 안 옴(Wi-Fi)
    - 위치 상태가 READY가 아니거나 2.5초 넘게 안 옴
    - 위치가 0.5 m 넘게 튐(VIO 재시작), 또는 위치 창을 다시 채우는 중
    - 1.5 m/s보다 빠른 위치 보정 한 번은 움직임으로 보지 않고 창만 다시 채웁니다.
  - 핸드브레이크가 걸리거나 풀린 직후 0.1초의 ERPM은 쓰지 않습니다.
  - 키보드 조종 모드(다른 ROS 도메인)에서는 위치가 없어서 늘 off입니다. 환산값(3번)이 미검증이라 미끄러짐 '비율'은 내지 않습니다.
- **5a. 출력 경로 시험:**
  - ROS 시험의 가짜 VESC가 받은 모든 모터 명령을 순서대로 기록합니다.
  - 감속 정지 시나리오마다 세 가지 규칙을 검사합니다.
    - 핸드브레이크는 12.5~30 A
    - 0 A는 VESC 고장 때만
    - 자율주행 속도 명령은 0~1000 ERPM
  - 새 시나리오:
    - ERPM이 튀는 감속 정지
    - 끊김 뒤 재출발
    - 정지 유지 중 종료(핸드브레이크가 나가고 구동 명령은 없음)
    - 주행 중 종료(브레이크 전류가 나가고 구동 명령은 없음)
    - 미끄러짐 표시
- 시험: 단위 시험 `slip_monitor`, `raw_commissioning_contract`(감속 정지 끝 3샘플·ERPM 튐), ROS `planning_stop_ramp_ros`.

현재 설정은 이 README와 YAML을 기준으로 확인하세요. 날짜가 붙은 기존 `.ko.txt` 문서는 당시 시험·배포 기록이며 최신 실행 설정과 다를 수 있습니다.

### GUI가 읽는 로그 줄이기 (2026-11-19, 19:43 배포)

팀 GUI는 이 제어기 로그를 1초에 한 번 마지막 1 KiB만 읽고(`board_tools/module_log_tail.cpp`), 한 줄을 400자에서 자릅니다. 그래서 어떤 줄 뒤에 다음 읽기 전까지 1 KiB 넘게 쓰이면 그 줄은 GUI에 안 보입니다. 11-18 밤 강의실 복도 주행(52초, 27890e1)에서는 Pi4 로그 갱신 62번 중 25번에서 앞 줄이 잘렸고, `[STOP_RAMP] end` 5개 중 2개가 빠졌습니다. 크기 비중은 `[RESTART_TRACE]` 32 %(재출발마다 3초 동안 50 Hz, 약 150줄), `control_trace` 22 %, `vesc_trace` 19 %, `[PLANNING_WAIT]` 10 %, `[STOP_RAMP]` 7 %, `link_trace` 1 %였습니다. 이번 변경은 로그만 바꿉니다. 멈춤·출발 판단과 출력 전류는 같습니다.

- **매 주기 줄은 기본으로 끔:** 새 설정 `trace_every_tick`(기본 `false`, `config/commissioning.yaml`). `true`면 11-19 전처럼 `[RESTART_TRACE] t=`·`[STOP_RAMP] t=` 줄도 찍습니다. 전체 로그 파일을 읽는 벤치 분석에서만 켭니다.
- **재출발 끝 요약**(`[RESTART_TRACE] end t= reason=` 뒤, `trace_summary.hpp` `RestartSummary`):
  - `reason`: `window`(`restart_trace_s` 지남), `stopped`(정지 출력), `stop_ramp`(LPP 감속 정지 시작, 뒤는 `[STOP_RAMP]` 줄), `restarted`(다음 재출발이 덮음), `shutdown`(노드 종료).
  - `ticks`: 기록한 제어 주기 수.
  - `reach_s`: 측정 ERPM이 그 순간 명령의 90 %(같은 방향)에 처음 닿은 시각. 못 닿았으면 `na`. 감속 정지 구간은 넣지 않습니다(떨어지는 명령에 닿은 것으로 잘못 셀 수 있어서).
  - `cmd_erpm`: 마지막 명령. `erpm_min`·`erpm_max`: 측정 ERPM 범위. 전진 출발에서 `erpm_min`이 음수면 뒤로 밀린 것입니다.
  - `current_max_a`: `current_a` 크기의 최대. `iq_peak_a`·`iq_peak_t`: `avg_iq` 크기가 가장 컸던 값(부호 포함)과 그 시각. 둘 다 출발 뒤 **두 번째** VESC 상태값부터 셉니다. 그 전 값은 핸드브레이크 유지 전류가 섞입니다(11-18 10° 경사: 30 A 유지 뒤 t=0 30.4 A, t=0.019 29.3 A, t=0.041 14.0 A). 유지 전류는 시작 줄에 있습니다. 그런 값이 없으면 `na`.
  - `telemetry_age_max_ms`: 가장 오래된 VESC 상태값 나이. 없었으면 `na`.
- **감속 정지 끝 요약**(`[STOP_RAMP] end`·`cancelled` 줄 뒤, `StopRampSummary`):
  - `wait_s`: 속도 명령이 끝 속도(200 ERPM)에 닿은 뒤 감속이 끝날 때까지. 0.60이면 측정 속도를 기다리다 시간 상한으로 끝난 것입니다. 닿기 전에 끝나거나 취소되면 `na`.
  - `end_samples`·`end_above`: 그동안 받은 VESC 상태값 수와, 그중 200보다 컸던 수.
  - `erpm_last`: 감속 시작부터 마지막 5개 측정 ERPM(오래된 것부터, 감속을 끝낸 값 포함). 마지막 3개 중 2개가 200 이하면 '최근 3개 중 2개' 규칙으로 끝난 것입니다(명령이 200에 닿기 전에도 적용). 200 근처에서 오르내리는 HFI 값도 여기서 봅니다.
  - `iq_peak_a`: 감속 중 `avg_iq` 크기가 가장 컸던 값(음수 = 제동).
- **줄 순서:** `[STOP_RAMP] end`는 정지 출력·`raw transition`·대기 처리 뒤에 찍습니다. 그 주기에서 뒤에 올 수 있는 것은 1초 줄(`control_trace`·`vesc_trace`·`link_trace`)뿐입니다. GUI는 각 읽기의 마지막 1 KiB를 보므로, 뒤에 쓰이는 바이트가 적을수록 남습니다. 그 주기의 `[PLANNING_WAIT]`(약 420바이트)은 2초 미룹니다(끝 줄에 같은 `state=`·`reason=`이 있음). 그때도 대기 중이면 그때의 자세한 줄이 나옵니다. 고장·발행자 중복처럼 잠기는 정지는 미루지 않고 바로 찍습니다(그 뒤로 다시 찍히지 않고, 연결 수·고장 값은 끝 줄에 없음).
- **`[PLANNING_WAIT]`:** 내용(사유, 상태, 상태값 문제, 드라이버 연결 수, 고장 코드, 리셋 필요)이 바뀌면 찍되 1초에 1번까지만 찍습니다. 2초 넘게 대기 기록이 없다가 새 대기가 시작돼도 찍고, 같은 내용이 이어지면 10초마다 찍습니다. 예전에는 대기 중 매초 약 400자였습니다. 나이·전류·전압은 바뀌어도 새로 찍지 않습니다. 시계는 ROS 시계에서 단조 시계로 바꿨습니다. 이 줄은 앞머리까지 약 450자라 GUI에서는 끝이 잘립니다(예전과 같음).
- **종료할 때:** 노드가 꺼지면 진행 중이던 재출발 기록과 감속 정지의 끝 줄을 정지 출력 뒤에 찍습니다(`reason=shutdown`, `reason=node shutting down`).
- **크기(검토 계산, 실차 미확인):** LPP 주행 중 `control_trace`와 `vesc_trace`가 1초에 약 560바이트입니다. 정지 한 번은 begin 약 200, 유지 약 125, `raw transition` 약 146, end 약 250바이트입니다. 예전에는 end 뒤에 `[PLANNING_WAIT]`까지 약 560바이트가 더 쓰였고, 같은 읽기 안에 1초 줄 두 개가 오면 end가 빠졌습니다(검토 모의 약 50 %). 이제 보통은 end 뒤에 1초 줄 두 개와 `link_trace`(약 127)가 와도 약 930바이트라 1 KiB 안에 남습니다. 다만 0.6초 상한으로 끝나 유지 줄(약 123)이 뒤따르거나, 같은 1초 안에 다시 출발해 `[RESTART_TRACE]` 시작 줄(약 160)까지 겹치면 약 1.1 KiB가 되어 아직 빠질 수 있습니다(검토 계산). 정지가 잦으면(2~3초마다) 평균이 1 KiB/s를 넘어, 그 앞의 begin·유지·재출발 끝 줄도 빠질 수 있습니다. 빠짐 없는 기록은 Pi4의 전체 로그 파일에 있습니다. GUI 쪽 읽기 한도(`module_log_tail.cpp`의 1 KiB)를 늘리는 것이 근본 해결입니다.
- 시험: 단위 시험 `field_diagnostics`(요약 값, 감속을 끝낸 샘플, 시간 상한 끝, 유지 전류 제외, 반복 규칙), ROS `planning_stop_ramp_ros`(매 주기 줄 없음, 끝 줄의 `erpm_last`에 규칙을 만족한 샘플, 끝 줄 주기에 `[PLANNING_WAIT]` 없음(풀리는 대기), VESC 고장으로 끝난 감속은 대기 줄을 바로 찍음, 재출발이 `reason=stop_ramp`로 끝남, 주행 중 종료에 `reason=shutdown`, `trace_every_tick: true`면 매 주기 줄이 다시 나옴). ROS 시험은 rcutils 로그 출력기를 감싸 줄을 모읍니다(화면 출력은 그대로). 독립 검토(에이전트 19개)에서 확정된 12건을 반영했습니다.

### 배포 설정·팀 GUI 계약 시험과 `[CONFIG]` 경고 (2026-11-19)

차 동작은 바뀌지 않습니다. 시험이 늘었고, 시작할 때 경고 한 줄이 생길 수 있습니다.

- **`[CONFIG]` 경고:** 제어기가 시작할 때, 설정 파일(또는 `-p`)에 있지만 이 노드가 읽지 않은 이름을 한 줄로 알립니다. 예: `[CONFIG] set but not used (typo, or not used in this mode; default kept): planning.stop_ramp_sec`. 오타이거나 이 모드에서 쓰지 않는 설정이고, 그 값은 기본값으로 남습니다. commission 모드에서는 보정 모드 전용 설정 13개(`current_limit_a`, `manual_timeout_s`, `trial_duration_s` 등, 이 모드는 `commissioning.<이름>`을 읽음)도 여기에 나옵니다. Pi4의 설정 파일은 보드에서 손으로 고치는 일이 있어서, 오타가 소리 없이 기본값으로 돌던 것을 보이게 합니다. 지금 배포 설정 4개로는 나오지 않습니다(아래 시험).
- **배포 설정 시험**(ROS `deployed_config_ros`, `controller_ros_tests --deployed-config-only`):
  - Pi4 `pi4_output_only.sh`와 같은 순서로 `commissioning.yaml` → `planning.yaml` → `planning_headless.yaml` → `nomad_vehicle_ready.yaml`을 rcl의 YAML 해석기로 읽습니다(뒤 파일이 앞을 덮음). 그 값으로 노드를 띄웁니다. 토픽만 시험용으로 바꾸므로, 시험이 VESC 드라이버 토픽으로 명령을 보내는 일은 없습니다.
  - 설정 이름마다 노드가 실제로 읽는지 확인합니다. `[CONFIG]`에 배포 설정 이름이 없어야 합니다. 일부러 틀린 이름은 `[CONFIG]`에 그대로 나와야 합니다.
  - VESC 토픽 설정 8개를 이름과 값의 짝으로 확인합니다. 각각 드라이버 이름공간 `/nomad_bench`(`pi4_output_only.sh`의 `-r __ns:=/nomad_bench`) 아래의 맞는 드라이버 토픽이어야 합니다. 그래서 속도↔듀티처럼 서로 바뀐 짝이나 틀린 설정 이름도 잡습니다.
  - 드라이버 조향 설정(`vesc_steering_test.yaml`)의 서보 범위가 제어기 서보 범위를 덮는지 확인합니다.
  - `status_json`에 배포 값이 그대로 나오는지 확인합니다: raw 단위, 설정 확인됨, 최고 1850·후진 800 ERPM, 서보 중앙 0.5, 감속 0.3초, 끊김 뒤 대기 0.3초, 미끄러짐 표시 켬.
  - Pi4 운영 전용 드라이버 파일 `commission_session/vesc_commission.yaml`은 저장소에 없어서 확인하지 못합니다.
- **팀 GUI 계약 시험:** 팀 GUI 원본(`NOMAD_실물시험앱_20261106/laptop_monitor`)이 읽는 것을 시험 코드의 목록(`kGuiLogLines`, `kGuiStatusKeys`)으로 고정합니다. GUI를 바꿀 때 이 목록도 같이 고칩니다.
  - 자율주행 로그 줄(ROS `planning_stop_ramp_ros`의 모든 장면): `control_trace`·`vesc_trace`·`link_trace`의 키가 GUI가 읽는 키와 정확히 같아야 합니다. 값 안에 `단어=`가 섞이면 GUI가 앞 키 값을 잘못 자르므로 이것도 잡습니다. 숫자 값은 숫자여야 하고(`link_trace`의 나이는 `na` 허용), 앞머리 포함 400자 안이어야 합니다(보드 꼬리 읽기가 400자에서 자름). `[SLIP]` 줄은 GUI의 start·end 정규식에 맞아야 합니다.
  - 키보드 조종 HTTP `/api/status`: `keyboard_drive.py`, `vesc_status.py`, `drive_record_dialog.py`, `scripts/ensure_gui.sh`가 읽는 키 68개가 `status_json`에 모두 있어야 합니다(배포 설정 시험에서 확인). `connected`·`token`·`server_time_s`는 `gui_server`·`gui_http`가 붙이므로 목록에서 뺍니다. 키보드 창이 조종을 허용하는 시간 값 규칙(`keyboard_drive.control_timing`: Wi-Fi 프로필이면 2초·1.5초·1.5초·1500 ms 정확히)도 같은 식으로 확인합니다. `ground_drive_supported`는 false여야 합니다.
  - `control_trace`의 `erpm_valid`·`reset_required`는 GUI가 `0`/`1`만 받으므로 정확히 그 값인지 봅니다.
- **가끔 실패하던 시험 고침:** `controller_ros_integration`의 'stale IMU stops owned bench output'과 'excessive tilt stops bench output'은 상태값을 고정 시간 뒤 한 번만 읽어서, PC가 바쁘면 가끔 실패했습니다. 상태값이 올 때까지 최대 1초 기다리도록 바꿨습니다. 확인하는 내용은 같습니다.
- 독립 검토(에이전트 16개)에서 확정된 7건을 반영했습니다: 토픽을 이름·값 짝으로 확인, HTTP 키 5개 추가, 키보드 시간 값 규칙, `0`/`1` 값, commission 모드의 보정 모드 설정 경고.

### 통신 상태를 한 클래스로 (2026-11-19, 리팩토링)

차 동작, 로그 줄, `status_json`, 설정 이름은 바뀌지 않습니다. 제어 노드 곳곳에 흩어져 있던 LPP 통신 상태를 `include/nomad_control_cpp/link_monitor.hpp`의 `LinkMonitor` 하나로 모았습니다.

- **모은 것:**
  - 1초 통신·제어 주기 창(`link_stats.hpp`의 `LinkStats`를 품음)
  - 마지막으로 받은 명령 시각과 '명령이 이어진 시간'(간격 0.15초 넘으면 다시 셈)
  - 끊김 표시와 끊김 뒤 재출발 대기(`planning.link_resume_stable_s`)
  - `link_trace`를 찍을지 정하는 규칙과 줄 내용
- **노드가 부르는 곳:**
  - 명령 수신: `onCommand`, 받아들인 명령: `onAccepted`
  - 끊김·무효 입력: `onLost`. 끊김이 아닌 정지(운영자·전역 STOP, RESET, 재개, 시작, 감속 취소): `clearPause`. 플래너 STOP은 11-19에는 `clearPause`였고 11-20부터는 `onPlannerStop`(대기를 지우지 않고 `[LPP_BACK]`용으로 기록만). 거부된 이동 명령: `onRefused`, RESET: `endOutage`
  - 매 제어 주기: `onControlTick`, 매 플래닝 주기: `updateFlow`, 재출발 판단: `resumeWaiting`
  - 상태값: `roll`/`last`, 로그: `traceDue`/`traceFields`
- **시험:** 지금까지 ROS 시험으로만 확인하던 재출발 대기를 단위 시험 `field_diagnostics`로도 확인합니다.
  - 끊김 뒤 늦은 명령 하나로는 출발하지 않음
  - 0.3초 이어진 명령으로 대기가 끝남
  - 0.15초 넘는 간격이나 살아 있는 명령이 없으면 다시 셈
  - 정지·리셋은 대기를 풂, 0초 설정이면 기다리지 않음
  - `link_trace` 규칙과 글자 형식

### 상태값·1초 로그 줄 만드는 부분 나누기 (2026-11-19, 리팩토링)

`publishStatus`(약 230줄)를 역할별 함수로 나눴습니다. 코드 줄은 옮기기만 했습니다.

- **나눈 함수:**
  - `publishTextStatus`: `~/status` 텍스트 줄
  - `baseStatusJson`: 상태·모드·안정·키보드 시간 값·raw/보정 한계
  - `addVescStatus`: ESC 온도와 VESC 값
  - `addStopOutputStatus`: 정지 출력과 오래 유지 경고 `[HOLD_LONG]`
  - `addLinkStatus`: 통신 창
  - `addPlanningStatus`: 플래닝·감속 정지·미끄러짐
  - `logStatusTraces`: `control_trace`·`vesc_trace`·`link_trace`
- **같은지 확인:** 바꾸기 전후의 실제 `controller_node`를 배포 설정 4개로 띄워 비교했습니다. 결과는 다음과 같습니다.
  - `status_json` 키 147개, 값의 형식, 시간에 따라 변하는 것을 뺀 값이 모두 같음
  - 텍스트 상태 줄이 같음
  - `control_trace` 수가 같음
  - JSON 객체는 키를 정렬해 내보내므로, 어느 함수가 키를 넣는지는 결과에 영향이 없습니다.
- **의도한 변경 하나:** `control_trace`·`vesc_trace`의 1초 간격을 ROS 시계 대신 단조 시계로 잽니다(`trace_summary.hpp` `PeriodGate`).
  - 전에는 `RCLCPP_INFO_THROTTLE`이 'ROS 시각 ≥ 마지막 + 1초'일 때만 찍었습니다. 그래서 Pi4 시계를 뒤로 맞추면 그 차이만큼 두 줄이 안 나와, GUI의 제어기 표시가 '미수신'이 될 수 있었습니다(Pi4 시계는 며칠씩 틀려 손으로 맞춘 적이 있음).
  - 평소에는 같습니다. 첫 줄은 바로 찍고, 그 뒤 1초마다 두 줄을 함께 찍습니다.
- **시험:** 단위 시험 `field_diagnostics`(`PeriodGate`), ROS `deployed_config_ros`(두 줄이 1초에 한 번).

### 감속 시작 판단·끊김 뒤 플래너 STOP·끊김 기록 줄 (2026-11-20, 11-19 저녁 주행 분석)

11-19 저녁 11번 주행(GUI 기록의 VIO와 Pi4 `controller.log`)을 분석한 뒤 사용자가 고른 세 가지입니다. 감속 중 실제 제동과 평지 유지 전류가 큰 문제는 주행 시험이 필요해 보류했습니다. 독립 검토(네 관점, 지적마다 반박 확인)를 거쳐 ②와 ③은 처음 안을 고쳤습니다.

- **① 감속 시작을 최근 측정값 3개로 판단(`RawTrialController::kStopRampStartWindow`, `recentMaxErpm()`):**
  - 감속 정지는 '명령'과 '측정 ERPM' 중 작은 값에서 시작합니다. 이 값이 200 이하이면 감속 없이 바로 정지 출력(핸드브레이크)으로 갑니다.
  - 지금까지는 가장 최근 측정값 하나만 봤습니다. 11-19에는 HFI가 한 번 낮게 읽어서(820/613/−52) 88번 중 2번이 0.23~0.37 m/s로 달리는 중에 30 A 핸드브레이크를 바로 잡았습니다. 그날 가장 센 정지였습니다.
  - 이제 최근 3개(가장 최근 값에서 0.1초 안) 중 가장 큰 값을 씁니다. 셋이 모두 200 이하일 때만(막힘·뒤로 밀림) 바로 정지합니다. 시계가 뒤로 맞춰지면(rebase) 그 뒤 값만 셉니다.
  - 셋 중 하나만 높으면 감속은 시작합니다. 다만 시작 판단에 쓴 가장 최근 값도 끝 판정(최근 3개 중 2개가 200 이하)에 들어가므로, 다음 한두 샘플에 또 200 이하가 나오면 20~40 ms 만에 끝날 수 있습니다.
  - 효과는 '바로 잡지 않고 잠깐이라도 감속한다'까지입니다. 감속은 거의 제동하지 않고(11-19 VIO 감속도 약 0.2 m/s²), 유지 전류도 그대로입니다(11-19의 그 정지는 직전 정지와 같은 자리라 `same_place` 30 A). 더 부드러운 정지는 보류한 '감속 중 실제 제동'이 필요합니다.
  - `[STOP_RAMP] begin`의 `measured_erpm`은 여전히 가장 최근 값 하나라서, 이제 `ramp_from_erpm`이 그보다 클 수 있습니다(예: `measured_erpm=-52 ramp_from_erpm=613`).
- **② 끊김 뒤에는 플래너 STOP이 재출발 대기를 지우지 않음(`LinkMonitor::onPlannerStop`):**
  - 끊김 뒤에는 0.3초 동안 명령이 끊김 없이 와야 다시 출발합니다(11-17 2번). 지금까지는 플래너 STOP이 오면 이 대기가 지워졌습니다.
  - 그런데 끊긴 동안 플래너도 STOP을 보냅니다(`VEHICLE_STATE_STALE`, WAIT). 이 STOP들이 연결이 돌아올 때 도착해서 대기를 지웠습니다.
    - 11-19 끊김 정지 8번 중 6번이 명령이 돌아온 지 16~101 ms 만에 출발했습니다.
    - 그중 3번은 0.6초 안에 다시 섰습니다.
  - 처음 안은 'STOP의 나이가 `valid_for` 안이면 지움'이었습니다. 하지만 검토에서 막지 못한다는 것이 확인됐습니다. 끊긴 동안 보낸 STOP 중 마지막 것(연결이 돌아오기 직전 한 주기 안에 보낸 것)과 연결이 돌아온 직후 보낸 STALE STOP은 제때 도착하기 때문입니다. 끊김 초반에 보낸 STOP은 끊김 길이만큼 늦게 오거나, 구독 깊이 1 때문에 덮입니다.
  - 그래서 끊김 뒤에는 플래너 STOP이 대기를 아예 지우지 않습니다.
  - 대가: '끊김 → 플래너 STOP → 출발'이 이어지면 출발이 0.3초 연속 명령을 더 기다립니다. 끊김이 없었던 플래너 STOP 정지는 예전처럼 다음 명령에 바로 출발합니다.
  - 운영자 STOP·RESET, 재출발, 새 시작, 감속 취소는 예전처럼 대기를 지웁니다.
- **③ 끊김마다 두 줄: 끊길 때 `[LPP_LOST]`, 다시 받을 때 `[LPP_BACK]`:**
  - `[LPP_LOST] since_ms= age_ms= valid_ms= why=`: 입력을 잃은 순간입니다.
    - `since_ms`: 마지막으로 받아들인 명령 뒤 지난 시간
    - `age_ms`, `valid_ms`: 그 명령이 도착했을 때의 나이와 유효기간
    - `why`: 플래닝 guard의 사유(`planning command expired`, `invalid/stale planning command`, `planning speed outside configured range` 등). 띄어쓰기가 있어 줄 끝에 둡니다.
  - `[LPP_BACK] gap_ms= first_ms= refused= refused_age_max_ms= stops= stop_age_max_ms= stop_valid_ms= age_ms=`: 끊김 뒤 처음 받아들인 이동 명령에서 한 번 찍힙니다. 운영자 RESET을 하면 기록을 버리므로, 다음 주행의 첫 명령에 지난 기록이 붙지 않습니다.
    - `gap_ms`: 끊기기 전 마지막으로 받아들인 이동 명령부터 지금까지. 연결이 돌아온 뒤 플래너가 STOP을 유지한 시간도 들어갑니다.
    - `first_ms`: 같은 기준으로, 끊김 뒤 처음 도착한 메시지(STOP·늦은 명령 포함)까지. 연결이 실제로 돌아온 시점입니다. `gap_ms`보다 훨씬 작으면 그 뒤로는 플래너가 STOP을 유지한 것입니다.
    - `refused`, `refused_age_max_ms`: 그사이 늦거나 잘못돼 거부된 이동 명령 수와 그중 가장 늦은 나이. 구독 깊이가 1이라 한꺼번에 몰려 오면 일부는 콜백까지 오지 않으므로, 수는 최소값입니다.
    - `stops`, `stop_age_max_ms`, `stop_valid_ms`: 그사이 받은 플래너 STOP 수, 가장 늦은 나이, 마지막 STOP의 `valid_for`. 플래너가 STOP에 넣는 `valid_for`는 아직 모릅니다.
    - `age_ms`: 이번 명령의 나이. 다시 받은 첫 명령 자체가 늦었는지는 이 값으로 봅니다.
  - **읽는 법:**
    - 늦게 온 명령은 만료 '뒤에' 도착합니다(새 명령은 만료된 명령보다 늘 젊음). 그래서 원인은 `[LPP_BACK]`에서 봅니다.
    - `refused`나 `stops`가 있고 그 나이가 `valid_ms`보다 크면 늦게 도착한 것(Wi-Fi 지연·몰림)입니다.
    - 둘 다 0이어도 `[LPP_BACK]`의 `age_ms`가 평소(`[LPP_LOST]`의 `age_ms`, 보통 몇 ms)보다 훨씬 크고 `valid_ms`에 가까우면, 다시 받은 첫 명령 자체가 늦게 온 것(Wi-Fi 지연)입니다.
    - 둘 다 0이고 `age_ms`도 평소와 같으면 그사이 콜백까지 온 것이 없는 것입니다. 플래너가 건너뛰었거나, 링크가 아예 끊겼거나, 몰려 와서 마지막 것만 받은 경우이고, 구독 깊이가 1이라 이 셋은 이 줄만으로 구분되지 않습니다.
  - `[STOP_RAMP] cancelled` 줄 끝 요약 앞에 `cmd_age_ms=`(감속을 취소한 새 명령의 도착 나이)를 더했습니다.
  - 팀 GUI는 이 줄들을 해석하지 않습니다(Pi4 `controller.log`에서 봄, GUI가 읽는 줄 4종은 그대로). 다만 GUI가 1초에 읽는 1 KiB 예산은 씁니다.
    - 만료 정지 1번마다 `[LPP_LOST]` 약 140 B가 `[STOP_RAMP] begin`과 같은 주기에 쓰입니다.
    - 출발 3초 안의 만료라 `[RESTART_TRACE] end`까지 겹치면, 검토 모의에서 GUI가 놓치는 `control_trace`가 18 %에서 27~33 %로 늘었습니다. 감속·재출발 줄은 그대로였습니다.
    - `[LPP_BACK]`(약 190 B)은 끊김 1번에 한 번입니다.
- **시험:**
  - 단위 `raw_commissioning_contract`: 820/613/−52(노드 순서 포함), 150/640/90, 0.1초보다 오래된 값, 시계 되돌림 뒤
  - 단위 `field_diagnostics`: 플래너 STOP이 대기를 지우지 않음, `[LPP_LOST]`·`[LPP_BACK]` 필드
  - ROS `planning_stop_ramp_ros`: 늦은 STOP·제때 온 STOP 모두 대기 유지, `[LPP_BACK]`(늦은 이동 명령·STOP 2개), 만료 줄, 취소 줄, 막힌 차에 감속 시작 줄 없음
  - ROS `controller_ros_integration`: 잘못된 이동 명령의 `[LPP_LOST]`
- **끊김 자체:** 11-19 밤에는 감속 84번 중 44번이 명령 만료였습니다. 원인은 Jetson→Pi4 Wi-Fi 전달입니다(플래너는 제때 보냈고 Pi4 주기도 정상). 근본 해결은 유선 연결입니다. 데이터로 본 플래너 명령 유효시간은 아직 약 0.15초였습니다(11-16에 0.3초로 합의, 남은 일 4).

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
```

이 검사는 실제 VESC·Pi 연동 시험을 대신하지 않습니다. 11-16부터 `-Wall -Wextra`가 켜져 있고 경고 0개가 기준입니다(새 경고가 나오면 고치고 커밋).

**시험 라벨(2026-11-19):** 모든 시험에 라벨이 있습니다. ROS 빌드 기준 27개입니다.
- `unit`(19개): ROS 없는 C++ 시험과 명령줄 시험. **Pi4 빠른 시험은 `ctest -L unit`**입니다. 예전의 `ctest -E "_ros$|controller_ros_integration|module_pause_auto_resume"`와 같은 19개입니다.
- `ros`(5개): 노드 단위 ROS 시험. PC에서 약 3분 걸립니다(`controller_ros_integration`, `planning_stop_ramp_ros`, `deployed_config_ros`, `module_pause_auto_resume`, `rpm_running_floor_ros`). 시험마다 자체 DDS domain(226, 227, 229, 230, 232)과 `ROS_LOCALHOST_ONLY=1`로 팀 그래프와 섞이지 않습니다.
- `script`(3개, `node`가 있으면 4개): 저장소의 실행 스크립트·도구 시험. `linux_launcher_contract`, `lpp_entry_contract`, `control_trial_recorder`, 그리고 `gui_manual_browser_contract`(Node)입니다. 앞의 셋은 11-19까지 손으로만 돌리던 Python 시험입니다. Pi4는 배포 때 `scripts/`를 Pi4 것 그대로 두므로 PC에서만 돌립니다.

ROS 환경에서는 `colcon test`(또는 빌드 폴더에서 `ctest`)로 27개 시험이 돕니다(ROS 통합시험은 약 100초, 제한 150초).

**Pi4 컴파일러(GCC 11) 경고를 PC에서 미리 보기(2026-11-19):** `bash src/nomad_control_cpp/test/gcc11_check.sh`. PC의 GCC 13은 11-17 Pi4에서만 나온 경고(집합 초기화 목록, f7d4382)를 놓쳤습니다.
- `g++-11`(우분투 24.04: `sudo apt-get install g++-11`)로 Release 빌드합니다. 오프라인 빌드와 `unit` 시험을 돌리고, ROS와 `vesc_msgs`·`nomad_interfaces`가 든 작업공간을 source해 두었으면 ROS 빌드도 합니다.
- 경고가 하나라도 있으면 실패합니다. 임시 폴더에서 빌드하므로 다른 것은 바꾸지 않습니다.
- 11-19 결과: 오프라인·ROS 모두 경고 0, `unit` 19/19 통과.
- PC의 ROS는 Jazzy라 Humble API 차이까지는 잡지 못합니다. 그것은 Pi4 빌드에서 확인합니다. `rpm_running_floor_ros`는 11-15에 제어기에 없던 100 ERPM 요청 데드존(`planning.stop_request_erpm`) 검사 2개를 빼서 이제 통과합니다(그 전 기록의 '의도된 실패 2건'이 이것). 'stale IMU' 받침대 시험은 타이밍 때문에 가끔 실패했으나 11-19에 상태값 대기로 고쳤습니다. 'stale telemetry cannot save measurement'도 PC 부하 때 한 번 실패해 11-15에 대기를 0.6초로 늘렸습니다.

## 작업 내역 (2026-11-11 ~ 11-13, BAE)

e173e9f까지는 저장소에서 Pi4 `~/nomad_cpp_ws`로 배포했습니다(실행 파일 sha256 `f03e70a3…`). 그 뒤 Pi4에서 직접 온도 보호(11-12 야지 시험 전)와 정지 출력(11-12 저녁)을 넣어 21:33에 빌드했고(sha256 `c4a86cf9…`, 백업 `log/stop_brake_20261112`, `log/handbrake_20261112`), 그 소스를 아래 11-13 커밋으로 옮겼습니다. 07668df까지는 Pi4 설치본과 동작이 같고 문구·`status_json` 정지 필드·스타일만 다릅니다. 11-13 17:24에 f6a662b와 수동 GUI VESC 점검 수정을 Pi4에 배포했습니다(실행 파일 sha256 `d65b65c9…`, Release 빌드 5분, Pi4 시험 17/18 통과·실패는 의도된 `rpm_running_floor_ros` 2건, 백업 `release_backups/feat_control_f6a662b_pre_20261113_172018`). 11-13 22:52에 472ad91·1afcbb1(감속 정지)을 Pi4에 배포했습니다(실행 파일 sha256 `369e3700…`, Release 빌드 6분 9초·경고 0, 사용자 요청으로 빠른 단위시험 15/15만 실행하고 ROS 시험은 PC에서만 확인, 백업 `release_backups/feat_control_1afcbb1_pre_20261113_224455`; 감속만 끄려면 설치본 `install/nomad_control_cpp/share/nomad_control_cpp/config/planning_headless.yaml`의 `planning.stop_ramp_s`를 0으로, 다시 빌드 불필요). Pi4의 실행 스크립트(`scripts/*.sh`, `desktop/`)는 Pi4 것을 그대로 두었습니다(저장소 판은 경로 일반화판, Pi4 미검증). 11-15 14시 20분쯤 GUI 팀원이 그 Pi4 소스(1afcbb1)에 수동 후진을 넣어 Pi4에서 빌드했습니다(실행 파일 sha256 `915e9894…`, Pi4 빠른 시험 7/7, 드라이버 yaml `speed_min` −800; 그 변경분을 04b3886 위에 얹은 것이 11cfb8b). 11-15 17:01(실제 시각, Pi4 시계는 약 5.6일 늦음)에 1d47cf0을 Pi4에 배포했습니다(실행 파일 sha256 `d2bbe876…`, 시계가 틀려 예전 빌드 폴더를 백업으로 옮기고 새로 Release 빌드 7분 36초·컴파일 경고 0(시계 차이 경고만), Pi4 빠른 단위시험 16/16·후진 시험 94/94, ROS 시험은 PC에서 확인, 백업 `release_backups/feat_control_1d47cf0_pre_20261115_165157`, 드라이버 yaml `speed_min` −800 그대로). 04b3886의 감속 정지 수정(0.6초 대기 등)도 이때 처음 Pi4에 들어갔습니다. 같은 날 18:33에 팀원이 Pi4 src에서 `manual_servo_rate_s`를 1.0으로 바꿨고(빌드 안 함, e076d48로 커밋), 20:32에 e076d48(445cfef 포함)을 배포했습니다(실행 파일 sha256 `68827bc0…`, Pi4 시계가 맞은 상태에서 새로 Release 빌드 6분 57초·경고 0, Pi4 빠른 단위시험 16/16, 백업 `release_backups/feat_control_e076d48_pre_20261115_202341`). 11-16 16:40에 4e64c7e를 배포했습니다(e076d48 + 동작을 바꾸지 않는 리팩토링·설정 범위 오류 문구·컴파일 경고 옵션, 서보 표시는 되돌려 빠짐, 실행 파일 sha256 `92523911…`, 배포 전 Pi4 소스는 e076d48과 같았음, 새로 Release 빌드 7분 7초·경고 0, Pi4 빠른 단위시험 16/16, ROS 시험은 PC에서 확인, 백업 `release_backups/feat_control_4e64c7e_pre_20261116_163143`, 드라이버 yaml `speed_min` −800 그대로). 11-16 22:44에 515c704를 배포했습니다(4e64c7e + 팀 GUI용 VESC 표시 값, 제어 판단은 같음, 실행 파일 sha256 `c515287e…`, 배포 전 Pi4 소스는 4e64c7e와 같았음, 새로 Release 빌드 6분 52초·경고 0, Pi4 빠른 단위시험 17/17(새 `vesc_telemetry_display` 포함), ROS 시험은 PC에서 확인, 백업 `release_backups/feat_control_515c704_pre_20261116_223622`, 드라이버 yaml `speed_min` −800 그대로). 이날 저녁 Pi4는 학교 Wi-Fi에서 `lpp.local` = 192.168.0.10이었습니다. 11-17 13:15에 f6f4946을 배포했습니다(현장 진단 기록·차량 상태 50 Hz·정지 유지 경고, 유지 전류와 멈춤·출발 판단은 같음, 실행 파일 sha256 `53db03f2…`, 배포 전 Pi4 소스는 515c704와 같았음, 새로 Release 빌드 6분 43초·경고 0, Pi4 빠른 단위시험 18/18(새 `field_diagnostics` 포함), ROS 시험 22/22는 PC에서 확인, 백업 `release_backups/feat_control_f6f4946_pre_20261117_130757`, 드라이버 yaml `speed_min` −800 그대로, Pi4 주소 192.168.0.10). 11-17 13:37에 58c83fc를 배포했습니다(명령이 없는 창은 `link_trace`를 매초 찍지 않음, 로그만 바뀜, 실행 파일 sha256 `afde4fa8…`, 배포 전 Pi4 소스는 f6f4946과 같았음, 새로 Release 빌드 6분 44초·경고 0, Pi4 빠른 단위시험 18/18, 백업 `release_backups/feat_control_58c83fc_pre_20261117_133023`). 11-17 16:22에 27890e1을 배포했습니다(2차 묶음: 감속 정지 끝 3샘플, 끊김 뒤 재출발 0.3초 대기, 미끄러짐·막힘 표시, 출력 경로 시험, 실행 파일 sha256 `9f5e479e…`, 배포 전 Pi4 소스는 58c83fc와 같았음, 새로 Release 빌드 7분 20초·경고 1(StopRamp 초기화 목록, 동작 무관, f7d4382에서 고침), Pi4 빠른 단위시험 19/19(새 `slip_monitor` 포함), ROS 시험 23/23은 PC에서 확인, 백업 `release_backups/feat_control_27890e1_pre_20261117_161506`). 11-19 10:30에 f5caad8을 배포했습니다(감속 정지 끝 '최근 3개 중 2개' + f7d4382 경고 수정, 실행 파일 sha256 `c55f03b8…`, 배포 전 Pi4 소스는 27890e1과 같았음, 새로 Release 빌드 7분 25초·경고 0, Pi4 빠른 단위시험 19/19, ROS 시험은 PC에서 확인, 백업 `release_backups/feat_control_f5caad8_pre_20261119_102208`). 11-19 19:43에 4b8af61을 배포했습니다(1·2·3·4·6순위: GUI가 읽는 로그 줄이기 b2d006a, 배포 설정·팀 GUI 계약 시험과 `[CONFIG]` 경고 a3f6daf, 통신 상태 `LinkMonitor` a9598e1, `publishStatus` 나누기와 1초 로그 줄 단조 시계 64505f5, 시험 라벨 4b8af61. 실행 파일 sha256 `11519bd9…`, 배포 전 Pi4 소스·설치본 설정은 f5caad8과 같았음, Pi4는 켠 직후 시계가 11-09로 늦었다가 1분 안에 NTP로 맞춰진 뒤 진행, 새로 Release 빌드 7분 24초·경고 0, Pi4 빠른 시험 `ctest -L unit` 19/19, ROS 시험 27/27과 GCC 11 확인은 PC에서, 백업 `release_backups/feat_control_4b8af61_pre_20261119_193549`, 드라이버 yaml `speed_min` −800 그대로). 배포 전에는 Pi4 소스를 가져와 마지막 배포 커밋과 비교하세요(Pi4 시계가 며칠씩 틀려 파일 시각은 믿을 수 없음).

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
| 04b3886 | (11-14) 2차 리뷰 반영: 200 ERPM 유지 최대 0.6초(가파른 내리막에서 브레이크 20 A로 넘기지 않게), 명령이 200에 닿을 때 감속 샘플을 앞 구간으로도 판정(근거 2개 이상일 때만, 내리막 경계 사각지대 12~20°에서 2.3~3.8 cm로 예전보다 좁게), 시험 보강(대기 시간·취소 뒤 새 기록), 로그 순서 문서, 472ad91·1afcbb1 Pi4 배포 기록(`369e3700…`) |
| 11cfb8b | (11-15) 팀원이 Pi4에서 직접 넣은 GUI 수동 후진을 그대로 옮김(`manual_reverse_max_erpm` 800, 드라이버 `speed_min` −800, `test_manual_reverse.cpp`). 측정 속도만으로 방향 전환을 판단하는 문제가 있음 |
| ba0d3e5 | (11-15) 방향 전환은 운영자가 반대 방향을 눌렀을 때만(최대 0.5초 대기), 같은 방향 명령은 뒤로 밀림·속도 튐과 상관없이 계속 구동, `status_json` `manual_reversal_wait`, 시험 보강(밀림·튐·대기 상한·MOTOR 시험, 바퀴 걸림 감시 켬), 문서 |
| 1d47cf0 | (11-15) 2차 리뷰 반영: 브레이크가 지면(옛 방향 속도가 최저값보다 100 ERPM 넘게 오름) 대기를 바로 끝냄, `[REVERSAL]` 로그, 시험 보강(멈춘 바퀴 반전·후진 바퀴 걸림·STOP·새 세션·−800 경계·대기 상한 한 틱, ROS 시험 `runManualReverse`: 후진 명령 발행·상태 필드·반전 대기), `commissioning.yaml` 주석, 출처·문구 정정 |
| feb3dd3 | (11-15) 1d47cf0 Pi4 배포 기록(`d2bbe876…`) |
| 445cfef | (11-15) 수동 조종에도 ESC 80 ℃ 정지(세션 종료, 70 ℃ 미만까지 START 거부), RESET 때 과전류 피크 기록 초기화(RESET 직후 피크 한 번에 다시 잠기던 것). |
| e076d48 | (11-15) 팀원이 Pi4에서 바꾼 수동(=LPP) 서보 변화율 0.40 → 1.0/s(끝에서 끝 0.5초, 팀 합의)를 그대로 옮김 |
| 7629970 | (11-15) e076d48 Pi4 배포 기록(`68827bc0…`) |
| ecffb52 | (11-15) 리팩토링: 주행 상태 초기화를 `clearDriveState()`·`beginRunning()` 한 곳으로(`start`·`resumePlanning`·`stop`에 복사돼 있던 것) |
| e1efe81 | (11-15) 리팩토링: `raw_trial` `tick()`을 `tickStopRamp()`·`updateReversalWait()`·`rampMotorCommand()`로 나눔(순서 그대로) |
| acf3b15 | (11-15) 리팩토링: `controlTick()`에서 상태 발행을 `publishStatus()`로 옮김. 플래닝을 끈 빌드가 `[REVERSAL]` 변수 때문에 컴파일 안 되던 것 수정 |
| 7cd79ba | (11-15) 리팩토링: 감속 정지 끝 속도를 노드가 `raw_trial`과 `StopOutput`에 같은 값으로 넘김(경사 작업에서 한쪽만 바뀌어 내리막 판정이 꺼지는 일 방지) |
| 398c9a8 | (11-15) 설정 범위 오류에 값 이름 표시(예: `commissioning.manual_servo_rate_s=1.5 outside 0 < x <= 1`). 검사 조건은 그대로, 시험 없던 범위 9개 추가 |
| 5e59e26 | (11-15) 리팩토링 기록, ROS 시험 'stale telemetry' 대기 0.4 → 0.6초(부하 때 한 번 실패) |
| c74bad9 | (11-15) `test_rpm_floor_ros.cpp`에서 제어기에 없던 100 ERPM 데드존 검사 2개와 그 설정을 뺌(사용자 결정). 나머지 검사(STOP·과속·만료 때 해제 등)는 그대로, 이제 통과 |
| 22760dc | (11-16) 리팩토링: 'headless 자율주행은 대기, 나머지는 잠금' 규칙을 `pauseOrLatch()` 하나로(6곳·3가지 모양이던 것), 제어 주기 지연 검사도 `timerStalled()` 하나로 |
| 230259c | (11-16) 컴파일 경고 `-Wall -Wextra` 켬(제어 코드·ROS 빌드 경고 0), 시험 경고 3곳 정리, 시험에 남은 없는 설정 `planning.startup_boost_enabled` 삭제 |
| 4e64c7e | (11-16) 4d48523 서보(조향) 표시를 사용자 결정으로 되돌림: `status_json`의 `servo_moving`·`steering_estimate_deg`·`steering_target_estimate_deg`, `control_trace`의 서보 값, 웹 GUI 조향 게이지, 미리보기 서보 왕복, `PlanningCalibration::radFor()`와 그 시험을 모두 뺌(코드는 230259c와 같음) |
| 8d90802 | (11-16) 4e64c7e Pi4 배포 기록(`92523911…`) |
| 515c704 | (11-16) 팀 GUI용 VESC 값: `status_json`에 입력 전류·전력·iq·id·duty·vd/vq·MOSFET 3개·Ah/Wh·타코·VESC ID·고장 이름·상태값 나이·추정 속도, 자율주행용 1초 `vesc_trace` 로그 줄, 시험 `vesc_telemetry_display`와 ROS 시험 검사. 표시 전용(제어 판단 바뀜 없음) |
| 5a457c2 | (11-16) 515c704 Pi4 배포 기록(`c515287e…`) |
| f6f4946 | (11-17) 현장 진단:
  - `link_trace` 로그와 `link_*`·`control_tick_max_ms` 상태 키(명령 간격·지연·만료·제어 주기)
  - `vehicle_state` 50 Hz
  - 정지 유지 발열 추정·`[HOLD_LONG]` 경고·`vesc_trace hold_s`(전류는 그대로)
  - 조향 범위 서비스의 음수 시각 종료 수정(`wire_stamp.hpp`)
  - 시험 `field_diagnostics`와 ROS 검사

  독립 검토 4건 반영: 시각 계산은 예전대로, QoS는 reliable 유지, 로그 줄 빈도 제한, 시험 null 처리 |
| 03c3233 | (11-17) f6f4946 Pi4 배포 기록(`53db03f2…`) |
| 58c83fc | (11-17) 명령이 아예 없는 1초 창은 `link_trace`를 매초 찍지 않음(플래너가 꺼져 대기할 때 GUI 로그 읽기 1 KiB/s 초과 방지). 로그만 바뀜 |
| 598863a | (11-17) 58c83fc Pi4 배포 기록(`afde4fa8…`) |
| 27890e1 | (11-17) 2차 묶음: 감속 정지 끝 3샘플 판정, 끊김 뒤 재출발 대기 0.3초, 미끄러짐·막힘 표시(`slip_monitor.hpp`, 표시 전용), 출력 경로 시험. 독립 검토 반영 |
| f7d4382 | (11-17) StopRamp 초기화 목록 경고 수정(동작 같음, 11-19 f5caad8 배포로 Pi4 반영) |
| 69ab7fd | (11-17) 27890e1 Pi4 배포 기록(`9f5e479e…`) |
| f5caad8 | (11-19) 감속 정지 끝 판정을 '연속 3개'에서 '최근 3개 중 2개'로(11-18 지하주차장에서 200 근처 오르내림으로 정지가 중앙값 0.63초, 2번은 상한까지). ROS 'link resume' STOP 확인을 상태값 대기로 |
| 5c0cbf4 | (11-19) f5caad8 Pi4 배포 기록(`c55f03b8…`) |
| b2d006a | (11-19) GUI가 읽는 로그 줄이기: 재출발·감속 정지 매 주기 줄을 끝 요약 줄로(`trace_summary.hpp`, `trace_every_tick` 기본 꺼짐), `[STOP_RAMP] end`를 주기 맨 끝에, 그 주기 `[PLANNING_WAIT]` 생략, 대기 줄은 바뀔 때·10초마다, 종료 때 끝 줄. 로그만 바뀜(11-19 19:43 4b8af61 배포로 Pi4 반영) |
| a3f6daf | (11-19) 배포 설정 시험(yaml 4개를 Pi4 순서로, 드라이버 토픽·조향 범위, 상태값의 배포 값), 팀 GUI 계약 시험(로그 줄 4종, HTTP 상태 키 68개, 키보드 시간 값), 시작 때 `[CONFIG]` 쓰이지 않는 설정 이름 경고, 가끔 실패하던 stale IMU·기울기 시험을 상태값 대기로. 차 동작은 같음 |
| a9598e1 | (11-19) 리팩토링: LPP 통신 상태(1초 창, 끊김 뒤 재출발 대기, `link_trace` 규칙)를 `link_monitor.hpp` `LinkMonitor` 하나로. 재출발 대기 단위 시험. 동작·로그·상태값 같음 |
| 64505f5 | (11-19) 리팩토링: `publishStatus`를 역할별 함수로(실제 노드 비교로 상태값·텍스트 줄 같음 확인). `control_trace`·`vesc_trace` 1초 간격을 단조 시계로(시계를 뒤로 맞추면 두 줄이 멈추던 문제) |
| 4b8af61 | (11-19) 시험 정리: `unit`·`ros`·`script` 라벨(Pi4 빠른 시험 `ctest -L unit`), 손으로만 돌리던 Python 시험 3개를 ctest에 등록, GCC 11 경고 확인 스크립트(`test/gcc11_check.sh`, 경고 0) |
| 635c31d | (11-19) 4b8af61 Pi4 배포 기록(`11519bd9…`) |
| (이 커밋) | (11-20) 11-19 저녁 주행 분석 반영: 감속 시작을 최근 측정값 3개로(HFI 한 번 낮은 값에 30 A 바로 정지 2/88), 끊김 뒤에는 플래너 STOP이 재출발 대기를 지우지 않음, 끊김마다 `[LPP_LOST]`·`[LPP_BACK]` 줄과 감속 취소 줄 `cmd_age_ms`. 독립 검토 반영. Pi4 미배포 |

통합 GUI(`feat/gui` ddfb8c3, a1c24af): 플래닝 창 '제어기 복구' 버튼, 복구 결과 전용 표시, 복구 중 창 닫기 보호.

### 남은 일

1. VESC 전류 한계 정리: 11-13 VESC에 적용(모터 ±50 A, 배터리 30 A, 회생 −30 A, 같은 날 `s_pid_ki` 0.04·`s_pid_kp` 0.010). 정지 브레이크 때문에 `l_current_min`은 −30 A 이하로 둡니다(지금 −50 A). 설정 파일 `vesc_config_20261112/VESC_설정파일/mcconf_20261113_PID_kp010.xml`.
2. 수동 GUI VESC 점검(`ensure_gui.sh`): 11-13 새 VESC 값으로 맞춤(모터 ≤50 A·하한 −50~−30 A, 배터리 ≤30 A·회생 −30~0 A, PID 최소 50). 자율주행 경로(`run_planning_output.sh`)에는 VESC 설정 점검이 없습니다. Pi4 `~/vesc_ws/inspect_vesc.py --bench-current-a`는 VESC 전류 한계를 써 버리므로(회생 0 A) 쓰지 마세요.
3. 리뷰에서 나온 개선 후보(미수정): 전류 피크 직후 재출발 대기 없음, 과속 감속 명령이 VESC PID 최소값 아래로 내려감, 운영자 STOP 후 복구하면 곧바로 출발.
4. 플래너 쪽 합의: 명령 유효시간 0.15 → 0.3초(11-16 짧은 끊김 정지 12번 해당), `vehicle_state` 구독 best_effort, STOP을 매 주기 반복 발행(그 뒤 제어기 명령 구독 best_effort 가능), 플래너가 1000 ERPM 하한을 모름, 종료 시 드라이버·제어기 동시 종료, `vesc_commission.yaml` 서보 범위 중복, LPP STOP 뒤 감속 정지로 STOP→핸드브레이크 약 0.4~0.46초(예전보다 약 0.35~0.4초 더, 가파른 내리막은 200 유지 최대 0.6초 뒤 브레이크로 넘어가면 상한 없음)·거리 약 4~6 cm 증가(11-13 감속 정지).
5. HFI 적용 후 통합시험(ERPM 모드)에서 턱 넘기·바퀴 걸림 감시·모터 발열 확인. VIO duty 모드는 계속 사용 금지(Pi4의 `nomad_vio_duty_control`에도 11-12에 같은 정지 출력과 온도 보호가 들어갔지만 실차 미시험, 이 저장소 밖).
6. 정지 출력 개선(다음 경사 시험 전): 1단계(유지 고정·표시, VESC 고장 0 A, 실제 설정 ROS 시험, 대기 사유)는 11-13에 완료. 재출발은 VESC 속도 PID 조정(VESC Tool, `vesc_config_20261112` 6절, 11-13 `s_pid_ki` 0.04·`s_pid_kp` 0.010 적용, `mcconf_20261113_PID_kp010.xml`)과 `[RESTART_TRACE]` 기록으로 진행(출발 보조는 넣지 않음). 3단계(부하에 맞춘 유지 전류 12.5~30 A)는 11-13 구현, 실차 확인 남음(`[STOP_OUTPUT] handbrake hold` 로그로 경사별 전류와 미끄러짐을 같이 보고 구름저항 6 A·배율 1.45·여유 8 A를 맞춤. 7°·10° 경사의 오르막 전류와 미끄러지지 않는 핸드브레이크 값, 평지 주행 전류 측정이 있으면 좋음). 남은 것: 4단계 VIO로 미끄러짐 감지(그때 유지 중 RESET·재출발이 ERPM 0을 정지로 믿지 않게), 5단계 밀리는 중·내리막 정지 때 능동 정지 후 유지.
7. 데스크톱 Qt GUI 도움말(`desktop/main.cpp`)의 '실제 차량에서는 브레이크가 아닙니다' 문구는 Qt6 빌드 확인 뒤 고칩니다. 웹 GUI는 실제 정지 설정을 표시합니다.
8. 수동 후진(11-15): 1d47cf0 배포 완료(11-15 17:01). 경사 후진·방향 전환을 실차로 확인(`[REVERSAL]` 로그). 키보드 조종 GUI 문제 2가지(포커스 잃을 때 닫기 누락, 연결 끊기 때 핸드브레이크 해제)를 GUI 팀원에게 전달.
