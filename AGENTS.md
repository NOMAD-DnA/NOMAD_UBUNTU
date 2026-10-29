# NOMAD 작업 지침

## 시작
- 현재 구조는 [docs/STRUCTURE.md](docs/STRUCTURE.md), 사용자 설치·실행 안내는 [docs/실행안내.md](docs/실행안내.md). 사람이실행하는루트wrapper는scripts/에모으고루트에중복파일·링크를두지않는다.
- 기존 변경과 사용자 실행 프로세스를 먼저 확인한다. 사용 중인 시뮬레이션을 요청 없이 종료·초기화하지 않는다.
- 과거 설정·실험은 [docs/archive/AGENTS_before_integration_refactor.md](docs/archive/AGENTS_before_integration_refactor.md)에 보존한다. 과거 값보다 현재 config와 이 문서를 우선한다.

## 모듈 경계
- nomad_gazebo: 센서·GNSS 모사·차량 물리·ROS bridge. GNSS 오류노드는 opt-in.
- nomad_vio / nomad_localization: Basalt·OpenVINS 관리, 공통 출력, Ceres GNSS/VIO 융합.
- nomad_semantic: B0 추론만. nomad_perception: 촬영시각 깊이 사영, 원본 지면점 TLS, 누적지도·비용.
- nomad_path_planning: GPP/LPP/복구/목표속도·조향. nomad_control: 명령실행·피드백. Planning은 Twist/PWM을 직접 발행하지 않고 Control은 지도·목표·경로를 구독하지 않는다.
- 교체 인터페이스는 nomad_interfaces와 nomad_bringup/config/{topics,modules,vehicle}.yaml. 타입·좌표·유효기간·QoS 변경 시 관련 README와 테스트를 갱신한다.

## 현재 실행 정책
- `./scripts/run_terrain_navigation.sh`: 맵 → 기존 GPP/LPP 메뉴 → 실제 VIO/GNSS 융합·B0·인지지도·Planning·Control·RViz.
- `./scripts/run_module.sh`: 같은 각 파트를 단독 실행. 전체 실행과 중복 실행 금지. 기본 GT 실행은 run_autonomy.sh로 보존.
- environment: .nomad/env.sh, domain42/LOCALHOST. 모든 시뮬레이션 소비 노드는 use_sim_time=true. 같은 domain/partition에 Gazebo 중복 실행 금지.
- 외부 VIO일 때 fused provider만 odom→base_link 동적TF, Gazebo GT TF off. /odom은 GT/제어속도 피드백용이고 Planning위치는융합출력. 센서고정TF는 robot_state_publisher.
- 전체 실행은 초기 정렬 조종 창 자동 표시(initial_teleop=true)→2m 이상 수동 이동→위치 정렬 완료→조종 정지 ACK·발행자 종료 확인→지도 READY·실제 /odom 정지3초→자율주행. 초기 창 옵션을 끄면 기존 수동 조종/종료 흐름 사용. GNSS는정지yaw제공안함. VIO원본/융합auto_restart=false,Control재시작확인연결미완료.

## 인지 정책
- 카메라 관측지원 점수는 마지막 직접 촬영 후 10sim초 감쇠 없이 유지하고, 10~30sim초 선형 감쇠 후 0이 된다. 관측지원은 통행 허가가 아니며 새로 확정된 장애물·차단 경사를 덮어쓰지 않는다. 장애물 확정의 최근3초 증거 창과 별도이다.
- 20cm셀, 원본 지면점 TLS 반경.3m/XYbin.05m. 셀높이는표시용이며경사입력아님. 부족한관측을heightfallback으로채우지않음.
- 의미분할 입력과 깊이는동일capture로맞추고촬영시각TF사용. palette/10클래스ID는모델metadata계약유지.
- 사용자 요청으로 경사 즉시 차단을 철회했다. 기준 초과 유효 경사 또는 지면대비.2~.8m돌출은 최근3sim초 내3촬영/.5초/75%동의로 확인한다. 잔디 예외도 이전 동작으로 복구. 경사 임계각 기본10도, GUI 입력값 우선. 해제는4회 반복직접관측. 경사 확정은 이번 촬영 지면점이 있는 셀의 재사용하지 않은 유효 평면만 집계. slope10°는실측차량한계아님.
- 관측기억:실측negative-ray global누적/local30sim초. 미관측-1 유지, 미관측후방을가짜free로만들지않음.
- 2026-10-19 사용자 요청으로 미관측 초기 free 가정은 기본 비활성: bootstrap_radius=0, bootstrap_override=false. 명시적 과거 실험의 양수 반경은 재현용으로만 보존. 센서 근거 없는 전방/후방을 free로 만들지 않음.
- 장애물주변1.2m 추가비용100*(1-(d/r)^2),기존비용과max. unknown보존/80차단임계유지. 원본solid source100.
- 확신잔디ID3(최근10점·60%)는현재시뮬통행가정. 실제식생통과보장아님.
- timestamp는integer나노초변환,미세부동소수점허용1µs. 진짜미래관측·만료는허용하지않음.
- 기본 B0는 nomad_b0_map04(4번 맵600train/100val)이며 기존 nomad_b0(2번 맵1000train/200val)는 보존한다. 같은 맵 검증이며 타맵·실물·변경된 카메라 화각의 성능을 보장하지 않는다. 모델출처·연구용라이선스보존.

## 수정·검증
- active 알고리즘/토픽/TF/차량한계를리팩토링목적으로바꾸지않는다. 후진복구history/freshness미해결을완료로표시하지않는다.
- run_module은실행facade, terrain_layers/navigation_costmap은ROS없는계산층, ROS노드는IO계층. 새설정은config,개인절대경로는코드에넣지않는다.
- .nomad/vio/sources.json의고정SHA/patches로외부소스재현. third_party를임의reset/update하지않음.
- 생성SDF/OBJ/URDF만수정하지말고 authoring·생성입력을보존한다. maps1–8/원본자료/src/nomad_sim·mvsim삭제금지.
- build/install/log/.nomad/deps/sessions는생성물. 상세시험/학습자료는별도보관하고실행경로에넣지않는다.
- 변경범위에맞는회귀·빌드·ROS계약검증,실제성공/실패와미해결을구분한다. 이번정리커밋·푸시는사용자구조검증후에만수행.

## GUI 로그 기록
- 기본 기록 비활성. 기록 시작 버튼에서 파트 선택 후 선택한 실행 메시지만 저장, 종료 시 flush/manifest 종료시간 저장. 화면 필터와 저장 선택은 독립이다.
- GUI는 NOMAD_RUN_LOG_DIR을 자식 환경에서 제거하고 record_results:=false로 지도 결과 저장을 끈다. 자동 CSV/지도/그림 저장은 하지 않는다. 기존 CLI의 명시적 결과 기록 기능과 nomad_logging 옵트인 패키지는 보존한다.
- 실시간/저장 실행 로그 필터는 시뮬레이션·위치추정·인지·플래닝·제어·실행 관리. 로그 폭주 시 화면 최근200줄/100ms로 갱신 작업을 제한하며 선택된 콘솔 기록은 별도 buffered stream으로 저장한다. VIO tail은 활성 실행만1초주기로 확인한다.
- Gazebo transport 분리는 .nomad/env.sh에서 공통 적용한다: 기본 partition=nomad_gazebo_<hostname>_<domain>, GZ_IP=127.0.0.1. 통합 실행이 run_forest.sh를 우회하더라도 같은 분리 정책을 적용한다. ROS LOCALHOST만으로 Gazebo transport가 분리되지 않는다. 명시적 NOMAD_GZ_PARTITION/GZ_IP override는 유지한다.

- GUI 실행 전 /proc에서 동일 ROS domain의 기존 모듈을 검사한다. 다른 GUI/고아 실행과 겹치면 차단한다. QProcess 부모가 끝나도 소유 PGID 자식에 TERM→2초 후 KILL 정리를 적용한다. 다른 그룹을 종료하지 않는다. 원격 Gazebo 격리와 로컬 중복 실행 방지는 별개다.

- run_gui.sh 시작 시 cleanup_previous.py로 이전 NOMAD 실행을 정리한다. 같은 UID·정확한 NOMAD_WS_ROOT·ROS_DOMAIN_ID·프로젝트 명령 증거가 필요하며, 자식도 같은 workspace/domain이어야 한다. PID starttime으로 재사용을 방어한다. 일반 Python/다른 workspace/domain/기존 GUI 자체는 자동 종료하지 않는다. SIGINT3초→TERM2초→KILL1초 후 남으면 GUI 시작을 중단한다.

## 2026-10-20 기본 플래닝 프로파일
- 사용자 요청으로 일반 sensor-based 실행은 메뉴1번 Hybrid A* + RPP, terrain_planning.yaml의 observed_corridor·증분 경유점·다중 비용·관측 회전 검사 설정을 기본 적용한다. 최종목표3m 진입 후 목표를 유지하고 실패 시 정지·재계획하며 후진복구는 비활성이다. 맵·출발·목표는 사용자 선택을 유지한다.
- 설정 승격은 완주 보장이나 지그재그 해결을 뜻하지 않는다. 검증·백업·복구 기록: experiments/default_profile_20261020/README.md.

## 2026-10-26 GPP 정상 경로 재발행 생략 결정
- 사용자 합의: GPP는 주행 중 기존 경로의 유효성을 검사하되 정상일 때 global_path와 동일 중간 목표를 재발행하지 않는다. LPP는 마지막 유효 경로로 계속 추종한다. 목표/경로 변경 또는 무효화 시에만 필요한 발행을 수행한다.
- HTML 시연에는 이 정책을 적용한다. 정상 검사를 GPP 발행 단계나 LPP와의 역할 이관처럼 표시하지 않고 별도 관리 상태·내부 로그로 표시한다.
- 2026-10-27 실제 ROS의 observed_corridor + retain_safe_path에도 적용했다. 정상 검사는 corridor_status만 발행하며, 빈 경로 이후에는 유효 경로를 다시 발행한다. 명령 선택기는 목표 발행 시각과 토픽 수신 순서에 대응한다. 지역 명령 freshness/정지 계약은 유지한다. direct·retain_safe_path=false까지 같은 정책이라고 설명하지 않는다.

## 2026-10-27 플래닝 시나리오 실험실
- 사용자 선택: 첫 검증은 실제 Gazebo 물리 + GT 위치·관측 지도 + 실제 Hybrid A*/RPP/Control. B0/VIO 통합 시험과 구분한다.
- 실행은 scripts/run_scenario_lab.sh, 기본 domain142. 시나리오 입력/생성기는 nomad_bringup/nomad_bringup/scenario_lab, 생성 맵·명시적 시험 결과는 sessions/planning_lab. 기존 maps1–8과 개인 ABCD 백업은 보존한다.
- 큐의 명시적 시험 실행은 metrics/trajectory/events/진단 로그 저장을 허용한다. 일반 GUI의 기본 로그 기록 비활성 정책은 유지한다. 5번 안전 정지·6번 전진 불가를 목표 완주로 집계하지 않는다. 3번 합성 비용과 추가한 물리 곡면의 차이는 docs/플래닝_시나리오_실험실.md 참조.

## 2026-10-28 전륜 구동·후륜 조향 통합
- 현재 기본은 front_drive_rear_steer. 기하/한계는 nomad_bringup/config/vehicle.yaml, 계산은 ROS 독립 nomad_vehicle_model. 이전 전륜 조향은 vehicle_legacy.yaml로 명시한다. 1단계 단독 시험 기록은 GPT log/차량_후륜조향에 보존.
- DriveCommand speed=비조향 앞축 m/s, steer=등가 자전거 각 rad(전진 좌회전 양수). 물리 뒤각은 반대 부호이며 후진에서 다시 반전하지 않는다. 메시지 v2 의미를 사용하며 wire 필드는 동일하다.
- base_link는 차체 기준, +X 새 전방. 앞축이 +wheelbase/2이므로 reference_offset=-wheelbase/2. GPP/RPP/Rollout/observed_guard/복구와 Control 피드백은 이 기준을 공유한다. 공통 vehicle_file과 다른 기하의 모듈 override는 거부한다.
- forest/GUI/시나리오 실험실은 같은 기하로 차량을 생성한다. 기존 맵의 지형·spawn·추가 contact 센서는 보존하고 이전 구동/조향 링크·플러그인은 교체한다. SDF·URDF·vehicle.json은 생성기와 함께 유지한다.
- Planning→Control Twist→watchdog→rear_steering_mixer(input_mode=twist)→4개 joint 출력. 단독 domain152 시험만 전용 DriveCommand 모드. 하나의 믹서에서 입력 모드 두 개를 동시에 받지 않는다. mixer 종료 시 전체 sim을 종료한다.
- 실제 /odom·뒤 steering joint로 VehicleState를 만든다. 명령 echo 금지·ROS lease/wall timeout/clock reset/정지 계약 유지. 후미 외곽 반길이.52m/반폭.42m, 미관측후방을 free로 꾸미지 않는다.
- 카메라 optical/IMU/GNSS/안테나 lever arm은 실제 장착 정보 없이 뒤집지 않는다. 실제 TF·센서 캘리브레이션과 실차 치수/질량·ERPM/서보/제동 보정은 별도이다. 실물 VESC/서보에 시뮬4출력을 그대로 연결하지 않는다.
- 단독 시험 scripts/run_rear_steering_sim.sh, 통합은 기존 scripts/run_terrain_navigation.sh·GUI. 짧은 실행 안내 docs/후륜조향_시뮬레이션.md. 사용자가 승인하기 전 commit/push 금지.
