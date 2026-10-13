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
- 20cm셀, 원본 지면점 TLS 반경.3m/XYbin.05m. 셀높이는표시용이며경사입력아님. 부족한관측을heightfallback으로채우지않음.
- 의미분할 입력과 깊이는동일capture로맞추고촬영시각TF사용. palette/10클래스ID는모델metadata계약유지.
- 확인차단은지면대비.2~.8m돌출또는validground slope20° 이상에4촬영/.5초/75%동의. 해제는반복직접관측. slope20°는실측차량한계아님.
- 관측기억:실측negative-ray global누적/local30sim초. 미관측-1 유지, 미관측후방을가짜free로만들지않음.
- 초기고정전방3m/bootstrap_override=true는사용자지정출발가정. 차량을따라움직이는freebubble아님.
- 장애물주변1.2m 추가비용100*(1-(d/r)^2),기존비용과max. unknown보존/80차단임계유지. 원본solid source100.
- 확신잔디ID3(최근10점·60%)는현재시뮬통행가정. 실제식생통과보장아님.
- timestamp는integer나노초변환,미세부동소수점허용1µs. 진짜미래관측·만료는허용하지않음.
- B0는2번맵1000train/200val미세학습모델. 분할성능의타맵·실물일반화를보장하지않음. 모델출처·연구용라이선스보존.

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
