# NOMAD 실험 자동화

기존 `nomad_bringup forest.launch.py`, 알고리즘 선택 목록, 토픽 설정, ROS bag을 이용한다. GPP/LPP·인지·제어 알고리즘은 변경하지 않는다. 실행 전에 `.nomad/env.sh`를 읽는 루트 `run_experiment.sh`를 사용한다.

```bash
./scripts/run_experiment.sh record 비교01       # 생성 → 파일럿 6회 → 본실험 54회
./scripts/run_experiment.sh status 비교01       # 현재 실행 / 진행률
./scripts/run_experiment.sh resume 비교01       # 완료 결과 보존, 중단/평가불가 실행 재시도
./scripts/run_experiment.sh evaluate 비교01     # 저장 기록 평가, CSV/HTML/PNG 갱신
./scripts/run_experiment.sh show 비교01         # 결과 갱신 후 HTML 경로 표시
./scripts/run_experiment.sh prepare 비교01      # 스냅샷·실험표 생성만
```

실험명을 생략하면 입력 또는 기존 목록 번호 선택. `--pilot-only`는 작은 시험만 수행한다. Ctrl+C는 이 도구가 시작한 프로세스 그룹에만 종료 신호를 보내고 기록기를 정상 종료한다. 기존 사용자 프로세스는 종료하지 않는다. 기본 실험 ROS domain 186은 시작 전 점유 여부를 검사하며 `NOMAD_EXPERIMENT_DOMAIN`으로 변경할 수 있다. Gazebo partition은 시도마다 고유하다. 이 도구는 장기 실행을 담당하는 기존 노드들을 별도로 재구현하지 않는다.

## 고정 조건

6개 조합은 기존 `select_algorithms.py`에서 읽는다. 4/7번은 (-28,0)에서 (28,0), 8번은 (-28,0)에서 막다른길 뒤편 (34,0)을 목표로 한다. 원래 SDF의 시작 자세·높이·차량·센서·물리를 유지한다. 같은 맵에서 조합별 3회 반복, 반복 블록 내 순서는 seed 741/742/743으로 고정해 섞는다. 맵 자체를 새로 무작위 생성하지 않으므로 3회는 환경 다양성 검증이 아니다. 파일럿은 4번 맵의 차량 전방 2m 목표다.

목표 접수부터 본실험은 600 시뮬레이션 초, 파일럿은 15초 제한. 시작 준비는 wall 120초, 본 실행 wall 7200초가 상한이며 wall 상한 도달은 인프라/실행 문제로 분리한다. 현재 GPP 시간제한 해제 설정은 변경하지 않는다. 각 시도마다 전체 스택을 재시작해 지도·복구 이력을 초기화한다. 센서 READY·실제 위치·차량 피드백·접촉 관측·bag 구독이 확인돼야 목표를 전송한다.

## 충돌과 안전 성공

실험용 SDF 복사본에 Gazebo Contact system과 차량의 각 충돌 링크 접촉 센서만 추가한다. 원본 SDF, 질량, 마찰, 충돌 형상은 변경하지 않는다. 관측용 복사본과 브리지 설정은 실행 폴더에 저장한다. 정상 바퀴–terrain 접촉만 허용하며 차체–terrain 및 차량–장애물 접촉은 실패다. 이름을 해석할 수 없는 접촉, 접촉 관측 상실은 평가 불가다. 접촉 쌍은 events.jsonl과 raw bag으로 보존한다. 정상 접지를 관측해 충돌 파이프라인이 살아 있는지도 확인한다.

안전 성공은 목표 반경 0.35m 안에서 실측 속도 절댓값 0.05m/s 미만을 1 시뮬레이션 초 유지하며 금지 충돌이 없는 경우다. roll/pitch 60° 이상 0.5초를 실험상 전복으로 정의하며 실차 허용 경사각이 아니다. /odom에 1초 초과 누락 또는 시간 역행이 있으면 평가 불가다. 센서 끊김으로 멈추는 현상은 상태 로그에 기록한다.

## 정답·지표·그림

- 사람 라벨: 시험 유형과 허용/금지 접촉 규칙. 관측 없이 주행 가능하다고 라벨링하지 않는다.
- 기하 기준: SDF 충돌 형상·지면 메시 기반의 독립 기준 경로/시간이 필요하다. 설계 `route.csv`, `bypass.csv`는 최단경로 정답이 아니다.
- 실제 시뮬레이션 관측: /odom 궤적, /clock 시간, 차량 피드백, Contact 메시지. 실차 성능 증명이 아니다.
- 안전 도착률: 유효한 본실험의 안전 성공 비율, Wilson 95% 구간 및 유효 횟수를 병기한다. 평가 불가 실행을 실패/성공으로 변환하지 않는다. 전체 54개가 확보되기 전 비교는 부분 결과다.
- 안전 성공 가중 SPL = S × L*/max(L*,L), SCT = S × T*/max(T*,T). L은 /odom의 XY 누적 거리, 3D 거리도 별도 보존한다. T는 목표 전송 이후 정지 확인까지 시뮬레이션 시간이다. 기하 참조도 XY 기준과 동일한 목표 반경을 사용해야 한다. SCT 기준은 동일 차량 모델·속도·조향·후진·도착 조건을 적용해야 한다.
- `manifest.json`의 references는 기본 미검증이다. 독립 기준 생성·검증의 근거가 없으면 SPL/SCT는 null이며 **공식 종합 순위는 보류**한다. 최선 실험 결과나 직선거리/최대속도를 최적 정답으로 둔갑시키지 않는다.
- 종합 점수 = 100 × (0.5 SR + 0.25 SPL + 0.25 SCT). 안전 성공을 사용하므로 도착만 요구한 문헌 SPL/SCT와 구분한다. 각 조합 9개 유효 본실험과 검증된 참조가 모두 필요하다.
- 그림: 맵×조합 안전 도착률, 불확실성, 거리–시간 산점도, 실패 원인, 실제 궤적, 종합 점수(미검증이면 보류 안내). 실패 실행의 짧은 경로를 높은 효율로 해석하지 않는다.

## 저장·재개·버전

`experiments/실험명/manifest.json`, `snapshot/`, `status.json`, `runs/조건/attempt_001/{bag,metadata.json,trajectory.csv,events.jsonl,outcome.json,launch.log}`, `results/{metrics.csv,summary.json,report.html,figures,notion_summary.md}`.

기준 commit과 소스·선택 맵·설정의 SHA256을 저장하고 복사본을 보존한다. Python 노드는 snapshot에서 로드한다. 기존 설치 launch/config를 포함한 소스 해시는 실행 전후에 검사하며 변경 시 배치를 중단하고 해당 실행을 무효화한다. 수정 중인 코드와 결과가 섞이지 않도록 새 실험명을 사용한다. 기존 시도는 덮어쓰지 않는다. 완료된 유효 실행은 resume에서 건너뛰며 작은 시험의 충돌·시간 초과도 유효한 결과로 보존하며 기록 파이프라인이 정상일 때 본실험으로 진행한다. 작은 시험에서 기록 누락/실행 오류가 나면 본실험을 시작하지 않는다. 디스크 여유 8GiB 미만이면 중단한다.

원시 영상은 기본 기록에서 제외한다. 위치·TF·clock·전역/지역 costmap·경로·모듈 상태·명령·피드백·접촉은 MCAP에 저장한다. 이미지 기반 인지 재평가를 위한 데이터셋은 아니며 센서 입력을 재생해 다른 정책의 실제 성능을 평가할 수 없다. 새 실행을 별도로 수행해야 한다.

노션용 요약 파일은 로컬 결과에서 생성한다. Notion 계정 토큰을 코드에 저장하지 않으며 페이지 업데이트는 연결된 브라우저/커넥터에서 수행한다.

## 전체 비영상 원시 기록 (2026-10-06)

사용자 합의로 `sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`만 제외하고, 격리 ROS domain에서 발견되는 모든 비영상 토픽(숨김 토픽 포함)을 저장한다. joint_states·IMU·scan·camera_info·디버그 지도·상태·파라미터 이벤트도 포함된다. 발행되지 않는 토픽은 만들지 않는다. `topic_inventory.json`의 실제 발행자·타입·QoS와 `topics.csv`의 실제 저장 건수를 구분한다. 원본 RGB/깊이 영상 재처리가 필요한 새 인지 분석은 이 데이터만으로 불가능하다.

MCAP의 Zstd 무손실 청크 압축을 쓰며 512MiB 단위로 분할한다. 각 종료된 실행은 모든 저장 메시지를 순회해 metadata의 건수와 비교하고 `archive_audit.json`, 파일별 `checksums.sha256`을 생성한다. 이 검사는 저장 전 DDS 유실까지 완전 증명하지 않는다. 기존 종료·위치·접촉 신선도 검사와 함께 사용한다. 이상 기록은 보존하고 성능 실패와 별도로 분류한다. 보관 위치는 로컬 디스크이며 별도 장치 백업은 아니다.

`timing/*.csv`는 실험 환경변수가 설정됐을 때만 GPP 탐색·경로 추출, A* fallback, 선택 GPP, 참조 어댑터, LPP 알고리즘 호출의 wall/해당 thread CPU 시간을 ns 단위로 저장한다. stage별 범위가 다르므로 서로 다른 stage를 같은 총 계획 시간으로 혼동하지 않는다. 입력 준비·지도 갱신·ROS 대기·발행과 전체 노드 처리 시간은 포함하지 않는다. 계측은 계획 입력·출력·제한을 바꾸지 않는다.

원시 MCAP은 전체 메시지 필드를 보존한다. 나중에 시뮬레이터 없이 다음 명령으로 특정 토픽의 모든 필드를 열린 형식(JSONL gzip)으로 내보낼 수 있다. `bag_sim_ns`는 bag 수신 시뮬레이션 시간이며 원래 header stamp도 message 안에 남는다. 선속도는 `twist.twist.linear.x`를 그대로 쓴다.

```bash
./scripts/run_experiment.sh export 실험명 --topic /odom
./scripts/run_experiment.sh export 실험명 --topic /nomad/control/vehicle_state
```

오늘의 수집 모드에서는 각 실행 후 그림·노션을 생성하지 않는다. `progress.json`, 각 시도의 원시 파일과 검사 결과만 갱신한다. 수집 완료 후 필요할 때 `evaluate`를 명시적으로 실행한다. 원시 기록·중단 기록·기존 샘플은 자동 삭제하지 않는다.

## 4번 맵 순환로만 실행 (2026-10-07)

```bash
./scripts/run_experiment.sh route map4_loop_20261007 --pair 3
./scripts/run_experiment.sh status map4_loop_20261007
```

노션 `ROS Gazebo 환경 → 맵 구성 → 04. 혼합림 순환로`의 같은 방향 2바퀴
시나리오를 실행한다. 기존 54회 실험과 별개의 이름을 사용한다. 맵4의
`route.csv`에서 약 4m 간격으로 순서 있는 경유점을 생성한다. 중간 경유점
1m 이내에서 다음 `/goal_pose`를 전달하며 정지를 요구하지 않는다. 마지막
출발점에서 기존 도착·정지 조건을 확인한다. GPP/LPP와 인지·차량 설정은
유지하고 경유점 사이 실제 경로는 계획기가 생성한다. 기준선을 엄밀하게
추종하거나 정확한 304m 주행을 보장하지 않으며, 이 선은 최단경로 정답이 아니다.

기본 조합3은 D* Lite + Ackermann rollout. `--pair 1`~`6`으로 선택한다.
최대1200초 시뮬레이션 시간 / 4시간 실제 시간. headless 실행하며 모든 비영상
ROS 토픽, 물리 접촉, 경로·제어·실측 속도와 경유점 도달 이벤트를 기존 방식으로
저장한다. 진행은 해당 attempt의 `mission_progress.json`과 `events.jsonl`에
남는다. 중단은 실행 터미널 Ctrl+C, 재시도는 `resume 실험명`이며 중단 지점이
아닌 새로운 시뮬레이션에서 해당 1회를 다시 시작한다. 이전 파일은 보존한다.
이 명령은 다른 시뮬레이터를 자동 종료하지 않는다.

## 전체 1~8번 맵 기록 (2026-10-07)

`./scripts/run_experiment.sh batch all_maps_routes_20261007`로 6개 조합 × 8개 맵 × 3회,
총144회와 기록 확인용 파일럿6회를 실행한다. 1·2번은 기존 목표(23,16),
3번은 기본5° 레인의 끝(20,-25)을 새 실험 목표로 사용한다.
4~7번은 기준 순환로2바퀴, 8번은 진입→분기점복귀→우회로를 순서대로 전달한다.
후진을 직접 강제하지 않고 기존 복구/계획 모듈에 맡기며 실제 후진거리를 기록한다.
`stop 실험명`은 기록 정리를 기다리는 정상 중단 요청, `status 실험명`은 상태 확인,
`resume 실험명`은 미완료 조건 재시도다. 취소 표시된 예전54회 실험은 재개하지 않는다.

맵1·2의 외부 차량 SDF include도 실험 복사본에서 확장해 접촉 센서를 부착한다.
원본 위치·관성·충돌체는 유지한다. 맵3의 평지와 경사로는 바퀴 접촉만 허용하고
차체 접촉은 충돌로 기록한다. 8개 맵 모두 계측 파일 생성·물리 보존을 검사한다.

무진전 종료: 본실험에서 시뮬레이션60초 동안 기준 위치로부터0.5m 이상
이동하지 않으면 `no_progress`로 기록하고 다음 조건으로 진행한다.0.5m 이상
전진/후진하면 기준 위치와 타이머를 갱신한다. 제자리 회전·작은 흔들림은
이동으로 인정하지 않는다. 파일럿에는 적용하지 않으며 센서/기록 품질 오류와
구분한다. 이 수치는 본 프로젝트의 실험 종료 기준이며 문헌의 표준값이 아니다.
판정 전에 충돌·전복·odom/접촉 관측 오류를 먼저 검사한다.

실험 루트와 새 실험 폴더에는 `COLCON_IGNORE`를 생성해 스냅샷의 ROS 패키지를
colcon 검색에서 제외한다. 기존 기록도 `experiments/COLCON_IGNORE`로 제외되며
원시 데이터와 스냅샷은 유지한다. 워크스페이스 루트의 `colcon build`를 사용할 수 있다.

## 4번 맵 수동 경유점 비교 — 2026-10-12

2026-10-08 RViz에서 입력한 43개 경유점을 원래 순서 그대로 재사용한다.
기본은 기존 메뉴 6개 조합 × 3회 = 본실험18회이며, 조합별 전방2m 파일럿6회가 먼저 진행된다.
파일럿은 실행·센서·기록 검증을 통과해야 본실험으로 넘어간다. 검증된 기록의 도착 실패(충돌·시간 초과 등)는 알고리즘 성능 결과로 보존하고 본실험을 계속한다. 파일럿 성공 수와 성능 실패 수는 별도로 표시하며 본실험 성공률에 섞지 않는다.

```bash
./scripts/run_experiment.sh compare map4_clicked_20261012
./scripts/run_experiment.sh status map4_clicked_20261012 --watch
./scripts/run_experiment.sh stop map4_clicked_20261012
./scripts/run_experiment.sh resume map4_clicked_20261012
./scripts/run_experiment.sh show map4_clicked_20261012
```

`compare`와 이 비교 실험의 `resume`은 기본으로 백그라운드에서 실행한다.
터미널을 닫아도 실행 프로세스가 유지되지만 컴퓨터 종료·절전까지 극복하는 서비스는 아니다.
`--foreground`는 현재 터미널 실행, `--prepare-only`는 기록/실험표 생성만,
`--pilot-only`는 파일럿만 실행한다. `--pairs 1,2,3,4,5` 또는 `--repeats 1`로
새 실험의 범위를 지정할 수 있다. 기존 실험 조건은 이름을 재사용해 바꾸지 않는다.
`--route-source`는 experiments 아래 경유점 기록 폴더명이며 기본값은
`manual_map4_waypoints_20261008`이다. 원본 체크섬·XY 순서·좌표계·출발 자세를 검증하고
route_input에 복사한다. 원본이나 복사본을 변경하면 기존 결과에 조용히 섞지 않는다.

각 시험의 출발 자세는 맵4의 `(-28,0,0.03,0,0,-pi/2)`다. 중간 경유점1m,
최종0.35m와 정지 조건은 기존 실행기를 사용한다. 수동 입력 중 수정한 점도 유지하므로
기존 수동 조작의 시각/궤적을 정확히 재현하는 시험은 아니다. 클릭 방향은 원본에만
보존하고 실행기는 XY 순차 목표를 전달한다. 새 지도/차량/경로 추종기를 만들지 않는다.

`status`는 본실험 완료/18, 성공·성능 실패 수, 파일럿 진행, 현재 조합·반복 번호,
현재 경유점/43, 경유점까지 거리, 실측 속도, 시뮬레이션 시간, 노드 상태를 표시한다.
`--watch`에서 Ctrl+C는 화면 갱신만 끝내며 실험은 계속된다. 초기화·기록 정리 중에는
그 상태를 표시한다. `background.log`는 실행 로그이며 `runner.json`에 PID를 저장한다.
이미 실행 중인 실험은 파일 잠금으로 중복 시작을 차단한다. stop은 현재 기록을 정리한 뒤
멈추고, resume은 완료한 성공/성능 실패를 보존하며 중단된 시험만 처음부터 재시도한다.
운전 도중의 물리 상태에서 이어가는 기능은 아니다. 다른 사용자 세션은 종료하지 않는다.

기존 모든 비영상 MCAP 기록·접촉 판정·무진전60초 종료·디스크 여유 검사·검증을 재사용한다.
코드/설정은 시작 시 스냅샷으로 고정하고 ROS 모듈 설정도 스냅샷 YAML을 전달한다.
시뮬레이터 잔디 정답 모드는 모든 조합에 동일하게 적용되며 실제 잔디 인지 성능 시험이 아니다.
본실험의 충돌·timeout·no_progress는 성능 실패로 기록하고 다음 조건으로 진행하지만,
기록 누락/센서 오류 등의 평가불가는 자동 중단한다. 기존 취소 실험은 재개하지 않는다.

완료 또는 평가불가 중단 시 기존 report.py로 부분/전체 결과를 자동 생성한다.
실행 중에도 show/evaluate로 종료된 시험들을 비교할 수 있다.

- `results/report.html`: 비교표, 안전 도착률 및95%구간, 주행 거리/시간, 실패 사유, 궤적 그림
- `results/metrics.csv`: 시험별 기존 지표 + 경유점 도달 수
- `results/pair_summary.csv`, `summary.json`: 실제 계획 횟수 기준 집계, 성공 주행 평균 거리/시간/후진 거리
- `results/timing_summary.csv`: 조합·stage별 호출 수, 평균/P95 wall 시간, 평균 thread CPU 시간
- `results/figures/`: 저장된 결과로 생성한 그림. 실패한 짧은 주행을 효율 우수로 해석하지 않는다.

가중치는 기존 SR50%/SPL25%/SCT25%를 유지한다. 수동 클릭을 이은 선은 검증된 최단경로나
최소시간 정답이 아니므로, 독립 기준값이 없는 SPL/SCT/종합점수는 N/A로 둔다.
이 비교는 시뮬레이션 성능이며 실차 성능의 증명이 아니다. 노션에는 자동 게시하지 않는다.


## Map 4 multi-factor optimization

`./scripts/run_experiment.sh optimize map4_multicost_20261019` starts one bounded
baseline in background. Use the same command with `--stage status`, `--stage stop`
or `--stage sweep`. The sweep reuses measured feature snapshots; it does not
create a synthetic map or use route.csv as a ground-truth path. No automatic
infinite retry. Existing baseline IDs cannot be overwritten. Startup 240 wall s,
clock stall 20 wall s, stationary 120 wall s, live run 600 sim s / 1800 wall s;
parent watchdog terminates a blocked child. Only owned process groups are stopped.
Initial alignment is straight 0.25m/s for at most 3m, stopped immediately on fusion
READY / other command publisher detection. No automatic goal is sent before
normal initialization, observed contacts, recording and planning readiness.
The experiment is isolated in ROS domain 63 and a dedicated Gazebo partition.
World goal (28,0) uses the verified shared simulator/fusion ENU datum.
Instantaneous pose-estimation error is never used to transform the target.
GT is used only for evaluation, not localization.
All non-image ROS topics are stored in MCAP, feature snapshots every 5 simulated
seconds plus settings, logs and outcomes. Invalid infrastructure runs are not
planner failures or optimization successes. No SPL/SCT without verified references.

미관측 비용+관측 영역 실행 제한 시험:
```bash
./scripts/run_experiment.sh optimize map4_unknown_guard_20261019 --stage validate --weights 8,0.5,0.5 --unknown-surcharge 9 --trial guarded_01
./scripts/run_experiment.sh optimize map4_unknown_guard_20261019 --stage status
./scripts/run_experiment.sh optimize map4_unknown_guard_20261019 --stage stop
```
새 이름은 수정 소스 snapshot을 보존한다. trial params.yaml/metadata.json/outcome.json의
unknown_surcharge가 실제 시험값이며 기존 실험 기본값 4.5와 구분한다.
정체 120초/시작240초/본주행1800초 wall/600초 sim 제한을 유지한다.

### Cluster-cost comparison
`optimize NAME --stage optimize --cluster-sweep --mapfile SNAPSHOT --mission MISSION`
uses the existing bounded runner: 10 frozen-map candidates (baseline C=0 plus
R={2,4,6}m × C={8,32,128}), followed by one live validation. Other weights are
O=8,S=T=0.5,U=18. All candidates use a common screening score
`length + 18*unknown_length + 128*integral(rho_4m ds)`; this is an explicitly
chosen heuristic preference, not learned ground truth. Density is the fraction
of observed solid source cells in a full disk. The snapshot is offline input only;
live mapping starts fresh. `--cluster-weight` and `--cluster-radius` also work
with `--stage validate --weights 8,0.5,0.5 --unknown-surcharge 18`.
Compare against C=0 with identical mission/localization/guard. Inspect success,
collision, progress, unknown exposure and density exposure; lower frozen-map
score alone is not successful navigation. All non-image bags and snapshots are
retained. RViz remains opt-in; existing watchdog limits and private domain remain.

`--influence-sweep --mapfiles PRE_ENTRY_1.npz PRE_ENTRY_2.npz PRE_ENTRY_3.npz`
compares zero cluster, old C=128, and nine influence-area configurations. The
common diagnostic metric uses influence 1m, assessment disk 4m, saturation .4
for every candidate. Automatic optimize selection requires successful searches
on all three snapshots and ranks mean L+18U+16*influence_exposure. It is heuristic
screening, not an arrival or unseen-forest avoidance guarantee. `--stage sweep`
allows reviewing all candidates before a separate bounded `validate` run.

`--cluster-priority` compares the previous C=8/U=18 against C={32,64,128}
× U={2,4.5,9}, holding influence=1.5m, assessment radius=4m and saturation=.4.
Selection requires solving all three pre-entry snapshots. Lowest mean observed
cluster exposure is primary; within 2% of that minimum, prefer less unknown
length, then shorter length. No weighted L+18U score is used for this selection.
Live GPP still uses additive costs; this is a priority rule for choosing weights,
not a guarantee that all online paths avoid clusters. Unknown areas may hide
obstacles. Existing observation guard and hard blocked-cell constraints remain.

Forward broad comparison: `--gpp hybrid_astar --broad-sweep --stage sweep` compares
48 conditions: proximity {4,8}, cluster {8,16,32,64}, unknown {2,4.5,9}, influence
{.75,1.5}m; slope/terrain .5, cluster disk 4m, saturation .4. Each measured input
must contain actual `start_yaw`; no synthetic zero yaw for kinematic comparisons.
The existing forward Hybrid A* uses rear-axle integration and base_link output,
wheelbase .72m, max steer .4rad, offset .36m. Behind-start paths are rejected.
`rank_balanced_forward` requires success on all three snapshots; caps length at
1.35 times the shortest candidate per snapshot and extra unknown length at .25
of that reference. Among remaining candidates, minimize observed cluster
exposure; 5% ties favor less proximity exposure, then unknown length and length.
These are explicit screening preferences, not a verified optimal reference.
Compare several shortlisted conditions in fresh, bounded live runs; no defaults
are promoted from frozen-map scores alone. GUI remains opt-in.

최적화 재주행은 `--lpp rpp`로 LPP만 교체할 수 있다. 기본값은 `rollout`이며 자식 실행·manifest·metadata·outcome에도 선택값을 보존한다.

ROS 실행은 기본→vehicle→실험 설정을 단일 YAML로 병합하여 노드별 조향 여유가 공통 설정에 덮이지 않도록 한다.

`--unknown-direction-weight W`는 근거리 미관측 방향 추가비용을 설정한다. 기본 0, 반경6m/전방30° 면제, live/search와 자식 실행 및 결과 기록에 동일 적용된다.

`--unknown-direction-free-angle DEG`로 비용 시작 각도를 지정한다(기본30°, 이번 비교15°). 센서의 실제 시야각을 뜻하지 않는다.

## Map04 관측 통로 가중치 비교 (2026-10-20)

`corridor_sweep.py`는 기존 `optimization.py`의 관측·기록·제한시간·소유 프로세스 정리를 재사용해 12조건을 순차 실행한다. Hybrid A* + RPP, 아래쪽 출발, 최종 목표 (28,0), 관측 통로 모드, 복구 비활성 조건이다. 고정 조건표는 스크립트의 CONDITIONS에 있고 각 실행의 cNN.json에 실제 명령과 가중치를 남긴다.

```bash
source .nomad/env.sh
python3 src/nomad_bringup/experiments/corridor_sweep.py 실험명
python3 src/nomad_bringup/experiments/corridor_sweep.py 실험명 --status
python3 src/nomad_bringup/experiments/corridor_sweep.py 실험명 --stop
```

완료 outcome이 있으면 보존하고 다음 조건으로 이어간다. 명시적 중단 표식 STOP_REQUESTED가 있으면 자동 재개하지 않는다. 재실행은 새 실험명을 권장한다. 실패 조건도 valid 성능 결과이면 계속하며 인프라 평가불가이면 중단한다. 이번 무진행 제한은 45 wall초/0.5m이다.

`optimization.py --planning-overrides YAML`은 허용된 Planning 노드의 ros__parameters만 병합하고 실행 폴더에 사본을 보존한다. `--no-motion-seconds`는 최소 10초이며 기존 기본값은 120초다. corridor.goal_weight/length_weight/narrow_weight/heading_weight/visit_weight/clearance_weight는 통로 후보 선택 비용이며, GPP 자체의 proximity/cluster와 별개다. recovery.enabled=false를 LPP와 recovery supervisor에 모두 지정하면 복구 동작을 끈다. 기존 실행 기본값은 true다.

실행 결과: experiments/map4_weight_trials_20261019/결과.md 및 results.json/csv. 12/12 완료, 목표 도착 0회. 10번은 55.74m 주행하고 목표 18.68m를 남겼다. 성공 가중치로 적용하지 않았다. 관련 변경은 corridor_selector.py, recovery_gpp.py, recovery_supervisor.py, local_planner_node.py, optimization.py, corridor_sweep.py. 실험 전 관련 테스트 243개 및 두 패키지 빌드 통과. 기본 실행으로 복귀하려면 실험 override 없이 기존 스크립트를 사용한다.
