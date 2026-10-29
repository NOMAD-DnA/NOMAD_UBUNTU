# 알고리즘 비교 실험실

## 세 시험 환경의 구분

| 환경 | 목적 | 지도·입력 | 알고리즘 | 실행·보관 위치 |
|---|---|---|---|---|
| 1. 플래닝 시나리오 실험실 | 6개 상황에서 고정 조합 검증 | Gazebo 물리 + GT 위치 + 누적 LOS 관측 지도 | Hybrid A* + RPP | `scripts/run_scenario_lab.sh`, `sessions/planning_lab` |
| 2. 과거 LPP 비교 | 같은 GPP에서 LPP 선정 | 맵4 주 비교·맵7 보조, 당시 실행별 기록 조건 | Hybrid A* + RPP / PP / Stanley / Ackermann rollout | 2026-10-26 개인 백업의 `workspace/scripts/run_lpp_experiment.sh`, `LPP테스트` |
| 3. 이 비교 실험실 | 같은 시나리오에서 GPP/LPP 조합 비교 | **1번의 6개 맵·차량·관측·목표·판정 그대로 공유** | 아래 5조합 | `scripts/run_algorithm_comparison.sh`, 이 폴더의 `results` |

2번 자료는 당시 소스·센서·속도·지도·하드웨어 조건을 가진 역사적 결과다. 그 수치와 3번 수치를 같은 조건의 순위표로 합치지 않는다. PP·Stanley는 현재 production LPP registry에 없으므로 백업 코드를 이식하지 않았다. 현재 알고리즘을 바꾸거나 과거 ABCD 개선 코드를 복원하지 않는다.

## 실행

`nomad_ws`에서 `./scripts/run_algorithm_comparison.sh`.
창을 여는 것만으로 시뮬레이션은 시작되지 않는다. 기본 ROS domain은 **144**이며 1번 실험실의 142와 분리한다. `NOMAD_COMPARISON_DOMAIN_ID`로 변경할 수 있다. 같은 domain/partition 실행 충돌은 공통 runner가 차단한다. 사용자 프로세스를 임의 종료하지 않는다.

- **실행 큐**: 시나리오·조합·반복 횟수·sim 시간 제한을 선택해 추가한다. `6개 모두 추가`는 선택 조합으로 여섯 시나리오를 추가한다.
- **선택 시나리오 × 5조합 추가**: 현재 선택한 시나리오에서 다섯 조합을 반복한다. 반복마다 조합 순서를 순환시킨다.
- **6시나리오 × 5조합 추가**: 30 × 반복 횟수만큼 시험을 추가한다.
- **큐 시작** 후 순차 실행. 현재 시험 정지/일시정지, 대기 항목 삭제, 실행 중 Gazebo 보기, 재시작 시 interrupted 보존은 1번과 같다.
- **시나리오 결과**: 시나리오를 선택하고 알고리즘이 표시된 개별 실행의 실제 궤적·속도·여유를 확인한다.
- **알고리즘 비교표**: 결과 탭에서 선택한 시나리오의 다섯 조합 통계를 본다. 버튼으로 CSV를 저장한다. 여기서 일괄 추가할 때도 결과 탭의 시나리오를 사용한다.

단일 시험 예:

```bash
./scripts/run_algorithm_comparison.sh --run --scenario 1 \
  --algorithm hybrid_astar+rpp --timeout 180 \
  --output 알고리즘_비교실험/results/runs/manual_001
```

출력 폴더는 새 이름이어야 한다. 과거 큐와 결과는 불러오거나 덮어쓰지 않는다. `--root`로 다른 결과 폴더를 열 수 있다.

## 다른 PC에서 pull 후 준비

ROS 2 Jazzy/Gazebo Harmonic과 프로젝트 의존성이 설치된 Ubuntu 환경에서:

```bash
git switch feat/plan
git pull --ff-only origin feat/plan
sudo apt install python3-pyqt5 python3-matplotlib python3-numpy python3-yaml
source .nomad/env.sh
colcon build --base-paths src --symlink-install --packages-select \
  nomad_interfaces nomad_logging nomad_gazebo nomad_semantic \
  nomad_perception nomad_path_planning nomad_control
./scripts/run_algorithm_comparison.sh
```

최초 ROS/Gazebo 설치는 [설치 및 실행 안내](../docs/실행안내.md)를 따른다. 기존 실행을 마친 후 빌드한다. 실행 스크립트가 빌드된 환경과 소스 GUI를 직접 로드하므로 이 폴더를 별도로 설치할 필요는 없다. 개인 LPP 백업, 과거 시험 결과, B0 모델 추론 및 VIO 실행은 이 시험에 필요하지 않다. 원본 6개 시나리오와 맵 생성 입력은 저장소에 포함돼 있으며 시험마다 맵을 생성한다.

`results/`와 `validation/` 원시 로그·스크린샷은 로컬 생성물이므로 pull에 포함되지 않는다. [검증기록](검증기록.md)은 구현 당시 검증 요약이며, 다른 PC에서는 새 시험을 실행해 결과를 만든다.

## 비교 조합과 통제 조건

| 번호 | GPP | LPP | 선택 이유 |
|---|---|---|---|
| 1 | Hybrid A* | RPP | 시뮬레이션 1과 동일한 기준선 |
| 2 | Hybrid A* | Ackermann rollout | GPP를 고정한 LPP 차이 |
| 3 | State Lattice | RPP | RPP를 고정한 차량 운동학 기반 GPP 차이 |
| 4 | D* Lite | Ackermann rollout | 증분 격자 계획과 후보 궤적 평가 조합 |
| 5 | A* | Ackermann rollout | Rollout을 고정한 격자 GPP 비교 |

공통 `scenario_lab/config.yaml`에서 **두 ROS 노드의 gpp_algorithm/lpp_algorithm만 교체**한다. 실제 planner factory가 해당 구현을 실행한다. 알고리즘 이름만 바꾸거나 주행 궤적을 미리 생성하지 않는다. 차량 기하·속도·unknown·후진복구 비활성·관측·탐색 예산은 유지한다. 격자 경로의 차량 추종 적합성과 공통 observed_corridor 정책도 결과에 영향을 준다. 두 알고리즘이 모두 다른 조합 사이의 차이를 GPP 또는 LPP 하나의 효과로 단정하지 않는다.

## 핵심 지표

과거 LPP 시험의 임무 성공, 여유, 시간, 계산 부담을 가져오되 **측정 정의는 현 6개 시나리오 환경에 맞춘다**.

- 기록 수·목표 도착·기대 결과 충족·실패/오류/중단 수. 5번의 벽 앞 정지와 6번의 전진 불가 정지는 목표 도착이 아니다.
- 장애물 여유 p05의 실행별 중앙값, 성공한 시험 중 최소 여유. 차량 XY 외곽과 GT 장애물 거리이며 접촉력/실물 안전 인증이 아니다.
- 주행 sim 시간의 실행별 중앙값.
- GPP/LPP 프로세스 누적 CPU core·s의 실행별 중앙값. 실행 시간이 다르면 CPU 누적량도 달라진다.
- **LPP 전략 호출 wall 지연 p95(ms)**의 실행별 중앙값. 2번 `lpp_metrics.py`와 동일하게 `lpp_strategy`의 `elapsed_wall_ns`를 사용한다. CPU 사용률이나 제어 주기 보장을 의미하지 않는다.

시간·여유·계산 통계는 기대 결과 충족 및 소유 프로세스 정리 성공 실행만 집계한다. 실패한 짧은 시험을 빠른 알고리즘으로 평가하지 않는다. 실패 원시 결과는 개별 결과 탭에 남는다. 결측은 —/빈 CSV 값이며 0으로 대체하지 않는다. 지표 표본 수는 적격 시험 수이고 지표별 결측에 따라 실제 유효 표본 수는 더 작을 수 있다. 0회 실행과 전부 실패를 구분한다. 비교표는 자동 우승자를 지정하지 않는다.

시간 제한이 혼합되면 화면에 주의를 표시한다. 정식 비교는 같은 소스·설정·시간 제한·하드웨어에서 새 `--root`를 사용한다. 소스 버전이 다른 실행을 자동 통합해 우열을 판정하지 않는다. 같은 시나리오의 최소 3회 이상 반복을 권장하되 충분한 통계 검증을 보장하지 않는다.

## 구현 및 기록

이 폴더의 `algorithm_comparison`은 조합 목록·통계·GUI 확장이다. 큐/프로세스 관리, 시나리오 원본 `authoring.json`, 맵 생성기, ROS recorder는 `nomad_bringup.scenario_lab`을 공유해 맵 복제에 따른 차이를 방지한다. 기본 1번 GUI는 계속 한 조합만 제공한다.

`results/queue.json`, `results/runs/<id>/`에 입력·config·source SHA256·manifest·trajectory·events·timing·outcome·로그·프로세스 정리 기록을 보존한다. 실행 기록은 Git에서 제외한다. 3번은 명시적 시험 기록이며 일반 GUI의 기록 비활성 정책을 바꾸지 않는다. 공통 실행 의존성/설치 안내는 `docs/플래닝_시나리오_실험실.md`를 따른다.
