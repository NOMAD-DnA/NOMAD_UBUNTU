# 비교 GUI worker_error 수정 — 2026-10-29

## 원인과 근거

`알고리즘_비교실험/results/20261029_160833_808762.runner.log`에 `ModuleNotFoundError: No module named 'nomad_vehicle_model'`이 기록됐다. 원격 dev에서 추가된 공통 차량 모델 패키지가 설치 공간에 없었다. 알고리즘 registry가 Hybrid A*를 불러오는 단계에서 실패했으며 시뮬레이터 시작 전 오류다. 앞선 소스 PYTHONPATH 기반 테스트는 설치 누락을 검출하지 못했다.

또한 runner의 알고리즘 설정 초기화가 예외 기록 영역 밖에 있어 outcome.json이 생성되지 않았다. GUI는 결과 부재를 worker_error로 표시했다.

## 조치

```
source .nomad/env.sh
colcon build --base-paths src --symlink-install --packages-select nomad_vehicle_model nomad_gazebo nomad_path_planning nomad_control nomad_bringup
```

5개 패키지 빌드 성공. runner.py의 알고리즘 설정 초기화를 예외 처리 안으로 옮겨 startup_error 결과와 error.log를 남긴다. gui.py는 준비 오류 시 큐를 정지하고 원인과 runner 로그 위치를 표시한다. 기존 큐/실패 로그는 변경하지 않았다.

변경 파일: src/nomad_bringup/nomad_bringup/scenario_lab/{runner,gui}.py 및 test/{test_scenario_lab_gui,test_scenario_lab_startup}.py.

## 검증

GUI와 같은 PYTHONPATH(비교 GUI 및 bringup만 직접 추가)로 관련 테스트 16개 통과. 차량 모델은 build/install 경로에서 로드됨을 확인했다.

```
NOMAD_COMPARISON_DOMAIN_ID=194 ./scripts/run_algorithm_comparison.sh --run --scenario 1 --algorithm hybrid_astar+rpp --timeout 5 --output /home/user/nomad_ws/sessions/comparison_worker_fix_20261029/smoke_01
```

실제 Gazebo/ROS 시작, GPP 경로 1회 발행, LPP 연산 51회, 마지막 상태 DRIVING. 5.05 sim초에서 의도한 timeout으로 종료. cleanup_ok=true, 잔여 소유 프로세스 없음. 이 시험은 시작·주행·기록 확인이며 목표 완주 시험이 아니다. 사용자의 기본 domain144 큐와 분리된 domain194를 사용했다.

## 사용자 재실행

열려 있는 비교 GUI를 닫고 `./scripts/run_algorithm_comparison.sh`로 다시 연다. 새 설치 환경을 로드해야 한다. 기존 대기 항목은 유지되며 큐 시작으로 실행할 수 있다. 기존 worker_error 항목은 자동 재시도하지 않으므로 같은 조건을 큐에 다시 추가한다.

## 실패 시 확인과 복구

새 시험의 outcome.json 및 error.log, 큐 루트의 해당 ID.runner.log를 확인한다. 설치 누락이면 위 선택 빌드 후 GUI를 다시 연다. 실패 결과를 지우거나 기존 시험 폴더에 덮어쓰지 않는다. Git commit/push 및 사용자 프로세스 종료는 수행하지 않았다.
