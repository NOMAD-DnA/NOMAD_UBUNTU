# 인지·지도

```bash
./scripts/run_module.sh semantic  # 별도 터미널: RGB 분할
./scripts/run_module.sh map       # 깊이·분할·융합 pose → 지도·코스트맵
```

핵심은 `terrain_layers.py`(지면·클래스), `navigation_costmap.py`(비용·관측기억), `navigation_terrain.py`(ROS IO).
설정: `config/navigation_terrain.yaml`. 입력 RGB/깊이/라벨은 같은 capture·TF로 정렬한다.
출력 `/nomad/navigation_costmap`, `/nomad/navigation_local_costmap`, `/nomad/perception/status`.
20cm셀·120cm 이차 장애물 비용. 미관측은 -1, 통행0–79, 차단100.
세부 계약·기존 비교 도구는 [기존 문서](docs/archive/README_details.md).

### 지도 지연 진단 (선택)

다음 실행에만 `NOMAD_MAP_DIAGNOSTICS_DIR` 환경변수 또는 지도 노드의
`diagnostics_directory` 파라미터를 지정한다. 기본값은 빈 문자열(기록 없음).
GUI의 일반 로그 선택과 별개인 명시적 진단이며 `record_results=false`여도 기록한다.
이미 실행 중인 노드에는 소급 적용되지 않는다. 사용자 실행을 종료한 뒤 다시 실행한다.

```bash
NOMAD_MAP_DIAGNOSTICS_DIR="$HOME/nomad_ws/log/map_diagnostics" ./scripts/run_gui.sh
```

JSONL에는 integer `stamp_ns`(촬영 시각), `sim_ns`, `wall_ns`, `mono_ns`와
수신, 동기화 성공/제외 사유, 대기 입력 덮어쓰기/만료, TF 실패 이유/대기 시간,
계산 단계별 소요시간, 지도 발행, health 실패 조건을 기록한다.
`input_received`는 ROS callback 실행 시각이며 DDS 패킷 도착 시각이 아니다.
`tf_wait`는 조회 시도별 기록이다. 영상/점군 본문은 저장하지 않는다.
TF base 조회가 실패하면 기존 동작대로 상위 process의 만료 검사 전 반환한다.
이 계측은 그 동작을 수정하지 않고 기록한다.
파일은 실행마다 고유 이름이며 별도 스레드에서 기록·1초마다 flush한다.
8192건 큐가 차면 주행을 막지 않고 진단을 버리며 종료 요약에 dropped를 남긴다.
강제 종료 시 버퍼 기록이 유실될 수 있고, 정상 종료의 diagnostics_closed가 없으면
기록 완전성을 보장하지 않는다. 계측 오버헤드를 감안해 비활성 실행과 구분한다.

### 지도 계산 프로세스 분리

`navigation_terrain`은 ROS 수신/촬영시각 TF 조회/발행/health를 부모 노드에서 처리하고,
지형 누적·경사 평가·스캔 통합·코스트맵 계산·주기적 파일 저장은 `spawn`한 작업 프로세스
한 개에서 순차 처리한다. 지도 누적 상태는 작업 프로세스가 소유한다.
실행 중 계산은 한 건만 허용하며, 다음 입력은 기존 exact-capture 동기화를 통과한 최신
묶음 하나만 유지한다. 수신된 모든 프레임을 계산한다는 보장은 없다.
TF를 확보한 스캔도 최대 6개를 대기시키며 교체를 진단 로그에 남기고 전달 시 0.6초 만료를 검사한다.
부모는 실제 완료된 지도만 원래 촬영 시각으로 발행한다. 기존 map/odom 신선도 조건,
health 유효기간, 시작 전 3초 정상/정지 조건은 유지한다.

작업 프로세스가 종료되거나 오류를 보고하면 FAULT이며 자동으로 빈 지도를 정상 발행하지 않는다.
계산이 `worker_timeout_wall_sec`(기본 실제시간 30초)를 넘거나 시뮬레이션 시계가 되돌아가도
FAULT로 재시작을 요구한다. 그보다 앞서 지도가 만료되면 기존 freshness 검사로 WAITING이다.
종료 시 자신이 만든 작업 프로세스만 정리한다. 별도 `live_terrain` 실행은 기존 동기 처리다.

진단 `compute_submitted`는 부모의 작업 전달, `compute_complete`의 `elapsed_ms`는 작업
프로세스 계산 시간이고 `dispatch_to_result_ms`는 IPC/스케줄링/선택적 저장을 포함한 결과 수령까지다.
`measured_in=worker` 단계 이벤트는 결과 수령 시 기록되므로 이벤트 wall 시각을 실제 계산 시작·종료로
해석하지 않는다. 이 변경은 콜백 차단을 해결하며, 지형 계산 자체의 연산량 최적화는 아니다.


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

LiDAR/RGB-D time alignment: the terrain worker defers scans newer than the current
camera capture (1 microsecond tolerance), then integrates them in timestamp order
when a subsequent capture catches up. This avoids replacing older free-ray evidence
with a newer timestamp that scan_free_memory would reject. Deferred scans retain
sensor time; no artificial current-time refresh or unobserved free cells. A bounded
256-scan queue reports clock divergence rather than growing indefinitely. Grid
headers remain the camera capture time; existing age checks stay active.

## 2026-10-19: sensor-only initialization

Default navigation bootstrap_radius is 0 and bootstrap_override is false. Zero disables both synthetic grid seen/free writes and output free-area filling. No vehicle-footprint free bubble is added. Existing positive-radius settings remain available only for explicit legacy experiment reproduction; loading an old experiment YAML or saved GUI override can re-enable the old assumption.

With record_results enabled, terrain/obstacle_diagnostics.jsonl records each saved capture's projected tree/rock points, collision-height points, candidate cells, grass-veto cells and confirmed cells. These are global capture counts, not proof that a particular tree was observed. Normal GUI recording policy is unchanged.

## 경사 즉시 차단 (2026-10-20)
유효 지면 높이와 경사가 있는 셀은 설정 임계각 이상이면 즉시 전역/지역 비용100으로 처리한다. 기본 임계각10도. 경사에는 반복 관측 대기 및 잔디 예외를 적용하지 않는다. 유효성 부족/NaN은 초과 판정이 아니다. 고형물 확인 절차는 유지한다. 검증·복구: experiments/immediate_slope_block_20261020/README.md.

## 경사 즉시 차단 철회 (2026-10-20, 위 절보다 우선)
사용자 요청으로 반복 관측 방식으로 복구: 최근2sim초 내4회·.5초 이상·75%동의. 임계각 기본10도/GUI 설정값 유지. 직전 즉시 차단 설명은 현행이 아니다. 테스트25개 통과. 시간창/직접관측에 따른 확정 지연 분석: experiments/slope_repeat_restore_20261020/README.md.

## 반복 확정 조건 개선 (2026-10-20, 위 절보다 우선)
경사/고형물 공통 확정은 최근3sim초 내3회·최소.5초·75%동의로 변경. 해제는4회·최소1초·최근1초 초과증거 없음 유지(증거 창은3초). 경사는 이번 촬영 지면점이 있는 셀이고 slope_reused가 아닌 경우만 새 초과 증거로 인정. 오래된 경사만 있을 때에는 확정/해제 투표를 추가하지 않는다. GUI 경사 임계각은 유지. 실제 지면점 누락 시 확정 지연은 여전히 가능하며 1.58초는0.79초 간격의 유효 관측3회가 이어질 때의 계산값이다. 기록: experiments/slope_confirmation_3hits_20261020/README.md.

## 카메라 관측 기억 (2026-10-21)

카메라 관측지원 점수는 마지막 직접 촬영 후 10sim초 감쇠 없이 유지하고, 10~30sim초 선형 감쇠 후 0이 된다. 관측지원은 통행 허가가 아니며 새로 확정된 장애물·차단 경사를 덮어쓰지 않는다. 장애물 확정의 최근3초 증거 창과 별도이다.

`navigation_terrain.yaml`의 `observation_support_hold_seconds: 10.0`, `observation_support_expire_seconds: 30.0`으로 설정한다. 점수 품질은 마지막 관측 시점의 증거로 계산하며, 새 관측 없이 강해지거나 유효기간이 연장되지 않는다. OccupancyGrid의 미관측/장애물 값이나 local/global 지도 보존 정책을 변경하지 않는다. 재시작한 인지 노드부터 적용된다.
