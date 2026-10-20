# 플래닝

```bash
./scripts/run_module.sh planning  # 기존 메뉴에서 GPP/LPP 선택
```

입력: 융합 pose·전역/지역 costmap·목표·센서 상태·Control 피드백.
출력: `/nomad/planning/drive_command`의 목표 속도·조향각·정지 요청.
설정: `config/planning.yaml`; 통합 override: `../nomad_perception/config/terrain_planning.yaml`.
GPP/LPP 전략은 `gpp/`, `lpp/`; 후진 복구는 `recovery_supervisor.py`.
후진은 기록된 복귀 경로와 최신 센서가 필요하다. 현재8번 맵에서 복구 이력 부족 정지는 미해결이다.
알고리즘·복구·인터페이스 계약은 [상세](docs/archive/README_details.md).

## GPP 전용 장애물 근접 비용

`gpp.proximity_radius: 2.0`m, `gpp.proximity_weight: 4.0`이 기본이다.
인지 지도 원본을 변경하지 않고 값 100인 확정 장애물 셀 중심까지의 유클리드
거리를 OpenCV distanceTransform으로 계산한다. 80–99인 팽창 셀은 거리 원점으로
재사용하지 않는다. 입력 지도별로 결과를 캐시하며 새 ROS 토픽은 추가하지 않는다.

통행 가능한 관측 셀의 GPP 가중치는 `1 + 4*C/79 + w*max(0,1-d/r)^2`이다.
80 이상은 계속 통행 불가, 미관측 -1의 기존 정책도 유지한다.
LPP와 인지의 공용 코스트맵, 기존 1.2m 비용 반경은 변경하지 않는다.
D* Lite(복구 A* 포함), A*, Weighted A*, Hybrid A*, State Lattice, Field D* 변형과
참조경로 보정에서 같은 추가 비용을 사용한다. D* 갱신은 원본 값이 같더라도
장애물 변경으로 주변 추가 비용이 달라진 셀까지 반영한다.

가중치 0은 추가 선호를 끈다. 반경은 양수여야 한다. GUI 세부 설정에서 조정 후
다음 실행부터 반영된다. 장애물을 인지하지 못하면 추가 비용도 생기지 않는다.
이는 최근접 거리 선호이며 장애물 개수의 합산이나 넓은 길 선택 보장이 아니다.
좁은 길만 있으면 통과 가능성은 그대로 유지된다.


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

### 미관측 구간의 잠정 계획과 실행 제한 (2026-10-19)

GPP의 `/nomad/global_path`는 미관측(-1)을 포함할 수 있는 **잠정 경로**다.
최종 목표와 원본 전역 경로를 유지하며 LPP 내부 참조만 현재 위치에서 연결되는
관측 통행 가능 구간까지 자른다. 희소한 경로점 사이도 검사하며 yaw를 보존한다.
`LPP_WAIT_OBSERVATION`은 다음 관측을 기다리는 상태이고 목표 도착이 아니다.
별도 frontier 탐색/관측 자세 선택기는 이번 변경에 포함하지 않는다.

`observed_guard.enabled=true`일 때 명령 선택기는 정상/복구 모두 현재 융합 자세에서
명령 조향각의 Ackermann 원호를 검사한다. 미관측/지도 밖은 차량과 바퀴의 외곽을
포함하는 반경 0.65m 원으로 보수적으로 검사한다(차체 .85×.5m, 축간 .72m,
윤거 .6m, 바퀴 반경 .15m/폭 .08m). 기존 >=80 장애물 비용은 이미 팽창됐으므로
다시 이 반경으로 팽창시키지 않고 중심 경로 검사를 유지한다.

안전 속도 상한은 `v*(지도 나이 + .3s) + v²/(2*.4m/s²) + .1m <= 관측 거리`로
계산한다. 측정 속도가 상한을 넘거나 여유가 없으면 정지 명령을 보낸다.
`.4m/s²`는 실차 검증값이 아닌 시뮬레이션 시험 가정이다. 지도 신선도·센서
건강·기존 충돌 검사는 계속 적용된다. 초기 bootstrap free 영역은 기존 출발 가정이며
관측으로 새로 인정한 데이터가 아니다. 이산 지도/모델 오차에 대한 실차 안전 보장은 아니다.

GPP multi 비용의 `gpp.unknown_surcharge`는 미관측 구간 길이에 누적되는 추가 비용이다.
기본 4.5는 유지하며 시험에서는 9.0을 사용한다. 경사/지면 결측 prior와 구분하고
같은 미관측 비용을 두 번 더하지 않는다. 군집 비용/사전지도/기준경로 입력은 없다.


### 새로 진입하는 영역 검사로 수정 (2026-10-19 후속)

위의 반경 .65m 전체 관측 요구는 폐기했다. `observed_guard.half_length=.52`,
`half_width=.40`의 차량 방향에 맞춘 사각형(바퀴 포함)을 사용한다.
전진 원호를 검사할 때 시작 차체와 겹치는 미관측 셀만 해당 원호 검사 내에서
예외로 둔다. 예외 집합을 예측 경로를 따라 이동시키지 않는다. 새로 진입하는
미관측/지도 밖 영역은 제한하고 기존 장애물 검사와 제동 여유는 유지한다.
후진에는 이 예외를 적용하지 않는다. 지도 데이터 자체를 free로 바꾸거나
관측 기록을 생성하지 않는다. 기존 LPP/명령의 중심 경로 충돌·신선도 검사도 유지한다.

### Observed obstacle cluster preference (multi mode, opt-in)
`gpp.cluster_weight` (default 0) and `gpp.cluster_radius` (default 4 m)
add `wC * rho_R` per metre to the existing multi-factor GPP objective.
`rho_R` is source occupancy==100 area divided by the full discrete disk area.
Inflated cells 80..99 and unknown cells are not obstacle sources. This is observed
obstacle **area density**, not a count of trees or inferred unseen forest density.
The fixed disk denominator includes unseen/out-of-map space; this is a lower-bound
observation, not evidence of free space. Known neighbouring sources also penalize
unknown destinations, in addition to the existing unknown surcharge. Hard blocked
cells remain infinite and the LPP observation guard remains active. Radius and
weight affect soft route preference only, not occupancy or collision geometry.

Influence-area variant: `gpp.cluster_influence > 0` first takes the union of
metric disks around observed source cells (==100); `gpp.cluster_saturation`
normalizes disk coverage, clipped at 1. Proposed trials use influence .75/1/1.5m,
assessment radius 4m, saturation .4, weights 4/8/16. This soft field does NOT
change occupancy, hard inflation, observation guards or unseen obstacle labels.
Zero influence and saturation 1 preserve the previous area-density definition.

RPP는 보간 수치 오차만 최대 1e-5 rad 허용하고 조향을 차량 한계 내로 제한한 뒤 속도·궤적·충돌 검사를 수행한다. 실제 추종 오차로 인한 큰 조향 초과는 계속 거부한다.

미관측 방향 비용(옵트인): `gpp.unknown_direction_weight` 기본 0. `radius` 기본 6m, `free_angle_deg` 기본 30°. 미관측 셀 중심의 방향과 현재 차량 yaw 사이 각도 θ에 대해 `w * clip((|θ|-30°)/60°,0,1) * clip(1-d/6,0,1)`를 단위거리 비용에 더한다. 관측된 셀/차단 셀은 추가 비용 0이며 기존 차단은 유지한다. 사전지도·좌우 편향·카메라 시야 보장 없이 관측 지도와 융합 자세만 사용한다. 관측 처리된 장애물 미확정 영역에는 적용되지 않는다.


## 관측 통로 목표 선택 (2026-10-19, opt-in)

`navigation.goal_mode: observed_corridor`는 관측된 통행 가능 공간에서 다음 임시 목표를 고른 뒤 기존 GPP/LPP를 사용한다. `direct`가 기본값이며 최종 목적지까지 계획하던 기존 동작이다. 모드는 `dstar_lite_gpp`, `nomad_planning_command`, `nomad_recovery_supervisor` 세 노드에 동일하게 설정해야 한다.

최종 목표 `/nomad/goal`은 보존한다. 새 `/nomad/navigation_target`(PoseStamped, transient-local depth1)은 GPP가 검증한 임시 끝점이며 `/nomad/global_path`는 그 끝점까지다. `/nomad/corridor_status`(String JSON)는 후보/점수/실패를 제공한다. 두 토픽은 bringup의 topics.yaml에 등록되어 있다. 명령 선택은 최종 도착 판정을 유지하고 복구는 임시 목표 변경 시 주행 이력을 지우지 않는다.

선택기는 관측된 셀(0..79)만 연결하며 8m 범위·끝점 여유 .7m·접근 1m로 갱신한다. 미관측 공간을 가짜 free로 만들지 않는다. 점수는 폭·거리·진행 방향·재방문을 반영하며 최초값이지 학습된 최적값은 아니다. 선택 .35초/20,000 확장, 후보별 GPP 2초/30,000 확장·최대3후보, 실패 후보3 simulation 초 후 재검사. 시간 제한은 엄격한 실시간 보장이 아니다. 센서 최신성/기존 실행 안전 검사를 유지하고 후보 부재 시 정지한다.

제한시간 실험은 기존 실행기 `src/nomad_bringup/experiments/optimization.py`에 `--goal-mode observed_corridor`를 추가한다. `--goal-mode direct`로 복귀한다. 전체 조건/재현 명령/결과: `experiments/map4_observed_corridor_20261019/구현_검증.md`. 이 기능은 관측되지 않은 장애물 뒤편을 알지 못하며 완주·기준 경로 재현·실차 안전을 보장하지 않는다.

관측 통로 모드의 최종 GPP 경로는 `min(1.5×horizon, 1.8×격자접근거리+1m)` 길이 이하만 허용한다(기본 최대12m). 가까운 임시 목표에 대한 긴 루프 경로를 그대로 실행하지 않는다.

## Map04 후속 옵션 (2026-10-20)

통로 후보는 `corridor.risk_weight`(기본0)로 GPP 합성 가중치의 정규화 위험 `clip((weight-1)/risk_scale,0,4)`을 접근 경로 적분/평균에 반영한다(`risk_scale` 기본50). `corridor.bearing_weight`(기본0)는 현재 yaw와 실제 후보 방위각 차이/π에 비용을 준다. `corridor.min_clearance`(기본0m)는 >=80셀 중심까지의 거리가 설정값 이하인 관측 셀을 계획 복사본에서 차단한다. unknown은 유지되며 차체 여유 보장은 아니다. 0.4m 실험은 실패하여 채택하지 않았다.

`rpp.adaptive_lookahead`(기본false)는 기존 추종거리 실패 시 .6/.8/1/1.2/1.5m를 기본값과 가까운 순서로 검사한다. 모든 후보에 기존 조향 한계와 1.5m 예측 충돌 검사를 적용하고 전부 실패하면 정지한다. ROS 경로·제어 계약은 유지한다.

실험 실행기의 `--trial-wall-seconds`(기본1800,30..7200), `--trial-sim-seconds`(기본600,30..3600)는 유한 상한이며 엄격한 실시간 보장이 아니다. 결과와 복귀: `experiments/map4_goal_followup_20261020/결과.md`. Map04 단일 목표 도착 판정 성공이 있으나 terrain 소속 rock/바퀴 접촉은 기존 감시기가 허용하므로 무충돌 안전성 증명이 아니다.

### 관측 부족 방향 회전 억제 시험 (2026-10-20)
옵트인 corridor.observation_turn_weight(기본0), corridor.observation_guard(기본false). 동일촬영/geometry의 camera_observation_support를 받아 미지원방향15deg 초과에 soft cost를 부여하고, 전진경로1.1~2.0m에서 지원도>=.35인 샘플비율60%를 요구한다. 근거리사각/후방을 관측free로 만들지 않으며 기존 전체경로 충돌검사는 유지. support는 반복 RGBD 셀 샘플링 근거이며 free-volume 보장이 아니다. 시험 결과는 experiments/map4_observation_turn_20261020/결과.md. 완주 실패, 기본 활성화하지 않음.

### 최종 목표 접근 (2026-10-20)

`observed_corridor`는 `corridor.final_approach_radius: 3.0`m 안에 들어오면 최종 목표를 고정한다. 이후 목표가 뒤쪽이 되거나 반경 밖으로 나가도 중간 경유점으로 전환하지 않는다. 경로 계산 실패/시간 초과/차단/미관측이면 기존 빈 경로 정지 후 같은 목표를 재계획한다. 새 목표를 받으면 고정을 해제한다. 도착 허용35cm, 관측·충돌·운동학 검사는 유지한다. `0.0`은 이전 동작 재현용이며 `direct` 모드는 영향 없다. 이번 수정은 정지·재계획 정책으로, 도달 불가능한 목표의 성공을 보장하지 않는다.

## 안전한 기존 경로 유지 (2026-10-20)

일반 인지지도 실행은 `corridor.retain_safe_path=true`를 적용한다. 기존 경로의 known/free·gate·위치/방향 정렬·입력 최신성을 재검사해 유지하고, 새로운 경로에만 반복 관측지원 비율을 요구한다. 경로 이탈(25cm/15도), 경유점 도착, 최종목표 전환, LPP 실패는 재계산한다. LPP 안전 검사는 유지한다. 세부 검증·복구 기록: `experiments/path_retention_20261020/README.md`.

### 경유점 도달 가능성 재검사
경유점 좌표 자체 또는 남은 경로가 차단/미관측으로 바뀌면 기존 목표와 경로 캐시를 함께 해제하고 즉시 빈 경로를 발행한 뒤 재탐색한다. 최종목표 잠금은 유지한다. 승인 당시 대비 남은 경로의 최대 정규화 경사비용 증가가 `corridor.retained_slope_increase`(기본0.2) 이상이면 경유점을 재평가한다. 기존 경로 유지가 등판 가능성을 보장하지는 않는다. 기록: `experiments/waypoint_reachability_20261020/README.md`.
