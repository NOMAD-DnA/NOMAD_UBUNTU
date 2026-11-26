# GPP(Hybrid A* · 경유점 선택) 작업 기록 — 2026-11-25

`docs/jetson_planning_snapshot_20261125/`(커밋 eb4a2c6)에 올라간 Jetson 소스가 **최종 상태**다. 이 폴더는 그 상태에 이르기까지 GPP에 적용한 변경을 기능별 패치로 보존한 기록이다. 소스 자체는 바꾸지 않는다.

- 패치 기준 경로: 패키지 폴더(`nomad_planning_core/`, `nomad_planning_policy/`, `nomad_planning_cpp/`). 이 저장소에 적용하려면 `git apply -p1 --directory=src <patch>`.
- 기준선: Jetson `~/nomad_planning_ws/src` 11-24 버전. 아래 순서대로 차례로 적용하면 스냅샷 소스와 같아진다(실험 패치 제외).
- 새 기능은 라이브러리 기본값을 이전 동작으로 두고, ROS 노드(`planner_node`) 파라미터 기본값으로 켠다.
- LPP(RPP·경로 추종) 변경은 이 기록에 포함하지 않는다.

## 적용 순서

| 순서 | 시각(KST) | 패치 | 내용 | 주요 파라미터(노드 기본값) | 테스트 |
|---|---|---|---|---|---|
| 1 | 11-25 12:00 | `waypoint_reissue_20261125.patch` | 안전한 경로를 따라 일정 거리 진행하면 경유점을 다시 고르는 재발행. 기본 꺼짐 | `corridor.reissue_enabled` false | `waypoint_reissue_test` |
| 2 | 11-25 14:46 | `waypoint_handover_20261125.patch` | 경유점 도착 시 즉시 재계획, `replan_period` 파라미터화 | `replan_period` | `waypoint_handover_test` |
| 3 | 11-25 15:56 | `waypoint_reissue_log_20261125.patch` | 재발행 시도마다 `[GPP_REISSUE]` 로그 | — | `waypoint_reissue_test` |
| 4 | 11-25 16:23 | `reissue_search_0.5s_20261125.patch` | 재발행 탐색 시간 한도 0.3 → 0.5 s | `corridor.reissue_search_seconds` 0.5 | — |
| 5 | 11-25 18:30 | `goal_cost_scoring_20261125.patch` | 경유점 방향 점수에 장애물 반영(장애물 뒤 목표 방향 경유점 회피) | `corridor.goal_cost_scoring` true, `corridor.goal_cost_clearance_weight` 1 | `goal_cost_scoring_test` |
| 6 | 11-25 18:43 | `clearance_factor_20261125.patch` | 장애물 근접 상대 비용 w = base·(1+F·q) + clearance_weight·q (코너 붙기 방지) | `cost.clearance_factor` 2 | `clearance_factor_test` |
| 7 | 11-25 19:47 | `goal_approach_20261125.patch` | 최종 목표 접근(0.30 m 도착, 2 m 접근 구역), GPP 조향 여유 0.85, 막다른 경유점 검사 | `corridor.final_goal_tolerance` 0.30, `corridor.goal_approach_radius` 2.0, `corridor.gpp_steer_ratio` 0.85, `corridor.exit_check_length` 0.6 | `goal_approach_test` |
| 8 | 11-25 20:52 | `start_exempt_20261125.patch` | 차 주변 반경 안에서는 `min_clearance` 띠를 적용하지 않음(띠 안에서 출발 가능). 비용지도 자체 팽창(비용 ≥ 80)과 차체 박스 검사는 그대로 | `corridor.start_clearance_exempt` 0.2 | `start_exempt_test` |
| — | 11-25 | `preferred_clearance_experiment_NOT_DEPLOYED_20261125.patch` | 선호 여유(0.30 m) 우선 탐색 실험. **도착률이 떨어져 미적용** | — | — |

## 참고

- `경유점_재발행_구현계획.md`: 1번 재발행 기능의 설계 문서(인터페이스 불변 원칙, 설정 항목).
- 각 단계 적용 전 Jetson 백업: `~/nomad_planning_ws/backups/<기능>_before_20261125_*`.
- 주행 기록(`.nomad`), 로그, 시뮬레이션 결과 파일은 포함하지 않는다.
- 2026-11-26 실차 기록 분석: 중간 경유점 도착 거리 `corridor.arrival_tolerance`가 보드 설정에서 1.5 m로 되어 있어, 첫 경유점이 바로 "도착" 처리되며 출발 시 정지·재출발이 반복되고 코너에서 재탐색이 시간 한도를 넘어 멈추는 현상이 있었다. 권장값 0.25 m(설정 변경만 필요).
