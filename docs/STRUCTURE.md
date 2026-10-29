# 통합 구조

```text
양안 + IMU → nomad_vio / nomad_localization ─┐
GNSS ───────────────────────────────────────┴→ 위치·자세
RGB → nomad_semantic → 라벨 ─┐                    │
깊이 ───────────────────────┴→ nomad_perception ←┘
                                ↓ 전역/지역 코스트맵
                         nomad_path_planning
                                ↓ 속도·조향각
                            nomad_control → 차량
```

| 파트 | 핵심 코드 | 설정 |
|---|---|---|
| VIO 실행·공통 출력 | `nomad_vio`, `nomad_localization/scripts/vio_supervisor.py` | `nomad_vio/config/vio/` |
| GNSS/VIO 융합 | `nomad_localization/src/` | `nomad_localization/config/fusion.yaml` |
| 분할 | `nomad_semantic/node.py`, `runtime.py` | `nomad_semantic/models/nomad_b0/metadata.json` |
| 지면·클래스 누적 | `nomad_perception/terrain_layers.py`, `quality_mapping_core.py` | `nomad_perception/config/navigation_terrain.yaml` |
| 비용·관측 기억 | `nomad_perception/navigation_costmap.py` | 같은 YAML: 20cm, 120cm 이차 장애물 비용 |
| 인지 ROS 입출력 | `live_terrain.py`, `navigation_terrain.py` | 같은 YAML |
| GPP·LPP·복구 | `nomad_path_planning/gpp/`, `lpp/`, `recovery_supervisor.py` | `config/planning.yaml`, 통합 override `terrain_planning.yaml` |
| 제어 | `nomad_control/` | `config/parameters.yaml` |

패키지 경로는 모두 `src/` 아래. 토픽 계약은 `nomad_bringup/config/topics.yaml`, 공통 차량 기하는 `config/vehicle.yaml`.
융합만 `odom → base_link` 동적 TF를 발행한다. `/odom`은 Gazebo GT와 제어 피드백용이며 플래닝 위치는 융합 출력이다.
실험·학습·이전 기록은 실행 경로에 넣지 않는다. 생성 결과는 `sessions/`, 빌드는 `build/install/log`, 이전 자료는 `docs/archive/` 또는 GPT 기록에 보관한다.

실행 진입점은 `scripts/`, 사용자 안내는 `docs/실행안내.md`, 최초 설치 도구는 `.nomad/`에 모은다.

플래닝 단독 검증은 `scripts/run_scenario_lab.sh` → `nomad_bringup/nomad_bringup/scenario_lab/`.
GT 관측 입력·6개 맵 생성·실행 큐·결과 GUI를 묶고 기존 GPP/LPP/Control을 사용한다.
생성 맵과 결과는 `sessions/planning_lab/`, 상세 안내는 [플래닝 시나리오 실험실](플래닝_시나리오_실험실.md).


알고리즘 5조합 × 6시나리오 비교는 `scripts/run_algorithm_comparison.sh` → `알고리즘_비교실험/`. 기존 scenario_lab 맵·물리·runner를 공유하고 결과는 해당 폴더의 `results/`에 분리한다.
