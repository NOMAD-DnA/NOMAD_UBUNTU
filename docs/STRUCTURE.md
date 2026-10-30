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
| 실물 인지 | `nomad_perception/real/`, 선택 LiDAR 드라이버 `.nomad/real_sensors/` | `config/real/`, `real_perception.launch.py` |
| 인지 ROS 입출력 | `live_terrain.py`, `navigation_terrain.py` | 같은 YAML |
| GPP·LPP·복구 | `nomad_path_planning/gpp/`, `lpp/`, `recovery_supervisor.py` | `config/planning.yaml`, 통합 override `terrain_planning.yaml` |
| 공통 차량 운동학 | `nomad_vehicle_model/` | `nomad_bringup/config/vehicle.yaml`: 앞축 적분·뒤 조향 |
| 제어 | `nomad_control/` | `config/parameters.yaml` |

패키지 경로는 모두 `src/` 아래. 토픽 계약은 `nomad_bringup/config/topics.yaml`, 공통 차량 기하는 `config/vehicle.yaml`.
융합만 `odom → base_link` 동적 TF를 발행한다. `/odom`은 Gazebo GT와 제어 피드백용이며 플래닝 위치는 융합 출력이다.
실험·학습·이전 기록은 실행 경로에 넣지 않는다. 생성 결과는 `sessions/`, 빌드는 `build/install/log`, 이전 자료는 `docs/archive/` 또는 GPT 기록에 보관한다.

실행 진입점은 `scripts/`, 사용자 안내는 `docs/실행안내.md`, 최초 설치 도구는 `.nomad/`에 모은다.

플래닝 단독 검증은 `scripts/run_scenario_lab.sh` → `nomad_bringup/nomad_bringup/scenario_lab/`.
GT 관측 입력·6개 맵 생성·실행 큐·결과 GUI를 묶고 기존 GPP/LPP/Control을 사용한다.
생성 맵과 결과는 `sessions/planning_lab/`, 상세 안내는 [플래닝 시나리오 실험실](플래닝_시나리오_실험실.md).

현재 전륜 구동·후륜 조향은 [차량 실행 안내](후륜조향_시뮬레이션.md) 참조. Planning·Control·맵/GUI가 같은 기하를 사용한다.

실물 인지는 `scripts/run_real_perception.sh`를 사용하며 Pi 5 입력과 외부 센서를 받는다. [입력 계약과 파일 목록](../src/nomad_perception/docs/real_perception.md). 실물 모듈 연결은 `modules_real_perception.yaml`의 external 설정으로 내장 인지 중복 실행을 막는다.
