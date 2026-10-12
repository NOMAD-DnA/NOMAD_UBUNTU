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
