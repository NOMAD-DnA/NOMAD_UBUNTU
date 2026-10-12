# NOMAD

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic. 위치추정 → 인지 지도 → 플래닝 → 제어를 독립 실행할 수 있는 차량 시뮬레이션.

- **처음 실행:** [실행안내](실행안내.md)
- **코드·설정 위치:** [구조](docs/STRUCTURE.md)

```bash
./run_terrain_navigation.sh viewer:=false  # 맵·플래너 선택 → B0·융합·지도·제어·RViz
./run_module.sh help                      # 각 파트 독립 실행
```

| 파트 | 문서 |
|---|---|
| 센서·차량·맵 | [nomad_gazebo](src/nomad_gazebo/README.md) |
| VIO·GNSS 융합 | [nomad_vio](src/nomad_vio/README.md) · [융합](src/nomad_localization/README.md) |
| 의미분할 | [nomad_semantic](src/nomad_semantic/README.md) |
| 원본 점·경사·코스트맵 | [nomad_perception](src/nomad_perception/README.md) |
| GPP·LPP·후진 복구 | [nomad_path_planning](src/nomad_path_planning/README.md) |
| 차량 제어 | [nomad_control](src/nomad_control/README.md) |
| 토픽·메시지 계약 | [nomad_bringup](src/nomad_bringup/README.md) · [nomad_interfaces](src/nomad_interfaces/README.md) |

기존 GT 기반 실행은 `./run_autonomy.sh`, 수동 시뮬레이션은 `./run_forest.sh`. 실제 VIO 융합 실행과 동시에 켜지 않는다.
기존 설계·시험 기록은 [보관 문서](docs/archive/). 빌드·실행 생성물은 Git에 포함하지 않는다.
현재 막다른 길 후진 복구 이력과 지도 지연 문제는 남아 있다. 단독 연결·일반 전진 주행 확인을 완주 보장으로 해석하지 않는다.
