# 모듈 통합

사용자 명령은 루트 [실행안내](../../실행안내.md). 전체 실행과 단독 실행을 함께 켜지 않는다.

- `config/topics.yaml`: 각 파트 토픽 이름·remap.
- `config/modules.yaml`: 기본 GT 공급자. 실제융합 통합은 `nomad_perception/config/modules_terrain_navigation.yaml`.
- `config/vehicle.yaml`: 공통 좌표·차량기하·시뮬레이션 시각.
- `launch/{planning,control,vio,perception,autonomy}.launch.py`: 공통 설정으로 모듈 조립.
- `nomad_bringup/configuration.py`: 공급자·토픽 계약 검증. `launching.py`: 노드 생성·중복 실행 잠금.

위치 공급자만 `odom → base_link` 동적 TF를 가진다. 메시지 계약은 [nomad_interfaces](../nomad_interfaces/README.md).
기존 교체·실차 설정·실험 사용법은 [상세](docs/archive/README_details.md).
