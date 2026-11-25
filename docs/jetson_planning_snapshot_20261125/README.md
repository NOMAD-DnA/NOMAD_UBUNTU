# Jetson planning source snapshot — 2026-11-25

젯슨 `~/nomad_planning_ws/src`의 최신 소스 46개 파일을 그대로 가져온 백업이다. 원격 `feat/plan`의 기존 시뮬레이션 코드는 보존하고 다음 패키지를 추가했다.

- `src/nomad_planning_core`: 비용장 및 Hybrid A*.
- `src/nomad_planning_policy`: 경유점 선택, RPP, 시작점 여유거리 예외와 회귀 테스트.
- `src/nomad_planning_cpp`: ROS 노드, 실행 스크립트와 프로필.

`source.sha256`은 저장소 루트 기준 경로이며 수집 후 젯슨 원본과 46개 모두 일치함을 확인했다. `planner_board_overrides.yaml`은 수집 시점의 별도 보드 설정을 보존한 참고 자료다. 자동 적용되지 않으며 20:39 주행 당시 설정과 동일하다는 뜻은 아니다. 수집 당시 `corridor.min_clearance`는 0.10m다.

이번 작업은 소스 백업이다. 젯슨의 소스·설정·빌드·실행 프로세스를 변경하지 않았다. 빌드 산출물, 주행 로그, 모델 가중치, 접속 자격 증명은 포함하지 않았다. 최신 로직의 설명 및 과거 검증 주장은 각 패키지의 README를 참고한다.

## 이번 백업 검증

- 노트북 x86_64 / ROS 2 Jazzy에서 세 패키지 Release 빌드 성공. 기존 로컬 `nomad_interfaces` 설치를 의존성으로 사용했다.
- 등록된 CTest 18개 통과(core 1, policy 13, cpp 4). `ros_contract_test` 실행 파일은 빌드했으나 ROS 실통신 시험 및 Jetson 재빌드·실차 시험은 수행하지 않았다.
- 0.10m/0.125m 코너 시나리오는 모두 도착했다. 0.10m right_corner는 일시 정지 6회가 있으므로 무정지 주행으로 해석하지 않는다.
- 패키지 소스 46개는 젯슨 원본과 SHA-256 일치. 이 문서와 설정 스냅샷만 백업용으로 추가했다.
