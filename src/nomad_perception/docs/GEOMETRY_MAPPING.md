# 경사 비용 지도

기존 `nomad_perception` ROS 패키지 안의 `nomad_geometry_mapping` Python 모듈이다.
원본은 `nomad_ws_geometry_mapping/src/nomad_geometry_mapping`이며, 비용 계산과
평면 품질 기준은 원본 구현을 유지했다. 실행 이름과 설치 경로를 통합 패키지에 맞췄다.

## 빌드와 실행

워크스페이스에서 한 번 빌드한다.

```bash
cd ~/nomad_ws
source .nomad/env.sh
colcon build --base-paths src --packages-up-to nomad_perception nomad_gazebo --symlink-install
```

첫 번째 터미널에서 경사 매핑 전용 통신 영역을 설정한다.

```bash
cd ~/nomad_ws
export NOMAD_ROS_DOMAIN_ID=91 NOMAD_GZ_PARTITION=nomad_geometry_mapping_91
source .nomad/env.sh
ros2 run nomad_perception geometry_session --map 7
```

맵 번호를 생략하면 선택 메뉴가 나온다. `--list`는 맵 목록만 출력한다.
`--headless`, `--no-rviz`, `--bare`도 지원한다.

두 번째 터미널에서 같은 환경을 설정한 뒤 직접 운전한다.

```bash
cd ~/nomad_ws
export NOMAD_ROS_DOMAIN_ID=91 NOMAD_GZ_PARTITION=nomad_geometry_mapping_91
source .nomad/env.sh
./scripts/run_teleop.sh
```

W/S는 속도, A/D는 조향, Space는 정지다. 실행 중인 다른 시뮬레이션은
자동 종료하지 않는다. 세션은 자기 실행에서 시작한 프로세스만 관리한다.

## 계산과 출력

입력은 깊이와 Gazebo GOOSE64 정답 클래스로 만든 `/nomad/semantic_points`다.
촬영 시각의 TF로 `odom`에 정합한다. 기본 셀은 0.2m이고 원본 지면점의
반경 0.3m 이웃에서 TLS 평면을 적합한다. 지지점 분포가 부족하면 연결된
관측 높이로 재추정하고, 모순되지 않는 기존 평면만 유지한다.

설정은 [`../config/geometry_mapping.yaml`](../config/geometry_mapping.yaml)이다.
현재 유효한 20도 미만 경사의 비용은 다음 식을 따른다.

```text
min(99, floor(99 * (slope_deg / 20)^2 * (1 + 0.25 * (1 - effective_q_g)) + 0.5))
```

20도 이상 또는 추정 불가는 100이고 미관측은 -1이다. 주변 8셀의 원본
신뢰도 보정과 평면 품질 검사는 원본 설정대로 적용한다. 신뢰도는 내부
지지량 지표다. 출력은 팽창 전 경사 비용으로, 모든 장애물을 나타내는
자율주행 안전 지도와 다르다.

`/manual_mapping/costmap`, 경사각, 진단과 실제 궤적을 발행한다.
기존 `perception` 실행 항목과 Planning 입력에는 자동 연결하지 않는다.
RViz에서 경사 비용, 셀별 숫자와 실제 궤적을 확인한다.

## 종료와 저장본 확인

Ctrl+C 또는 다음 명령으로 이번 세션을 종료하고 최종 저장한다.

```bash
ros2 run nomad_perception geometry_session stop
ros2 run nomad_perception geometry_session list
ros2 run nomad_perception geometry_session view
```

저장 위치는 `~/nomad_ws/sessions/geometry/`이고, 의미 지도 기록과 구분한다.
`view`에 기록 폴더를 지정하면 해당 저장본을 Gazebo 없이 표시한다.
최종 `map.npz`, 체크포인트, CSV, PNG와 숫자 SVG에 비용, 경사, 지지자료,
설정과 궤적을 기록한다. 기존 저장본은 자동으로 변환하지 않는다.
