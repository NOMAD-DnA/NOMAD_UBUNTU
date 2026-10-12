# 의미 비용 지도

기존 `nomad_perception` ROS 패키지 안의 `nomad_semantic_mapping` Python 모듈이다.
원본은 `nomad_ws_semantic_mapping/src/nomad_semantic_mapping`이며, 2026-10-12의
점 만료 처리와 신뢰도 비용 보정을 포함한다. 실행 이름과 설치 경로를 통합했다.

## 빌드와 실행

```bash
cd ~/nomad_ws
source .nomad/env.sh
colcon build --base-paths src --packages-up-to nomad_perception nomad_gazebo --symlink-install
```

첫 번째 터미널에서 의미 매핑 전용 환경으로 실행한다.

```bash
cd ~/nomad_ws
export NOMAD_ROS_DOMAIN_ID=92 NOMAD_GZ_PARTITION=nomad_semantic_mapping_92
source .nomad/env.sh
ros2 run nomad_perception semantic_session --map 7
```

맵 번호를 생략하면 메뉴가 나온다. `--list`는 맵 목록만 출력한다.
`--headless`, `--no-rviz`, `--bare`도 지원한다.

두 번째 터미널에서 같은 환경으로 수동 조종한다.

```bash
cd ~/nomad_ws
export NOMAD_ROS_DOMAIN_ID=92 NOMAD_GZ_PARTITION=nomad_semantic_mapping_92
source .nomad/env.sh
./run_teleop.sh
```

W/S는 속도, A/D는 조향, Space는 정지다. 실행 중인 다른 시뮬레이션은
자동 종료하지 않고, 세션 관리자는 자기 실행의 프로세스만 관리한다.

## 계산과 출력

입력은 `/nomad/semantic_points`이고 원본 클래스 ID 0~63을 보존한다.
현재 시뮬레이션 공급자는 Gazebo 정답 라벨이며 학습 모델 추론 결과가 아니다.
촬영 시각 TF로 `odom`에 정합한다. 기본 셀 크기는 0.2m다.

설정은 [`../config/semantic_mapping.yaml`](../config/semantic_mapping.yaml),
클래스 비용표는 [`../config/semantic_class_costs.json`](../config/semantic_class_costs.json)이다.
undefined(0), ego_vehicle(8), sky(53), outlier(56)는 비용에서 제외한다.

```text
S = sum(class_count * class_cost) / sum(eligible_class_count)
q_s = min(1, eligible_class_count / 20)
S_adj = min(99, S + 0.25 * (1 - effective_q_s) * (99 - S))
output_cost = floor(S_adj + 0.5)
```

원래 S가 100이면 100을 유지하고 미관측은 -1이다. 대표 클래스 하나의
비용으로 평균 비용을 대체하지 않는다. 최근의 원본 지지량과 클래스 분포가
유사한 주변 8셀만 비용 계산 신뢰도를 보완한다. 보완된 값을 다시 전파하지 않는다.
신뢰도는 내부 지지량 지표다.

원시 점이 만료되어도 계산 가능한 과거 비용과 대표 클래스를 주행 지도에
유지하며, 새 비용 대상 관측으로 갱신한다. 원시 점 보관량과 비용 지지량을
구분해 기록한다. 경사 계산이나 자율주행 Planning 연결은 이 모듈에서 실행하지 않는다.

`/semantic_mapping/costmap`을 발행한다. 원본 대표 클래스는
`/semantic_mapping/representative_class`, 원시 및 보정 비용은 진단 이미지로
발행한다. RViz에서는 의미 비용과 대표 클래스 색상을 번갈아 확인할 수 있다.

## 종료와 저장본 확인

```bash
ros2 run nomad_perception semantic_session stop
ros2 run nomad_perception semantic_session list
ros2 run nomad_perception semantic_session view
```

Ctrl+C로 종료해도 최종 저장한다. 저장 위치는 `~/nomad_ws/sessions/semantic/`이며
경사 지도 기록과 구분한다. `view`에 기록 폴더를 지정하면 해당 저장본을
Gazebo 없이 표시한다. 최종 NPZ, 체크포인트, CSV, PNG와 숫자 SVG에 비용,
원본 클래스, 지지량, 계산 시각, 신뢰도 보정과 실제 궤적을 기록한다.
