# LPP 지역 우회와 GPP 재계획 협력 — 2026-10-21

## 변경 목적과 범위
사용자 요청: LPP 담당자가 최적화로 전역 경로 밖의 장애물 회피 궤적을 만들 때,
GPP가 정상적인 우회를 실패로 취급해 경유점을 계속 바꾸지 않도록 준비한다.
현재 기본 `observed_corridor` 실행의 GPP 측 수신·검증·재계획 로직을 추가했다.
Hybrid A*와 비용식, 인지, 기존 LPP, 명령 선택·제어의 안전 검사는 변경하지 않는다.
`direct` 모드에서는 이번 협력 메시지를 사용하지 않는다.

```text
기존: 최신 지도 → 기존 경로 차단 → 경로 폐기 → 경유점/전역 경로 재계획
추가: LPP AVOIDING + 최신 검증 궤적 + 유효한 전방 합류 구간
                    ↓
      제한 시간 동안 기준 경로/경유점 유지 (차단 구간을 통행 가능으로 바꾸지 않음)
                    ↓
      TRACKING 복귀 / BLOCKED / 재계획 요청 / 만료 / 진행 정체
                    ↓
      현재 위치·자세에서 기존 비용식으로 재계획
```

## LPP 담당자 연결 계약
새 토픽 `/nomad/planning/lpp_progress`, 타입 `nomad_interfaces/msg/LppProgress`.
Reliable / Volatile / depth 1, 권장 10 Hz. 상태만 자동으로 생성하는 어댑터는 추가하지 않았다.
기존 `/nomad/lpp_status` 문자열과 `/nomad/lpp_failed` Bool은 유지한다.

- `header`: 상태 생성 ROS 시각과 planning_frame (현재 odom).
- `reference_stamp`: 최적화에 사용한 `/nomad/global_path.header.stamp`를 정확히 복사.
  GPP가 새 기준 경로를 보내면 새 stamp를 사용. 이전 경로의 상태는 새 경로에 적용하지 않는다.
- `state`: TRACKING=0 / AVOIDING=1 / BLOCKED=2.
- `request_replan`: 합류 불가·우회 불가 등의 명시적 GPP 재계획 요청.
- `rejoin_index`: 참조 Path의 전방 합류점 인덱스(0부터). AVOIDING 한 회 동안 고정.
  합류점 변경이 필요하면 request_replan. 합류 이후 마지막 경유점까지 알려진 통행 가능 구간이어야 한다.
- `motion`: `/nomad/planning/local_motion`에도 보내는 **동일한** 검증 PlannedMotion.
  상태와 다른 궤적을 발행하지 않는다. base_link 위치·yaw를 planning_frame으로 표현.
  현재 계약은 양의 속도·단일 조향 명령이다. 시간별 속도/조향 궤적 실행으로 확장하는 것은 별도 작업이다.
- 정상 AVOIDING 중 기존 `/nomad/lpp_failed`는 false. true가 함께 오면 실패를 우선한다.
- 최적화 실패·충돌 예상이면 BLOCKED/request_replan과 기존 빈/정지 motion을 함께 발행한다.
  GPP 상태 메시지만으로 차량 정지를 대신하지 않는다.
- 성공적인 재합류 시 TRACKING을 발행하면 현재 위치부터 GPP가 연결 경로를 재계산한다.
  이전 경로 시작점으로 돌아가도록 만들지 않는다.

이번 변경만으로 기존 RPP에 우회 능력이 생기지 않는다. LPP의 `known_prefix` 전처리,
전역 경로가 후방인지 확인하는 command guard 등은 그대로다. 담당자가 새 LPP를 연결할 때
해당 조건으로 정상 우회가 거절되는지도 통합 시험해야 한다. 검사 자체를 끄지 말고
실제 지역 궤적·참조 구간을 기준으로 검증하는 후속 변경을 별도로 검토한다.

## GPP의 유지 허용 조건
1. 최신 상태·motion, 올바른 frame/유한 pose, 현재 참조 경로 stamp 일치.
2. motion 시작 위치 현재 차량에서 0.3 m 이내, yaw 차이 0.35 rad 이내.
3. 속도/조향 한계, 후륜축 환산 곡률·전진 접선 검사. 샘플 간 후륜 간격 ≤0.25 m,
   ≥2점, 길이 ≥0.1 m, 최대500점. 곡률 허용오차0.001/m, 접선 오차0.08 rad.
4. 새 지도에서 지역 궤적 중심선과 합류 이후 전역 경로를 검사. >=80/미관측/지도 밖/진입 gate 차단.
   팽창 지도에 차량 크기를 다시 더하지 않는다. 새로 진입하는 미관측 footprint도 검사한다.
5. 현재 위치에서 참조 샘플까지 최대3 m, 합류점은 최근접 참조 인덱스보다 앞.
6. 고정 합류점과의 실제 거리 감소를 확인. 단순 heartbeat로 정체 타이머를 갱신하지 않는다.

차단된 참조 prefix는 **기준선으로만** 보존하며 안전한 신규 전역 해로 재발행/재타임스탬프하지 않는다.
GPP는 AVOIDING 동안 `GPP_LOCAL_AVOIDANCE` 및 corridor_status `lpp_avoidance_hold`를 보낸다.
취소 시 `GPP_LPP_REPLAN`/`lpp_avoidance_replan`에 사유를 남기고 기존 경로를 비운 뒤 재계획한다.
목표 변경·지도 오류·경로 폐기 시 협력 상태도 초기화한다. 최종 목표 잠금은 유지한다.

이 검사는 최신 격자 기반의 기하 검사이며 움직이는 장애물의 미래 위치 예측이나 충돌 방지 보장이 아니다.
실제 실행은 기존 command selector가 별도 최신 지도·센서·명령 유효기간·관측/정지거리 검사를 적용한다.
GPP의 `hold`는 실행 승인 신호가 아니다.

## 설정과 실행
`config/planning.yaml`의 `dstar_lite_gpp.ros__parameters`:

| 파라미터 | 기본값 | 의미 |
|---|---:|---|
| cooperation.enabled | true | 상태 발행자 없으면 기존 동작 |
| status_timeout | 0.5 | 상태·궤적 ROS초 유효기간 |
| wall_timeout | 1.0 | 새로운 상태 수신 wall초 한계 |
| max_duration / max_wall_duration | 30 / 120 | 한 우회의 ROS초 / wall초 한계 |
| no_progress_timeout | 5.0 | 합류점 방향 진행 없음, ROS초 |
| progress_distance | 0.15 | 진행 갱신 최소 거리 m |
| max_deviation | 3.0 | 참조 경로 샘플과 최대 거리 m |
| max_speed | 0.5 | 협력 허용 상한 m/s; 주행 속도 설정 아님 |

모두 `cooperation.` 접두사. 차량 wheelbase/max_steer/reference_offset는 기존 GPP 파라미터 공유.
시간·거리 값은 초기 협력 정책이며 실차 검증값/최적값이 아니다. 느린 시뮬은 wall 상한에도 걸릴 수 있다.

```bash
cd ~/nomad_ws
source .nomad/env.sh
colcon build --base-paths src --symlink-install --packages-select nomad_interfaces nomad_path_planning nomad_bringup
source install/setup.bash
ros2 interface show nomad_interfaces/msg/LppProgress
# 실행은 기존 GUI/CLI 그대로
./scripts/run_terrain_navigation.sh viewer:=false
# 다른 터미널에서 상태 확인
ros2 topic echo /nomad/corridor_status
```

복구: `cooperation.enabled: false`로 변경 후 planning 재시작. 기존 RPP만 쓰거나 상태 발행자가 없으면
설정을 끄지 않아도 원래 GPP 동작이다. 실행 중 시뮬을 임의 종료하지 않는다.

## 검증 기록
- 빌드: nomad_interfaces / nomad_path_planning / nomad_bringup 3개 성공 (10.5초).
- 초기 회귀: 316 통과 / 13 실패. 새 planning.lpp_progress를 topics.yaml에 추가했으나
  중앙 topic_defaults 등록을 놓쳐 통합 실행 설정 검사가 실패했다.
- 수정: 중앙 기본 토픽 등록, 예전 사용자 topics.yaml에 새 선택적 키가 없어도 기본값으로 보완,
  사용자 지정 remapping 및 예전 파일 호환 테스트 추가. 다른 필수 키 검사는 유지.
- 수정 후 전체 planning + bringup 회귀: **330 passed in 23.94s**.
- 이후 최대 조향각 좌/우 회전 허용 시험 2개 추가. 해당 코어 테스트 **29 passed in 0.16s**.
  전체 330회 이후 프로덕션 코드 변경은 없고 테스트 2개만 추가했다.
- 실제 ROS 송수신: 분리된 domain188에서 LppProgress 수신 및 참조 stamp 전달 확인,
  메시지 수신만으로 우회 허가가 생기지 않음/빈 경로 발행 시 상태 초기화 확인.
  기존 알고리즘 조합 송수신 테스트는 기존 domain187 격리 설정 사용.
- 기존 경로 prefix 차단 상태에서 검증 지역 궤적 허용, suffix 차단/unknown 시 거부,
  stale/replay/clock reset/명시 실패/진행 정체/최대 시간/기하 위반 거부 확인.
- 설치된 `ros2 interface show nomad_interfaces/msg/LppProgress` 성공.
- `git diff --check` 통과. 사용 중인 시뮬레이터를 종료하거나 새 Gazebo를 실행하지 않았다.
- **미검증:** 새 최적화 LPP와 실제 동적 장애물 Gazebo 주행, 장애물 미래 위치 예측,
  정상 우회 시 기존 command guard/known_prefix와의 최종 통합. 완주/안전 성능 향상으로 주장하지 않는다.

검증 명령:
```bash
source .nomad/env.sh
export ROS_DOMAIN_ID=188 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_LOG_DIR=/tmp/nomad_cooperation_ros_logs PYTHONDONTWRITEBYTECODE=1
python3 -m pytest -p no:cacheprovider src/nomad_path_planning/test src/nomad_bringup/test -q
```

변경 파일 (기존 다른 작업의 변경은 보존):
- nomad_interfaces/msg/LppProgress.msg, CMakeLists.txt, README.md
- nomad_bringup/config/topics.yaml, nomad_bringup/topic_defaults.py,
  nomad_bringup/configuration.py, test/test_configuration.py
- nomad_path_planning/nomad_path_planning/lpp_cooperation.py, recovery_gpp.py
- nomad_path_planning/config/planning.yaml, README.md, docs/LPP_GPP_우회협력.md
- nomad_path_planning/test/test_lpp_cooperation.py, test/test_lpp_cooperation_ros.py

파일 경로는 모두 src/ 아래 패키지 기준. Git commit/push는 하지 않았다.
