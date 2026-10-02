# 모듈 분리 검증 — 2026-10-02

## 확인한 범위

- 새 공통 메시지와 모듈 여섯 패키지, Gazebo launch를 포함해 원본 워크스페이스에서 7개 패키지 colcon 빌드 성공.
- 기존 경로·인지 테스트 117개 및 신규 명령/설정/모듈 경계 테스트 포함 148개 통과.
- 단위 검증: 전진/후진 속도·조향 부호, 실측 피드백, 방향 전환 전 정지, 명령 lease/wall timeout, 중복·과거 명령, 같은 시각의 정지 우선, 잘못된 외부 지도, 복구 대기 우선권, 토픽 설정·공급자 선택.
- 실제 Planning ROS 노드를 같은 프로세스에서 구동한 합성 막다른길 시험 통과. 뒤쪽 전역 경로에 전진 중단, 0.40 시뮬레이션 초 뒤 후진, 10.698m 연속 후진 후 분기점 탈출, 후방 장애물 정지/재개, REVERSE→SWITCH→EXIT→FOLLOW, 원래 목표 (18,5) 도착. 최종 위치 (17.720,4.862). 이 시험의 Control/차량은 운동학 fixture다.
- 별도 프로세스 ROS 연결 시험 통과: 모든 토픽을 /interface_test/...로 변경하고 외부 인지·VIO stub → 실제 GPP/LPP → DriveCommand → 실제 Control/Twist 연결, 실측 피드백, 인지 READY 해제 시 정지를 확인했다. Control의 구독 목록에 Path/costmap이 없고 /cmd_vel 토픽을 사용하지 않는 것도 확인했다.

실행 중인 사용자 Gazebo는 중단·초기화하지 않았다. 이 변경에 대해 실제 Gazebo 숲에서 차량을 움직이는 물리 시험이나 실차/PWM 검증은 수행하지 않았다.

- 기본 네 모듈(8개 노드)을 모두 실행한 합성 센서 시험 통과: 목표 전 정지 → 전진 → 목표 도착 후 정지, 이동 2.667m, 모든 경로 odom frame, 센서 입력 중단 시 정지. 이 검사는 테스트에서만 LiDAR 융합 모드/terrain 비활성 override를 사용해 카메라 계산과 모듈 배선을 분리했다. 기본 카메라 설정은 바꾸지 않았다.

## 재현

```bash
cd ~/nomad_ws
source .nomad/env.sh
python3 -m pytest -q src/nomad_path_planning/test src/nomad_perception/test src/nomad_control/test src/nomad_bringup/test
# 0/42 이외의 비어 있는 domain을 지정. 스크립트가 기존 노드를 발견하면 중단한다.
ROS_DOMAIN_ID=86 python3 src/nomad_bringup/scripts/verify_interfaces.py
ROS_DOMAIN_ID=87 python3 src/nomad_path_planning/scripts/verify_history_recovery.py
```

verify_interfaces는 모든 모듈 토픽 이름을 /interface_test/...로 바꾸고 외부 인지·VIO stub에 실제 Planning/Control을 연결한다. verify_history_recovery는 실제 GPP/LPP/복구/명령 선택기에 합성 지도·측정·운동학을 연결한다. 둘 다 사용자 /cmd_vel에 차량 명령을 보내지 않는다.
