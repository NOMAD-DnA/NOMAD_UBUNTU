# 차량 제어

```bash
./scripts/run_module.sh control  # Planning 명령 → 구동 출력·실측 피드백
```

입력 `/nomad/planning/drive_command`, 출력 `/cmd_vel`, 피드백 `/nomad/control/vehicle_state`.
Planning 속도는 비조향 앞축 m/s, 조향은 등가각 rad(전진 좌회전 양수)다. 실측 뒤 관절각을 공통 운동학으로 역산한다.
Gazebo 출력은 Twist(v, v·tan(등가각)/wheelbase)이며 뒤 관절의 부호 변환은 Gazebo 믹서가 한 번만 수행한다.
설정: `config/parameters.yaml`, 공통 차량 기하: `../nomad_bringup/config/vehicle.yaml`.
지도·경로·목표 판단은 하지 않는다. 만료·정지·피드백 계약은 [상세](docs/archive/README_details.md).

## Pi 4 VESC 실차 backend

`vesc_controller`는 `/nomad/planning/drive_command`를 VESC ERPM·서보 위치 명령으로 바꾼다. Pi 4의 기존 `vesc_driver`를 사용한다. `real_vesc.yaml`은 기본 비구동이며, `scripts/run_pi4_vesc_cycle.sh`는 VESC 펌웨어에서 모터·배터리 입력 전류 상한이 각각 30A 이하이고 명령 타임아웃이 300ms 이하임을 읽어 확인한 뒤에만 구동을 활성화한다. `current_limit_a=30`은 telemetry 초과를 감지하면 정지 상태를 래치하는 추가 검사다. ROS 드라이버의 `current_max`는 전류 **명령** 범위라서 ERPM 주행의 모터 전류 한도를 보장하지 않는다.

Control의 `max_speed: 0.0` 잠금은 제거했다. 속도 목표는 Planning에서 정하고 Control은 별도의 0 m/s 한도로 거부하지 않는다. 현재 Planning의 목표 속도 설정은 별도로 유지된다. 전류 한도만으로 차량 속도가 제한되는 것은 아니다. `max_steer=0.4`는 시험 후보값이다.

4614.0 ERPM/(m/s), -1.2135 servo/rad, 중립 0.5304는 기존 F1TENTH 예제에서 복사한 후보이며 이 차의 보정 실측값이 아니다. 바퀴를 든 시험에서 +0.10 듀티가 새 전방 기준 전진인 것만 확인했다. 양수 ERPM·서보 각도 변환은 별도 실물 확인이 필요하다. `trial_candidate_conversion=true`는 이 후보를 명시적으로 쓰는 단기 시험 옵션이고 `calibration_valid=false`는 그대로 유지한다.

VESC에는 조향 관절 센서가 없으므로 `VehicleState.steering_valid=false`를 유지한다. Planning selector는 기본적으로 이 상태에서 정지하고, 실차 한 사이클 시험에서만 `allow_unmeasured_steering_trial=true`를 명시해 진행할 수 있다. 그 옵션은 조향을 측정했다고 표시하지 않는다.
