# 차량 제어

```bash
./scripts/run_module.sh control  # Planning 명령 → 구동 출력·실측 피드백
```

입력 `/nomad/planning/drive_command`, 출력 `/cmd_vel`, 피드백 `/nomad/control/vehicle_state`.
Planning 속도는 비조향 앞축 m/s, 조향은 등가각 rad(전진 좌회전 양수)다. 실측 뒤 관절각을 공통 운동학으로 역산한다.
Gazebo 출력은 Twist(v, v·tan(등가각)/wheelbase)이며 뒤 관절의 부호 변환은 Gazebo 믹서가 한 번만 수행한다.
설정: `config/parameters.yaml`, 공통 차량 기하: `../nomad_bringup/config/vehicle.yaml`.
지도·경로·목표 판단은 하지 않는다. 만료·정지·피드백 계약은 [상세](docs/archive/README_details.md).
