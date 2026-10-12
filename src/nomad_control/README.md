# 차량 제어

```bash
./run_module.sh control  # Planning 명령 → 구동 출력·실측 피드백
```

입력 `/nomad/planning/drive_command`, 출력 `/cmd_vel`, 피드백 `/nomad/control/vehicle_state`.
Planning의 조향각은 rad, 양수 왼쪽. Gazebo 출력은 linear.x와 angular.z로 변환한다.
설정: `config/parameters.yaml`, 공통 차량 기하: `../nomad_bringup/config/vehicle.yaml`.
지도·경로·목표 판단은 하지 않는다. 만료·정지·피드백 계약은 [상세](docs/archive/README_details.md).
