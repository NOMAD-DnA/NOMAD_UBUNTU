# 센서·차량·맵 시뮬레이션

```bash
./run_module.sh sim     # 맵 선택·센서·차량, 위치TF는외부융합
./run_module.sh teleop  # 수동 초기정렬; Space정지, 창종료후Control사용
```

GT 기반 수동실험은 `./run_forest.sh`. 실제융합과 동적TF를 동시에 켜지 않는다.

| 대상 | 설정·코드 |
|---|---|
| 맵1–8 | `maps/`, `scripts/select_map.py` |
| 차량·센서 | `models/nomad_vehicle/model.sdf`, `config/sensors.yaml`, `urdf/` |
| Transport↔ROS | `config/bridge.yaml`, `launch/forest.launch.py` |
| GNSS 기본값 | `/gps/fix`; 오류모사 `scripts/gnss_fault_injector.py`·`config/gnss_sim.yaml`,기본자동실행off |
| 구동명령 제한 | `scripts/command_watchdog.py` |

ROS센서: `/oak/{rgb,depth,left,right}/...`, `/oak/imu`, `/scan`, `/gps/fix`.
`/odom`은 Gazebo GT, 카메라·IMU의 최종 실물보정은별도. 기본최대속도1m/s.
생성된파일만고치지말고authoring·생성기입력을함께관리한다. 기존맵·물리·GNSS 세부설명은 [상세](docs/archive/README_details.md).
