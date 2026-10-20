# 센서·차량·맵 시뮬레이션

```bash
./scripts/run_module.sh sim     # 맵 선택·센서·차량, 위치TF는외부융합
./scripts/run_module.sh teleop  # 수동 초기정렬; Space정지, 창종료후Control사용
```

GT 기반 수동실험은 `./scripts/run_forest.sh`. 실제융합과 동적TF를 동시에 켜지 않는다.

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

## 카메라 프로파일

기본은 ZED 2 최대 화각의 이상적 핀홀 근사: 가로110°·세로70°, 양안 간격12cm. 640×480/30Hz는 리샘플링 운용 설정이며 네이티브 ZED 모드나 실물 보정이 아니다. 깊이0.4–8m/IMU200Hz는 기존 시뮬레이션 운용값을 유지한다. `/oak` 토픽·프레임명은 호환 인터페이스다.

`forest.launch.py`는 `config/sensors.yaml`로 원본 맵의 실행용 복사본에 센서를 적용하고 동일 입력에서 정적TF를 생성한다. 원본 맵, 차량 물리 및 추가 실험용 contact 센서는 보존한다. GUI도 같은 생성 함수를 사용한다. 기본 Basalt/OpenVINS 보정은 해당 기본 프로파일과 일치한다. 다른 `sensor_config`로 실행할 때 VIO 보정도 함께 제공해야 한다. GUI의 센서 설정은 일치하는 VIO 보정을 자동 생성한다. 이전 OAK 입력은 `config/sensors_oak_d_lite.yaml`에 보존했다.
