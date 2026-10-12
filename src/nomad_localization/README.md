# NOMAD GNSS/VIO Ceres backend

팀 실행 진입점는 `ros2 launch nomad_vio fusion.launch.py`. [VIO 실행/계약](../nomad_vio/README.md) 참조.

`src/fusion_node.cpp`: 상대 VIO SE(3) + GNSS 안테나 위치 + 보존 경계 prior, robust Ceres LM 최적화. raw IMU/feature/GNSS bias 상태 없음. 최근 10초 포즈, 5Hz keyframe, 2Hz solve, 최대10iteration/50ms/1thread. 경계 prior는 정확 marginalization이 아닌 근사. posterior covariance는 계산하지 않음.

`scripts/vio_supervisor.py`: Basalt/OpenVINS 공통 pose 입력·소유 프로세스 epoch/재시작. 기존 사용자 프로세스를 종료하지 않음. 기본 launch는 manage_vio/auto_restart=false. `launch/basalt.launch.py`는 설치된 외부 Basalt를 구성할 때 사용.

`config/fusion.yaml`: 입력 토픽·datum·lever arm·윈도·sigma·실패 판단. 기본 GPS `/gps/fix`, 품질 `/gps/quality`(미제공 unknown 허용). 실험용 품질은 별도 오류노드의 `/experiment/gps/quality`로 지정.

설치 의존성: ROS2 Jazzy, Ceres/Eigen (`libceres-dev`, Eigen3), nomad_interfaces. `rosdep install --from-paths src --ignore-src -r -y` 후 VIO README 빌드 명령 사용. Basalt/OpenVINS는 `.nomad/install_vio.sh`로 설치하며 Gazebo 보정 파일은 nomad_vio/config/vio에 제공한다.
