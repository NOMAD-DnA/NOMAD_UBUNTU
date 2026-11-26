# GUI 실내 매퍼 소스 보관본

버전: `nomad-indoor-mapper-20261124-obstacle-lock-1`.
GUI에서 선택하는 별도 실내 매퍼의 Jetson 검증 소스입니다. 2026-11-24 회수본 전체 소스76개 파일의 SHA-256을 확인했고, 2026-11-25 인지 화면 작업 직전 기록의 매퍼 소스와도 대조했습니다. 2026-11-26에는 Jetson SSH 연결이 되지 않아 보드의 현재 상태를 다시 확인하지 못했습니다.

## 구성과 동작

- `ws/src/nomad_indoor_mapper`: C++17 ROS 2 노드, 계산부, 기본 설정, 테스트, nlohmann 헤더와 원본 라이선스.
- `ws/src/nomad_interfaces`: 독립 작업공간용 메시지 패키지.
- `ws/active_config/indoor.json`: 2026-11-25 마지막으로 확인한 보드 설정의 JSON 값. 원본 파일의 공백 형식까지 복제한 파일은 아닙니다.
- `verification`: 기존 ARM 검증과 장애물 고정 켬/끔 합성 ROS 검사 결과. 바이너리와 빌드 생성물은 포함하지 않습니다.

지도 입력은 LiDAR와 VIO입니다. 격자는12.5cm이며 전방 반각은 `lidar_half_fov_deg`로10~100도 사이 소수를 사용합니다. 기본값은80도입니다. 반환 시각의 차량 중심 XY 반경2m 안 실제 반환 종점은 Z와 관계없이 장애물입니다. 종점 전 광선 통과 셀은 반경 안에서 포장면/경사0으로 갱신하며 같은 스캔의 장애물 종점이 우선합니다.

`lock_obstacle_cells=true`이면 같은 지도 세션에서 관측한 장애물을 광선 통과로 해제하지 않습니다. false이면 광선 통과로 해제합니다. 시간 만료로 셀을 해제하지 않지만 기존 지도 창 밖 제거와 셀 상한 정책은 유지합니다. 초기 반경1.5m 자유 공간과0도 가정을 유지합니다. 실제 장애물은 자유 공간 덮어쓰기보다 우선합니다.

기본 설정의 VIO 고정Z는0m이고 보드 설정 사본은0.19m입니다. `floor_z_cm`은 현재 광선 방식에서 분류에 사용하지 않는 호환용 값입니다. 소스에 남아 있는 초기 버전의 `config/indoor_doc.json` 설명과 일부 이전 ROS 시험은 과거 높이 분류 정책을 담고 있습니다. 현행 동작은 이 문서와 `ros_obstacle_lock_integration.py`, `ros_fov_config_integration.py`를 기준으로 구분하세요. 원본 소스의 바이트를 보존하기 위해 과거 설명은 수정하지 않았습니다.

## 독립 빌드

상위 colcon에서 중복 메시지 패키지를 발견하지 않도록 이 보관 폴더에 `COLCON_IGNORE`를 둡니다. 이 폴더에서 `ws`로 이동해 별도로 빌드합니다. Eigen3, colcon 및 ROS 2 의존 패키지가 필요합니다. Jetson의 ROS 설치 위치에 맞는 setup.bash를 먼저 source하세요.

```bash
# 이 README가 있는 폴더에서 실행
source /opt/ros/humble/install/setup.bash
cd ws
colcon build --base-paths src --packages-up-to nomad_indoor_mapper --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
ctest --test-dir build/nomad_indoor_mapper --output-on-failure
```

일반 ROS apt 설치는 `/opt/ros/humble/setup.bash`를 사용합니다. ROS 없는 PC 계산부 검사는 다음과 같습니다.

```bash
cmake -S ws/src/nomad_indoor_mapper -B build-core -DINDOOR_WITH_ROS=OFF -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build build-core -j2
ctest --test-dir build-core --output-on-failure
```

기존 합성 ROS 시험은 원래 배포 폴더의 `ws/install`, `tmp` 및 로그 위치를 가정합니다. 저장소의 임의 위치에서 그대로 실행하는 범용 테스트 런처는 아닙니다. 과거 ARM CTest5개와 켬/끔 합성 ROS30개 통과는 `verification`에 보존한 당시 결과이며 실주행 성능 증거는 아닙니다.

## GUI 연결

GUI와 원격 시작 관리는 [feat/gui의 실물시험앱](https://github.com/NOMAD-DnA/NOMAD_UBUNTU/tree/feat/gui/tools/visual_trial_20261106/laptop_monitor)에 있습니다. GUI는 Jetson의 `/home/nvidia/nomad_indoor_ws` 설치와 `active_config/indoor.json`을 사용합니다. 이 저장소 보관 작업만으로 보드 설치 경로나 GUI를 변경하지 않습니다.

OAK RGB와 ZED indoor5 의미분할은 인지 화면 표시용이며 실내 지도 입력이 아닙니다. 표시 기능의 기존 GUI 커밋은 [967bcb6](https://github.com/NOMAD-DnA/NOMAD_UBUNTU/commit/967bcb6b2b77fc24055d610cb987c1f5769cfc95)입니다. 이 폴더는 매퍼 본체의 소스 보관 범위입니다.
