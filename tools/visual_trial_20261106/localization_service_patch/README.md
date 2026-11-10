# Pi5 native 위치추정 GUI 연동 수정본

이 디렉토리는 이미 설치된 `nomad_localization` 패키지에 적용하는 수정 파일 모음이다. 독립 SDK·전체 패키지 배포본이 아니다. 같은 상대경로를 Pi5 `/home/user/nomad_ws/src/nomad_localization/`에 적용한다. 기존 나머지 소스·SDK·실측 config는 유지한다.

```bash
source /home/user/nomad_ros.sh
source /home/user/nomad_ws/install/setup.bash
cmake -S /home/user/nomad_ws/src/nomad_localization -B /home/user/nomad_ws/build/nomad_localization -DBUILD_TESTING=ON
cmake --build /home/user/nomad_ws/build/nomad_localization --target fusion_node localization_viewer_test heading_reset_scenario -j4
ctest --test-dir /home/user/nomad_ws/build/nomad_localization -R '^localization_viewer$' --output-on-failure
cmake --install /home/user/nomad_ws/build/nomad_localization
```

현재 보드에는 native 빌드·설치를 완료했다. 실행 중 모듈을 자동 재시작하거나 실제 방향 초기화·VIO 재실행을 하지 않았다. 다음 시험 시작부터 새 서비스를 사용한다.

- `/fusion/reset_heading_from_magnetometer`: VIO 단독 지속·정차·독립 지자계 관측 조건을 확인한 뒤 public 차체 위치를 보존하며 yaw를 초기화한다. 보정 미확인 관측은 임시 방위 상태를 유지한다.
- 기존 `/fusion/status`에 `viewer_frame`, `viewer_epoch`, `viewer_gnss_*`, `viewer_vio_*`, `heading_reset_*`를 추가한다. 새 고용량 센서 구독은 없다.
- 비교 VIO는 구간 시작 때 public 출력에 고정 정렬한다. GNSS 최적화가 이 비교선을 보정하지 않는다. GNSS는 실제 datum 및 안테나/IMU/차체 변환을 사용한다.
- GUI VIO 재실행은 기존 `/fusion/vio_epoch`를 사용한다. 살아 있는 fusion을 다시 켜지 않고 마지막 위치·방향에 새 VIO를 부착한다. fusion이 없으면 새 원점으로 시작하며, 이전 위치 복구라고 표시하지 않는다.
- 기본 설정: `/home/user/nomad_ws/config/fusion_independent.yaml`. 새 `heading_reset_vio_only_s=1.0`, `heading_reset_stationary_s=0.5`, `heading_reset_max_speed_mps=0.1`, `heading_reset_max_rotation_rps=0.15`는 소스 기본값이다. 장착 보정값과 기존 캘리 플래그를 변경하지 않는다.

검증은 native 빌드·핵심 unit 및 localhost 합성 ROS로 수행했다. 실제 센서 정확도·실차 주행 검증을 대신하지 않는다.
