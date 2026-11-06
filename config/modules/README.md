# Jetson 개별 실행

이 폴더는 기존 Jetson 개별 모듈의 설정 보관본이다. 최신 C++ 비용지도와 실측
보정 시험 구성은 [Jetson C++ 실행 안내](../../docs/Jetson_Cpp_실행안내.md)를 따른다.
아래 루트 wrapper 명령은 기존 Jetson 설치에 대한 기록이며 이 저장소의 신규
실행 진입점은 `scripts/`에 있다. ZED 단독 실행은 `scripts/run_zed_sensors.sh`다.

```bash
cd ~/nomad_ws
./zed.sh       # ZED 영상·IMU·지자계
./lidar.sh     # G2B 스캔 — 별도 터미널
./semantic.sh  # ZED 영상 → ADE20K 150종 라벨 — 별도 터미널
./slope.sh     # OAK 점군·G2B 스캔 + TF → 기하학 경사 — 별도 터미널
```

각 터미널에서 `Ctrl+C`로 해당 모듈만 종료. 다른 모듈은 자동 실행하지 않음.

설정: `~/nomad_ws/config/modules/*.yaml`. 기본 ROS 도메인 42.
라이다 포트 지정: `NOMAD_LIDAR_PORT=/dev/ttyUSB0 ./lidar.sh`.

- 의미분할 출력: `/perception/semantic/ade20k/class_ids` (`mono8`, 640×360). 기존 팀 모델의 클래스 번호와 다름.
- 경사도 입력: `/oak/depth/points`, `/scan`, 촬영 시각의 `odom → base_link → 센서` TF. `odom`은 중력 기준 Z축이 위인 좌표계여야 함.
- 경사도 출력: `/perception/slope/points`의 `slope_deg` 필드(도), `confidence` 필드. 현재 의미분할 지면 필터 없이 기존 평면 계산부를 사용함.
- 입력·TF가 없거나 오래되면 `WAITING`. 평면을 계산할 충분한 3D 표면 점이 있어야 경사도가 나옴.
- 상태: `/perception/semantic/status`, `/perception/slope/status`.
