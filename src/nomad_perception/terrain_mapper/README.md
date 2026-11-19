# GitHub 보관 및 빌드 범위

이 폴더는 NOMAD_UBUNTU의 feat/percep 브랜치에 보관한 Jetson terrain 전용 작업공간입니다.
바깥 Python 인지 패키지와 메시지 중복 빌드를 막기 위해 COLCON_IGNORE를 둡니다.
별도 작업공간에 이 폴더의 src/를 복사하여 ROS 2 Humble 환경에서 빌드하세요.
Jetson 빌드와 결과는 SSD /mnt/ssd 아래에 둡니다. 기존 시스템에 자동 배포하지 않습니다.
GUI 연동 사본은 참고용이며 실제 GUI 원본은 별도 feat/gui 브랜치에서 관리합니다.
이 패키지의 활성 설정은 수집 당시 값입니다. 설치 기본값 12cm와 달리 현재 저장값은 5cm입니다.
기존 저장소의 공통 nomad_interfaces를 덮어쓰지 않고 수집 당시 메시지를 이 독립 작업공간에 보존했습니다.

아래는 공유폴더 패키지의 원본 안내입니다.

---

# 현재 사용하는 기존 매퍼 개선 코드

Jetson에서 2026-11-19T20:11:00.545902+09:00에 직접 읽어 온 소스와 설정 사본입니다.
대상은 `nomad_terrain_mapper`이며 GUI의 "기존 매퍼 개선"에 해당합니다.

## 구성

- `src/`: terrain core, semantic, cost, mapper 전체 소스와 vendor 헤더 및 라이선스, 의존 메시지 패키지 nomad_interfaces.
- `active_config/terrain.json`: 수집 당시 Jetson에 저장된 설정 원문.
- `runtime-config/`: 기존 GUI 실행 경로에서 사용하는 semantic.real.json, slope.json, LiDAR 보정 원문.
- `gui-integration/`: 현재 공유 GUI의 terrain 설정 화면, 기본 설정, 설명 및 관련 검사 사본. 독립 실행 앱이 아니며 전체 GUI는 기존 공유폴더 것을 사용합니다.
- `snapshot_manifest.json`: 파일별 원본 위치와 SHA-256, 수집 시각 및 설치 바이너리 해시.

11월 18일 OAK 전방 수평거리 제한과 11월 19일 OAK/LiDAR 높이 차이 GUI 설정이 포함됩니다.
핵심 소스는 11월 19일 배포 완료 기록과 해시가 일치합니다. 저장된 설정은 이후 변경될 수 있으므로 이 묶음의 active_config를 기준으로 확인하세요.
수집 당시 OAK 최대 수평거리 값은 2.0m, OAK/LiDAR 높이 차이 기준은 5.0cm입니다.

## 빌드와 실행 참고

Jetson의 ROS 2 Humble 및 Eigen3, C++17과 colcon 환경을 전제로 합니다.
새 작업 디렉터리에 압축을 풀고 해당 디렉터리에서 ROS 환경을 불러온 뒤 `colcon build --symlink-install`로 빌드합니다. `nomad_interfaces`도 함께 들어 있습니다.

기존 실행 스크립트는 `src/nomad_terrain_mapper/scripts/run_terrain_mapper.sh`이며 기존 Jetson 설치 경로와 nomad_ws overlay를 참조합니다. 다른 PC에서 실행하려면 환경과 경로를 맞춰야 합니다.
현재 GUI의 `start_jetson.sh`는 저장된 terrain 설정의 gui_overrides를 실행용 semantic/slope 설정에 적용합니다. 따라서 보관한 runtime-config 원문만으로 실행하면 GUI에서 적용하던 경계 여유와 초기 영역 설정까지 동일하다고 보장할 수 없습니다. 실제 통합 실행에는 기존 공유 GUI를 사용하세요.

이번 작업은 코드 공유만 수행했습니다. 새 빌드, 보드 파일 변경, 센서나 지도 시작 및 중단은 하지 않았습니다. 설치 실행 파일과 주행 기록 및 인증 정보는 이 소스 패키지에 넣지 않았습니다.

## 매퍼 내부 VIO 외부파라미터 보정 (2026-11-19)

Pi5 VIO의 카메라 외부파라미터와 일치하도록 매퍼 내부 공통 센서 변환에 2.49도 보정을 추가했습니다. [변환 정의와 검증 범위](docs/vio_extrinsic_correction_20261119.md)를 참고하세요. PC ROS 2 Jazzy Release 빌드와 CTest 3개가 통과했으며 Jetson에는 아직 설치하지 않았습니다. 위의 snapshot_manifest와 active_config는 최초 수집 당시 기록으로 유지합니다.
