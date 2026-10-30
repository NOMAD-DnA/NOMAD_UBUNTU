# 실내 VIO GUI 시험 설정 — 2026-10-31

## 이번 실행 방식
- 사용자가 추가 센서 점검을 중단하고 GUI에서 직접 초기화·실행하기로 요청했다.
- PC GUI를 실행했다. 시작 버튼은 사용자가 누른다.
- Pi 5는 명시적인 `vio_only` 시험 모드. GNSS 수신·융합 노드를 시작하지 않는다.
- 실제 VIO가 유효하고 최신일 때 기존 어댑터로 `/nomad/localization/odometry`, 상태와 `odom → base_link` TF를 발행한다.
- 첫 유효 base pose를 로컬 원점으로 삼고 VIO의 중력·yaw 축을 유지한다. 실행 중 GNSS 좌표로 자동 전환하지 않는다.
- GNSS 절대 위치·정북 기준이 아니다. 새 실행은 원점이 바뀌므로 목표를 다시 선택한다.

## 적용 파일과 설정
### Jetson
루트: `/home/nvidia/nomad_semantic_20261030`
- `jetson_autonomy.json`: `stereo_fps: 10`, `semantic_fps: 2`, RGB 10Hz 유지.
- `opt/nomad/scripts/autonomy_local.py`: 의미분할 `--target-fps 2` 전달.
- `scripts/run_jetson_oak_local.sh`: 단독 실행도 동일 속도 적용.
- 두 로컬 wrapper에서 `RMW_FASTRTPS_PUBLICATION_MODE=ASYNCHRONOUS` 적용.
- `bin/semantic_perception_rgbd_local.pyz`: 대기 중 IMU 묶음을 합쳐 영상보다 먼저 전달. IMU 시각·순서·측정값 보존. 양안 발행 대기는 최신 한 쌍으로 제한.
- 수정 원본 `opt/nomad/local_patches/sensor_publisher.py` 보존. 기존 실행 파일은 backups에 보존.
- `config/fastdds_wifi.xml`의 Pi 5 peer를 192.168.3.95로 갱신.

### Pi 5
- 현재 주소 192.168.3.95. `~/nomad_gnss_pi5/config/fastdds_wifi.xml`의 인터페이스와 self peer 갱신.
- GNSS 장치 한 개에 chmod 666 적용, user 계정 실제 read/write open 성공. USB 재연결·재부팅 시 이 일회성 권한은 초기화될 수 있다.
- `~/nomad_gnss_pi5/config/localization.env`: `NOMAD_PI5_LOCALIZATION_MODE=vio_only`.
- native workspace의 `nomad_vio/fusion_adapter.py`, `fusion.launch.py` 소스와 설치 사본을 함께 갱신.
- GNSS 융합으로 복귀하려면 localization.env 값을 `gnss_vio`로 바꾸고 해당 실행을 종료·재시작한다.
- VIO stale/유효 자세 검사 유지. 좌표계 변경·큰 불연속은 FAULT로 막고 재시작 요구.

## 사용자 실행
1. PC의 열린 NOMAD 실차 운영 화면에서 시작 버튼을 누른다.
2. 영상이 뜨면 **VIO 초기화 해!** — 차량을 천천히 움직인다.
3. 유효 VIO가 나오면 로컬 위치를 사용해 인지·지도·플래닝 준비가 진행된다.
4. 지도와 위치가 나온 뒤 새 목표를 선택한다. 기존 GUI의 실차 구동 확인 및 Control 조건은 유지된다.

## 확인 범위
- 주기 조정·주소 수정·chmod는 적용했다. GUI 창 표시 확인.
- 변경 전 측정에서는 영상 대기 때문에 IMU 지연이 누적됐고 GNSS는 실내에서 UTC 미확정이었다.
- 사용자의 점검 중단 요청 이후 IMU 큐/비동기 발행/VIO 전용 모드를 배포했고 문법만 확인했다. 이후 사용자가 GUI에서 실행한 결과를 종료 전에 수집했다. 상세 결과는 아래 로그 기록을 따른다.
- 의미분할 2Hz는 목표 상한이며 처리 성능에 따라 낮을 수 있다. 10Hz는 양안 입력 설정이며 실제 VIO 출력 주기를 보장하지 않는다.
- 이번 작업에서 모터 제어를 시작하거나 주행 명령을 발행하지 않았다.
- GUI 패키지와 최신 실행 설정 예시는 `feat/gui`에 게시. 실제 원본 로그는 공유폴더에 별도 보존.


## 마지막 GUI 실행 로그까지 반영한 결과

**전체 루프 완료 아님.** Jetson 마지막 실행은 준비 제한시간 120초 안에 READY에 도달하지 못했고 GUI가 실행을 정리했다. Pi 4 제어까지 성공했다고 기록하지 않는다.

| 실행 | 구성 / 결과 |
|---|---|
| Jetson `1a993d937624` / Pi 5 `6dca0bfb0d1d` | 이전 IP 설정, GNSS `Permission denied`, IMU·양안 만료. |
| Jetson `rates2vio10_20261031a` / Pi 5 `ea7a3e4dd527` | 주소·chmod와 2Hz/10Hz 설정 반영. GNSS 포트 열림. 실내에서 UTC 미확정. 센서 지연 측정 후 직접 시작했던 시험만 중지. |
| Jetson `f2571153b68c` / Pi 5 `2741cd947670` | 사용자가 GUI로 실행. VIO 전용 모드 적용 확인. 의미분할 발행은 진행됐지만 양안 최신성 검사에서 시간 초과. |

### 마지막 실행의 실제 로그 발췌

Pi 5:
```text
Local VIO test mode: GNSS fusion disabled; initializing from actual camera and IMU
basalt started: owned pid 7222, epoch 0
VIO_ONLY_TEST: local origin; GNSS fusion disabled for this run
```

Jetson `supervisor.log`:
```text
Startup timeout: left: Capture or receipt expired
```

Jetson 마지막 의미분할 발행 기록 요약:
```json
{"event":"published","frames":155,"camera_sequence":1070,"fps":1.49}
```
`fps`는 프로그램이 기록한 실행 중 누적 처리율이다. 설정 목표 상한 2Hz와 구분한다.

Jetson 마지막 실행 중 인지 상태 기록 요약:
```json
{"mapping_initialized":false,"pose_samples":0,"semantic":{"label_frames":150,"depth_frames":108},"canonical_planning_published":false,"waiting_reason":"left: Capture or receipt expired"}
```

마지막 health 파일에는 `/nomad/localization/odometry`의 수신 이력이 있으나 종료 시점에는 오래된 상태였고, 인지 모듈의 유효 위치 표본은 0이었다. 따라서 VIO 전용 어댑터가 실행됐다는 것과 최신 위치로 지도·플래닝이 이어졌다는 것은 구분한다. `health.json`은 종료 과정의 마지막 스냅샷이므로 그 시점의 age 수치를 정상 실행 중 지연 통계로 해석하지 않는다.

종료 과정에서는 `rcl_shutdown already called`, `BrokenPipeError`, `ExternalShutdownException`, `KeyboardInterrupt`도 기록됐다. 이들은 종료 시점 로그이며, 위 준비 실패 원인과 구분해 원본에 보존한다.

### 큐 수정 전 15초 수신 측정

| Pi 5 수신 토픽 | 수신 Hz | 측정시각 기준 중앙 지연 |
|---|---:|---:|
| `/oak/left/image_rect` | 1.40 | 1.059초 |
| `/oak/right/image_rect` | 1.47 | 1.142초 |
| `/oak/imu` | 39.80 | 6.301초 |
| `/basalt/odom` | 0.40 | 7.730초 |
| `/gps/fix` | 0 | 없음 |

이 표는 IMU 큐 보완·비동기 발행·VIO 전용 모드 적용 **전** 측정이다. 최신 배포 성능 수치로 사용하지 않는다. 최신 배포 이후 별도 측정은 사용자의 요청대로 하지 않았으며 GUI가 남긴 기존 로그만 수집했다.

### 원본 보관과 다음 작업
- 공유폴더 원본 로그: `NOMAD_실내_VIO_GUI_원본로그_20261031/` (Jetson 3회, Pi 5 3회).
- 로그를 수집한 뒤 사용자 요청으로 Jetson에 `shutdown -h now`를 전달했다. SSH 연결은 원격에서 종료됐다.
- 다음 작업은 양안·IMU가 최신 상태로 전달되는 구간과 VIO 지속 출력의 병목 해결이다. 이번 종료/기록 작업에서는 추가 실차 시험을 수행하지 않았다.
- 이번 GUI 코드는 기존 로그 기록 기능을 포함한다. 기록 버튼으로 만든 별도 GUI 시도 폴더는 이번 수집에서 발견되지 않았다. 위 원본은 보드의 stdout/상태 파일이다.
