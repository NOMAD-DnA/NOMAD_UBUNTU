# Jetson C++ 비용지도와 실측 보정 시험 구성

2026-11-06 17:38 KST Jetson 소스 스냅샷 기준이다. 비용지도 빌드는
`nomad-costmap-cpp-20261105-2`, 소스 정리 개정은 `jetson-source-sync-20261106-1`이다.
현재 Jetson에서 가져온 소스와 설정을 보관하며 기존 시뮬레이션 실행은 별도다.

## 파일 위치

| 경로 | 역할 |
|---|---|
| `src/nomad_perception/traversability/source/` | 변경하지 않은 계산 원본 113개, 공통 메시지, LiDAR 원본과 고정 해시 |
| `src/nomad_perception/traversability/semantic_adapter/` | ADE20K 150종에서 프로젝트 ID 0..11로 변환하는 C++ 코드와 검사 |
| `tools/visual_trial_20261106/config/costmap_runtime/` | 실제 통합시험 runtime에서 회수한 비용지도 설정 |
| `tools/visual_trial_20261106/config/` | 공장 보정, VIO 보정, ZED 실행 조건, DDS 연결과 preview 변환표 |
| `tools/visual_trial_20261106/` | 외부 OAK 스테레오 점군, frame alias, 모니터, ZED launch 소스 |
| `tools/indoor_fcn_20261106/` | 별도 FCN ResNet18 실내 의미분할 시험 소스와 SUNRGBD 21종 변환표 |
| `config/` | 기존 Jetson OAK와 개별 모듈 및 유선 DDS 설정 |

모델 가중치와 TensorRT engine, SDK 설치 파일, Python 배포 라이브러리, build,
install, runtime 실행 파일과 시험 로그는 Git에 넣지 않는다.
`source/vendor/source.tar.gz`는 재빌드에 필요한 고정 LiDAR 소스이며 빌드 생성물이 아니다.

## 기존 Jetson에서 빌드

대상은 기존 Jetson Xavier NX의 Ubuntu 20.04와 ROS 2 Humble 환경이다.
ROS underlay `/opt/ros/humble/install`, 빌드된 workspace `install/local_setup.bash`,
TF2 C++ 개발 패키지, Eigen, OpenSSL과 CMake가 필요하다. 센서 도구는 기존
ZED SDK 및 `zed_wrapper`, `nomad_oak_cpp`, depthai SDK, OpenCV 설치를 사용한다.
이 문서는 JetPack, ROS와 센서 SDK의 신규 설치를 자동화하지 않는다.

```bash
cd /home/nvidia/nomad_ws
./scripts/build_jetson_traversability.sh --check
./scripts/build_jetson_traversability.sh
```

`COLCON_IGNORE`가 있는 독립 CMake 빌드다. 일반 workspace colcon 빌드만으로는
이 C++ 비용지도를 만들지 않는다. 기본 출력은 `traversability/runtime-semantic-test`다.
기존 runtime이 있으면 덮어쓰지 않고 멈춘다. 재빌드는 모듈 폴더 안의 새 경로를 선택한다.

```bash
NOMAD_RUNTIME_DIR="$PWD/src/nomad_perception/traversability/runtime-semantic-rebuild" \
  ./scripts/build_jetson_traversability.sh
```

`NOMAD_ROS_PREFIX`, `NOMAD_WORKSPACE_ROOT`, `NOMAD_BUILD_JOBS`로 기존 환경과
병렬 작업 수를 지정할 수 있다. 새 runtime 선택은 이후 실행에도 같은 값을 사용한다.
비용지도와 ADE 변환기의 운용 코드는 CPU C++이며 외부 의미분할 추론은 별도다.

시험 보조 프로그램은 다음으로 빌드한다. 센서나 모델 전환은 시작하지 않는다.

```bash
./scripts/build_jetson_trial_tools.sh
# 기존 CUDA 및 TensorRT 개발 환경이 준비된 경우 FCN 노드도 빌드
./scripts/build_jetson_trial_tools.sh --with-fcn
```

FCN에는 TensorRT 8.5와 CUDA runtime 개발 헤더 및 라이브러리가 필요하다.
현재 소스는 512×512 엔진을 요구한다. 모델과 엔진은 별도로 준비해야 하며
`switch_model.sh`가 요구하는 엔진 이름은 `model/fcn_resnet18_512x512_fp16.engine`이다.
`config/source_config.json`은 공개 모델의 원래 512×400 계약 기록이다.
현재 512×512 bilinear 시험 전처리와 같은 것으로 해석하지 않는다.

## 실측 보정 설정으로 시험 runtime 만들기

기본 C++ 소스와 base runtime은 원본 `plumb_bob`, 왜곡 0인 D 5개 계약을 보존한다.
실제 통합시험은 같은 K와 외부 변환을 쓰되 ROS wrapper의 `rational_polynomial`,
왜곡 0인 D 8개 CameraInfo 계약을 사용한다. `config/costmap_runtime/`이 그 설정이다.
새 빌드만으로 시험 설정까지 자동 적용됐다고 해석하지 않는다.

아래 명령은 아직 없는 시험 runtime을 만들 때 사용한다. 실행 중인 runtime을
교체하는 명령이 아니다. 다른 경로에 빌드했다면 `base_runtime`을 그 경로로 바꾼다.

```bash
(
set -e
cd /home/nvidia/nomad_ws
module="$PWD/src/nomad_perception/traversability"
trial="$PWD/tools/visual_trial_20261106"
base_runtime="$module/runtime-semantic-test"
trial_runtime="$trial/runtime-map-preview"
test ! -e "$trial_runtime" && mkdir "$trial_runtime"
cp -a "$base_runtime/lib" "$base_runtime/share" "$trial_runtime/"
cp -a "$trial/config/costmap_runtime/." "$trial_runtime/share/nomad_costmap_cpp/config/"
python3 "$module/source/tools/make_manifest.py" "$trial_runtime"
NOMAD_RUNTIME_DIR="$trial_runtime" ./scripts/run_jetson_costmap.sh --action check
)
```

기존 runtime이 있으면 위 묶음은 중단된다. 각 명령의 성공을 확인하며
진행한다. 두 launcher의 기본 ROS domain은 59이고 기본 `ROS_LOCALHOST_ONLY`는 0이다.
시뮬레이션의 domain 42와 구분하고 Pi5와 같은 domain 및 시계를 사용한다.
유선 DDS 설정은 Jetson `10.77.0.1`, Pi5 `10.77.0.2`를 기록한 장치별 설정이다.

## 입출력과 실행

전체 토픽과 capture timing 진단은
`src/nomad_perception/traversability/source/TOPICS_JETSON.txt`를 따른다.
Pi5 OAK 점군과 factory 정보, VIO 및 READY, ZED CameraInfo와 의미 영상,
LiDAR 및 촬영 시각 TF가 외부 입력이다. 지도 발행은 계산 완료 직후다.
단순 launcher 실행은 이 외부 입력들을 모두 시작하는 전체 실행 도구가 아니다.

```bash
cd /home/nvidia/nomad_ws
NOMAD_RUNTIME_DIR="$PWD/tools/visual_trial_20261106/runtime-map-preview" \
  ./scripts/run_jetson_costmap.sh --action start --run-id measured_trial_001
# 동일 run-id로 종료
NOMAD_RUNTIME_DIR="$PWD/tools/visual_trial_20261106/runtime-map-preview" \
  ./scripts/run_jetson_costmap.sh --action stop --run-id measured_trial_001
```

run-id는 새 실행마다 고유하게 정한다. 외부 LiDAR를 이미 사용하는 구성은
감독기에 `--external-lidar`를 전달한다. 장치와 중복 실행 상태를 먼저 확인한다.
기존 원격 전체 통합시험의 GUI와 `/tmp/nomad_integrated_jetson_run_dir` 상태는
이 소스 보관만으로 새로 만들어지지 않는다.

ADE 변환기는 별도 프로세스다. 기존 150종 모델을 사용하는 경우에만 시작한다.

```bash
./scripts/run_jetson_semantic_adapter.sh
# preview 입력 /nomad_preview/ade20k/class_ids를 쓸 때
NOMAD_SEMANTIC_MAPPING="$PWD/tools/visual_trial_20261106/config/ade20k.preview.json" \
  ./scripts/run_jetson_semantic_adapter.sh
```

ADE 입력 `/perception/semantic/ade20k/class_ids`는 640×360 mono8이고 출력은
`/perception/semantic/class_ids`다. 원래 촬영 시각과 optical frame을 유지한다.
ADE의 zero-based floor ID 3은 프로젝트 paved ID 6으로 변환한다.
FCN SUNRGBD 21종의 floor는 원본 ID 2이며 같은 프로젝트 ID 6으로 간다.
서로 다른 원본 클래스 번호를 혼용하지 않는다. FCN과 ADE 변환기를 동일 출력
토픽에 동시에 붙이지 않는다. FCN 전환 스크립트는 이미 실행 중이고 상태 파일이
있는 기존 통합시험에서만 사용하는 도구다. 이 저장소 반영 과정에서는 실행하지 않았다.

## 실측 보정과 검증 범위

`config/costmap_runtime/oak_g2b_selected_20261030.json`과
`zed2_g2b_selected_20261030.json`은 선택 원본과 바이트 단위로 일치한다.
입력은 `g2_laser`, 출력은 각각 `oak_cam_b_optical_frame`과
`zed2_23472639_left_rectified_optical_frame`, 변환 단위는 m다.
ZED launch와 `zed_measured_20261030.yaml`은 self-calibration 비활성,
시리얼 23472639, HD720 조건을 보존한다. 장착이나 스캔 축이 바뀌면 이전
보정값의 적용 가능성을 다시 확인한다.

Jetson에서 원본 113개와 배포 소스 131개의 무결성, ADE mapping과 ROS pipeline
검사 2개가 통과했다. 주요 실행 파일 5개와 실측 보정 파일도 기존 적용 기록과
일치했다. 수신한 소스의 공통 메시지 6개는 저장소의 메시지와 같다.
PC에서도 비용지도와 ADE 변환기를 ROS 2 Jazzy 환경에서 다시 빌드했고 CTest
5개가 통과했다. 외부 점군, frame alias와 모니터도 PC에서 빌드 확인했다.
이번 Git 반영은 새 FCN 엔진 생성, 카메라 재시작, 센서 정합이나 주행 시험을
수행한 것이 아니다. FCN 512×512 전환은 별도 실내 시험 소스로 취급한다.

과거 floor 수정 후에도 최종 비용이 -1과 100만 출력된 문제가 있었다.
이번 소스 동기화로 해결됐다고 표시하지 않는다. 실제 CameraInfo와 점군 정합,
경사 품질, 최종 비용 분포 및 지도 반영 지연은 현재 모델과 실제 입력으로 재검증한다.

구조 분리와 명시적 메시지 계약의 원칙은
[ROS 2 원논문 III-B](https://arxiv.org/html/2211.07752v1#S3.SS2)를 참고한다.
Git 반영은 [Git 공식 push 문서](https://git-scm.com/docs/git-push)에 따른 일반 push이며
공유 브랜치의 이력을 강제로 교체하지 않는다.
