# OAK 점군 생성 방식 선택

기능 ID: `oak-cloud-select-20261110-1`

공유 GUI에 `OpenCV StereoSGBM`, `OAK 내부 StereoDepth` 선택과 `점군 방식 적용`, `현재 방식 확인` 버튼을 추가했다. 의미분할 선택과 독립적이며, 의미분할 전환과 점군 전환 및 시험 종료는 GUI에서 동시에 실행하지 않는다.

## 사용

기존 `실행.sh`로 GUI 시험을 정상 시작한다. 초기 점군 방식은 OpenCV다. 실행 중 OAK 점군 메뉴에서 방식을 선택하고 적용한다. 선택한 방식에서 촬영한 새 유효 점군이 실제로 수신되고 발행자가 하나임을 확인한 뒤 적용 확인으로 표시한다. `현재 방식 확인`은 읽기만 수행한다. SSH 없는 화면과 관전 화면에서는 적용할 수 없다.

기존 지도를 자동으로 지우거나 재시작하지 않는다. 현재 지도는 100 미만 경사 평면을 고정하므로 기존 셀은 전환 전 경사를 유지할 수 있다. 두 방식의 지도 결과를 비교하려면 각각 새 지도 세션에서 시작한다. 전환은 측정 방식의 변경이며, 주행 중 지도의 모든 경사가 즉시 다시 계산된다는 뜻이 아니다.

깊이 입력이 없거나 새 유효 점군을 확인하지 못하면 이전 방식으로 복원을 시도한다. 복원도 새 점군 수신으로 검증하며, 검증되지 않은 결과는 미확인으로 표시한다. 원격 명령이 성공 종료했다는 이유만으로 전환 성공을 표시하지 않는다.

## 프로그램 구조

- OAK USB 장치 소유자는 기존 Jetson `oak_publisher` 한 개다. GUI가 실행하는 `run_oak_cpp.sh`가 시작한다. Pi5 VIO는 기존 좌우 영상과 IMU를 구독한다.
- 새 GUI의 시험 시작은 OAK 발행기에 내부 깊이 출력을 준비하도록 요청한다. 기본 단독 발행기의 `enable_device_depth=false`는 유지한다.
- GUI 시험에서는 깊이 경로를 처음부터 준비한다. 따라서 OpenCV 모드에서도 OAK 내부 깊이 처리 자원과 깊이 출력은 사용된다. 이 선택으로 실행 중 카메라 재연결과 VIO 재시작 없이 점군 방식을 바꿀 수 있다. 이전의 깊이 출력 비활성화 구성과 전체 장치 부하가 완전히 같은 조건은 아니다.
- `external_stereo_cloud` 한 개만 `/oak/depth/points`를 발행한다. OpenCV 모드에서만 좌우 영상 구독과 SGBM 계산을 수행한다. 내부 깊이 모드에서는 깊이 영상만 구독하고 미터 단위 XYZ로 변환한다.
- 이 OAK-D Lite 구성에서 내부로 옮기는 것은 스테레오 깊이 계산이다. XYZ 변환과 ROS 발행은 여전히 호스트의 C++ 프로그램에서 수행한다.
- 전환과 최종 발행은 같은 잠금으로 보호한다. 이전 방식의 진행 중 계산은 끝날 수 있지만 전환 응답 후에는 이전 세대 결과를 발행하지 않는다. 전환 이전 촬영 프레임도 새 방식 확인에 사용하지 않는다.
- 같은 사용자와 ROS domain에서 새 점군 생성기 중복 실행을 차단한다. 전환 확인기는 실제 토픽 발행자 수가 1인지도 확인한다.

## 기하와 시각 계약

입력 내부 깊이는 `/oak/device_depth/image_rect`, `16UC1`, 640x480, mm 단위의 rectified-left 축방향 Z다. 0은 유효 깊이가 아니다. 내부 깊이의 confidence 기준 200, 좌우 일치 검사와 3비트 subpixel을 사용한다. 시간 필터와 공간 구멍 채우기 및 decimation은 켜지 않는다. 내부 깊이는 기존 좌우 촬영 sequence와 연결해 왼쪽 원래 촬영 시각을 그대로 내보낸다. 깊이와 왼쪽 촬영 시각 차이가 1ms를 넘으면 버린다. IMU는 기존 독립 발행 스레드와 축을 유지한다.

기존 `oak_basalt_calib.json`의 rectified K를 깊이 변환에도 사용한다. 장치의 disparity-to-depth focal length와 baseline을 그 K 및 공장 실측 baseline으로 명시한다. EEPROM에는 쓰지 않는다. 실제 장치 serial과 공장 B->C->A 및 rectification 회전과 이동이 보관 보정과 같은지 시작 시 검사한다. 별도 `/oak/device_depth/camera_info`는 정적 보정이며 관측 시각을 만들지 않는다.

출력은 두 모드 공통으로 `/oak/depth/points`, `oak_cam_a_optical_frame`, 미터 단위 float32 XYZ, point_step=12, height=1이다. rectified-left 역회전과 실제 B->C->A 회전 및 이동을 적용한다. 이름만 CAM_A로 바꾸지 않는다. 행206부터 4픽셀 간격, 깊이 0.2m 초과 10m 미만, 기본 점군 1Hz를 유지한다. 내부 깊이 모드에서는 두 1Hz 제한기의 도착 지터로 발행률이 반감되지 않도록 촬영 시각 기준으로 제한한다.

주행가능성 소스와 실행 파일, 보정 파일 및 최종 비용 계산은 변경하지 않는다. 기존 경사 고정, 사람 점유, LiDAR 통과 복원과 의미 비용 정책을 유지한다. 비용지도 build_id는 기존 배포 ID 그대로다.

## 파일과 빌드

소스는 이 폴더 `source/nomad_oak_cpp`, `source/external_cloud`, `source/gui`에 있다. Jetson 실제 원본은 PC의 `original_jetson.tar.gz`, 공유 GUI 원본 목록은 `original_gui_manifest.json`에 보존했다. 기존 비교용 주행가능성 패키지를 덮어쓰지 않는다.

Jetson은 기존 DepthAI 2.29.0과 ROS2 Humble로 빌드한다. DepthAI 3.x 설치나 SDK 업그레이드는 수행하지 않는다. `source/gui/laptop_monitor/remote/start_jetson.sh`가 새 발행기 옵션을 전달하며 세 전환 스크립트는 native `cloud_source_control`을 실행한다.

## 검증과 제한

검증 결과와 설치 파일 해시는 `completion.json`을 따른다. PC 합성 ROS 검사는 실제 카메라 검사와 구분한다. 실제 OAK 검사는 정지 상태의 별도 ROS domain에서 카메라와 점군만 실행한다. 실제 주행과 Pi5 VIO 및 전체 지도 및 차량 제어의 통합 주행 정확도를 입증한 검사는 아니다.

GUI 전체 회귀 검사에서는 기존 공유 원본에서도 동일하게 실패하는 플래닝 로그 검사 1개가 있었다. 이 기능의 신규 GUI 검사 8개는 통과했다. 원본의 해당 실패를 이번 점군 기능의 통과로 합산하지 않는다.

## 원본 근거

- [DepthAI 2.29.0 StereoDepth API 소스](https://github.com/luxonis/depthai-core/blob/v2.29.0/include/depthai/pipeline/node/StereoDepth.hpp): RAW16 깊이, rectified-left 정렬, subpixel, focal length 및 baseline override의 단위와 적용 범위.
- [DepthAI 2.29.0 StereoDepth 구현](https://github.com/luxonis/depthai-core/blob/v2.29.0/src/pipeline/node/StereoDepth.cpp): 정렬과 깊이 설정의 실제 구현. 최신 3.x 문서의 별도 PointCloud 노드를 이 2.29 구성에 가정하지 않는다.
- [Hirschmüller, Stereo Processing by Semi-Global Matching and Mutual Information, TPAMI 2008 원문](https://elib.dlr.de/55367/1/Stereo_Processing-Hirschm%C3%BCller.pdf): 좌우 일치 검사, subpixel 보정과 가림 및 저텍스처 한계. 내부 깊이 선택을 실측 경사 정확도 향상의 증거로 해석하지 않는다.
