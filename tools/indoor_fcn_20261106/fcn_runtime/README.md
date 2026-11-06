# FCN 실내 모델 별도 실행 노드

이 문서는 Jetson에서 회수한 실내 시험 소스의 계약을 설명한다. 2026-11-06
17:38 KST 소스 스냅샷에는 512×512 입력 전환 코드가 포함돼 있다. 이번 저장소
반영에서는 해당 엔진 생성과 실물 추론을 다시 실행하지 않았다. 아래 소스 계약과
실물 검증 상태를 구분하며, 전체 설치 및 검증 범위는
[Jetson C++ 실행 안내](../../../docs/Jetson_Cpp_실행안내.md)를 따른다.
기존 B2와 카메라 실행 파일은 별도로 관리한다.

이 노드는 기존 ZED 컬러 토픽만 구독한다. 카메라 장치를 열지 않는다. 기본 field12 출력은 별도 미리보기 토픽이며, 상위 실행 도구가 기존 frame alias를 연결해 기존 GUI·mapper용 `/perception/semantic/class_ids`를 공급한다. FCN 모드에서는 기존 B2의 최종 ID 발행 어댑터를 중지하여 동일 토픽에 서로 다른 모델이 동시에 발행하지 않도록 해야 한다.

| 항목 | 내용 |
|---|---|
| 입력 토픽 | `/zed/zed_node/left/image_rect_color` 기본값 |
| 입력 형식 | `sensor_msgs/msg/Image`, rgb8/bgr8/rgba8/bgra8 |
| 원본 클래스 출력 | `/nomad_preview/sunrgbd/class_ids`, SUNRGB-D21 ID |
| 우리 클래스 출력 | `/nomad_preview/sunrgbd/field12_raw`, field12 ID 0..11, `field12_output_topic` 파라미터로 지정 가능 |
| 출력 형식 | 두 토픽 모두 `sensor_msgs/msg/Image`, mono8, 640×360 |
| 헤더 | 입력의 capture stamp와 optical frame 그대로 유지 |
| 처리 주기 | `rate_hz:=1.0` 기본값, 최신 입력 한 장만 보관, 중첩 추론 없음 |
| 추론 | 기존 native TensorRT 8.5 + CUDA Runtime, 보드용 Python 코드 없음 |

필수 파라미터는 `engine_path`, `classes_path`, `mapping_path`다. `mapping_path`에는 `../config/sunrgbd_to_field12.json`을 사용한다. 클래스 파일 21개 이름, engine 출력 채널 21개, source ID 전체 0..20, target ID 0..11, source 이름 일치, field12 이름·순서, schema 1 및 unknown ID 0을 검증한다. 누락·중복·잘못된 범위는 시작 실패로 처리한다.

사용자 요청으로 B2와 비교 입력·출력 크기를 맞춘다. 전처리는 **512×512 bilinear resize** → RGB float32 → `(RGB/255−mean)/std` → NCHW다. mean은 `(0.485, 0.456, 0.406)`, std는 `(0.229, 0.224, 0.225)`다. crop/letterbox는 사용하지 않는다. 원래 공개 모델의 512×400 nearest 입력 계약과 구분되는 시험 설정이며, 노드는 `[1,3,512,512]` 입력 엔진만 받는다. 이전 512×400 엔진은 사용할 수 없다.

후처리는 각 클래스 score plane을 **512×512 bilinear 보간 → 채널 argmax → 완성된 ID 영상을 640×360 nearest resize** 순서로 처리한다. float32/float16 출력 모두 지원하며, 원본 score와 보간 score의 non-finite 값은 출력하지 않는다. 512×512 입력 FCN의 고유 score 격자는 **16×16**으로 남는다. 입력·최종 영상 크기와 후처리 순서를 맞춰도 B2와 동일한 공간 정밀도를 보장하지 않는다.

`CMakeLists.txt`는 이미 설치되고 source된 ROS Humble의 ament/rclcpp/sensor_msgs와 OpenCV/CUDA/TensorRT를 찾아 사용하는 standalone CMake다. 다운로드·설치·전체 워크스페이스 빌드를 수행하는 규칙은 없다. 엔진 빌드와 실행 전환은 상위 실행 도구에서 별도로 처리한다.

현재 매핑은 실내 시험용으로 floor→paved(6), person→person(1), other/ceiling→ignore(0), 나머지 실내 객체→artificial(10)이다. 이는 야지 12클래스를 새로 학습했다는 의미가 아니다.
