C++ 비용지도 실행 패키지
기준: nomad-pi5-oak-zed-native-20261105-7 보존
빌드: nomad-costmap-cpp-20261105-2

1. 범위
Pi5가 OAK 점군과 VIO를 발행합니다. 별도 Jetson 모듈이 ZED2 영상을 취득하고
의미 영상 및 CameraInfo를 발행합니다. 이 패키지는 LiDAR를 직접 취득합니다.
수신기, 지면 계산기, 비용지도 발행기, 상태 감시기, 실행 감독기는 C++17입니다.
Python이나 ros2 launch 및 ros2 run을 운용 실행에 사용하지 않습니다.
PC의 가상 센서, 시험 기록기, 웹 시각화는 Python 및 JavaScript를 유지합니다.
ZED2 깊이, GPU 및 별도 가림 검사는 사용하지 않습니다.

2. Jetson에서 빌드
기존 ROS 2 Humble 개발 환경을 사용합니다. 기본 경로:
/opt/ros/humble/install/setup.bash
다른 환경이면 NOMAD_ROS_PREFIX로 지정합니다.
필요한 개발 도구: g++ 9 이상, CMake 3.16 이상, make, Eigen3, OpenSSL 개발 헤더.
ROS의 ament_cmake, rclcpp, tf2_ros, sensor_msgs, nav_msgs, diagnostic_msgs,
rosidl_generator_cpp 및 C++ typesupport 생성기가 필요합니다.
ROS 메시지 생성과 빌드 도구가 Python3을 사용하는 것은 허용된 범위입니다.
nlohmann/json과 nanoflann 헤더 및 LiDAR 원본은 vendor에 포함했습니다.

압축을 풀고 소스 폴더에서:
./build_on_jetson.sh

결과는 runtime-jetson입니다. 이전 runtime-jetson이 있으면 덮어쓰지 않습니다.
재빌드할 때는 NOMAD_RUNTIME_DIR="$PWD/runtime-jetson-new"로 새 출력 경로를 지정합니다.
빌드 중 오류가 생기면 로그와 소스를 보존하고 자동 배포하지 않습니다.
기존 Jetson의 costmap_runtime이나 20261105-1을 자동 교체하지 않습니다.

3. 실행
runtime-jetson/lib/nomad_costmap_cpp/nomad_runtime_manager --action check
runtime-jetson/lib/nomad_costmap_cpp/nomad_runtime_manager --action start --run-id cpp_01
runtime-jetson/lib/nomad_costmap_cpp/nomad_runtime_manager --action status --run-id cpp_01
runtime-jetson/lib/nomad_costmap_cpp/nomad_runtime_manager --action stop --run-id cpp_01

실행 파일이 자기 경로에서 runtime 루트를 찾습니다. --prefix로 별도 지정할 수 있습니다.
기본 ROS_DOMAIN_ID는 기존 설정과 같은 42입니다. 환경 변수로 명시한 값은 유지합니다.
기본 LiDAR 경로는 이전 선택 설정의 CP2102 USB by-id 경로입니다.
다른 장치 경로는 --lidar-port로 지정합니다.
외부에서 LiDAR를 발행하는 시험에만 --external-lidar를 사용합니다.
--ros-prefix로 ROS C++ underlay 경로를 지정할 수 있습니다.
실행 때 local_setup.bash를 source할 필요가 없습니다.
상태와 로그는 runtime 루트의 runs/실행ID에 저장됩니다.
동일 실행ID를 재사용하지 않습니다. 다른 프로그램이나 PID를 임의 종료하지 않습니다.

4. 유지한 동작
점군 DDS KEEP_LAST 5, BEST_EFFORT, VOLATILE.
LiDAR 및 OAK 전달 FIFO는 80개와 8개이며 계산 대기는 최대 200개입니다.
초과하면 오류를 남깁니다. 정상 계산 중 입력을 최신 1개로 덮어쓰지 않습니다.
OAK 촬영과 수신 만료는 각각 1.5초입니다.
기존 LiDAR, VIO, 의미 영상, 지도 만료 조건을 유지합니다.
고정 500ms나 1초 대기를 넣지 않았습니다. 결과가 준비되면 발행합니다.
촬영 시각은 int64 나노초로 보존합니다. 계산 시각으로 새로 찍지 않습니다.
외부 보정, 의미 클래스 비용, 지면 및 경사 계산 기준과 미관측 처리를 유지합니다.

5. 검증의 범위
Python 기준 구현과 고정 입력의 수치 및 셀 판정을 비교합니다.
C++ Eigen과 NumPy/LAPACK의 부동소수점 결과가 모든 가능한 입력에서 비트 단위로
동일하다는 보장은 하지 않습니다. 시험한 입력의 판정 일치와 수치 허용 오차를 기록합니다.
ARM64 QEMU 빌드와 시험은 실제 Jetson의 처리 성능이나 센서 정확도 검증이 아닙니다.
1Hz 입력과 1초 목표는 최대 갱신 간격 1초의 보장을 뜻하지 않습니다.
최종 검증 수치와 알려진 제한은 analysis/event_dispatch.html을 참고하세요.

6. 완료 알림과 상태 기록 변경
계산 완료와 입력 도착은 ROS guard로 실행기를 깨웁니다.
입력 처리 시작과 프레임 경계에서도 완료 결과를 먼저 확인합니다.
지도 상태 변경은 원래처럼 한 ROS 실행 흐름에서 처리합니다.
15ms 입력 폴링을 제거했습니다. 100ms 유지 관리 타이머는 남아 있습니다.
큰 상태 파일의 JSON 직렬화 및 I/O는 별도 기록 스레드에서 수행합니다.
불변 결과를 공유하며 최신 진단 파일만 합쳐 기록할 수 있습니다.
이는 입력 점군의 폐기나 DDS 깊이 변경을 뜻하지 않습니다.
파일 기록 실패는 snapshot_log_error 진단으로 알립니다.

7. 검증 범위
PC와 ARM64 QEMU 원본 대조 각 125개, C++/RMW 및 이벤트 검사 각 3개 통과.
실행 관리기 수명주기 각 8개 통과. 기존 시작 시각 검사도 leader task의
커널 stat을 읽도록 수정해 QEMU의 자기 stat 합성 값과 구분했습니다.
Python과 CUDA 동적 의존성이 없는 C++ 실행 파일을 포장합니다.
실물 Jetson 설치와 실제 LiDAR 장치 취득은 수행하지 않았습니다.
전체 주행의 순차 비교 수치와 알려진 제한은 analysis/event_dispatch.html을 따릅니다.
최대 1초 지도 간격을 보장하는 하드 실시간 설계는 아닙니다.

8. PC 순차 비교 결과
Python-7 / C++-1 / C++-2의 지도 간격 p95는 967.94 / 996.32 / 1011.83ms다.
지도 간격 개선은 입증하지 못했으며 최대 1초 목표도 미충족이다.
입력→지도 중앙값은 C++-1 104.94ms에서 C++-2 100.96ms로 관측됐다.
최악 지도 간격 1132.80ms는 기여 입력 발행 간격 1125.04ms와 지연 변동 7.76ms의 합이다.
각 구현을 한 번씩 PC에서 측정한 값이다. 실제 Jetson 성능을 뜻하지 않는다.
