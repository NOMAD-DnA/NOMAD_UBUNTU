NOMAD C++ Qt GUI · 2026-11-08

최신 갱신: 조향 입력0=좌회전/1=우회전, raw 서보 출력=1-입력.
모의 주행: Windows desktop/Simulation.cmd / Linux bash desktop/start-linux.sh --simulate
모의 화면에서 고정/추적 시점 선택. 자세한 조작은 GUI의 사용 안내 버튼.
ROS Humble 로컬 빌드 및 모의 통합 검사도 완료됨: ROS_WSL_VALIDATION.ko.txt 참조.

기존 웹 GUI와 동일한 Pi HTTP/ROS 제어 서비스를 사용하는 네이티브 Qt Widgets 앱입니다.
창을 열거나 연결이 복구되는 것만으로 차량이 시작하지 않습니다.

Linux (Ubuntu 22.04 이상 / Debian 12 이상, Qt6 개발 패키지 제공 환경)
  최초 설치:
  sudo apt install build-essential cmake qt6-base-dev openssh-client curl util-linux
  bash linux/settings.sh
  bash desktop/start-linux.sh
  이후: bash desktop/start-linux.sh
  앱 메뉴 등록: bash desktop/install-linux-desktop.sh
  연결 종료: 차량 STOP 후 bash linux/disconnect.sh
첫 실행 때 해당 PC에서 C++ 앱을 빌드합니다. 이후에는 변경된 파일만 빌드합니다.
기존 웹 GUI: bash linux/start.sh

Windows
  desktop/start-windows.cmd 실행 또는 바탕화면의 NOMAD Qt Control 바로가기.
  Pi 연결 설정은 기존 scripts/connection.json을 사용합니다.
  Qt 실행 파일만 실행하면 터널을 만들지 않습니다. 실행기를 사용하세요.

조종
  1. 수동 조종과 마우스/키보드를 선택합니다.
  2. 연결 후 수동 제어 시작을 누릅니다. 속도 0으로 시작합니다.
  3. 마우스: 슬라이더. 키보드: W/↑ +50 ERPM, S/↓ -50 ERPM,
     A/← 조향 입력 -0.02(좌), D/→ 조향 입력 +0.02(우).
  4. Esc 또는 Space 또는 STOP: 모터 구동 해제(제어기 설정에 따라 0A coast 또는 VESC 브레이크·핸드브레이크 유지).
키보드 조종 중 다른 창으로 이동하면 STOP합니다.
입력 방식 또는 수동/LPP 모드를 바꾸면 STOP하며 다시 시작해야 합니다.
  현재 수동 범위: 0~1850 ERPM, 조향 입력 0~1 (서보 출력은 반전).
m/s·km/h는 기존 영상 비례 환산 표시이며 실측 이동 속도가 아닙니다.

LPP 자율 제어
  AUTONOMY_GUIDE.ko.txt의 ROS 빌드·보정 설정이 먼저 필요합니다.
  계획 명령이 들어오고 변환값이 검증되어야 LPP 자율 제어 시작 버튼이 활성화됩니다.
  GUI는 승인 세션만 유지하며 m/s·rad 목표는 ROS DriveCommand에서 받습니다.
  LPP 명령 만료/연결 상실/STOP 후 자동 재시작하지 않습니다.
  현재 배포에는 실제 차량 보정값을 만들어 넣지 않았습니다.

검증 범위
  Windows 및 Debian Qt 빌드와 가짜 HTTP 제어기의 입력/STOP 시험.
  WSL ROS Humble 빌드와 가짜 LPP/VESC 통합 시험 통과. 실제 차량 연동은 미검증.
  기존 Pi 실행 프로그램은 교체하지 않았습니다.
