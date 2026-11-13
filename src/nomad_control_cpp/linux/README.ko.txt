NOMAD GUI · Linux

바로 실행
1. 이 폴더 전체를 압축 해제합니다.
2. PC를 차량 Pi와 같은 Wi-Fi에 연결하고 Pi/VESC 전원을 켭니다.
3. 이 폴더에서 터미널을 열고 다음 명령을 실행합니다.

   bash start.sh

4. 처음 접속할 때 Pi 호스트 키를 확인하고, Pi user 계정의 비밀번호를 입력합니다.
   비밀번호를 입력해도 글자가 표시되지 않는 것이 정상입니다.
5. 브라우저가 열리면 '수동 제어 시작'을 누르고 속도/조향 슬라이더로 조종합니다.
   종료는 STOP 또는 Esc입니다. STOP은 구동 해제이며, 제어기 설정에 따라 VESC 브레이크·핸드브레이크 출력을 보냅니다(경사 정차 보장 아님).

기본 연결 정보
   Pi: 192.168.3.142
   계정: user
   작업 폴더: /home/user/nomad_cpp_ws
   GUI: http://127.0.0.1:8765/

기본값으로 바로 연결됩니다. IP가 바뀌었거나 다른 계정을 쓰면:
   bash settings.sh
설정은 connection.local.conf에 저장되며 비밀번호는 저장하지 않습니다.
SSH 키는 각 사용자가 Pi에 등록한 본인 키를 지정합니다. 키 파일은 배포판에 포함되지 않습니다.

바로가기 만들기 (선택)
   bash install-desktop.sh
앱 메뉴의 NOMAD GUI로 시작할 수 있습니다. 바탕화면 경로를 찾으면 바탕화면에도 추가합니다.
배포 폴더를 이동하면 바로가기를 다시 만들어 주세요.

연결 종료
먼저 GUI에서 STOP을 누른 뒤:
   bash disconnect.sh
실행 터미널을 닫아도 SSH 터널은 유지됩니다. 끊긴 뒤에는 start.sh를 다시 실행합니다.
조종자는 한 사람씩 사용합니다. 다른 사람이 조종 중이면 기존 조종자의 STOP 후 시작합니다.
GUI 연결이나 실행 스크립트만으로 차량을 자동 출발시키지 않습니다.

필요한 프로그램
- Bash, OpenSSH 클라이언트, curl, flock, sha256sum 및 브라우저.
- 자동 브라우저 열기/바탕화면 경로 탐색에는 xdg-utils를 사용합니다.
- Ubuntu/Debian에서 없는 경우: sudo apt install openssh-client curl util-linux coreutils xdg-utils
- Linux PC에는 ROS, C++ 컴파일러, Python 또는 별도 GUI 빌드가 필요 없습니다.
- x86_64 / ARM64 모두 같은 스크립트를 사용합니다.
- 기존 NOMAD 제어기가 설치된 Pi에 연결하는 클라이언트입니다. 새 Pi 설치용 이미지는 아닙니다.

문제 해결
- 8765 포트 사용 중: 기존 SSH 터널을 종료하거나 이 폴더의 disconnect.sh를 실행합니다.
- Pi 준비 실패: Pi/VESC 전원, USB 연결과 다른 제어 프로그램 실행 여부를 확인합니다.
- Permission denied: Pi 계정/비밀번호 또는 등록된 SSH 키를 확인합니다.
- 브라우저가 열리지 않음: http://127.0.0.1:8765/ 를 직접 엽니다.
- 설정만 검사: bash start.sh --check
- 브라우저 없이 연결: bash start.sh --no-browser

구성
- start.sh: 로그인, 서비스 준비, SSH 터널, 브라우저 열기
- settings.sh / connection.conf: 연결 정보 설정
- install-desktop.sh: Linux 앱 메뉴·바탕화면 바로가기
- disconnect.sh: 이 실행기가 만든 SSH 연결 종료
- common.sh: 공통 설정 검사와 연결 처리

실제 GUI와 C++ 제어기는 Pi에서 실행됩니다. 조향 0~1 및 속도 제어는 현재 Pi 설정을 그대로 사용합니다.
