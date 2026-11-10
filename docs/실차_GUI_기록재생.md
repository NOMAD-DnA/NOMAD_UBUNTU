# 실차 GUI · 주행 데이터 기록과 재생

Linux 노트북에서 실행한다. 운영·관전·위치추정/인지/주행가능성/플래닝/제어 개발 창에 주행 데이터 기록·재생을 추가한 GUI다.

팀원은 기존 공유 GUI 폴더를 계속 사용한다. 다시 다운로드하거나 작업 위치를 옮길 필요가 없다.

```bash
cd "/home/user/samba/shared/NOMAD_모듈별_통합_20261104/NOMAD_실물시험앱_20261106/laptop_monitor"
bash run_app.sh
```

GitHub `NOMAD-DnA/NOMAD_UBUNTU`의 기존 `feat/gui`는 이 공유 GUI의 백업 대상이다. Git 백업본을 별도로 실행할 때는 저장소 루트의 `bash scripts/run_live_trial_gui.sh`를 사용한다.

최초 의존성 설치가 필요한 PC에서만:

```bash
sudo apt install python3 python3-pyqt5 python3-matplotlib python3-numpy openssh-client iproute2 util-linux
```

- **운영:** 보드 주소·SSH 비밀번호 입력 → 시험 시작 또는 기존 실행에 화면 연결. 보드 찾기도 입력한 비밀번호를 사용한다.
- **관전:** 운영 PC의 중계 주소로 연결한다. 관전은 ROS·보드 비밀번호 없이 운영 PC가 받은 데이터를 본다.
- **기록:** 실시간 창의 기록 시작 → 기록 정지. 기록 폴더에서 `.nomad` 파일을 찾는다.
- **재생:** 시작 창 또는 실시간 창의 기록 열기 → `.nomad` 선택. 같은 개발 창에서 재생/일시정지·시간 이동·배속을 사용한다. 재생은 보드 연결·구동 명령을 하지 않는다.

기록은 기본 `~/.local/state/nomad-trial/recordings/`에 날짜·시각 파일명으로 저장된다. 저장 완료 후 파일 한 개를 팀원에게 전달하고 같은 버전의 GUI에서 연다. 주행 기록은 Git에 넣지 않는다.

GUI 코드와 팀원 수정 위치: `tools/visual_trial_20261106/laptop_monitor/README.md`.
전방 표시 설정: `tools/visual_trial_20261106/config/oak_basalt_calib.json`.

실시간 연결은 기존 native 보드 작업공간과 전달기·Pi4 플래너/제어기가 설치돼 있다는 전제다. 이 브랜치는 노트북 GUI 백업이며 보드 전체 설치를 새로 수행하지 않는다. 재생은 보드 없이 가능하다. 기존 GUI가 받은 영상 미리보기·지도 이미지/격자 위치·연속 위치·경로/제어·로그를 기록하며, GUI에 오지 않는 원본 점군·수치 비용배열을 새로 수집하지 않는다. 과거에 저장하지 않은 주행 데이터는 복원하지 않는다.

업데이트 규칙은 공유폴더와 GUI 폴더의 `AGENTS.md`를 따른다: **공유폴더에서 수정·확인 → 관련 변경 commit → feat/gui에 push → 원격 반영 확인**. 작업 전 pull이나 clone은 강제하지 않는다. 원격 head를 확인해 그 커밋을 부모로 공유 GUI 변경만 백업할 수 있으며, 공유폴더를 원격 사본으로 덮어쓰거나 force-push하지 않는다. 기존 사용자 수정과 무관한 원격 파일을 보존한다. GUI 코드는 오프라인 검사·가상 입력 UI 확인을 수행했으며, 실물 전체 기록·재생 검증은 이후 현장 시험과 구분한다.
