# 현재 로컬 GUI 보존본 (2026-11-23)

사용자 요청으로 `feat/percep`에 현재 노트북에서 사용하는 GUI 소스를 보존했다.

- 기준 배포 버전: `daf6ecec7cca`, 설치 기록 2026-11-22 21:52:24.
- `laptop_monitor/`, 표시용 `config/oak_basalt_calib.json`과 안내 문서는 이 배포 사본에서 복사했다.
- 소스 코드와 실행 모드의 원래 파일 권한을 보존했다. 캐시, 주행 기록, 개인 GUI 상태, SSH 인증 정보와 로컬 설치 표식은 포함하지 않는다.
- 기존 매퍼 C++ 소스, Jetson 설치 파일과 active_config는 이번 커밋에서 바꾸지 않는다.
- GUI 기본 설정과 특정 노트북 또는 보드에서 마지막으로 저장한 실행 설정은 다르다. 예전에 GUI에 입력한 값이 clone만으로 보드에 자동 적용되는 것은 아니다.

## Git 사본 실행

저장소 루트에서 다음 명령을 사용한다.

```bash
bash scripts/run_live_trial_gui.sh
```

Linux와 Python 3, PyQt5, matplotlib, numpy, OpenSSH 등의 의존성은 `laptop_monitor/run_app.sh`를 따른다. 이 명령은 실제 운영 GUI이므로 보드 실행과 차량 제어는 장비 및 운영 절차를 확인한 뒤 사용한다.

GUI 코드는 보드의 기존 센서, 매퍼 및 실행 환경을 호출한다. 이 커밋이 Jetson과 Pi의 전체 소스 또는 실행 환경을 복원하는 패키지는 아니다. Pi5의 별도 시험 키트 등 필요한 보드 구성은 해당 문서를 함께 확인한다.

공유폴더가 개발 원본이고 노트북 current는 배포 사본이라는 기존 운영 구조는 그대로다. 이번 보존 대상 브랜치는 사용자가 지정한 feat/percep이며, 복사한 AGENTS.md의 기존 feat/gui 운영 지침은 원래 출처의 규칙이다.
