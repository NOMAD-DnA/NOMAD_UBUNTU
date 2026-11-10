# 실차 GUI 작업 지침

- 이 폴더는 기존 운영/관전 GUI와 주행 데이터 기록·재생의 기준 소스다. 사용자 승인 범위만 수정하고 실차 보드 런타임과 구별한다.
- 저장소 `NOMAD-DnA/NOMAD_UBUNTU`, 브랜치 `feature/gui-record-replay`.
- **수정 전에 반드시 `git status`/브랜치를 확인하고 `git pull --ff-only origin feature/gui-record-replay`를 한다. 수정·필요 검사 후 관련 파일만 commit하고 `git push origin feature/gui-record-replay`하여 원격 반영까지 확인한다.**
- pull 충돌·기존 미커밋 변경·다른 에이전트 수정이 있으면 덮어쓰지 않고 알린다. 임의 reset/stash/force-push 금지. 공유폴더의 오래된 복사본을 Git 작업본 대신 수정하지 않는다.
- GUI 수정 승인 범위에는 위 동기화·커밋·푸시 절차를 기본으로 포함한다. 별도 보드 코드 수정·설치·모듈 실행·모터 시험·다른 작업은 승인 범위를 확인한다.
- Python은 노트북 GUI/시각화에 사용한다. 보드 센서·실시간 코드의 C/C++ 정책을 임의로 바꾸지 않는다.
- 실시간 데이터는 기존 단일 운영 PC 수신·관전 중계를 재사용한다. 개발 창이나 기록 때문에 새 보드 구독을 추가하지 않는다.
- 재생은 파일 입력만 사용하고 SSH·중계·플래너/목표/arm/모델/종료 명령을 만들지 않는다. 원본 센서 시각을 보존하고 재생 가상 시계로 freshness를 계산한다.
- 지도/영상은 보간하거나 현재 좌표를 임의 변경하지 않는다. 시간 이동 때 미래 프레임·경로·로그를 초기화한다.
- 주행 기록 `*.nomad`, 로그·영상·캐시·초기화 스냅샷·known_hosts·비밀번호·토큰·키를 커밋하지 않는다. finder 비밀번호도 GUI 입력 또는 환경에서만 받는다.
- 변경 범위에 맞는 오프라인 회귀 검사와 UI 확인을 한다. 테스트 가상 입력을 실제 주행 결과로 표현하지 않는다.
- 실행은 저장소 루트의 `bash scripts/run_live_trial_gui.sh`, 기록 위치는 `~/.local/state/nomad-trial/recordings/`가 기본이다. 필수 표시 config는 인접 `../config/oak_basalt_calib.json`을 유지한다.
