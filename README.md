# NOMAD

**2026-10-27 플래닝 변경·106개 시험 자료를 받으신 분은 [먼저 읽어주세요 — 인수인계](docs/먼저_읽어주세요_플래닝_인수인계.md)를 먼저 확인해 주세요.**

야지 차량 시뮬레이션: 위치추정 → 의미·경사 지도 → 플래닝 → 제어.

## 처음 프로젝트를 사용하는 경우

Ubuntu 24.04·ROS 2 Jazzy·NVIDIA 드라이버가 준비된 PC에서 시작한다.
환경 준비부터 소스 받기·설치·빌드까지 **[설치 및 실행 안내](docs/실행안내.md#1-처음-설치할-때)** 순서대로 진행한다.

## 설치를 끝낸 경우

```bash
cd ~/nomad_ws
./scripts/run_terrain_navigation.sh viewer:=false  # 맵·플래너 선택 → 전체 실행 + RViz
```

다른 터미널에서 `./scripts/run_teleop.sh`로 초기정렬 이동 후 정지·창 종료.
위치추정·인지가 준비되면 RViz `2D Goal Pose`로 목표를 찍는다.

## 각 파트만 켜고 싶은 경우

```bash
./scripts/run_module.sh help  # sim / localization / semantic / map / planning / control / rviz
```

실행 순서와 설명은 **[파트별 실행](docs/실행안내.md#3-각-파트를-따로-실행할-때)**.

## 파일을 찾을 때

- `scripts/`: 사람이 실행하는 쉘 스크립트
- `docs/`: 설치·실행 안내와 [코드 구조](docs/STRUCTURE.md)
- `src/`: 각 파트 코드·config
- `.nomad/`: 설치 도구·개발 환경

기존 GT 기반 자율주행은 `scripts/run_autonomy.sh`. 기술 상세·과거 문서는 각 패키지 README와 `docs/archive/`에 보관한다.
