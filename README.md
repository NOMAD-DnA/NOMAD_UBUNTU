# NOMAD 시뮬레이션 환경 설치

이 저장소는 NOMAD의 ROS 2 시뮬레이션 워크스페이스입니다. `mvsim`은 차량과 월드를 계산하는 시뮬레이터이고, `nomad_sim`은 우리가 만든 야지 맵·센서 설정·ROS 실행 파일을 담은 패키지입니다. 두 패키지는 함께 빌드합니다.

## 1. 준비

Ubuntu 24.04 데스크톱 환경에 [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)를 설치하세요. Gazebo나 Isaac Sim은 이 환경의 실행에 필요하지 않습니다. `git`, `python3-colcon-common-extensions`, `python3-rosdep`도 필요합니다.

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-rosdep
```

`rosdep`을 처음 쓴다면 `sudo rosdep init`을 한 번 실행하고, `rosdep update`를 실행하세요. 이미 초기화돼 있다면 다시 초기화하지 않아도 됩니다.

## 2. 워크스페이스 받기

홈 디렉터리에 `nomad_ws`가 **없는 경우** 아래처럼 복제합니다. 기존 워크스페이스가 있다면 덮어쓰지 말고 먼저 내용을 확인하세요.

```bash
git clone https://github.com/NOMAD-DnA/NOMAD_UBUNTU.git ~/nomad_ws
cd ~/nomad_ws
```

저장소의 `src/mvsim`에는 현재 프로젝트에서 사용한 MVSim 1.4.0 소스와 로컬 수정이 포함돼 있습니다. 별도의 MVSim 저장소를 다시 복제할 필요는 없습니다.

## 3. 의존성 설치와 빌드

```bash
cd ~/nomad_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
./.nomad/build.sh
```

빌드 결과인 `build/`와 `install/`은 각 컴퓨터에서 생성됩니다. `.nomad/env.sh`는 ROS 환경과 워크스페이스 경로를 설정하고, `.nomad/build.sh`는 `mvsim`과 `nomad_sim`을 빌드합니다. 두 파일은 평소에 직접 수정할 필요가 없습니다.

## 4. 실행

```bash
cd ~/nomad_ws
./run_forest.sh
```

MVSim 창에서 W/S는 전진·후진, A/D는 회전, Space는 정지입니다. 종료할 때는 실행 터미널에서 Ctrl+C를 누르세요. MVSim은 포트 23700을 사용하므로 같은 컴퓨터에서 두 번 실행하면 두 번째 실행이 거부됩니다.

기본 ROS 도메인은 42이고 검색 범위는 `LOCALHOST`입니다. 따라서 이 설정 그대로는 다른 컴퓨터의 ROS 노드와 자동으로 연결되지 않습니다. 센서 데이터를 쓰는 별도 노드는 `use_sim_time:=true`로 실행하세요.

### 다른 터미널에서 ROS 토픽 사용하기

`run_forest.sh`는 실행할 때 `.nomad/env.sh`를 내부에서 읽습니다. 하지만 그 설정은 **새로 연 터미널에는 전달되지 않습니다.** 토픽을 보거나 팀의 ROS 노드를 실행할 터미널마다 환경을 설정해야 합니다.

매번 긴 경로를 입력하지 않으려면 각자 `~/.bashrc` 맨 아래에 다음 함수를 한 번 추가하세요. 빌드·실행 스크립트가 개인의 `~/.bashrc`를 자동 수정하지는 않습니다.

```bash
nomad() {
  source "$HOME/nomad_ws/.nomad/env.sh"
}
```

새 터미널을 연 다음 `nomad`를 입력하고 ROS 명령을 사용하면 됩니다. 기존 터미널에서는 `source ~/.bashrc`로 함수를 불러올 수 있습니다.

```bash
nomad
ros2 topic list -t
```

시뮬레이터를 실행하는 첫 번째 터미널에는 `nomad`가 필요하지 않습니다. `./run_forest.sh`만 실행하세요. 토픽은 시뮬레이터가 실행 중일 때 보이며, 위 명령은 **같은 컴퓨터의 별도 터미널**에서 사용합니다.

## 폴더 위치

- `src/mvsim/`: 시뮬레이터 엔진과 ROS 노드. `externals/box2d` 소스도 포함됩니다.
- `src/nomad_sim/worlds/`, `assets/`: 현재 맵, 지형, 차량 주변 모델과 하늘 텍스처.
- `src/nomad_sim/config/`, `sensors/`: 카메라·라이다 설정과 장착 위치.
- `src/nomad_sim/launch/`, `scripts/`: 실행 연결, 맵 생성기와 ROS 센서 브리지.
- `.nomad/`: 환경·빌드 설정. 최상위에서 평소 사용하는 스크립트는 `run_forest.sh`입니다.

현재 차량은 MVSim의 Jackal 차동구동 모델이며, 자체 차량 모델이나 자율 탐색 알고리즘은 아직 포함되지 않았습니다. 카메라와 라이다 값은 실제 장비의 측정 보정값이 아닌 임시 시뮬레이션 설정입니다.
