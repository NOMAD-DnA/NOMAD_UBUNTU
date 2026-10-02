# 모듈 분리 전 구현 기록

이 문서는 2026-10-02 분리 전 설명을 보존한다. 현재 실행·인터페이스는 각 모듈 README와 nomad_bringup README를 우선한다. 센서 처리 구현은 nomad_perception으로 이동했다.

# NOMAD 패스플래닝

`~/ackermann_sim/src/nomad_planner_test`의 D* Lite GPP, Ackermann rollout LPP,
주행 이력 기반 후진 복구 및 Twist 제어를 NOMAD Gazebo 차량에 이식한 독립 패키지다.
기존 시뮬레이터/센서/TF/월드 파일은 수정하지 않는다. 이 패키지의 launch는
시뮬레이터를 실행하거나 차량 위치를 초기화하지 않는다.

## 빌드와 실행

한 번에 실행하려면 아래 명령을 사용한다. 기존 `nomad` 함수와 같은
`.nomad/env.sh`를 자동으로 읽고 Gazebo + 패스플래닝 + RViz를 모두 실행한다.

```bash
~/nomad_ws/src/nomad_path_planning/run_all.sh
```

통합 launch는 해당 터미널의 전경에서 실행되며, 종료는 Ctrl+C다.
스크립트가 별도 세션이나 백그라운드 작업을 만들지 않고 ROS launch가 자식 프로세스 종료를 관리한다.
같은 ROS domain/partition에 기존 NOMAD가 켜져 있으면 중복 실행을 거부하므로
기존 개별 launch를 먼저 Ctrl+C로 종료한다. `--check`는 실행 없이 환경/중복을 확인한다.
`headless:=true rviz:=false`를 덧붙이면 창 없이 실행할 수 있다.
처음 받았거나 launch 파일을 추가한 뒤에는 아래 개별 빌드 명령을 한 번 실행한다.

기존 시뮬레이터는 평소처럼 `~/nomad_ws/run_forest.sh`로 실행한다.
별도 터미널에서:

```bash
cd ~/nomad_ws
source .nomad/env.sh
colcon build --base-paths src --packages-select nomad_path_planning --symlink-install
source install/local_setup.bash
ros2 launch nomad_path_planning planning.launch.py rviz:=true
```

기존 `.nomad/build.sh`는 `nomad_gazebo`만 선택하므로 새 패키지는 위 명령으로
별도 빌드한다. 모든 소비 노드는 `use_sim_time=true`; 기본 ROS domain은 42다.
RViz의 Fixed Frame은 `odom`, **2D Goal Pose**로 목표를 지정한다.
`Vehicle`은 차량 모델을, `Vehicle position and heading`은 현재 위치의 축을 표시한다
(빨간 축: 차량 전방). 기본 화면은 `base_link`를 따라간다.
설정 변경을 현재 RViz에 적용하려면 File → Open Config에서
`src/nomad_path_planning/rviz/planning.rviz`를 연다. 시뮬레이터 재시작은 필요 없다.
도착 판정은 XY 거리 0.35m이며 목표 자세의 yaw 정렬은 수행하지 않는다.
목표를 받기 전에는 정지한다. 자율주행 중 teleop 등 다른 `/cmd_vel` 발행자를
함께 사용하지 않는다. 계획 launch는 같은 domain 내 중복 실행을 거부한다.
종료는 이 launch 터미널에서 Ctrl+C; 기존 Gazebo watchdog이 명령 만료 시 정지한다.

차량을 움직이지 않고 경로만 확인하려면:

```bash
ros2 launch nomad_path_planning planning.launch.py rviz:=true cmd_vel_topic:=/nomad/planning_preview/cmd_vel
```

좌표로 목표를 지정할 수도 있다. 아래 값은 예시이며 현재 위치 주변의 빈 공간을 고른다.

```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
```

## 입력과 출력

| 경계 | 토픽 / 의미 |
|---|---|
| 입력 | `/odom`: `nav_msgs/Odometry`, `odom → base_link`, 현재 Gazebo ground truth |
| 입력 | `/scan`: `sensor_msgs/LaserScan`, m/rad, 실제 측정 timestamp |
| 입력 | `/tf`, `/tf_static`, `/clock`: 기존 NOMAD가 제공 |
| 목표 | `/goal_pose`: `geometry_msgs/PoseStamped`, 반드시 `odom` 좌표 |
| 입력 | `/oak/{rgb,depth}/image_raw`, `/oak/{rgb,depth}/camera_info`: 지형 기하와 풀 구분용 정렬 RGB-D |
| GPP 입력 | `/nomad/navigation_costmap`: 관측 자유 공간 + 카메라 물체 기반 주행 지도; 미탐색 공간 허용 |
| LPP/제어/복구 입력 | `/nomad/navigation_local_costmap`: 최근 관측된 차체 전체 여유 공간, 장애물은 카메라 판정 |
| 기존 융합 지도 | `/nomad/costmap`, `/nomad/local_costmap`: 기존 LiDAR + 카메라 정보 유지, 기본 주행 입력에서 제외 |
| 지형 디버그 | `/nomad/terrain_costmap`: 깊이 기반 지형 비용·위험 영역 (팽창 전) |
| 영상/진단 | `/nomad/terrain_labels`: colour 모드의 영역 분류 영상, `/nomad/terrain_status`: 카메라 처리 상태 |
| 디버그 지도 | `/nomad/observed_map`: 사전 지도 없는 LiDAR 누적 점유 격자 |
| 경로 | `/nomad/global_path`, `/nomad/local_path`: `nav_msgs/Path`, `odom` |
| 연결 | `/nomad/current_pose`, `/nomad/goal`, `/nomad/sensors_ready` |
| 진단 | `/nomad/{mapping,gpp,lpp,controller}_status`: `std_msgs/String` |
| 제어 | `/cmd_vel`: `geometry_msgs/Twist`, m/s + yaw rad/s → 기존 watchdog |

지도 publisher는 reliable/transient-local, 센서 subscriber는 best-effort/volatile로
reliable 및 best-effort 입력을 모두 받는다. 계획 메시지는 reliable이다.
내부 목표 `/nomad/goal`과 전역 경로는 publisher/subscriber 모두 transient-local로
마지막 값을 유지해 노드 시작 순서에 따른 첫 메시지 유실을 방지한다.
새 TF를 발행하지 않으며 `map → odom` identity도 추가하지 않는다.

## 이식 차이와 동작

### 자율주행 LiDAR 제외 (2026-10-02)

`navigation.lidar_enabled: false`가 기본이다. `/scan` 수신과 기존 융합 코스트맵,
깊이 기반 기울기 계산은 유지한다. 주행용 두 지도에서는 LiDAR 장애물·팽창만
제외한다. LiDAR ray가 실제로 통과한 자유 공간은 기존 지도 정보로 유지한다.
LiDAR 반사 셀 자체는 카메라 지면 확인 전까지 미관측으로 남으며, 반사 뒤를 열지 않는다. GPP, LPP, 제어기와 후진 복구는 launch remap으로 모두 주행용 지도를 쓴다.
`true`로 바꾸고 계획 노드를 재시작하면 두 주행 지도는 기존 융합 지도와 같아진다.

LiDAR 장애물을 제외하는 주행은 `terrain.enabled: true`, `terrain.mode: geometry`,
`terrain.ground_filter_enabled: true`, `terrain.objects_only: true`를 요구한다.
연결된 관측 지면은 자유 공간, 지면 위 12cm 이상 물체는 장애물이며 여유 반경
0.70m는 유지한다. 기울기 계산은 계속하지만 기존 `terrain.slope_enabled: false`와
`objects_only: true`에 따라 경사 자체를 차단 비용으로 쓰지 않는다.
별도의 경사각 지도 토픽을 추가한 것은 아니다.

기준 지면 없는 풀만으로 자유 공간을 만들지 않으며, 두 센서 모두 미관측인 곳은
열지 않는다. 카메라 지형 TTL은 30초, 지역 자유 공간은 기존 2초 제한을 사용한다.
지역 지도는 차체 여유 원 전체의 최근 자유 공간 관측을 요구한다. 차량 바로 아래가
카메라에 보이지 않아도 기존 LiDAR 자유 공간 관측으로 출발할 수 있다. LiDAR 오인
차단 제거가 모든 오르막의 주행 가능성을 보장하지 않는다.
주행 health는 odom과 최근 처리된 깊이/TF 및 연결 지면을 요구하고,
scan 중단은 주행 health를 직접 낮추지 않지만 LiDAR로만 확인한 자유 공간은 2초 뒤
지역 지도에서 만료된다. 깊이 중단/오류는 정지시킨다.

센서 수신·위치·health 발행은 무거운 지도 계산과 별도의 callback group에서
실행한다. 지도 변경은 한 계산 작업만 수행하며 센서 대기열은 최신 메시지 1개를
우선한다. 주행 health는 **새 깊이 입력**의 0.6초 신선도와 **계산 완료한 지도**의
기존 지역 유효 기간 2초를 각각 확인한다. 둘 다 wall-clock 3초 제한을 적용한다.
새 영상이 도착해도 오래된 계산 결과의 시각을 갱신하지 않는다. 풀 voxel 평균은
전체 점군을 voxel마다 재검색하지 않고 한 번의 집계로 계산한다. 샘플 간격과
지면·물체 판정 기준은 유지한다.

목표가 이미 발행한 지도 안에 있으면 깊이 계산 완료를 기다리지 않고 즉시
`/nomad/goal`로 전달한다. 지도 확장이 필요한 목표만 지도 작업에서 처리하며,
더 새 목표가 도착하면 오래된 확장 요청은 발행하지 않는다. GPP는 새 목표에 대해
이전 목표의 재시도 지연을 적용하지 않고, 동일 지도 메시지를 pose/timer마다
다시 비교하지 않는다. LPP 계산은 wall-clock 10Hz이며 궤적·센서 신선도는 계속
시뮬레이션 시간 기준이다. 누적 LiDAR 높이 대조는 셀별 일괄 계산으로 처리한다.

RViz의 기존 지도는 참고용으로 유지한다. 실제 주행 지도를 보려면 기존 지도 표시를
끄고 `Navigation cost` 및 `Navigation local clearance` 항목을 켠다.

### 공통 동작

- 원본 전역/지역 계획 알고리즘을 재사용하며 기본 후진 복구는 별도 감독 노드가 담당한다. 원본의 Saye 월드,
  장애물 truth 조회, Gazebo 생성, OpenVINS 및 카메라 보정 파일은 가져오지 않았다.
- NOMAD wheelbase 0.72m, rear axle에서 base_link까지 0.36m를 반영한다.
  rollout은 rear axle에서 적분하고 출력 좌표를 차체 중심으로 변환한다.
  가상 중앙 조향각 ±0.40rad는 안쪽 바퀴의 ±0.55rad 제한 내 보수적 설정이다.
- 차체/바퀴를 감싸는 원형 여유 반경 0.70m에 격자 반대각선을 더해 장애물을
  팽창한다. 좁은 공간에서는 차량이 물리적으로 통과 가능해도 멈출 수 있다.
- 전진 최대 0.40m/s, 전진 rollout 최대 1.5m. 최종 목표가 더 가까우면 목표까지의
  거리로 후보 길이를 줄여 목표 뒤의 장애물 때문에 도착을 거부하지 않는다.
  목표 이전 구간의 충돌·미관측 검사는 유지한다. 기본 후진 복귀는 아래 주행 이력 방식을 쓴다.
  `managed_recovery: false`의 이전 LPP 단독 모드에만 0.6m·최대 3회 짧은 후진이 남아 있다.
  감독 노드를 함께 실행할 때는 반드시 `managed_recovery: true`를 유지한다.
- LiDAR는 **측정 시점의** `odom ← scan` 3D TF로 각 ray를 변환하고 XY에 투영한다.
  TF가 늦으면 잠깐 대기하며 최신 TF로 대체하지 않는다. 주행 health에 필요한 센서는
  위 LiDAR 사용 설정에 따르며, 기울기 정지는 `tilt_guard_enabled`를 켤 때만 적용한다.
- 시계가 뒤로 가거나 큰 odometry 위치 점프를 검출하면 정지 상태를 유지한다.
  위치/월드 초기화 뒤에는 계획 launch를 재시작해 누적 지도를 버린다.
- 제어 명령은 wall-clock 10Hz로 발행해 느리게 실행되는 Gazebo에서도 기존
  0.35초 watchdog과 호환된다. 시뮬레이션 시계가 0.5초간 멈추면 0을 발행한다.
  센서 신선도는 시뮬레이션 시간 0.6초와 wall-clock 3초를 함께 확인한다.
- 설정은 `config/planning.yaml`; `params_file:=/절대경로/config.yaml`로 교체 가능하다.

## 야지 지형 비용

기본 `terrain.mode: geometry`는 사전 월드 파일을 사용하지 않는다.
깊이 영상과 CameraInfo를 촬영 시각의 `odom ← optical` TF로 변환한다.
기울기·거칠기는 깊이만으로 계산하며, 풀 구분에는 정렬된 RGB-D가 필요하다. `odom`의 Z가 중력 수직이라는
현재 시뮬레이터 계약을 전제로 하며, IMU를 직접 융합하거나 새 TF를 발행하지 않는다.

0.2m 격자에서 직접 관측된 점이 최소 6개인 셀마다 반경 0.35m 이웃의 평면을 맞춘다.
기울기, 평면에 수직인 잔차의 RMS(거칠기), 잔차 최댓값-최솟값(높이 불연속 지표)을
계산한다. 평면을 결정할 수 없는 좁은 관측은 미관측으로 남기며, 높이 차가 큰
수직면은 장애물로 처리한다. 급경사와 깊이 경계를 미리 버리지 않는다.

| 항목 | 기본 차단 기준 | 설정 |
|---|---|---|
| 기울기 | 0.35rad, 약 20° | `terrain.slope_limit` |
| 거칠기 | RMS 0.04m | `terrain.roughness_limit` |
| 높이 불연속 지표 | 잔차 범위 0.18m | `terrain.step_limit` |

각 측정값을 차단 기준으로 나눈 비율 중 최댓값으로 0~79 비용을 만들고,
하나라도 기준 이상이면 100으로 차단한다. 이 값들은 **실측 차량 능력이 아닌 초기값**이다.
높이 불연속 지표는 정확한 단차 높이나 낙차 측정값이 아니다.
`terrain.patch_radius`, `terrain.geometry_min_samples`, `terrain.pixel_stride`로
평가 크기와 관측 밀도를 조절한다. 광학 깊이 최대 범위는 기본 6m다.

풀 또는 아래의 연결된 지면으로 확인되지 않은 LiDAR 장애물은 그대로 유지한다. 깊이 기반 위험 셀은 기존 0.70m 여유 반경의
커널로 팽창해 GPP/LPP/제어 costmap에 100으로 추가한다. 낮은 지형 비용으로
LiDAR 장애물이나 미관측 공간을 지우지 않는다. 미관측 공간은 GPP에서 탐색을
허용하지만 LPP/제어에서는 금지한다. LiDAR가 비어 있고 깊이 지형만 미관측인
셀은 중립 비용 35이며, 지면 지지력이 확인됐다는 의미는 아니다.

기하 관측은 `terrain.memory_seconds`(기본 시뮬레이션 30초) 동안 유지한다.
깊이/TF 입력이 끊겨도 관측된 위험은 이 기간 동안 유지하며, 만료 후 중립 비용으로
돌아간다. 카메라 단절만으로 정지시키지는 않는다. 기존 odom/scan/TF 신선도 및
차체 기울기 정지 조건은 유지한다. `/nomad/terrain_status`는 처리 셀 수, 차단 수,
최대 기울기·거칠기·잔차 범위와 stale/reject 이유를 표시한다.

RViz의 `Fused driving cost`는 합성 비용, `Depth terrain cost and hazards`는
팽창 전 깊이 지형 비용이다. `/nomad/terrain_labels` 색상 영상은
이전 `terrain.mode: colour` 모드에서만 발행한다. 기존 HSV 설정도 colour 전용이다.

## 현재 한계

이 구현은 관측된 표면의 기하 기반 비용이며 완전한 3D 주행 가능성 판정은 아니다.
가려진 지면, 깊이가 반환되지 않는 낭떠러지, 지반 지지력·마찰·바퀴 미끄러짐,
차량 하부 여유와 방향별 전복/등판 능력을 판정하지 않는다. 아래 조건으로 확인하지
못한 식생은 충돌체가 없어도 깊이 영상에서 거친 표면/장애물로 보일 수 있다.
텍스처에만 있는 거칠기는 깊이 기하에 반영되지 않는다. 센서 노이즈와 소수 이상점은
잔차 범위를 키워 보수적으로 차단할 수 있다.

깊이로 확인하지 못한 지면 반사는 여전히 장애물로 처리하며, 위쪽으로 향한 빈 ray가
지면의 통행 가능성을 증명하지 않는다. D* 격자 경로는 Ackermann 회전반경을 보장하지 않는다.
현재 `/odom`은 정답 위치이며 실제 localization 검증이 아니다. 실제 숲 주행과
처리 지연을 확인한 뒤 기준을 조정해야 하며, 완주나 물리적 안전을 보장하지 않는다.

실행 중인 시뮬레이터에 명령을 보내지 않고 확인하려면:

```bash
source ~/nomad_ws/.nomad/env.sh
cd ~/nomad_ws
python3 src/nomad_path_planning/scripts/verify_terrain.py
```

검증 노드는 출력을 `/nomad/terrain_validation/` 아래로 분리하고 `/cmd_vel`을 발행하지 않는다.

## 깊이 기반 지면 반사 구분

확인된 지면은 `terrain.ground_memory_seconds: 10.0`초 동안 월드 좌표로
보관한다. 신선한 깊이 입력이 있는 동안 시야 밖으로 나간 기존 관측 지면도
LiDAR 높이 대조와 과거 점유 해제에 사용한다. 이전 높이와 맞는 직접 재관측은
지면 연결을 다시 시작할 수 있다. 새 불일치 관측은 해당 셀의 옛 지면을 폐기한다.
빈 관측·TF 오류·시간 역행은 기록을 초기화한다. 영상이 0.3초 이상 오래되거나
wall-clock 입력이 끊기면 기록을 사용하지 않는다. 처음부터 미관측인 지면은 열지
않는다. 격자 경계는 인접 셀의 실제 관측 범위 ±4cm 안에서만 대조하며 높이 차
5cm 조건과 돌출 물체 유지는 그대로다.

현재 기본값은 `terrain.objects_only: true`다. 지면의 경사·거칠기·평면 잔차를
주행 위험 비용으로 쓰지 않고, 연결된 관측 지면 위의 물체를 판정한다.
셀 자체의 평면을 우선 맞춰 옆 나무/돌이 지면 추정을 오염시키지 않게 한다.
셀이 성기면 주변 평면을 사용하되 해당 셀의 모든 점도 검사한다.
접지 근처에서 이어진 지면 셀은 비용 0이다. 반경 0.45m 이내의 관측 지면 평면들이
높이에 동의할 때, 그 기준보다 `terrain.obstacle_height: 0.12`m 이상 높은 점이
3개 이상이면 물체 비용 100을 준다. 지면 기준이 없으면 미확인(-1)으로 남는다.
지면 평면의 잔차·연결성 검사는 물체를 지면으로 지우지 않기 위한 분류 조건이며,
그 조건에서 탈락했다는 이유만으로 지형 비용 100을 만들지는 않는다.
LiDAR 미분류 반사, 미관측 영역과 팽창 0.70m는 유지한다. 낮은 녹색 잡초는 기존
RGB-D 휴리스틱으로 통과 가능 여부를 판단하며 모든 식생/돌의 종류를 식별하는
의미론적 모델은 아니다. 지면처럼 매끄럽게 이어진 낮은 물체와 가려진 지면은 한계다.
`objects_only: false`에서만 아래 기존 기하 비용 규칙을 적용한다.

현재는 사용자 요청으로 **경사 판단을 비활성화**했다.
`terrain.slope_enabled: false`이면 경사 비용과 경사 임계값 차단, 연결 지면 후보의
경사 제한을 적용하지 않는다. `tilt_guard_enabled: false`이면 차량과 스캔 기울기로
센서 입력을 거부하지 않는다. 경사 측정값은 진단용으로만 남는다.
3D TF 보정과 잘못된 quaternion 거부, 거칠기·단차·수직면·고체 반사·팽창은 유지한다.
이 절의 각도 제한 설명은 해당 옵션을 다시 켤 때 적용되며 변경 후 계획 노드 재시작이 필요하다.

`terrain.ground_filter_enabled: true`가 기본이다. 깊이와 LiDAR 각각의 촬영 시각
TF로 관측점을 odom에 정렬하며, 깊이 촬영 시각의 `odom ← base_link`도 필요하다.
RGB 색상이나 맵 정답은 지면 판정에 사용하지 않는다.

0.2m 셀에서 직접 관측한 점이 최소 3개, 반경 0.30m 주변에 최소 6개 있어야
평면을 맞춘다. 기울기가 `terrain.slope_limit` 미만이고 수직 잔차 RMS가 2.5cm
미만, 개별 잔차가 5cm 미만이어야 후보가 된다. 수직면, 표본 부족, 혼합 표면은 제외한다.

차량 앞 0.35~1.5m·좌우 0.6m의 가까운 관측 중 차체 접지 기준면과 높이 차가
15cm 이내인 후보를 찾는다. 가장 가까운 후보에서 전방 25cm 이내의 띠를 잡고,
그 띠의 가장 낮은 표면에서 높이 5cm 이내인 셀만 시작점으로 삼는다.
곡면에서는 먼 지면의 접평면을 차량 위치까지 연장한 높이가 맞지 않을 수 있으므로
그 외삽 조건은 사용하지 않는다. 이는 현재 차량의 base_link가 지면 기준이라는
계약에 의존한다. 이후 네 방향으로 인접하며 서로의 평면 예측 높이가 4cm 이내로
이어지는 관측 셀만 연결한다. 관측이 비거나 단차가 있으면 연결을 끊는다.
따라서 떨어진 바위 윗면을 단지 평평하다는 이유로 지면으로 인정하지 않는다.

LiDAR 반사점이 연결된 지면의 관측 범위 안에 있고 평면 높이와 5cm 이내로
맞을 때만 해당 반사를 자유 끝점으로 누적한다. 깊이 관측은 스캔보다 미래이면
사용하지 않으며 최대 0.3초(시뮬레이션 시간)까지만 허용한다. 입력/TF 오류나
새로운 빈 관측은 지면 증거를 폐기하고, wall-clock 입력 지연도 검사한다.

기존 점유 기록은 반복된 자유 관측으로 감소한다. 또한 셀별 LiDAR 반사점의 3D
좌표를 보관해 새 깊이 영상에서 저장된 모든 반사점이 연결 지면으로 확인되면
그 셀만 해제한다. 차량 이동 후 ray가 짧아져 옛 지면 반사 위치까지 닿지 않는
경우에도 적용된다. 반복 반사는 셀 내부 2cm XY 구간별 3D 최솟값/최댓값 상자로
보관하며 개수가 많아져도 높이 증거를 버리지 않는다. 상자의 모든 모서리가 지면과
맞아야 해제하므로 높은 고체 반사가 평균으로 사라지지 않는다.
깊이 샘플 간격은 `terrain.pixel_stride: 2`다. 높이가 없는 기록, 고체 반사가 섞인 셀,
깊이 영상보다 새로운 스캔은 이 방식으로 해제하지 않는다.
`cleared_ground_cells`는 해당 영상에서 해제한 셀 수다.
측정 끝점 뒤로 ray를 연장하지 않고 미관측을 열지 않는다. 지면의 기울기·
거칠기 비용과 장애물 팽창 0.70m는 유지한다. 카메라 사각이나 급경사에서는 지면
반사도 계속 장애물로 남을 수 있다. 작은 높이 차가 허용 오차 안에 들어갈 수 있어
이 수치는 실측 센서 오차와 차량 여유에 맞춰 검증해야 한다.

`/nomad/terrain_status`의 `connected_ground_cells`로 관측된 연결 지면 셀 수를
확인한다. 실행 중인 노드는 코드가 자동 교체되지 않으므로 계획 launch 재시작이 필요하다.

## 통과 가능한 낮은 풀 인식

`terrain.grass_enabled: true`가 기본이다. Gazebo 메시/센서 마스크를 바꾸지 않고
RGB-D 관측에서 낮은 풀을 분리하는 초기 휴리스틱이다. 학습된 풀 의미 분할 모델이
아니므로 모든 종류·높이의 풀을 식별하지 못한다.

- 녹색 픽셀(OpenCV H=28~95, S≥60, V≥35)과 주변의 직접 관측된 비녹색 낮은
  표면으로 지면 평면을 추정한다. 표본이 적거나 평면 잔차 RMS가 2.5cm를 넘거나
  기울기가 0.30rad를 넘으면 통과 가능한 풀로 확정하지 않는다.
- 관측 지면 범위 안에서 지면 위 4~60cm(`terrain.grass_max_height`, 최대 0.60m)의
  후보만 인정한다. 10cm 3D voxel에 최소 3점이 있고 전부 후보이며 서로 다른
  두 깊이 프레임 이상에서 확인돼야 한다. 같은 voxel에 비풀 관측이 있으면 취소한다.
- 확인한 풀 점은 지면 기울기/거칠기 계산에서 제외한다. 같은 셀의 관측 지면 비용이나
  단단한 위험이 우선하며, 그것이 없는 풀 셀은 중립 비용 35를 사용한다.
- LiDAR 반사점이 최근 3초 이내의 확인된 풀 표면에서 10cm 이내이고 가까운
  비풀 관측이 없을 때 해당 반사를 통과 가능한 끝점으로 처리한다. 기존 점유 증거는
  정상 자유 관측 누적으로 감소한다. 반사점 뒤로 ray를 연장하지 않으며 다른
  단단한 반사가 같은 격자에 있으면 장애물이 우선한다.
- RGB·보정·동기화가 없으면 새 풀 확인을 하지 않고 깊이 기하 처리를 유지한다.
  관측 밖 증거는 3초 후 만료한다. `/nomad/terrain_status`의 `grass_points`와
  unavailable 이유로 확인한다. 장애물 여유 반경은 0.70m 그대로다.

전방 카메라에서 한 번도 보지 못한 측후방 풀, 높은 풀 벽, 가려진 지면 위 식생은
구분하지 못한다. 따라서 현재 출발점 측후방 풀의 정지를 이 기능만으로 해결한다고
보장할 수 없다. 녹색 물체나 낮은 가지 오분류 가능성이 있으며, 실물 적용에는
검증된 의미 분할과 추가 시야/센서가 필요하다. 센서가 본 풀과 가려진 지면의
물리적 주행 가능성은 서로 다른 정보다.

## 프론티어 없는 막다른 길 복귀

`recovery_supervisor`는 실제 주행한 `odom` 좌표와 **차량 yaw**를 약 0.15m 간격으로
최대 60m 저장한다. 미관측 경계나 정보 이득, 월드 정답 지도는 사용하지 않는다.
논문의 모드 전환·이력 복귀 개념을 적용한 설계이며 논문 알고리즘 전체의 재현은 아니다.

1. 정상 주행: GPP → LPP → 제어기로 움직이며 주행 이력을 기록한다.
2. 정체 감지: 복귀 가능한 연결된 이력이 쌓인 뒤부터 감시한다. 신선한 센서 입력이
   있는데 4초 동안 이동 범위의 대각선이 0.6m 이하면 복귀를 시작한다. 최초 GPP 계산
   대기·주행 이력 부족은 복귀 정지로 고정하지 않는다. 이는 정체 휴리스틱이다.
   별도로, 전역 경로의 다음 약 1m가 차량 뒤쪽을 향하면 제어기는 전진을 즉시 차단한다.
   뒤쪽 방향이 0.3초 유지되고 연결된 이력이 있으면 4초 정체 대기 없이 후진 복귀를 시작한다.
   판단은 현재 차량 yaw와 경로의 순서를 사용하며 최종 목표나 경로점 yaw를 사용하지 않는다.
   옆으로 꺾이거나 먼저 앞으로 진행하는 우회 경로는 뒤쪽 경로로 처리하지 않는다.
3. 연속 후진: 이력의 가장 이전 연결 지점까지 하나의 복귀 기준선으로 사용한다.
   기본 최대 범위는 저장된 이력 내 60m다. **2m/8m 중간 지점에서 종료하거나 전진을
   시도하지 않는다.** 0.6m 후진 rollout은 계속 갱신되며 yaw를 뒤집지 않는다.
   후진 0.20m/s, 축거 0.72m, 조향 한계 0.40rad를 사용한다.
4. 재진입 방지: 복귀 시작 위치에서 이력상 약 1m 뒤에 폭 4m의 방향성 통과 제한을
   한 번 둔다. GPP(D* Lite/A*)와 LPP에 적용하되 밖으로 후진하는 방향은 허용한다.
   복귀 도중에는 만료하지 않고, 탈출 완료 뒤 120초 동안 유지한다. 새 목표에서 해제한다.
5. 출구 검증: 후진하면서 0.5초마다 원래 목표로 향하는 GPP 경로를 확인한다.
   그 경로를 따라 전진하는 **약 3m의 조향 변화가 가능한 Ackermann 궤적**을 제한된
   후보 탐색으로 만든다. 현재 관측된 local costmap에서 모든 연결 구간이 통행 가능해야 한다.
   경로상 2m 이상 진행하며 종점은 GPP 경로에서 0.6m 이내, 방향 오차 0.6rad 이내이고,
   막힌 길의 진입 이력에서 1m 이상 떨어져야 출구로 인정한다. 단순히 GPP 경로가
   있거나 통로 안에서 짧게 전진할 수 있다는 이유로 후진을 종료하지 않는다.
6. 출구 진입: 검증된 출구가 발견되면 0.3초 정지 후, 그 전진 궤적을 복귀 감독기가
   직접 추종한다. 매 주기 최신 지도에서 충돌을 재검사한다. 검증한 궤적을 따라
   다른 통로로 진입한 뒤에 정상 GPP/LPP로 제어권을 넘긴다. 최종 목표는 바꾸지 않는다.
7. 정지 조건: 후방 장애물·미관측·센서 단절에는 정지한다. 후진 중 전역 경로가
   없거나 LPP가 실패했다는 이유만으로는 후진을 끊지 않는다. 이력 끝까지 와도
   출구가 없으면 멈춰 기다리고, 복귀 전체 제한 시간 360초가 지나면 STOP을 유지한다.
   목표당 복귀 시도는 최대 4회이며, 이력점 통과는 추가 시도로 세지 않는다.
   새 장애물이 검증된 전진 출구를 막으면 그 궤적도 즉시 정지/재검사한다.
8. 출구 각도 재조정: 출구 추종 실패가 0.3초 지속되면 감독기가 약 0.6m의
   짧은 후진 후보와 그 종점에서의 전진 출구를 함께 검사한다. 둘 다 현재 지도에서
   통행 가능할 때만 0.3초 정지 후 조향하며 후진한다. 실제 도달한 위치에서 전진
   출구를 다시 계산하고, 다시 0.3초 정지한 뒤 전진한다. 짧은 조정은 복귀당 최대
   3회다. 조정 후보가 없거나 횟수가 소진되면 실제 출구 진입 이력까지 포함해
   더 뒤로 복귀하고 실패한 진입 방향을 제한한다. 짧은 조정 중 새 장애물을
   만나거나 도착 자세에서 출구가 사라지면 연결된 후진 기준선으로 돌아간다.
   후방도 장애물/미관측이면 정지를 유지한다. 프론티어는 사용하지 않는다.
   `recovery.adjust_length`, `recovery.adjust_attempts`, `recovery.exit_blocked_seconds`로
   설정한다. GPP는 지도/출발 격자 변경 시 발행하므로 정지 중에는 과거의 동일 목표
   경로를 방향 안내로 재사용할 수 있다. 실제 이동 허가는 항상 최신 센서·local costmap·
   방향성 제한으로 검사하며, 3초가 지난 경로 자체를 통행 가능 증거로 사용하지 않는다.

| 토픽 | 타입 / 계약 |
|---|---|
| `/nomad/recovery_path` | `nav_msgs/Path`, odom, 10Hz 복귀 중 발행. 빈 배열=제어권 해제, 1점=정지 유지, 2점 이상=부호 있는 복귀 rollout(후진 또는 검증된 전진 출구) |
| `/nomad/recovery_reference` | `nav_msgs/Path`, transient-local. 현재 추종하는 전체 후진 이력선 또는 전진 출구 궤적, 디버그용 |
| `/nomad/recovery_gates` | `geometry_msgs/PoseArray`, transient-local. 위치=제한 중심, yaw=차단할 진입 방향 |
| `/nomad/recovery_status` | `std_msgs/String`, WAIT_HISTORY/MONITORING/REVERSE/SWITCH/EXIT/ADJUST_STOP/ADJUST_REVERSE/RETRY_RETREAT/COMPLETE 및 중단 이유 |

제어기는 활성 복귀 경로를 정상 LPP보다 우선하고, 이때는 전역 경로가 없어도 후진할 수 있다.
복귀 메시지가 0.5초 이상 끊기면 정지하며 정상 경로로 임의 전환하지 않는다. 명시적으로
해제된 후에도 이전 LPP 경로를 재사용하지 않는다. 센서 health·odom·시계 정지 보호는 유지한다.
`recovery.gate_half_width`는 YAML 공통 영역에서 세 노드에 같은 값으로 적용해야 한다.

정상 LPP는 충돌 없는 전진 후보라도 전역 경로의 다음 진행 방향이 뒤쪽이면 실패로 처리한다.
제어기는 후방 경로를 수신한 콜백에서 기존 전진 궤적을 지우고 0을 발행하며, 이후 늦게
도착한 전진 LPP 메시지도 거부한다. 제어 주기에서도 현재 차량 방향을 다시 확인한다.
`LPP_PATH_BEHIND`, `RECOVERY_PATH_BEHIND` 상태로 확인할 수 있다. 이 차단은 정상 전진에
적용하며, 이미 활성화된 감독기의 후진/검증된 출구 궤적은 전역 경로 갱신으로 중단하지 않는다.
`recovery.behind_seconds`는 후방 경로 확인 대기(기본 0.30초)이며 전진 정지는 즉시 수행한다.

이력은 목표 설정 이후부터 쌓인다. 시작하자마자 막혀 기록된 이력이 없으면 자동 후진하지 않고 정상 계획기에 제어를 맡긴다.
초기 경로가 늦게 생성되어도 새 목표 없이 전진할 수 있다.
출구 후보 탐색은 미래 3m의 국소 기하 검증이며 갈림길의 의미나 최종 목적지까지의
전체 주행 가능성을 증명하지 않는다. 제한된 후보 탐색이라 존재하는 출구를 놓칠 수 있다.
곡선의 조향 변화, 좁은 공간, 위치 오차, 관측 부족으로 후진/전진 후보가 없으면 멈출 수 있다.
숲 전체 탈출 성공은 실제 Gazebo 주행으로 별도 확인해야 한다.

격리된 ROS 연결 검증(사용하지 않는 도메인 선택):

```bash
source ~/nomad_ws/.nomad/env.sh
cd ~/nomad_ws
ROS_DOMAIN_ID=93 python3 src/nomad_path_planning/scripts/verify_history_recovery.py
```

이 검증은 실제 GPP/LPP/감독기/제어기와 합성 지도·Ackermann 위치 적분을 연결한다.
긴 막다른 통로에서 연속 후진 → 출구 전진 → 정상 주행을 확인한다. 시험 전용 명령
토픽을 사용하며 Gazebo 물리 시험은 아니다.

## 검증

```bash
source ~/nomad_ws/.nomad/env.sh
cd ~/nomad_ws
colcon test --packages-select nomad_path_planning --event-handlers console_direct+
colcon test-result --test-result-base build/nomad_path_planning --verbose
```

검증 범위와 실제 실행 결과는 `VALIDATION.md`에 기록한다.
원본 모듈은 Apache-2.0이며 `LICENSE`를 보존했다.
