# GPS 설정·실물 교체

```bash
./run_forest.sh                                      # GPS 포함 시뮬레이션
source ~/nomad_ws/.nomad/env.sh                       # 다른 터미널
ros2 topic echo /gps/fix sensor_msgs/msg/NavSatFix    # 위도·경도·고도
```

## 시뮬레이션 설정

`src/nomad_gazebo/config/sensors.yaml` → `gps` 수정 후 시뮬레이션 재시작. 지형 재생성 불필요.

| 설정 | 의미 | 현재 |
|---|---|---|
| `enabled` | Gazebo GPS 활성화 | true |
| `model_reference` | 구매 후 모델명 기록, 동작에 영향 없는 메모 | 구매 전 |
| `hz` | 실제 설정한 위치 출력 주기(시뮬레이션 시간) | 5Hz |
| `mount_*_m` | base_link→안테나 위상중심, 차량 +X 전방/+Y 왼쪽/+Z 위 | 0,0,0.45m |
| `position_noise_horizontal_sigma_m` | 수평 가우시안 위치 오차 표준편차(m) | 0 |
| `position_noise_vertical_sigma_m` | 수직 가우시안 위치 오차 표준편차(m) | 0 |
| `latitude_deg`, `longitude_deg`, `elevation_m` | Gazebo 월드 원점(실제 시작 위치 아님) | 37°,127°,0m |

기존 월드에 `spherical_coordinates`가 있으면 그 원점을 우선한다. 월드 +X 동쪽/+Y 북쪽/+Z 위(ENU). 출력 위치는 `gps_link` 안테나 기준이며 `/odom`의 base_link와 다르다. 장착 위치는 센서와 정적 TF에 함께 적용된다.

오차는 1σ 기준으로 입력한다. 제품의 CEP·95% 오차·수평RMS·분산을 그대로 넣지 않는다. 제조사 정의 또는 수집한 데이터로 환산/추정한다. 현재 sigma0은 이상적인 수신 시험이며 RTK 성능을 뜻하지 않는다. 단순 Gaussian 설정은 위성 수·차폐·멀티패스·RTK fixed/float·시간에 따른 바이어스를 재현하지 않는다.

## 실제 GPS 구매 후

1. `model_reference`에 수신기/안테나 모델 기록. 출력 주기·안테나 위상중심 위치를 위 설정에 반영.
2. 제조사 ROS2 드라이버에서 포트·baudrate·메시지·주기 설정. RTK를 사용하면 그 드라이버/수신기에서 보정 입력(NTRIP/기준국)을 별도 설정. 이 YAML은 실물 수신기를 설정하지 않는다.
3. 실제 드라이버의 NavSatFix를 `/gps/fix`로 remap하고 frame_id=`gps_link`로 설정. 실제 실행의 robot_state_publisher 또는 static TF에 동일한 안테나 장착 TF 제공. 현재 GPS TF는 Gazebo launch에만 추가되어 있다.
4. 실제 GPS와 Gazebo GPS가 같은 `/gps/fix`를 동시에 발행하지 않게 한다. 필요하면 시뮬레이션 `gps.enabled: false`. 실물은 ROS 시간, 시뮬레이션은 sim time으로 동기화하고, 고도 기준(타원체고/해발고)을 확인.
5. NavSatFix status·공분산 의미 확인 후 융합 모듈 연결. 위성수·RTK 상태는 NavSatFix에 모두 들어가지 않으므로 수신기 전용 상태 토픽을 별도로 확인.

현재 ros_gz_bridge는 공분산을 UNKNOWN으로 내보낸다. 위 sigma 설정을 변경해도 메시지 공분산이 자동으로 채워지지 않는다. VIO 융합 전 별도 공분산 처리가 필요하다. GPS↔VIO 융합은 아직 구현하지 않았다. 미터 좌표 변환은 아래 RViz 시각화에서 제공한다.

## GPS 시각화

```bash
./run_forest.sh                                    # GPS + 미터 좌표 계산 노드
# 다른 터미널
source ~/nomad_ws/.nomad/env.sh
ros2 launch nomad_gazebo gps_rviz.launch.py         # 보라색 GPS 위치·이동 궤적
```

`/gps/fix`의 WGS84 위도·경도·타원체고를 ECEF→ENU로 변환해 `/nomad/visualization/gps`(MarkerArray) 하나에 현재 안테나 위치와 최근1000점 궤적을 발행한다. 메시지 시각은 원본 GPS stamp, 좌표frame은 Gazebo 월드와 같은 `odom`. 고정 원점은 launch가 **실제 선택 월드**의 spherical_coordinates에서 읽는다. 첫 GPS를0으로 잡지 않는다. 현재 ENU/heading0 월드를 지원한다. 다른 원점·축의 외부VIO frame에 이름만 맞춰 겹치지 않는다. 그 경우 별도 좌표 정합/TF가 필요하다.

시각화는 안테나 기준이며 차량 중심 보정·GPS자세·GPS속도 추정·공분산 추정·융합·동적TF 발행을 하지 않는다. 최종 `/nomad/localization/odometry`는 기존 인터페이스로 유지하고 이 노드는 발행하지 않는다. `/gps/odometry`, `/gps/path`도 추가하지 않는다. `gps.visualization: false`이면 표시노드만 끈다.

No fix·잘못된 좌표는 표시/이력을 삭제하며 GPS시간 역행(rosbag rewind)시 궤적을 초기화한다. 새 fix 없이 ROS/sim시간2초가 지나면 표시가 만료된다. Gazebo/rosbag을 일시정지하면 sim시간도 멈추므로 만료시간도 멈춘다.

실물 GPS를 별도로 표시할 때는 동일 원점이 정의된 로컬 frame에 아래 노드를 실행한다(37/127/0은 예시 기본원점, 실제datum으로 변경).
```bash
ros2 run nomad_gazebo gps_visualizer.py --ros-args \
  -p origin_latitude_deg:=37.0 -p origin_longitude_deg:=127.0 \
  -p origin_elevation_m:=0.0 -p fixed_frame:=odom
ros2 launch nomad_gazebo gps_rviz.launch.py use_sim_time:=false
```
시뮬레이션이 이미 켜져 있으면 gps_visualizer를 중복 실행하지 않는다. RViz 설정 위치는 `src/nomad_gazebo/config/gps.rviz`.


## GNSS 오류 시험 — 수동 실행

```bash
source ~/nomad_ws/.nomad/env.sh
ros2 run nomad_gazebo gnss_fault_injector.py --ros-args -p use_sim_time:=true
# 이벤트 데모 설정으로 실행
ros2 run nomad_gazebo gnss_fault_injector.py --ros-args -p use_sim_time:=true \
  -p profile_path:=$HOME/nomad_ws/src/nomad_gazebo/config/gnss_demo.yaml
```

두 명령 중 하나만 실행한다. 기본 설정 gnss_sim.yaml은 오차 없이 통과한다. 입력 /gps/fix는 그대로, 출력은 /experiment/gps/fix(NavSatFix)와 /experiment/gps/quality(nomad_interfaces/GnssQuality). launch 자동 실행과 기존 GT 어댑터 변경 없음. fusion/odometry/TF는 발행하지 않는다. 실물은 use_sim_time:=false.

설정은 첫 입력 측정 stamp 기준 초, ENU=[동,북,상] m, 반개구간[start,start+duration). seed가 동일한 동일 입력은 재현된다. actual_sigma_enu_m는 실제 주입 잡음, reported_sigma_enu_m는 보고 공분산 1σ로 별도 설정한다(null이면 원본 공분산 유지). noise/offset/drift/spike(구간 첫 입력 딱1개)/dropout(위치·품질 모두중단)/no_fix(상태NO_FIX·좌표NaN)/delay(측정stamp유지·발행지연)/freeze(좌표·측정stamp반복)/quality 이벤트를 제공한다. 겹친 변위는 합산, 잡음·보고공분산·품질은 뒤 이벤트 우선. dropout은 새 출력 생성 중단이며 이전 delay 대기값은 예정대로 나올 수 있다.

Gazebo 센서 자체 sigma는 0으로 유지하고 meter 잡음은 이 노드에서 넣는다. 현재 설치 NavSat의 수평 noise Apply가 degree 좌표에 적용되므로 기존 sensors.yaml의 비영점 수평 sigma를 미터로 믿고 넣지 않는다. 데모의 잡음/상태/8m 튐 등은 설명용 가정이며 F9P 야지 실측값이 아니다. 시간역행 시 이벤트·seed·delay queue를 초기화한다. delay 후 원래 stamp의 메시지가 늦게 도착할 수 있으므로 소비자가 시각 검사를 해야 한다.

## 정상·실패 시험 프로필

- `config/f9p_single.yaml`: F9P-01B 수평1.5m CEP 기반 Gaussian 근사.
- `config/f9p_rtk.yaml`: 수평1cm+1ppm CEP, 기준국1km 가정(1.1cm).
- `config/f9p_failure.yaml`: 잡음증가·연속으로변하는bias, 정상사양과별도인실패시험.
- `config/gnss_demo.yaml`: 이전강한오류시험. 기본launch 자동실행없음.

위 수평 사양을 E/N 독립Gaussian의 σ로환산. 실제시간상관성·수직오차·receiver공분산은미검증. 위성수/PDOP/보정age는이프로필에서미제공;covarianceUNKNOWN유지. 실제F9P가해당필드를제공하지않는다는뜻이아님.

```bash
ros2 run nomad_gazebo gnss_fault_injector.py --ros-args -p use_sim_time:=true \
  -p profile_path:=$HOME/nomad_ws/src/nomad_gazebo/config/f9p_single.yaml
```

사양근거: https://content.u-blox.com/sites/default/files/documents/ZED-F9P-01B_DataSheet_UBX-17051259.pdf 표2. 구매보드칩/펌웨어미확정이며사양기준근사일뿐실물성능검증아님.

### 독립 실패 시험

`f9p_noise_010.yaml` / `f9p_noise_050.yaml` / `f9p_noise_200.yaml`: 수평축별σ0.1/0.5/2m를 각각별도시험. `f9p_bias.yaml`: 시간변화편향만시험. 각20–50초오류구간,앞뒤정상RTK근사. 이전f9p_failure.yaml은혼합이력용이며독립시험에는위4설정을사용. 모두수평만·인위적오류·수동실행,자동launch변경없음.
