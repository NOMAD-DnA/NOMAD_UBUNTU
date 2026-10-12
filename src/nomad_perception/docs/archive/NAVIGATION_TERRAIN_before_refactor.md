# 의미·경사 코스트맵 + GNSS·VIO 자율주행

```bash
cd ~/nomad_ws
./run_terrain_navigation.sh  # 맵 선택 → Basalt+GPS·실제 분할·20cm 지도·기존 Planning/Control
# 별도 터미널: 초기 GNSS/VIO 정렬은 수동으로 2m 이상 이동 후 정지
./run_teleop.sh             # 초기정렬 후 Ctrl+C, 자율주행 중 동시 사용 금지
# localization READY 후 RViz의 2D Goal Pose로 목표 지정
```

두 공급자의 READY와 최신 융합 속도(선속도/각속도 각각0.03이하)의 정지가3초 유지되면 기존 Planning/Control을 시작한다. 초기 수동 조종은 정렬 후 정지·종료한다.

기본 `run_autonomy.sh`는 바꾸지 않는다. 기존 `run_terrain_map.sh`는 GT TF 수동 지도 보기 그대로다. 동일 ROS 도메인에서 중복 시뮬레이션/인지/융합을 실행하지 않는다.

- 입력: 실제 SegFormer 10클래스·깊이·동일 촬영시각의 융합 TF.
- 경사: 반경0.3m 원본 지면 점 TLS, 셀높이 fallback 없음.
- 의미: Trees/Logs/Rocks 점이 관측 지면(없으면 촬영시각 차량 높이) 기준 +0.2~+0.8m에서 셀당3개 이상의 돌출점을 서로 다른 촬영시각 4회 이상·0.5초 이상 확인하고 최근2초 관측의75% 이상에서 일치할 때 차단. 수관만으로 차단하지 않는다. 다른 클래스는 부드러운 비용.
- 경사20° 이상도 동일하게 누적 확인 후 차단. 직접 비차단 관측이4회 이상·1초 이상 이어지면 차단 해제. 시간 경과나 반복된 같은 timestamp만으로 확인/해제하지 않음.
- 경사20° 이상 차단. **초기 설정값이며 실제 차량 등판 한계 검증값이 아니다.** `config/navigation_terrain.yaml`에서 변경.
- 통행 비용: 의미분포30%·경사제곱70%, 0~79. 차단100, 미확정-1.
- 전역은 관측 누적. 지역 자유영역은30초 유지 후 미관측; 정적 시뮬레이션 후진 확인용. 동적 환경 안전 보장 아님.
- 기존처럼 LiDAR 자유 ray를 관측 여유/차체 아래 공간에 사용, LiDAR hit로 주행 장애물은 추가하지 않는다. 원본점 경사/의미 차단은 자유 ray보다 우선.
- 장애물 팽창0.7m와 셀 여유, 지역 자유영역 침식 적용. Planning에서는 추가 팽창하지 않는다.
- 출력: `/nomad/navigation_costmap`, `/nomad/navigation_local_costmap`, `/nomad/perception/status`. odom·20cm·Reliable/TransientLocal 지도.
- GNSS·VIO만 동적TF 소유. `/odom`은 Gazebo 진단/평가에만 유지. 단일안테나GPS라 최초 이동정렬 필요.
- GNSS·VIO 실패·지도/위치 만료 시 WAITING, 시계 역행시 FAULT. 자동 VIO 재시작/Control 정지ack 연결은 이번 범위 외이며 기본off.
- 저장: `sessions/terrain_navigation/실행시간/map.npz` 높이·경사·클래스. `costmaps.npz`에 전역·지역 비용과 촬영시각을 저장. 목표는 odom좌표이며 위치 보정점프 대응은 별도 검증 필요.

현재 검증: 33개 단위/회귀 테스트·빌드·ROS 메시지/저장 계약 통과. 실제6번 공터 맵에서 Basalt+이상적 Gazebo GPS 정렬→실제NN코스트맵→기존GPP/LPP/Control 주행→ARRIVED 정지 확인. 정면3m 차단 목표에서는GOAL_BLOCKED 정지, 관측된통행가능목표(-27.5,-7.9)m로재지정후도착. GPS품질메시지 없는 조건이라 FUSED_GNSS_LOW_QUALITY는unknown품질의융합이며RTK시험이아님.

4번숲길초기시험에서한프레임지면오분류차단/START_BLOCKED발견,돌출높이확인과누적차단/해제를추가했다. 수정후4번숲길전체주행·모든맵·후진복구·실물·위치점프·실패재시작은이번시험에포함하지않음. 10Hz는목표이며실제처리시간은status에서확인.

LiDAR관측은같은촬영시각TF를최대0.6초기다려처리하며최신TF로대체하지않는다.

2번맵선택시입구길arc4m중앙에서다음3m길방향을향하는실행용시작자세를생성한다. 원본맵/기존실험시작자세는보존. src/nomad_perception/scripts/prepare_road_start.py. 뒤쪽미관측후진을허용하지않고전진으로초기정렬·출발한다. 초기가정영역은별도로유지하고출발영역반경을벗어나면지역지도에서는실관측만사용. 출발영역 안에서는 아래 초기가정이 우선이다. 선택형통합에서VIO초기출력2초대기(output_warmup_s),기존fusion단독기본0초유지.

초기반경3m는전방과차체직하(-.25m이내)만이다. 미관측뒤쪽을bootstrap으로통행가능하게하지않는다.

초기 출발 가정: `bootstrap_radius: 3.0`, `bootstrap_override: true`. 초기 융합 위치의 전방/차체 아래 영역은 차단·미관측보다 주행 가능(5)을 우선 출력한다. 뒤쪽 미관측은 열지 않는다. 차량이 초기 반경을 벗어나면 우선 적용을 종료하고 원래 센서 판정으로 복귀한다. 센서의 차단 기록 자체는 보존하며, 차량을 따라 움직이는 자유영역이 아니다.

알고리즘 선택: `./run_terrain_navigation.sh` → 맵 번호 → 기존 팀 GPP/LPP 6조합 번호(기본3). `NOMAD_PLANNER_PAIR=1 ./run_terrain_navigation.sh`는 Hybrid A*+RPP. 직접 지정은 `./run_terrain_navigation.sh gpp:=hybrid_astar lpp:=rpp`이며 gpp/lpp/params_file 명시 시 메뉴를 생략한다. 선택값은 초기 정지 대기 후 시작하는 기존 autonomy.launch.py까지 전달된다.

통합 RViz는 nomad_perception/rviz/terrain_navigation.rviz를 사용한다. 기본 Local drivable clearance 하나를 표시하며 Global semantic + slope cost / Observed RGB-D terrain은 필요 시 교대로 켠다. 동일 높이에 여러 Map을 동시에 겹쳐 켜지 않는다. 실제 관측 자유영역은 전역에 유지, 지역은 마지막 negative-ray 관측 후30초까지 유지하며 확인 차단이 우선한다. 통합 RPP 추종 거리는 config/terrain_planning.yaml의1.2m, 원본 Planning 전략/차량 조향 한계는 유지한다.

장애물 주변 최종 비용 정책: config/navigation_terrain.yaml의 obstacle_cost_radius=0.7m. 기존70cm전체차단 팽창 대신 원본 차단 셀 중심에서 거리 d에 따라 추가 비용 round(100*max(0,1-d/0.7))을 적용한다. 20cm간격에서100→71→43→14→0이며 기존의미/경사/관측비용과max로결합하므로총비용은0이아닐수있다. 미관측-1은열지않는다. GPP/LPP의기존80이상차단임계유지. 급경사 차단에는 slope_valid뿐 아니라height_valid/유한높이필수,지면없는경사로차단금지.
