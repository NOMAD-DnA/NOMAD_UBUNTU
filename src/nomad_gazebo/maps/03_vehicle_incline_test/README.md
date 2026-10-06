# 차량 시험장

기존 `./run_forest.sh` 또는 `./slam_experiment.sh sim` → **3번**.

| 경사 | 레인 중심 Y(m) |
|---|---|
| 5° | -25 |
| 10° | -15 |
| 15° | -5 |
| 20° | 5 |
| 25° | 15 |
| 30° | 25 |

평지 120×90m. 각 삼각 경사로는 X=0~16m, 수평 상승 길이 8m, 폭 4m. 시작은 5° 레인에서 +X 방향.

설정: `assets/authoring/vehicle_test.json`의 `payload_kg`(추가 kg), `friction_mu`(마찰), `incline_degrees`(각도).

```bash
python3 ~/nomad_ws/src/nomad_gazebo/scripts/generate_vehicle_test_map.py # 변경 후 재생성·시뮬레이션 재시작
```

기본 총질량 34.6kg + payload_kg. 추가 질량은 차체 CG에 반영하며 CG 위치는 고정한다.
시험: 각도·적재량·진입속도·마찰, 경사 중 정지/재출발, 후진, 정상부 차체 접촉.
현재 속도 제어 구동이므로 실제 모터 토크 한계에 따른 등판 시험에는 토크 제한 구동기가 추가로 필요하다.
