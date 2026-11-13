# VESC 드라이버 패치와 설정 (Pi4 `~/vesc_ws`, 2026-11-12)

제어기의 정지 출력(브레이크 → 핸드브레이크 유지)은 f1tenth `vesc_driver`에 핸드브레이크 토픽을 추가한 Pi4 드라이버가 있어야 동작합니다. 드라이버는 이 저장소 밖(Pi4 `~/vesc_ws/src/vesc`)에 있으므로, 같은 드라이버를 다시 만들 수 있도록 패치와 설정을 여기에 남깁니다.

| 파일 | 내용 |
|---|---|
| `vesc_driver_nomad_153998d.patch` | Pi4 드라이버 전체 변경분. upstream `https://github.com/f1tenth/vesc` 커밋 `153998d`(ros2_humble 병합)에 적용하면 Pi4 소스와 똑같아집니다(확인함). 11-12 이전 NOMAD 수정(소멸자에서 직렬 작업 정지, `packet_thread_run_` atomic)과 핸드브레이크를 포함합니다. |
| `vesc_driver_handbrake_20261112.patch` | 11-12 핸드브레이크 추가분만. `commands/motor/handbrake`(std_msgs/Float64, A) → `COMM_SET_HANDBRAKE`(ID 10, int32 A×1000). `handbrake_min/max`로 잘립니다. |
| `vesc_commission.yaml` | Pi4 `~/nomad_cpp_ws/commission_session/vesc_commission.yaml`(드라이버 파라미터, 같은 위치에 둡니다). 11-12에 `brake_max` 0→30, `handbrake_min 0`·`handbrake_max 30` 추가. |

## 다시 만들기

```bash
cd ~/vesc_ws/src
git clone https://github.com/f1tenth/vesc.git && cd vesc
git checkout 153998df8545fe1781b975df88e411b4e71d4bfe
git apply --check <저장소>/src/nomad_control_cpp/vesc_config_20261112/driver/vesc_driver_nomad_153998d.patch
git apply <저장소>/src/nomad_control_cpp/vesc_config_20261112/driver/vesc_driver_nomad_153998d.patch
cd ~/vesc_ws && colcon build --packages-up-to vesc_driver
```

## 주의

- **드라이버 한계값이 0이면 관성 정지가 됩니다.** 드라이버의 `brake_*`·`handbrake_*` 기본값은 0이고 명령을 그 범위로 자릅니다. 11-12 이전 Pi4 yaml(`brake_max 0`)이면 브레이크도, `handbrake_*`가 없는 yaml(upstream 포함)이면 핸드브레이크가 0 A로 잘려 VESC가 출력을 끕니다. 제어기 로그에는 표시가 없고 드라이버 로그에만 `... clipping`이 남습니다(11-12 21:10 시험이 이 상태였습니다). `brake_max ≥ stop_brake_current_a`, `handbrake_max ≥ stop_handbrake_current_a`로 두세요.
- **패치 안 한 드라이버:** 제어기는 핸드브레이크 토픽 구독자가 없으면 드라이버 준비가 안 된 것으로 보고 출발하지 않습니다(안전한 쪽). 사유는 처음 출발 때 `raw start refused: need exclusive driver ...`, 재출발 대기 중에는 `driver endpoints unavailable; waiting`으로만 나오므로 `ros2 topic info -v /nomad_bench/commands/motor/handbrake`로 구독자를 확인하세요.
- **VESC 모터 전류 하한:** VESC는 브레이크 전류를 `|l_current_min|`로 자릅니다. `l_current_min ≤ −30 A`를 유지하세요(지금 −60 A).
- **핸드브레이크 중 속도값:** 펌웨어 5.03은 핸드브레이크 중 위상을 0으로 고정하므로, VESC가 보고하는 ERPM과 타코가 실제 회전과 상관없이 0이 됩니다.
- **제어기가 꺼지면:** VESC는 마지막 명령 300 ms 뒤(`timeout_msec`) `timeout_brake_current`(지금 0 = 관성)로 풀립니다. 5.03에는 타임아웃 때 핸드브레이크를 거는 설정이 없습니다.
