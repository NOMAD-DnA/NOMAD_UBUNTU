# 제어·VESC 정리 (2026-11-11 ~ 11-12, BAE)

제어 파트(Pi4 `nomad_control_cpp`)와 VESC 설정에서 11-11~11-12에 바꾼 것, 시험 결과, 남은 일을 한곳에 모았다.
세부 근거는 GitHub `NOMAD-DnA/NOMAD_UBUNTU`의 커밋 메시지와 Pi4 `~/nomad_cpp_ws/release_backups/*/BUILD_NOTE.txt`에 있다.

## 1. 한눈에

- **턱 못 넘는 문제:** 원인은 센서리스 VESC가 정지 상태에서 회전자 위치를 못 잡는 것. **VESC 센서 모드를 HFI로 바꾸자 책 턱을 넘었다**(11-12 01시, VESC Tool 단위시험).
- **제어기 전류 보호:** 모터 전류 55A / 배터리 입력 30A로 분리. 출발 순간 30~35A 피크로 멈추던 문제 해결.
- **치명/비치명 잠금 분리:** 치명 고장만 잠그고(복구 버튼 필요), 나머지는 정지 출력 후 자동 재출발.
- **정지 출력(11-12 저녁, Pi4에서 수정 → 11-13 저장소 반영):** 0A 대신 VESC 브레이크 20A → 200 ERPM 이하에서 핸드브레이크 15A 유지. 드라이버 패치·설정은 [`driver/README.md`](driver/README.md).
- **ESC 온도 보호:** MOSFET 65℃ 경고, 80℃ 이상 자율주행 정지, 70℃ 미만 재출발.
- **통합 GUI 플래닝 창에 '제어기 복구' 버튼** 추가.
- Pi4 설치본: 11-12 21:33 Pi4에서 직접 빌드한 판(정지 출력·온도 보호 포함, 제어기 sha256 `c4a86cf9…`). 이전 저장소 배포본은 e173e9f(`f03e70a3…`).

## 2. VESC 설정 변경 (VESC Tool, 펌웨어 5.03)

| 항목 | 11-12 이전 | 지금 | 비고 |
|---|---|---|---|
| `foc_sensor_mode` | 0 (Sensorless) | **3 (HFI)** | 핵심 변경 |
| `s_pid_min_erpm` | 900 | **50** | 단독으로는 효과 없었음. 감속 구간에 필요해서 유지 |
| `foc_hfi_voltage_start` | 20 V | **8 V** | 배터리 약 16.5V라 20V는 낼 수 없는 값 |
| `foc_hfi_voltage_run` | 4 V | 4 V | 그대로 |
| `foc_hfi_voltage_max` | 10 V | **8 V** | 턱에서 머뭇거림이 줄어든 값 |
| `foc_hfi_samples` | 16 | 16 | 그대로 |
| `foc_sl_erpm_hfi` | 2000 | 2000 | 주행 1000 ERPM이 HFI 구간 안에 있음 |
| `foc_hfi_start_samples`, `foc_hfi_obs_ovr_sec` | 15, 1 ms | 그대로 | |

**그대로 둔 값(주의):** 모터 전류 ±60A, 배터리 최대 250A, 회생 −200A, 오픈루프 1500 ERPM, 관측기 ORTEGA, 모터 상수(R 0.0112 Ω, L 8.05 µH, λ 0.000769).

설정 파일(`VESC_설정파일/`, SHA256SUMS.txt 포함):
- `mcconf_20261112_before.xml` — 손대기 전 원본 (되돌릴 때 이것)
- `mcconf_20261112_before_2.xml` — `s_pid_min_erpm` 50만 바꾼 상태
- `mcconf_20261112_HFI_Setting.xml` — **지금 VESC에 쓴 값**
- `acconf_20261112_before.xml` — 앱 설정(바꾸지 않음)

되돌리기: VESC Tool → File → Load Motor Configuration XML → 원본 선택 → Write Motor Configuration.
**모터 설정 마법사(Setup Motors FOC)와 'Read Default Motor Configuration'은 누르지 말 것.** 11-09 이후 설정이 통째로 바뀐 원인으로 추정된다.

### 시험 경과 (VESC Tool 직접 구동, Pi4·제어기 없이)
1. duty 모드: 바퀴를 잡으면 떨면서 못 감. ERPM 모드: 돌고 있을 때 잡으면 밀고 나감 → 문제는 **정지 상태 출발**.
2. `s_pid_min_erpm` 900→50만: 책 턱 앞 정지에서 RPM 1000~4500 모두 떨기만 하고 못 넘음.
3. HFI(Start 8 / Run 4 / Max 6): 공중 정상, 정지 상태 전류 약 0A(명령 없을 때 출력 꺼짐). 책 턱: 처음 걸리는 듯하다가 넘어감.
4. HFI Max 6→8V: 머뭇거림 줄고 잘 넘음 → 지금 값으로 저장.

HFI 사용 시 저속 주행 중 고주파 '삐' 소리는 정상이다. 반복 시험, 더 높은 턱, 모터 발열, 출발 방향 오류 여부는 아직 충분히 보지 않았다.

## 3. 제어 코드 변경 (`feat/control`; e173e9f까지 저장소에서 Pi4로 배포, 11-13 커밋은 Pi4 직접 수정분을 옮긴 것)

| 커밋 | 내용 |
|---|---|
| 4a16650 | headless 플래닝은 commission 모드에서만 허용(다른 모드에서 null 역참조로 죽던 조합 차단) |
| e48d541 | 안전 상수 한곳(`control_limits.hpp`), LPP guard가 `motor_max_erpm`을 따름 |
| 2cd3580 | 정지 경로를 `stopAndRelease()` 하나로 |
| 2b7b5a5 | 제어 노드를 라이브러리로 한 번만 컴파일(시험이 배포 코드와 같은 -O1 코드를 검증) |
| 1ef60a2 | 손댄 .cpp에 `using namespace std` |
| 3dbbba3 | LPP 출력 소유를 `PlanningSupervisor` 상태머신으로(단위시험 포함) |
| 9ffea82 | 시험 include 정리 |
| ea5e603 | 상태값 이상 사유 분리, 치명만 잠금 |
| 95bc3e2 | 리뷰 수정(시계 역행·드라이버 준비·Idle 중 치명 고장·피크 즉시 해제) |
| cc4ef3e | **모터 55A / 입력 30A 분리**, 즉시 잠금 60A |
| e173e9f | ROS 시험 기대값을 현재 정책에 맞춤, CTest 제한시간 150초 (여기까지 저장소에서 Pi4로 배포) |
| 2da7b01 | (11-13, Pi4 11-12 수정분 반영) ESC MOSFET 온도 보호 |
| 07668df | (11-13, Pi4 11-12 수정분 반영) 정지 출력: 브레이크 20A → 200 ERPM 이하 핸드브레이크 15A |

### 지금 정지 정책
- **치명(잠김 → 복구 필요):** VESC 고장 코드, 전압 이상, 과속 2500 ERPM 초과, 바퀴 걸림(명령 중 50 ERPM 미만 2초), 다른 프로그램이 VESC에 명령, 운영자 STOP, 과전류(모터 55A·입력 30A 초과가 0.3초 지속 / 60A 1회 / 10초에 3회).
- **비치명(정지 출력 후 대기 → 자동 재출발):** 짧은 전류 피크, 제어 주기 0.2초 지연, 보드 시계 역행, LPP 명령 끊김·만료·STOP, 모터 상태 끊김, 드라이버 일시 사라짐, ESC 80℃ 이상(70℃ 미만까지).
- 수동 GUI 세션은 예전처럼 모두 잠근다.
- 사유가 값과 함께 표시된다. 예: `raw current spike: motor 33.0 A (limit 55), input 2.1 A (limit 30); output released`
- 정지는 브레이크 → 핸드브레이크 유지다(11-12 저녁부터). 핸드브레이크 중에는 VESC ERPM이 0으로 고정돼 미끄러져도 0으로 보이고, 제어기가 꺼지면 300ms 뒤 풀린다. 자율 최소 1000 ERPM 그대로다.

### 제어기 복구 버튼 (통합 GUI, `feat/gui` ddfb8c3·a1c24af)
- 플래닝 개발 창 '출력 정지' 옆 **'제어기 복구'**. 확인 창 후 Pi4 제어기의 RESET만 부른다(구동 명령 없음).
- 차가 0.5초 이상 멈춰 있어야 받아들여진다. **출력 허용 상태면 복구 직후 바로 다시 출발한다.**
- 결과는 플래닝 창 상태줄 아래 노란 줄에 시각과 함께 남는다.

## 4. 남은 일 / 주의

1. **VESC 전류 한계 정리(권장):** 모터 ±60→±50A(VESC가 먼저 제한하고 제어기 55/60A는 최후 보호로), 배터리 250→30A(제어기 입력 기준과 일치). 정지 브레이크 때문에 모터 하한(`l_current_min`)은 −30A 이하 유지. 회생(−200A) 0A는 속도 PID 감속·내리막 제동을 막을 수 있어 보류. 아직 안 바꿈.
2. **수동 GUI 경로(`commission_session/ensure_gui.sh`)가 VESC 설정 점검에서 막힘:** 기대값이 모터·배터리 30A, 회생 0, PID 최소 50이다. 위 1번을 정한 뒤 기대값을 새 값으로 맞춰야 한다. 자율주행 경로(`run_planning_output.sh`)에는 이 점검이 아예 없다.
3. **리뷰에서 나온 제어 개선 후보(미수정):** 피크 직후 곧바로 재출발해 '10초 3회'로 잠기는 문제(재출발 대기 필요), RESET 때 과전류 기록 초기화, 과속 감속 명령이 VESC PID 최소값 아래로 내려가는 문제, 운영자 STOP 후 복구하면 바로 출발하는 문제.
4. **플래너 쪽 합의 필요:** 플래너가 1000 ERPM 하한을 모름(감속 지시가 실제로는 0.27 m/s), 종료 시 드라이버·제어기 동시 종료로 마지막 정지 출력이 VESC에 못 갈 수 있음(VESC 300ms 타임아웃 뒤 관성 정지), `vesc_commission.yaml` 서보 범위 중복.
5. **VIO duty 모드는 계속 사용 금지**(VESC 저속 문제로 44A가 30초 넘게 흐른 기록). HFI 적용 후 재평가 필요. Pi4의 VIO duty 제어기에도 11-12에 같은 정지 출력·온도 보호가 들어갔지만 실차 미시험.
6. 다음 통합시험(ERPM 모드)에서 턱 넘기, 바퀴 걸림 감시 동작, 모터 발열을 확인할 것.
7. 시험 코드: `rpm_running_floor_ros`의 100 ERPM 기대값 2건은 의도적으로 예전 그대로 둠(실패 유지). 'stale IMU' 받침대 시험은 가끔 실패(실차 무관).

## 5. 되돌리기

- VESC: 위 `mcconf_20261112_before.xml`.
- Pi4 제어기: `~/nomad_cpp_ws/release_backups/feat_control_<커밋>_pre_<시각>/RESTORE.txt` (커밋별 백업 5개). 11-12 이전 제어기 소스는 e173e9f 백업(release_backups). `~/nomad_cpp_ws/log/handbrake_20261112`에는 핸드브레이크 추가 직전 소스(온도 보호·브레이크 포함)와 드라이버 원본, `log/stop_brake_20261112/vesc_commission.yaml.before`에는 브레이크 추가 전 드라이버 설정이 있습니다.
- 정지 출력 끄기: `commissioning.yaml`의 `stop_brake_current_a`, `stop_handbrake_current_a`를 0으로(예전 0A 관성 정지).
