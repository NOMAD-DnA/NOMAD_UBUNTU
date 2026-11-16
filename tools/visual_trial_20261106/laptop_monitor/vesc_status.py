"""2026-11-16 Pi4 제어기의 모터·서보(VESC) 값을 읽기 전용 표시 줄로 바꾼다 (Qt 없음, 시험용 순수 함수).

입력은 두 가지다.
- 키보드 조종: Pi4 제어기 HTTP `/api/status` JSON (keyboard_drive가 이미 0.1초마다 받는 값).
- 자율주행: 제어기 로그의 `control_trace`·`vesc_trace` 줄 (developer_control이 이미 읽는 로그).
제어기 4e64c7e 이전 배포본에는 새 키·`vesc_trace`가 없다. 그때는 값을 꾸미지 않고 '—'로 둔다.
서보 값은 모두 명령값이다(서보 피드백·실측 조향각은 없음). 입력 전류는 VESC 추정값이다.
"""
from __future__ import annotations

import math
import re

# VESC 펌웨어 5.03 mc_fault_code (제어기가 fault_name을 보내지 않는 옛 배포본용).
FAULT_NAMES = (
    "NONE", "OVER_VOLTAGE", "UNDER_VOLTAGE", "DRV", "ABS_OVER_CURRENT", "OVER_TEMP_FET", "OVER_TEMP_MOTOR",
    "GATE_DRIVER_OVER_VOLTAGE", "GATE_DRIVER_UNDER_VOLTAGE", "MCU_UNDER_VOLTAGE", "BOOTING_FROM_WATCHDOG_RESET",
    "ENCODER_SPI", "ENCODER_SINCOS_BELOW_MIN_AMPLITUDE", "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE", "FLASH_CORRUPTION",
    "HIGH_OFFSET_CURRENT_SENSOR_1", "HIGH_OFFSET_CURRENT_SENSOR_2", "HIGH_OFFSET_CURRENT_SENSOR_3",
    "UNBALANCED_CURRENTS", "BRK", "RESOLVER_LOT", "RESOLVER_DOS", "RESOLVER_LOS", "FLASH_CORRUPTION_APP_CFG",
    "FLASH_CORRUPTION_MC_CFG", "ENCODER_NO_MAGNET", "ENCODER_MAGNET_TOO_STRONG")
FAULT_KO = {"NONE": "없음", "OVER_VOLTAGE": "과전압", "UNDER_VOLTAGE": "저전압", "DRV": "게이트 드라이버",
            "ABS_OVER_CURRENT": "과전류", "OVER_TEMP_FET": "FET 과열", "OVER_TEMP_MOTOR": "모터 과열",
            "NO_TELEMETRY": "상태값 없음"}
DUTY_LIMIT_WARN = 0.9      # VESC 최대 듀티 0.95 근처 = 그 배터리 전압의 최고 속도
TELEMETRY_WARN_S = 0.2     # 드라이버는 50 Hz로 보냄
MISSING = "—"

_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_KEY = re.compile(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=")
_ROS_STAMP = re.compile(r"\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([0-9]+(?:\.[0-9]+)?)\]")
VESC_TRACE_NUMERIC = ("voltage_v", "input_current_a", "power_w", "iq_a", "duty", "fault_code",
                      "servo_cmd", "servo_target", "telemetry_age_ms")
VESC_TRACE_TEXT = ("fault", "hold_basis")


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _num(value, digits=1, sign=False):
    return (f"{value:+.{digits}f}" if sign else f"{value:.{digits}f}") if finite(value) else MISSING


def _number(text):
    try:
        value = float(text)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def fault_text(code, name=None):
    """'0 NONE (없음)'. 코드가 없거나 숫자가 아니면 '—'."""
    if not finite(code):
        return MISSING
    code = int(code)
    if not name:
        name = "NO_TELEMETRY" if code < 0 else FAULT_NAMES[code] if code < len(FAULT_NAMES) else "UNKNOWN"
    korean = FAULT_KO.get(name)
    return f"{code} {name}" + (f" ({korean})" if korean else "")


def fault_level(code, latched=False):
    if latched is True or (finite(code) and code > 0):
        return "fail"
    return "ok" if finite(code) and code == 0 else "unknown"


def thermal_level(state):
    return {"ok": "ok", "warn": "warn", "derate": "warn", "stop": "fail"}.get(state, "unknown")


def _servo_text(command, target):
    if not finite(command) and not finite(target):
        return MISSING
    return f"명령 {_num(command, 3)} · 목표 {_num(target, 3)} (명령값, 실측 조향 아님)"


def status_rows(data, fresh=True):
    """키보드 조종 HTTP 상태 → (제목, 값, 수준) 목록. 수준: ok/warn/fail/unknown."""
    data = data if isinstance(data, dict) else {}
    get = data.get
    rows = []
    voltage, current_in, power = get("voltage_v"), get("input_current_a"), get("input_power_w")
    rows.append(("배터리", f"{_num(voltage, 2)} V · 입력 {_num(current_in, 2)} A(VESC 추정) · {_num(power, 0)} W", "ok"))
    rows.append(("모터 전류", f"{_num(get('current_a'), 1)} A · 토크 전류 iq {_num(get('iq_a'), 1)} A", "ok"))
    duty = get("duty")
    rows.append(("듀티", _num(duty, 3, sign=True) + (" · 전압 한계 근처" if finite(duty) and abs(duty) >= DUTY_LIMIT_WARN else ""),
                 "warn" if finite(duty) and abs(duty) >= DUTY_LIMIT_WARN else "ok"))
    erpm, speed = get("raw_erpm"), get("estimated_speed_mps")
    if get("erpm_valid") is False:
        rows.append(("바퀴 속도", f"{_num(erpm, 0)} ERPM · 핸드브레이크 중(측정값 아님)", "unknown"))
    else:
        verified = get("estimated_speed_verified") is True
        rows.append(("바퀴 속도", f"{_num(erpm, 0)} ERPM" + (f" ≈ {_num(speed, 2)} m/s" + ("" if verified else " (환산 미검증)")
                                                           if finite(speed) else ""), "ok"))
    rows.append(("속도 명령", f"목표 {_num(get('target_motor_erpm'), 0)} · 명령 {_num(get('commanded_motor_erpm'), 0)} ERPM", "ok"))
    temp, state = get("esc_temp_c"), get("esc_temp_state")
    rows.append(("ESC 온도", f"{_num(temp, 1)} ℃ · {state or MISSING}", thermal_level(state)))
    stop = get("stop_output")
    rows.append(("정지 출력", f"{stop or MISSING} {_num(get('stop_output_a'), 1)} A · 근거 {get('stop_hold_basis') or MISSING}", "ok"))
    code, latched = get("fault_code"), get("vesc_fault")
    rows.append(("VESC 고장", fault_text(code, get("fault_name")) + (" · 제어기 잠금" if latched is True else ""),
                 fault_level(code, latched)))
    age, ready = get("telemetry_age_s"), get("driver_ready")
    age_text = f"{age*1000:.0f} ms 전" if finite(age) else MISSING
    ready_text = {True: "드라이버 준비", False: "드라이버 미준비"}.get(ready, "드라이버 " + MISSING)
    late = (finite(age) and age > TELEMETRY_WARN_S) or ready is False or not fresh
    rows.append(("VESC 상태값", f"{age_text} · {ready_text}" + ("" if fresh else " · 상태 끊김"), "warn" if late else "ok"))
    rows.append(("서보", _servo_text(get("commanded_servo_position"), get("requested_servo_position")), "ok"))
    return rows


def diagnostic_rows(data):
    """키보드 조종 HTTP 상태 → VESC 진단 줄 (값 확인용, 판단 기준 없음)."""
    data = data if isinstance(data, dict) else {}
    get = data.get
    temps = get("esc_mos_temps_c")
    temps_text = " / ".join(_num(value, 1) for value in temps) + " ℃" if isinstance(temps, list) and temps else MISSING
    count = lambda key: str(int(get(key))) if finite(get(key)) else MISSING
    return [
        ("MOSFET 온도 1/2/3", temps_text),
        ("d·q축", f"id {_num(get('id_a'), 2)} A · vd {_num(get('vd_v'), 2)} V · vq {_num(get('vq_v'), 2)} V"),
        ("사용량 (VESC 켠 뒤)", f"소모 {_num(get('charge_drawn_ah'), 3)} Ah / {_num(get('energy_drawn_wh'), 1)} Wh · "
                              f"회생 {_num(get('charge_regen_ah'), 3)} Ah / {_num(get('energy_regen_wh'), 1)} Wh"),
        ("타코미터", f"{count('tacho')} · 누적 {count('tacho_abs')} (전기 1회전 6, 거리 환산 미검증)"),
        ("VESC ID", count("vesc_controller_id")),
    ]


def controller_sends_vesc_values(data):
    """4e64c7e 이전 제어기는 새 VESC 키를 보내지 않는다."""
    return isinstance(data, dict) and "input_current_a" in data


def vesc_trace_from_line(line):
    """제어기 로그 한 줄의 `vesc_trace` → dict (없는 값은 None, 이전 값을 물려받지 않음)."""
    if not isinstance(line, str):
        return None
    line = _ANSI.sub("", line)
    marker = line.find("vesc_trace ")
    if marker < 0 or not line.lstrip().startswith("[controller]"):
        return None
    payload = line[marker + len("vesc_trace "):]
    matches = list(_KEY.finditer(payload))
    values = {match.group(1): payload[match.end():matches[index+1].start() if index+1 < len(matches) else len(payload)].strip()
              for index, match in enumerate(matches)}
    trace = {key: _number(values.get(key)) for key in VESC_TRACE_NUMERIC}
    for key in VESC_TRACE_TEXT:
        value = values.get(key)
        trace[key] = value[:80] if value else None
    if finite(trace["telemetry_age_ms"]) and trace["telemetry_age_ms"] < 0:
        trace["telemetry_age_ms"] = None  # -1 = 상태값 없음
    stamp = _ROS_STAMP.search(line[:marker])
    trace["stamp"] = _number(stamp.group(1)) if stamp else None
    return trace


def trace_rows(control, vesc):
    """자율주행 로그(control_trace의 추가 값 + vesc_trace) → (제목, 값, 수준) 목록."""
    control = control if isinstance(control, dict) else {}
    vesc = vesc if isinstance(vesc, dict) else {}
    rows = []
    rows.append(("배터리", f"{_num(vesc.get('voltage_v'), 2)} V · 입력 {_num(vesc.get('input_current_a'), 2)} A(VESC 추정) · "
                          f"{_num(vesc.get('power_w'), 0)} W", "ok"))
    rows.append(("토크 전류 iq", f"{_num(vesc.get('iq_a'), 1)} A", "ok"))
    duty = vesc.get("duty")
    rows.append(("듀티", _num(duty, 3, sign=True) + (" · 전압 한계 근처" if finite(duty) and abs(duty) >= DUTY_LIMIT_WARN else ""),
                 "warn" if finite(duty) and abs(duty) >= DUTY_LIMIT_WARN else "ok"))
    temp, thermal = control.get("esc_temp_c"), control.get("thermal")
    rows.append(("ESC 온도", f"{_num(temp, 1)} ℃ · {thermal or MISSING}", thermal_level(thermal)))
    rows.append(("정지 출력", f"{control.get('stop') or MISSING} {_num(control.get('stop_a'), 1)} A · 근거 "
                            f"{vesc.get('hold_basis') or MISSING}", "ok"))
    valid = control.get("erpm_valid")
    rows.append(("측정 ERPM 유효", {"1": "예", "0": "아니오 · 핸드브레이크 중(ERPM 0은 측정값 아님)"}.get(valid, MISSING),
                 {"1": "ok", "0": "unknown"}.get(valid, "unknown")))
    code = vesc.get("fault_code")
    rows.append(("VESC 고장", fault_text(code, vesc.get("fault")), fault_level(code)))
    age = vesc.get("telemetry_age_ms")
    rows.append(("VESC 상태값", f"{age:.0f} ms 전" if finite(age) else MISSING,
                 "warn" if finite(age) and age > TELEMETRY_WARN_S*1000 else "ok" if finite(age) else "unknown"))
    rows.append(("서보", _servo_text(vesc.get("servo_cmd"), vesc.get("servo_target")), "ok"))
    return rows


LEVEL_COLORS = {"ok": "#e2e8f0", "warn": "#fbbf24", "fail": "#f87171", "unknown": "#94a3b8"}
