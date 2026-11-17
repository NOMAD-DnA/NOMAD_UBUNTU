"""2026-11-16 Pi4 제어기의 모터·서보(VESC) 값을 읽기 전용 표시 줄로 바꾼다 (Qt 없음, 시험용 순수 함수).

입력은 두 가지다.
- 키보드 조종: Pi4 제어기 HTTP `/api/status` JSON (keyboard_drive가 이미 0.1초마다 받는 값).
- 자율주행: 제어기 로그의 `control_trace`·`vesc_trace` 줄 (developer_control이 이미 읽는 로그).
제어기 4e64c7e 이전 배포본에는 새 키·`vesc_trace`가 없다. 그때는 값을 꾸미지 않고 '—'로 둔다.
서보 값은 모두 명령값이다(서보 피드백·실측 조향각은 없음). 입력 전류는 VESC 추정값이다.

2026-11-17 (제어기 f6f4946 이후):
- 정지 유지 시간·모터 발열 추정·오래 유지 경고를 '정지 출력' 줄에 표시한다. 전류는 제어기가 바꾸지 않고, 모터 온도는 재지 않는다.
- '제어 주기·LPP' 줄을 추가했다.
  - 키보드 조종: HTTP `link_*`·`control_tick_max_ms` 키를 쓴다.
  - 자율주행: 로그의 `link_trace` 줄을 쓴다. 제어기는 이 줄을 문제가 있던 1초(만료, 명령 공백, 제어 주기 지연)에만 찍고, 그 밖에는 10초마다 찍는다.
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
# 정지 유지 경고: 제어기 stop_output.hpp와 같은 기준 (2026-11-17)
HOLD_LONG_A = 25.0         # 이 전류 이상을
HOLD_LONG_S = 10.0         # 이만큼 넘게 유지하면 경고
MOTOR_R_OHM = 0.0112       # VESC 모터 설정 R. 유지 중 모터 구리 손실 ≈ 1.5·R·I² (30 A → 약 15 W)
# 통신·제어 주기 (제어기 link_trace 기준과 같음)
LINK_GAP_WARN_MS = 200.0   # 명령이 오는 중 이보다 긴 공백
TICK_WARN_MS = 40.0        # 20 ms 제어 주기가 이보다 늦음 (CPU 부하)
LINK_STALE_S = 12.0        # link_trace는 평소 10초마다 오므로 이보다 오래되면 오래된 기록

_ANSI = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
_KEY = re.compile(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=")
_ROS_STAMP = re.compile(r"\[(?:DEBUG|INFO|WARN|ERROR|FATAL)\]\s*\[([0-9]+(?:\.[0-9]+)?)\]")
VESC_TRACE_NUMERIC = ("voltage_v", "input_current_a", "power_w", "iq_a", "duty", "fault_code",
                      "servo_cmd", "servo_target", "telemetry_age_ms", "hold_s")  # hold_s: 2026-11-17 이후
LINK_TRACE_NUMERIC = ("n", "gap_ms", "age_min_ms", "age_max_ms", "expired", "tick_max_ms")
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


def hold_heat_w(current_a):
    """정지 유지 중 모터 구리 손실 추정 W (모터 온도 측정값 아님)."""
    return 1.5 * MOTOR_R_OHM * current_a * current_a if finite(current_a) else None


def _stop_row(stop, amps, basis, hold_s=None, heat_w=None, long_hold=None):
    """'정지 출력' 줄. 핸드브레이크 유지 중이면 유지 시간·발열 추정을 붙이고, 오래 유지하면 주의."""
    text = f"{stop or MISSING} {_num(amps, 1)} A · 근거 {basis or MISSING}"
    holding = stop == "hold"
    if not holding:
        return ("정지 출력", text, "ok")
    if finite(hold_s):
        text += f" · {hold_s:.0f}초째"
    if finite(heat_w) and heat_w > 0:
        text += f" · 모터 발열 약 {heat_w:.0f} W(추정)"
    if not isinstance(long_hold, bool):
        long_hold = finite(amps) and finite(hold_s) and amps >= HOLD_LONG_A and hold_s >= HOLD_LONG_S
    if long_hold:
        text += " · 오래 유지 중(모터 온도 미측정)"
    return ("정지 출력", text, "warn" if long_hold else "ok")


def _link_text(tick, commands, gap, age_min, age_max, expired):
    """'제어 주기·LPP' 값과 문제 여부. 명령이 없으면 공백은 문제로 보지 않는다(플래너가 꺼져 있을 수 있음)."""
    parts = [f"제어 주기 최대 {_num(tick, 0)} ms"]
    trouble = finite(tick) and tick > TICK_WARN_MS
    if finite(commands) and commands > 0:
        parts.append(f"LPP {commands:.0f}개/초 · 최대 공백 {_num(gap, 0)} ms · 지연 {_num(age_min, 0)}~{_num(age_max, 0)} ms")
        trouble = trouble or (finite(gap) and gap > LINK_GAP_WARN_MS)
    elif finite(gap) and gap > 0:
        parts.append(f"LPP 명령 없음(마지막 명령 {gap/1000:.0f}초 전)")
    else:
        parts.append("LPP 명령 없음")
    if finite(expired):
        parts.append(f"만료 {expired:.0f}")
        trouble = trouble or expired > 0
    return " · ".join(parts), trouble


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
    rows.append(_stop_row(get("stop_output"), get("stop_output_a"), get("stop_hold_basis"), get("stop_hold_s"),
                          get("stop_hold_heat_w"), get("stop_hold_long")))
    code, latched = get("fault_code"), get("vesc_fault")
    rows.append(("VESC 고장", fault_text(code, get("fault_name")) + (" · 제어기 잠금" if latched is True else ""),
                 fault_level(code, latched)))
    age, ready = get("telemetry_age_s"), get("driver_ready")
    age_text = f"{age*1000:.0f} ms 전" if finite(age) else MISSING
    ready_text = {True: "드라이버 준비", False: "드라이버 미준비"}.get(ready, "드라이버 " + MISSING)
    late = (finite(age) and age > TELEMETRY_WARN_S) or ready is False or not fresh
    rows.append(("VESC 상태값", f"{age_text} · {ready_text}" + ("" if fresh else " · 상태 끊김"), "warn" if late else "ok"))
    rows.append(("서보", _servo_text(get("commanded_servo_position"), get("requested_servo_position")), "ok"))
    tick = get("control_tick_max_ms")
    if finite(tick):
        text, trouble = _link_text(tick, get("link_commands"), get("link_cmd_max_gap_ms"), get("link_cmd_age_min_ms"),
                                   get("link_cmd_age_max_ms"), get("link_expiries"))
        rows.append(("제어 주기·LPP", text + " (최근 1초)", "warn" if trouble else "ok"))
    else:
        rows.append(("제어 주기·LPP", MISSING + " (2026-11-17 이후 제어기)", "unknown"))
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


def link_trace_from_line(line):
    """제어기 로그 한 줄의 `link_trace` → dict (2026-11-17 이후 제어기). 'na'(명령·시각 없음)는 None."""
    if not isinstance(line, str):
        return None
    line = _ANSI.sub("", line)
    marker = line.find("link_trace ")
    if marker < 0 or not line.lstrip().startswith("[controller]"):
        return None
    payload = line[marker + len("link_trace "):]
    values = dict(re.findall(r"(?:^|\s)([A-Za-z_][A-Za-z0-9_]*)=(\S*)", payload))
    trace = {key: _number(values.get(key)) for key in LINK_TRACE_NUMERIC}
    stamp = _ROS_STAMP.search(line[:marker])
    trace["stamp"] = _number(stamp.group(1)) if stamp else None
    return trace


def link_row(link, age_s=None):
    """자율주행 '제어 주기·LPP' 줄. 문제 있던 1초의 줄이면 다음 평소 줄(최대 10초 뒤)까지 주의로 둔다."""
    title = "제어 주기·LPP"
    if not isinstance(link, dict):
        return (title, "link_trace 없음 (2026-11-17 이전 제어기, 또는 아직 기록 전)", "unknown")
    if finite(age_s) and age_s > LINK_STALE_S:
        return (title, f"마지막 link_trace {age_s:.0f}초 전 · 오래된 기록", "unknown")
    text, trouble = _link_text(link.get("tick_max_ms"), link.get("n"), link.get("gap_ms"), link.get("age_min_ms"),
                               link.get("age_max_ms"), link.get("expired"))
    text += " · 문제 있던 1초" if trouble else " · 평소(10초마다 기록)"
    if finite(age_s):
        text += f" · {age_s:.0f}초 전"
    return (title, text, "warn" if trouble else "ok")


def trace_rows(control, vesc, link=None, link_age=None):
    """자율주행 로그(control_trace의 추가 값 + vesc_trace + link_trace) → (제목, 값, 수준) 목록."""
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
    stop, amps = control.get("stop"), control.get("stop_a")
    rows.append(_stop_row(stop, amps, vesc.get("hold_basis"), vesc.get("hold_s"),
                          hold_heat_w(amps) if stop == "hold" else None))
    valid = control.get("erpm_valid")
    rows.append(("측정 ERPM 유효", {"1": "예", "0": "아니오 · 핸드브레이크 중(ERPM 0은 측정값 아님)"}.get(valid, MISSING),
                 {"1": "ok", "0": "unknown"}.get(valid, "unknown")))
    code = vesc.get("fault_code")
    rows.append(("VESC 고장", fault_text(code, vesc.get("fault")), fault_level(code)))
    age = vesc.get("telemetry_age_ms")
    rows.append(("VESC 상태값", f"{age:.0f} ms 전" if finite(age) else MISSING,
                 "warn" if finite(age) and age > TELEMETRY_WARN_S*1000 else "ok" if finite(age) else "unknown"))
    rows.append(("서보", _servo_text(vesc.get("servo_cmd"), vesc.get("servo_target")), "ok"))
    rows.append(link_row(link, link_age))
    return rows


LEVEL_COLORS = {"ok": "#e2e8f0", "warn": "#fbbf24", "fail": "#f87171", "unknown": "#94a3b8"}  # 어두운 창(제어 개발 창)
# 2026-11-17: 키보드 조종 창은 런처의 밝은 테마(흰 묶음 바탕)를 물려받는다. 위의 #e2e8f0는 거기서 거의 안 보였다.
LEVEL_COLORS_LIGHT = {"ok": "#17263b", "warn": "#b45309", "fail": "#b91c1c", "unknown": "#64748b"}
WIDE_ROWS = ("정지 출력", "제어 주기·LPP")  # 값이 길어 한 줄 전체를 쓰는 줄 (키보드 조종 창)
