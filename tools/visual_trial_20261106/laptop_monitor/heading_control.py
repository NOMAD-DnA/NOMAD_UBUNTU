"""One explicit Pi5 heading-reset request through the existing SSH transport."""
from __future__ import annotations

import json
import math
from pathlib import Path

from control import run_remote, validate_host


BASE_DIR = Path(__file__).resolve().parent
RESULT_MARKER = "NOMAD_HEADING_RESET_RESULT="
BACKEND_REASONS = frozenset({
    "VIO_INPUT_NOT_READY", "VIO_ONLY_REQUIRED", "OUTPUT_FRAME_INVALID",
    "WAITING_FOR_CONTINUOUS_VIO_ONLY", "HOLD_VEHICLE_STILL", "MAG_IMU_NOT_FRESH",
    "MAGNETOMETER_NOT_FRESH", "MAGNETIC_HEADING_UNAVAILABLE",
    "MAG_CALIBRATION_OR_EXTRINSICS_UNCONFIRMED", "invalid_mag_or_timestamp",
    "mag_calibration_or_extrinsics_unconfirmed",
    "mag_field_norm", "mag_field_norm_jump", "mag_tilt_orientation_unavailable",
    "mag_horizontal_field_too_small", "mag_heading_innovation", "reset_rejected",
})
REMOTE_REASONS = BACKEND_REASONS | frozenset({
    "wrong_board", "workspace_unavailable", "environment_unavailable", "profile_unavailable",
    "service_unavailable", "service_timeout", "service_call_failed", "invalid_service_response",
})
TRANSPORT_REASONS = frozenset({
    "timeout", "authentication_failed", "host_key_check_failed", "ssh_unreachable", "local_ssh_failed",
})

REASON_LABELS = {
    "VIO_INPUT_NOT_READY": "최신 VIO 위치를 기다립니다.",
    "VIO_ONLY_REQUIRED": "VIO 단독 모드에서만 초기화할 수 있습니다.",
    "OUTPUT_FRAME_INVALID": "위치 출력 좌표계 설정을 확인해야 합니다.",
    "WAITING_FOR_CONTINUOUS_VIO_ONLY": "VIO 단독 모드가 안정적으로 유지되기를 기다립니다.",
    "HOLD_VEHICLE_STILL": "차량을 멈춘 상태로 유지해 주세요.",
    "MAG_IMU_NOT_FRESH": "최신 지자계·IMU 입력을 기다립니다.",
    "MAGNETOMETER_NOT_FRESH": "최신 지자계 입력을 기다립니다.",
    "MAGNETIC_HEADING_UNAVAILABLE": "지자계 방향을 계산할 수 없습니다.",
    "MAG_CALIBRATION_OR_EXTRINSICS_UNCONFIRMED": "지자계 또는 센서 장착 보정이 확인되지 않았습니다.",
    "mag_calibration_or_extrinsics_unconfirmed": "지자계 또는 센서 장착 보정이 확인되지 않았습니다.",
    "invalid_mag_or_timestamp": "지자계 값 또는 측정 시각이 유효하지 않습니다.",
    "mag_field_norm": "자기장 세기가 유효 범위를 벗어났습니다.",
    "mag_field_norm_jump": "자기장 세기가 급격히 변했습니다.",
    "mag_tilt_orientation_unavailable": "기울기 보정용 자세를 기다립니다.",
    "mag_horizontal_field_too_small": "수평 자기장 성분이 부족합니다.",
    "mag_heading_innovation": "지자계 방향 변화가 허용 범위를 벗어났습니다.",
    "service_unavailable": "Pi5 방향 초기화 서비스가 보이지 않습니다.",
    "service_timeout": "Pi5 방향 초기화 응답 시간이 초과됐습니다.",
    "timeout": "Pi5 접속 또는 응답 시간이 초과됐습니다.",
    "reset_rejected": "Pi5에서 방향 초기화를 허용하지 않았습니다.",
}


def _failed(reason):
    return {"ok": False, "state": "HEADING_RESET_FAILED", "reason": reason}


def reset_heading(pi_host, password=None, timeout_s=14):
    """Return bounded reason codes only; never expose remote output or secrets."""
    try:
        host = validate_host(pi_host)
        if host.split("@", 1)[0] != "user":
            raise ValueError("Wrong board account")
        if isinstance(timeout_s, bool):
            raise ValueError("Invalid timeout")
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 60:
            raise ValueError("Invalid timeout")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("Invalid password")
    except (ValueError, TypeError):
        return _failed("invalid_pi5_address_or_options")
    try:
        result = run_remote(host, BASE_DIR / "remote" / "reset_heading_from_magnetometer.sh",
                            password, timeout_s=timeout_s)
        if not isinstance(result, dict):
            return _failed("invalid_remote_result")
        stdout = result.get("stdout", "")
        if not isinstance(stdout, str) or len(stdout) > 32768:
            return _failed("invalid_remote_result")
        markers = [line[len(RESULT_MARKER):] for line in stdout.splitlines() if line.startswith(RESULT_MARKER)]
        if len(markers) != 1:
            reason = result.get("reason")
            return _failed(reason if reason in TRANSPORT_REASONS else "missing_or_ambiguous_heading_result")
        value = json.loads(markers[0])
        if not isinstance(value, dict):
            return _failed("invalid_remote_result")
        if value.get("ok") is True:
            if (result.get("ok") is not True or value.get("state") != "HEADING_RESET" or
                    value.get("reason") != "ok"):
                return _failed("heading_reset_unconfirmed")
            return {"ok": True, "state": "HEADING_RESET", "reason": "ok"}
        reason = value.get("reason")
        return _failed(reason if reason in REMOTE_REASONS else "reset_rejected")
    except (ValueError, TypeError):
        return _failed("invalid_remote_result")
    except Exception:
        # Exception text may contain SSH/password values. Do not propagate it.
        return _failed("local_control_error")
