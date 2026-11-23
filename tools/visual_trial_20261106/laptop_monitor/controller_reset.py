"""Clear a critical latch of the running Pi4 controller (its ~/reset service).

Only the shipped remote/reset_controller.sh is sent. It never commands motor, servo or arm.
The controller accepts RESET only when stopped and stationary for 0.5 s.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
RESULT_MARKER = "NOMAD_CONTROLLER_RESET_RESULT="
REASONS = frozenset({
    "reset_accepted", "reset_refused", "wrong_board", "controller_not_running",
    "environment_unavailable", "service_unavailable", "call_timeout",
    "timeout", "authentication_failed", "host_key_check_failed", "ssh_unreachable", "local_ssh_failed",
})
REASON_LABELS = {
    "reset_accepted": "제어기 복구 완료 · 잠금이 풀렸습니다. 출력 허용 상태에서 유효한 명령이 오면 다시 출발합니다.",
    "reset_refused": "제어기가 복구를 거부했습니다. 차가 0.5초 이상 멈춰 있고 모터 상태가 들어오는지 확인해 주세요.",
    "wrong_board": "Pi4 제어 작업공간을 찾지 못했습니다. Pi4 주소를 확인해 주세요.",
    "controller_not_running": "Pi4에서 실행 중인 제어기가 없습니다. 플래너를 다시 연결해 주세요.",
    "environment_unavailable": "Pi4 ROS 환경을 불러오지 못했습니다.",
    "service_unavailable": "제어기 복구 서비스를 찾지 못했습니다. 제어기 실행 상태를 확인해 주세요.",
    "call_timeout": "제어기 복구 요청 응답이 없습니다.",
    "timeout": "Pi4 응답 시간이 초과됐습니다.",
    "authentication_failed": "Pi4 SSH 인증에 실패했습니다.",
    "host_key_check_failed": "Pi4 호스트 키 확인에 실패했습니다.",
    "ssh_unreachable": "Pi4에 연결할 수 없습니다.",
    "local_ssh_failed": "이 PC에서 SSH를 실행하지 못했습니다.",
}


def _failed(reason, message=""):
    return dict(ok=False, reason=reason, message=message)


def reset_controller(pi4_host, password=None, timeout_s=40):
    """Request one fixed shipped action; never return raw remote output or secrets."""
    try:
        host = validate_host(pi4_host)
        if host.split("@", 1)[0] != "user" or isinstance(timeout_s, bool):
            raise ValueError("Invalid address/options")
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 60:
            raise ValueError("Invalid timeout")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("Invalid password")
    except (TypeError, ValueError):
        return _failed("invalid_pi4_address_or_options")
    try:
        result = run_remote(host, BASE_DIR / "remote/reset_controller.sh", password, timeout_s=timeout_s)
        if not isinstance(result, dict) or not isinstance(result.get("stdout", ""), str):
            return _failed("invalid_remote_result")
        stdout = result.get("stdout", "")
        if len(stdout) > 32768:
            return _failed("invalid_remote_result")
        markers = [line[len(RESULT_MARKER):] for line in stdout.splitlines() if line.startswith(RESULT_MARKER)]
        if len(markers) != 1:
            reason = result.get("reason")
            return _failed(reason if reason in REASONS else "missing_or_ambiguous_reset_result")
        value = json.loads(markers[0])
        if not isinstance(value, dict):
            return _failed("invalid_remote_result")
        reason = value.get("reason")
        message = value.get("message", "")
        message = message[:300] if isinstance(message, str) else ""
        if value.get("ok") is True:
            if result.get("ok") is not True or reason != "reset_accepted":
                return _failed("reset_unconfirmed")
            return dict(ok=True, reason=reason, message=message)
        return _failed(reason if reason in REASONS else "reset_rejected", message)
    except (ValueError, TypeError):
        return _failed("invalid_remote_result")
    except Exception:
        return _failed("local_control_error")


def describe(result):
    """One-line Korean status for the planning window."""
    reason = result.get("reason", "")
    text = REASON_LABELS.get(reason, "제어기 복구 실패 · " + str(reason))
    message = result.get("message")
    return text + (" · 제어기: " + message if message else "")
