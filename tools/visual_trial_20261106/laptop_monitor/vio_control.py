"""Restart only the current tracked Basalt instance; keep live fusion intact."""
from __future__ import annotations

import json
import math
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
RESULT_MARKER = "NOMAD_VIO_RESTART_RESULT="
REASONS = frozenset({
    "wrong_board", "workspace_unavailable", "environment_unavailable", "profile_unavailable",
    "NO_TRIAL_RUN", "current_run_unknown", "current_run_invalid", "restart_busy", "trial_stop_busy",
    "vio_identity_mismatch", "fusion_identity_mismatch", "untracked_vio_running",
    "standalone_vio_session_active", "vio_stop_incomplete", "epoch_publish_failed",
    "vio_start_failed", "fusion_exited_during_restart", "fusion_preserved", "new_vio_origin",
    "timeout", "authentication_failed", "host_key_check_failed", "ssh_unreachable", "local_ssh_failed",
})
REASON_LABELS = {
    "fusion_preserved": "VIO 재실행 · 살아 있는 융합기의 마지막 위치에 다시 연결합니다.",
    "new_vio_origin": "VIO 재실행 · 융합기가 없어 VIO는 새 원점에서 시작합니다.",
    "current_run_unknown": "현재 시험 실행 폴더를 찾을 수 없습니다.",
    "NO_TRIAL_RUN": "실행 중인 시험이 없습니다. 시험을 시작한 뒤 VIO를 재실행해 주세요.",
    "current_run_invalid": "현재 시험 실행 폴더를 확인할 수 없습니다.",
    "restart_busy": "이미 VIO를 재실행 중입니다.",
    "trial_stop_busy": "시험 종료 작업이 진행 중입니다.",
    "untracked_vio_running": "현재 시험에서 관리하지 않는 VIO가 실행 중입니다.",
    "standalone_vio_session_active": "별도 VIO 세션이 실행 중입니다. 해당 세션에서 재실행해 주세요.",
    "vio_stop_incomplete": "이전 VIO 종료를 확인하지 못했습니다.",
    "epoch_publish_failed": "융합기에 VIO 재시작 정보를 전달하지 못했습니다. VIO는 종료된 상태입니다.",
    "vio_start_failed": "VIO 시작에 실패했습니다. 센서·융합기는 변경하지 않았습니다.",
    "fusion_exited_during_restart": "VIO는 시작됐지만 기존 융합기가 종료돼 위치 이어붙이기를 확인할 수 없습니다.",
}


def _failed(reason):
    return dict(ok=False, state="VIO_RESTART_FAILED", reason=reason, fusion_preserved=False)


def restart_vio(pi_host, password=None, timeout_s=24):
    """Request one fixed shipped action; never return remote output or secrets."""
    try:
        host = validate_host(pi_host)
        if host.split("@", 1)[0] != "user" or isinstance(timeout_s, bool):
            raise ValueError("Invalid address/options")
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or not 0 < timeout_s <= 60:
            raise ValueError("Invalid timeout")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("Invalid password")
    except (TypeError, ValueError):
        return _failed("invalid_pi5_address_or_options")
    try:
        result = run_remote(host, BASE_DIR / "remote/restart_vio.sh", password, timeout_s=timeout_s)
        if not isinstance(result, dict) or not isinstance(result.get("stdout", ""), str):
            return _failed("invalid_remote_result")
        stdout = result.get("stdout", "")
        if len(stdout) > 32768:
            return _failed("invalid_remote_result")
        markers = [line[len(RESULT_MARKER):] for line in stdout.splitlines() if line.startswith(RESULT_MARKER)]
        if len(markers) != 1:
            reason = result.get("reason")
            return _failed(reason if reason in REASONS else "missing_or_ambiguous_vio_result")
        value = json.loads(markers[0])
        if not isinstance(value, dict):
            return _failed("invalid_remote_result")
        reason = value.get("reason")
        if value.get("ok") is True:
            preserved = value.get("fusion_preserved")
            if (result.get("ok") is not True or value.get("state") != "VIO_RESTARTED" or
                    not isinstance(preserved, bool) or
                    reason != ("fusion_preserved" if preserved else "new_vio_origin")):
                return _failed("vio_restart_unconfirmed")
            return dict(ok=True, state="VIO_RESTARTED", reason=reason, fusion_preserved=preserved)
        return _failed(reason if reason in REASONS else "vio_restart_rejected")
    except (ValueError, TypeError):
        return _failed("invalid_remote_result")
    except Exception:
        return _failed("local_control_error")
