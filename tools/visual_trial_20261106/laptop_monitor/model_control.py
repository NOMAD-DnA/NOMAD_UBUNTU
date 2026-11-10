"""Laptop-only controls for the separately installed Jetson semantic model switch.

Only the semantic producer is switched by the board helper. Camera, VIO, map,
and laptop collectors are deliberately outside this API.
"""
from __future__ import annotations

import json
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
MODEL_NAMES = {"b2": "B2 · 기존", "fcn": "FCN · 실내"}
RESULT_MARKER = "NOMAD_MODEL_SWITCH_RESULT="


def _failed(reason):
    # Do not return raw SSH stdout/stderr or credentials to UI state/logs.
    return {"ok": False, "state": "UNCONFIRMED", "model": None, "reason": reason}


def switch_model(jetson_host, model, password=None, timeout_s=90):
    """Switch one existing Jetson semantic producer and verify its explicit result."""
    if model not in MODEL_NAMES:
        return _failed("invalid_model")
    try:
        jetson_host = validate_host(jetson_host)
        if jetson_host.split("@", 1)[0] != "nvidia":
            raise ValueError("Wrong board account")
    except ValueError:
        return _failed("invalid_jetson_address")
    try:
        value = run_remote(jetson_host, BASE_DIR / "remote" / ("switch_model_" + model + ".sh"),
                           password, timeout_s=timeout_s)
    except Exception:
        return _failed("local_control_error")
    if not value.get("ok"):
        reason = value.get("reason")
        safe_reasons = {"timeout", "authentication_failed", "host_key_check_failed", "ssh_unreachable",
                        "remote_script_failed", "local_ssh_failed", "invalid_host_script_or_options"}
        return _failed(reason if reason in safe_reasons else "model_switch_failed")
    markers = [line[len(RESULT_MARKER):] for line in value.get("stdout", "").splitlines()
               if line.startswith(RESULT_MARKER)]
    if len(markers) != 1:
        return _failed("missing_or_ambiguous_model_result")
    try:
        remote = json.loads(markers[0])
    except (ValueError, TypeError):
        return _failed("invalid_model_result")
    if not isinstance(remote, dict) or remote.get("state") != "MODEL_SWITCHED" or remote.get("model") != model:
        return _failed("model_switch_unconfirmed")
    return {"ok": True, "state": "MODEL_SWITCHED", "model": model, "reason": "ok"}
