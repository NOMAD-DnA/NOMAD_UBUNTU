"""Select the existing single Jetson cloud publisher and verify a fresh XYZ cloud."""
from __future__ import annotations
import json
from pathlib import Path
from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
SOURCE_NAMES = {"opencv": "OpenCV StereoSGBM", "device": "OAK 내부 StereoDepth"}
RESULT_MARKER = "NOMAD_CLOUD_SOURCE_RESULT="


def _failed(reason, source=None, rollback=False):
    return {"ok": False, "state": "UNCONFIRMED", "source": source,
            "reason": reason, "rollback_confirmed": rollback, "map_reset": False}


def switch_cloud_source(jetson_host, source, password=None, timeout_s=45):
    if source not in (*SOURCE_NAMES, "status"):
        return _failed("invalid_source")
    try:
        host = validate_host(jetson_host)
        if host.split("@", 1)[0] != "nvidia":
            raise ValueError("Wrong board account")
    except ValueError:
        return _failed("invalid_jetson_address")
    try:
        result = run_remote(host, BASE_DIR / "remote" / ("switch_cloud_" + source + ".sh"),
                            password, timeout_s=timeout_s)
    except Exception:
        return _failed("local_control_error")
    lines = [line[len(RESULT_MARKER):] for line in result.get("stdout", "").splitlines()
             if line.startswith(RESULT_MARKER)]
    if len(lines) != 1:
        safe = {"timeout", "authentication_failed", "host_key_check_failed", "ssh_unreachable"}
        return _failed(result.get("reason") if result.get("reason") in safe else "missing_or_ambiguous_cloud_result")
    try:
        value = json.loads(lines[0])
    except (ValueError, TypeError):
        return _failed("invalid_cloud_result")
    if not isinstance(value, dict):
        return _failed("invalid_cloud_result")
    actual = value.get("source")
    if actual is not None and actual not in SOURCE_NAMES:
        return _failed("invalid_cloud_source")
    if value.get("ok") is True:
        if (result.get("ok") is not True or value.get("state") != "SOURCE_CONFIRMED" or
                actual not in SOURCE_NAMES or (source != "status" and actual != source) or
                value.get("map_reset") is not False):
            return _failed("cloud_switch_unconfirmed")
        return {"ok": True, "state": "SOURCE_CONFIRMED", "source": actual,
                "reason": "ok", "rollback_confirmed": False, "map_reset": False}
    safe = {"source_unavailable", "new_source_no_fresh_cloud", "switch_service_failed",
            "control_error", "no_fresh_cloud", "switch_busy"}
    reason = value.get("reason")
    rollback = value.get("rollback_confirmed") is True and actual in SOURCE_NAMES
    return _failed(reason if reason in safe else "cloud_switch_failed", actual if rollback else None, rollback)
