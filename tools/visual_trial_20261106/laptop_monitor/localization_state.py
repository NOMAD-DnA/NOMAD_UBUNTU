"""Pure display helpers for existing localization packets; no ROS or I/O."""
from __future__ import annotations

import math
from collections.abc import Mapping


FRESHNESS_S = 2.0
RESET_FRESHNESS_S = 1.0
COLORS = {"ready": "#34d399", "warning": "#fbbf24", "stale": "#fb7185", "waiting": "#94a3b8"}
MODE_LABELS = {
    "FUSED": "VIO·GNSS 융합",
    "VIO_ONLY": "VIO 단독",
    "GNSS_HEADING": "GNSS 위치 + IMU·지자계 방향",
    "GNSS_ONLY": "GNSS 단독 · 방향 미확정",
    "NO_POSITION": "위치 입력 끊김",
    "WAITING": "위치 입력 대기",
}


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return result if math.isfinite(result) else None


def _flag(value):
    """DiagnosticStatus booleans are strings. Unknown is never true."""
    if isinstance(value, bool):
        return value
    if isinstance(value, str) and value.strip().lower() in ("true", "false"):
        return value.strip().lower() == "true"
    return None


def flatten(event):
    if not isinstance(event, Mapping):
        return {}
    result = dict(event)
    for name in ("data", "metrics", "fields", "values"):
        nested = event.get(name)
        if isinstance(nested, Mapping):
            result.update(nested)
        elif name == "values" and isinstance(nested, (list, tuple)):
            result.update({item["key"]: item.get("value") for item in nested
                           if isinstance(item, Mapping) and isinstance(item.get("key"), str)})
    return result


def _age(snapshot, kind, source, data=None):
    receipts = snapshot.get("received", {})
    receipt = receipts.get(f"{kind}:{source}") if isinstance(receipts, Mapping) else None
    if receipt is None and isinstance(receipts, Mapping) and isinstance(receipts.get(kind), Mapping):
        receipt = receipts[kind].get(source)
    now, receipt = _number(snapshot.get("now")), _number(receipt)
    if now is None or receipt is None or receipt > now:
        return None
    elapsed = now - receipt
    if data and "measurement_age_s" in data:
        measured = _number(data.get("measurement_age_s"))
        if measured is None or measured < -0.02:
            return None
        elapsed += max(0., measured)
    return elapsed


def _status_values(status):
    result = {}
    if not isinstance(status, Mapping):
        return result
    for item in str(status.get("detail", "")).split(";"):
        if "=" in item:
            key, value = item.split("=", 1)
            result[key.strip()] = value.strip()
    return result


def localization_summary(snapshot):
    """Describe measured mode, separately from public-output health.

    A fresh ModuleStatus may provide display fallback, but it cannot authorize
    a heading reset. FUSED only describes usable inputs, never solver success.
    """
    snapshot = snapshot if isinstance(snapshot, Mapping) else {}
    telemetry = snapshot.get("telemetry", {})
    statuses = snapshot.get("statuses", {})
    event = telemetry.get("fusion", {}) if isinstance(telemetry, Mapping) else {}
    data = flatten(event)
    status = statuses.get("localization", {}) if isinstance(statuses, Mapping) else {}
    status_age = _age(snapshot, "status", "localization")
    status_fresh = bool(status) and status_age is not None and status_age <= FRESHNESS_S
    diag_age = _age(snapshot, "telemetry", "fusion", data)
    diag_received = bool(data) and _flag(data.get("received")) is not False
    diag_fresh = diag_received and diag_age is not None and diag_age <= FRESHNESS_S
    diagnostic_mode = str(data.get("mode", "")).strip().upper()
    via = "fusion" if diag_fresh and diagnostic_mode in MODE_LABELS else "none"
    if via == "none" and status_fresh:
        fallback = _status_values(status)
        if str(fallback.get("mode", "")).upper() in MODE_LABELS:
            data = fallback
            via = "status"
    mode = str(data.get("mode", "")).strip().upper()
    if via == "none":
        mode = "STALE" if diag_received or status else "WAITING"
    elif mode not in MODE_LABELS:
        mode = "WAITING"
    position = _flag(data.get("position_valid")) if via != "none" else None
    orientation = _flag(data.get("orientation_valid")) if via != "none" else None
    mode_has_vio = mode in ("FUSED", "VIO_ONLY")
    mode_has_gnss = mode in ("FUSED", "GNSS_HEADING", "GNSS_ONLY")
    source = str(data.get("source") or {
        "FUSED": "VIO+GNSS", "VIO_ONLY": "VIO", "GNSS_HEADING": "GNSS+IMU/MAG",
        "GNSS_ONLY": "GNSS", "NO_POSITION": "NONE", "WAITING": "NONE"}.get(mode, "NONE"))[:120]
    label = MODE_LABELS.get(mode, "위치 상태 오래됨")
    color = COLORS["ready"] if mode in ("FUSED", "VIO_ONLY", "GNSS_HEADING") else COLORS["warning"]
    if mode == "WAITING": color = COLORS["waiting"]
    if mode == "STALE": color = COLORS["stale"]
    health = str(status.get("state", "UNKNOWN")) if isinstance(status, Mapping) else "UNKNOWN"
    if position is False and mode not in ("NO_POSITION", "WAITING", "STALE"):
        label += " · 위치 무효"
        color = COLORS["stale"]
    if status_fresh and health in ("WAITING", "FAULT", "STOPPED"):
        color = COLORS["stale"] if health == "FAULT" else COLORS["warning"]
    reason = str(data.get("reason", ""))[:300] if via != "none" else "유효한 최신 상태를 기다립니다."
    detail = reason
    if mode == "FUSED":
        detail = "VIO·GNSS 입력 유효 · 최적화 성공 여부는 별도 표시" + (" · " + reason if reason else "")
    if via == "status": detail = "공용 상태 기준 · " + detail
    if bool(status) and not status_fresh and mode != "STALE": detail += " · 후단 출력 상태 오래됨"
    if mode == "GNSS_ONLY" or orientation is False:
        orientation_text = "방향 미확정 · 마지막 방향 유지"
    elif via == "none": orientation_text = "방향 상태 미확인"
    elif _flag(data.get("global_heading_valid")) is True: orientation_text = "전역 방향 유효"
    elif _flag(data.get("heading_provisional")) is True: orientation_text = "임시 지자계 초기 방향"
    elif orientation is True: orientation_text = "로컬 방향 유효 · 전역 방위 미확정"
    else: orientation_text = "방향 상태 미확인"
    can_reset, reset_reason = False, "Pi5 방향 초기화 서비스 업데이트 필요"
    explicit = _flag(flatten(event).get("heading_reset_available"))
    if snapshot.get("replay") is True:
        reset_reason = "기록 재생 중에는 방향을 초기화할 수 없습니다."
    elif not diag_fresh or diag_age is None or diag_age > RESET_FRESHNESS_S:
        reset_reason = "최신 융합 상태를 기다립니다."
    elif diagnostic_mode != "VIO_ONLY":
        reset_reason = "VIO 단독 모드에서만 방향을 초기화할 수 있습니다."
    elif explicit is not None:
        can_reset = explicit is True
        reset_reason = ("현재 위치를 유지하며 지자계로 방향 초기화" if can_reset else
                        str(flatten(event).get("heading_reset_reason") or "지자계 입력 또는 초기화 조건을 기다립니다.")[:300])
    state = ("STALE" if mode == "STALE" else "WAITING" if mode == "WAITING" else
             "INVALID" if position is False or orientation is False or mode == "NO_POSITION" or
             (status_fresh and health in ("FAULT", "STOPPED", "WAITING")) else "READY")
    return dict(mode=mode, state=state, label=label, detail=detail, color=color, source=source,
                vio_text=("유효" if mode_has_vio else ("입력 끊김" if mode in ("GNSS_ONLY", "GNSS_HEADING", "NO_POSITION") else "입력 대기")),
                gnss_text=("융합에 사용" if mode_has_gnss else "융합에 미사용"),
                orientation_text=orientation_text, can_reset_heading=can_reset, reset_reason=reset_reason,
                position_valid=position, orientation_valid=orientation, status_state=health,
                status_fresh=status_fresh, diagnostics_fresh=diag_fresh, diagnostic_age_s=diag_age,
                mode_basis=via, heading_provisional=_flag(data.get("heading_provisional")) is True)


def comparison_samples(fusion_event):
    """Return only explicit backend-aligned position samples; invent no pose.

    The backend supplies a common viewer frame/epoch. The samples deliberately
    have no quaternion: a GNSS position is not an orientation measurement.
    """
    data = flatten(fusion_event)
    frame = data.get("viewer_frame")
    epoch = _number(data.get("viewer_epoch"))
    if (not isinstance(frame, str) or not frame.strip() or len(frame) > 128 or
            epoch is None or epoch < 0 or epoch != int(epoch)):
        return []
    samples = []
    for name, source in (("gnss", "gnss"), ("vio", "vio_compare")):
        if _flag(data.get(f"viewer_{name}_valid")) is not True:
            continue
        values = [_number(data.get(f"viewer_{name}_{key}")) for key in ("t", "x", "y", "z")]
        if any(value is None for value in values) or values[0] <= 0:
            continue
        samples.append(dict(kind="pose", source=source, frame_id=frame.strip(), viewer_epoch=int(epoch),
                            position_only=True, **dict(zip(("t", "x", "y", "z"), values))))
    return samples
