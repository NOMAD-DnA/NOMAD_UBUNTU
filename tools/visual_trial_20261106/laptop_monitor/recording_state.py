"""Versioned laptop display state and position-only baseline validation."""
from dataclasses import asdict
import hashlib
import math
from pathlib import Path

EVENT_SCHEMA_VERSION = 2
DISPLAY_SCHEMA_VERSION = 1
PROVENANCE_FILES = ("monitor.py", "session_ui.py", "session_recording.py", "recording_state.py",
                    "planning_footprint.py", "developer_planning.py", "planning_client.py",
                    "module_logs.py", "developer_control.py", "localization_state.py",
                    "developer_localization.py", "developer_traversability.py",
                    "developer_perception.py", "heading_indicator.py", "relay.py")


def source_digests():
    root = Path(__file__).parent
    return {name: hashlib.sha256((root/name).read_bytes()).hexdigest() for name in PROVENANCE_FILES}


def display_metadata(footprint):
    return {"event_schema_version": EVENT_SCHEMA_VERSION,
            "display_settings": {"schema_version": DISPLAY_SCHEMA_VERSION,
                                 "footprint": asdict(footprint)},
            "gui_source_sha256": source_digests()}


def replay_display(metadata):
    from planning_footprint import footprint_from_dict
    version = metadata.get("event_schema_version", 1)
    if isinstance(version, bool) or not isinstance(version, int) or not 1 <= version <= EVENT_SCHEMA_VERSION:
        raise ValueError("지원하지 않는 기록 이벤트 버전")
    settings = metadata.get("display_settings")
    if settings is None:
        return None, "기록 당시 윤곽 미저장 · 윤곽/겹침 셀 표시 안 함"
    if not isinstance(settings, dict) or settings.get("schema_version") != DISPLAY_SCHEMA_VERSION:
        raise ValueError("지원하지 않는 기록 표시 설정")
    note = "기록 당시 윤곽 설정 사용"
    saved = metadata.get("gui_source_sha256")
    if saved is None:
        note += " · 당시 GUI 코드 버전 미저장"
    elif not isinstance(saved, dict) or any(not isinstance(k, str) or not isinstance(v, str)
                                          or len(v) != 64 or any(c not in "0123456789abcdef" for c in v)
                                          for k, v in saved.items()):
        raise ValueError("잘못된 GUI 코드 버전 정보")
    else:
        current = source_digests()
        if saved != current:
            note += " · 기록 당시와 표시 코드 버전 다름"
    return footprint_from_dict(settings.get("footprint")), note


def comparison_baseline(event, limit=20000):
    """Validate an initial position history without inventing GNSS attitude."""
    source = event.get("history_source")
    if source not in ("gnss", "vio_compare"):
        raise ValueError("Invalid comparison baseline source")
    values, age = event.get("positions"), event.get("receive_age_s")
    if (not isinstance(values, list) or len(values) > limit or isinstance(age, bool)
            or not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0):
        raise ValueError("Invalid comparison baseline")
    result, last = [], 0.
    for value in values:
        if not isinstance(value, dict):
            raise ValueError("Invalid comparison position")
        row = {key: value.get(key) for key in ("t", "x", "y", "z")}
        if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
                   for v in row.values()) or row["t"] <= last:
            raise ValueError("Invalid comparison position time/coordinates")
        frame, epoch = value.get("frame_id"), value.get("viewer_epoch")
        if (not isinstance(frame, str) or not 0 < len(frame.strip()) <= 128
                or isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0):
            raise ValueError("Invalid comparison position frame/epoch")
        last = row["t"]
        result.append(dict(row, kind="pose", source=source, frame_id=frame,
                           viewer_epoch=epoch, position_only=True))
    return source, result, float(age)
