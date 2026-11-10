"""LPP candidate transitions from validated telemetry; no control or board IO."""
from datetime import datetime
import json
import uuid


class LppDiagnostics:
    def __init__(self):
        self.session = "lpp-ui-" + uuid.uuid4().hex
        self.sequence = 0
        self.previous = None

    def update(self, event):
        # executable is a planner result, independent of arming/output permission.
        # A goal can be waiting for inputs before its first global path exists.
        active = event.get("goal") is not None or bool(event.get("path"))
        if (event.get("executable") is not False or not active
                or event.get("state") in ("ARRIVED", "DISCONNECTED", "IDLE")
                or event.get("lpp_state") == "ARRIVED"):
            self.previous = None
            return None
        reason = event.get("lpp_reason") or event.get("reason") or "상세 사유 미제공"
        key = (tuple(event.get("goal") or ()), event.get("lpp_state"), reason)
        if key == self.previous:
            return None
        self.previous = key
        self.sequence += 1
        stamp = event["t"]
        try:
            clock = datetime.fromtimestamp(stamp).astimezone().isoformat(timespec="milliseconds")
        except (ValueError, OverflowError, OSError):
            clock = str(stamp)
        # JSON quoting preserves reason text while preventing multiline log injection.
        detail = json.dumps(reason, ensure_ascii=False)
        text = (f"[LPP PAUSE · GUI 수신] {clock} reason={detail}"
                f" | GPP={event.get('gpp_state', '미제공')}"
                f" path_valid={event.get('path_valid')}"
                f" | LPP={event.get('lpp_state', '미제공')} executable=false"
                f" | stamp_ns={event.get('stamp_ns', 0)}")
        return dict(kind="log", board="pi4", source="planning", t=stamp,
                    session=self.session, seq=self.sequence, text=text)
