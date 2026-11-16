"""2026-11-15 GUI '센서 녹화': Jetson SSD에 센서 원본(+선택 시 Pi5 GPS) rosbag을 녹화한다.

- 시작: (GPS 선택 시) Pi5 GPS 수신기 켜기 → Jetson remote/record_start_jetson.sh
  (Pi5 기준 시계 동기 확인, 센서가 꺼져 있으면 /mnt/ssd/ws/recording/start_sensors.sh로 켜기, ros2 bag record).
- 정지: Jetson remote/record_stop_jetson.sh (GUI 녹화가 켠 센서만 끔) → GUI가 켠 Pi5 GPS 수신기만 끔.
- 형식은 Jetson /mnt/ssd/README.md의 센서 녹화와 같다: /mnt/ssd/bags/<날짜_시각>_<장면>/bag/.
- 보드에 있는 기존 녹화 스크립트는 바꾸지 않는다(GUI가 보내는 스크립트만 실행).
"""
from __future__ import annotations

import json
import re
import tempfile
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
MARKER = "NOMAD_RECORD_RESULT="
SCENE_RE = re.compile(r"^[0-9A-Za-z가-힣_-]{1,40}$")
LABELS = {
    "recording": "녹화 중",
    "wrong_board": "보드 계정·녹화 폴더를 찾지 못했습니다(Jetson nvidia@ · /mnt/ssd/ws/recording).",
    "invalid_scene": "장면 이름은 한글·영문·숫자·_- 40자 이내로 적어 주세요.",
    "no_ssd": "Jetson SSD(/mnt/ssd)가 없습니다. 녹화하지 않습니다.",
    "already_recording": "이미 녹화 중입니다.",
    "ssd_full": "SSD 남은 공간이 30 GB보다 적습니다.",
    "clock_year": "Jetson 시계가 2026년이 아닙니다. 보드 시계부터 맞춰 주세요.",
    "clock_not_synced": "Jetson 시계가 Pi5에 맞춰져 있지 않습니다. Pi5·유선 연결 후 잠시 뒤 다시 누르세요.",
    "clock_offset": "Jetson과 Pi5 시계 차가 5 ms를 넘습니다. 잠시 뒤 다시 누르세요.",
    "rosbag_missing": "Jetson에 녹화 도구(rosbag2)가 없습니다.",
    "sensors_failed": "센서를 켜지 못했습니다.",
    "sensors_missing": "센서가 켜져 있지 않습니다.",
    "recorder_exited": "녹화기가 바로 종료됐습니다.",
    "not_recording": "녹화 중이 아닙니다.",
    "stopped": "녹화를 끝냈습니다.",
    "sensors_remaining": "녹화를 끝냈지만 센서가 남았습니다.",
    "stop_incomplete": "녹화 정지가 완전히 확인되지 않았습니다(metadata.yaml 없음).",
    "gnss_device_missing": "Pi5에 GPS 수신기(u-blox)가 연결돼 있지 않습니다.",
    "gnss_exited": "Pi5 GPS 수신기가 바로 종료됐습니다.",
    "timeout": "보드 응답 시간이 초과됐습니다.",
    "authentication_failed": "SSH 인증에 실패했습니다.",
    "host_key_check_failed": "호스트 키 확인에 실패했습니다.",
    "ssh_unreachable": "보드에 연결할 수 없습니다.",
}


def _fail(reason, **extra):
    return dict(ok=False, reason=reason, **extra)


def _marker(result):
    """The last marker line as a dict, or a failure dict carrying the SSH reason."""
    if not isinstance(result, dict):
        return _fail("invalid_remote_result")
    stdout = result.get("stdout", "") if isinstance(result.get("stdout", ""), str) else ""
    lines = [line[len(MARKER):] for line in stdout.splitlines() if line.startswith(MARKER)]
    if not lines:
        return _fail(result.get("reason") or "missing_result")
    try:
        value = json.loads(lines[-1])
    except ValueError:
        return _fail("invalid_remote_result")
    return value if isinstance(value, dict) else _fail("invalid_remote_result")


def _run(host, script, password, timeout_s, header=""):
    path = BASE_DIR / "remote" / script
    if not header:
        return _marker(run_remote(host, path, password, timeout_s=timeout_s))
    with tempfile.TemporaryDirectory(prefix="nomad-record-") as folder:
        temporary = Path(folder) / script
        temporary.write_text(header + path.read_text(encoding="utf-8"), encoding="utf-8")
        return _marker(run_remote(host, temporary, password, timeout_s=timeout_s))


def _hosts(jetson, pi=None):
    jetson = validate_host(jetson)
    if jetson.split("@", 1)[0] != "nvidia":
        raise ValueError("Jetson account")
    if pi:
        pi = validate_host(pi)
        if pi.split("@", 1)[0] != "user":
            raise ValueError("Pi5 account")
    return jetson, pi


def start(jetson, pi, password, scene, gps=True):
    """Start one recording. Returns {ok, reason, session, sensors_started, gnss}."""
    try:
        jetson, pi = _hosts(jetson, pi if gps else None)
        if not isinstance(scene, str) or not SCENE_RE.match(scene):
            return _fail("invalid_scene")
    except ValueError:
        return _fail("invalid_board_address")
    gnss = None
    if gps:
        gnss = _run(pi, "record_gnss_start_pi.sh", password, 25)
        if not gnss.get("ok"):
            return _fail(gnss.get("reason", "gnss_failed"), gnss=gnss, step="gnss")
    header = f"SCENE={scene}\nGPS={1 if gps else 0}\n"
    value = _run(jetson, "record_start_jetson.sh", password, 120, header=header)
    value["gnss"] = gnss
    if not value.get("ok") and gnss and gnss.get("started"):
        _run(pi, "record_gnss_stop_pi.sh", password, 20)  # undo only what this start did
    return value


def stop(jetson, pi, password):
    """Stop the current recording; then the GUI-started Pi5 GNSS receiver (if any)."""
    try:
        jetson, pi = _hosts(jetson, pi)
    except ValueError:
        return _fail("invalid_board_address")
    value = _run(jetson, "record_stop_jetson.sh", password, 150)
    value["gnss"] = _run(pi, "record_gnss_stop_pi.sh", password, 20) if pi else None
    return value


def status(jetson, password):
    try:
        jetson, _ = _hosts(jetson)
    except ValueError:
        return _fail("invalid_board_address")
    return _run(jetson, "record_status_jetson.sh", password, 12)


def describe(value):
    """One Korean line for the window."""
    reason = value.get("reason", "")
    text = LABELS.get(reason, "실패: " + str(reason))
    if value.get("message"):
        text += " · " + str(value["message"])[:160]
    if reason in ("stopped", "sensors_remaining"):
        size = value.get("bytes")
        text += (f" · {value.get('duration_s') or '?'} s · {round(size / 1e9, 2) if isinstance(size, (int, float)) else '?'} GB"
                 f" · GPS 위치 {value.get('gps_fix', 0)}개 · {Path(str(value.get('session', ''))).name}")
        if value.get("gps_fix") == 0:
            text += " · GPS 0개: 위성 미수신이면 Pi5 수신기가 위치를 내지 않습니다(실내 등)"
    return text + stop_details(value)


def stop_details(value):
    """2026-11-16: short Korean tail for a stop result: sensors stopped / leftover processes."""
    text = ""
    sensors = value.get("sensors")
    if value.get("reason") == "not_recording" and sensors == "stopped":
        text += " · 녹화가 켠 센서를 껐습니다"
    elif sensors == "recording_active":
        text += " · 녹화 중이라 센서 유지"
    if value.get("survivors"):
        text += " · 남은 센서 " + str(value["survivors"])[:120] + " (다시 정지)"
    gnss = value.get("gnss")
    if isinstance(gnss, dict):
        if gnss.get("survivors"):
            text += " · Pi5 GPS 남음 " + str(gnss["survivors"])[:60]
        elif gnss.get("ok") is False and gnss.get("reason") not in (None, "wrong_board"):
            text += " · Pi5 GPS 정지 미확인(" + LABELS.get(gnss.get("reason"), str(gnss.get("reason"))) + ")"
    return text
