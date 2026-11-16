#!/usr/bin/env bash
# Pi5 front video management (2026-11-11). The GUI prepends one line:
#   NOMAD_FRONT_PAYLOAD=<base64 JSON>
# with {"action": "note", "run": ..., "title": ..., "comment": ...}
#   or {"action": "delete", "run": ...}
# Notes live in ONE file keyed by trial name and are never removed by a delete:
#   ~/nomad_ws/log/front_video_notes.json
# A delete removes only ~/nomad_ws/log/<run>/front_video (never the trial's
# other logs) and refuses the trial that is recording now.
set -uo pipefail
exec 2>/dev/null
marker=NOMAD_FRONT_MANAGE_RESULT=
fail() { printf '%s{"ok":false,"reason":"%s"}\n' "$marker" "$1"; exit 0; }

[[ "$(id -un)" = user ]] || fail wrong_board
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = Raspberry\ Pi\ 5* ]] || fail wrong_board
[[ -n "${NOMAD_FRONT_PAYLOAD:-}" ]] || fail missing_payload
ws=/home/user/nomad_ws
exec 9>"/tmp/nomad_front_manage_$(id -u).lock"
flock -w 5 9 || fail busy

NOMAD_FRONT_PAYLOAD="$NOMAD_FRONT_PAYLOAD" NOMAD_WS="$ws" /usr/bin/python3 -I - <<'PY'
import base64, json, os, re, shutil, time
from pathlib import Path

MARK = "NOMAD_FRONT_MANAGE_RESULT="
def out(**value):
    print(MARK + json.dumps(value, ensure_ascii=False))
    raise SystemExit(0)

try:
    payload = json.loads(base64.b64decode(os.environ["NOMAD_FRONT_PAYLOAD"], validate=True))
except Exception:
    out(ok=False, reason="bad_payload")
ws = Path(os.environ["NOMAD_WS"])
run = payload.get("run")
if not isinstance(run, str) or not re.fullmatch(r"visual_trial_[A-Za-z0-9_-]{1,64}", run):
    out(ok=False, reason="bad_run")
notes_path = ws / "log" / "front_video_notes.json"

def load_notes():
    try:
        data = json.loads(notes_path.read_text(encoding="utf-8"))
        if isinstance(data, dict) and isinstance(data.get("runs"), dict):
            return data
    except (OSError, ValueError):
        pass
    return {"version": 1, "runs": {}}

action = payload.get("action")
if action == "note":
    title, comment = payload.get("title", ""), payload.get("comment", "")
    if not isinstance(title, str) or not isinstance(comment, str) or len(title) > 120 or len(comment) > 2000:
        out(ok=False, reason="bad_note")
    data = load_notes()
    entry = data["runs"].get(run, {})
    entry.update(title=title.strip(), comment=comment.strip(), updated=int(time.time()))
    data["runs"][run] = entry
    if notes_path.exists():
        shutil.copy2(notes_path, notes_path.with_suffix(".json.bak"))
    temporary = notes_path.with_suffix(".json.tmp")
    temporary.write_text(json.dumps(data, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    os.replace(temporary, notes_path)
    out(ok=True, state="NOTE_SAVED", run=run, note=entry)

if action == "delete":
    folder = ws / "log" / run / "front_video"
    if not folder.is_dir() or folder.is_symlink() or folder.resolve().parent.parent != (ws / "log").resolve():
        out(ok=False, reason="no_such_recording")
    pointer = Path("/tmp/nomad_integrated_pi_run_dir")
    try:
        current = pointer.read_text().strip()
    except OSError:
        current = ""
    if current == str(ws / "log" / run):
        pid_file = ws / "log" / run / "front_recorder.pid"
        try:
            pid = int(pid_file.read_text().strip())
            cmd = Path(f"/proc/{pid}/cmdline").read_bytes().split(b"\0")[0].decode()
            if cmd.endswith("/front_recorder"):
                out(ok=False, reason="recording_now")
        except (OSError, ValueError):
            pass
    size = sum(p.stat().st_size for p in folder.rglob("*") if p.is_file())
    shutil.rmtree(folder)
    note = load_notes()["runs"].get(run)  # untouched: notes survive the delete
    out(ok=True, state="DELETED", run=run, bytes=size, note_kept=note is not None)

out(ok=False, reason="bad_action")
PY
