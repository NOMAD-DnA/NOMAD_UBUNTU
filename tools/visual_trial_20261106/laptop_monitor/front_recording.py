"""Read-only check of the Pi5 front video recorder (OAK color JPEG, about 10 Hz).

The recorder starts and stops with the trial; the GUI only asks whether it is
recording and what it has saved. Never returns raw remote output or secrets.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
RESULT_MARKER = "NOMAD_FRONT_RESULT="
STATES = {
    "RECORDING": "녹화 중",
    "WAITING": "영상 대기 중",
    "NO_IMAGES": "영상 끊김",
    "DISK_LOW": "저장 공간 부족 · 새 파일 중단",
    "ERROR": "녹화 오류",
    "STOPPED": "녹화 종료",
}
_NUMBERS = ("segments", "frames", "dropped", "rejected", "duration_s", "bytes", "free_gb", "last_frame_age_s")


def _failed(reason):
    return dict(ok=False, state="UNCONFIRMED", reason=reason)


def _number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("number")
    return value


def check_front_recording(pi_host, password=None, timeout_s=15):
    try:
        host = validate_host(pi_host)
        if host.split("@", 1)[0] != "user":
            raise ValueError("account")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("password")
    except (TypeError, ValueError):
        return _failed("invalid_pi5_address_or_options")
    try:
        result = run_remote(host, BASE_DIR / "remote/check_front_recording.sh", password, timeout_s=timeout_s)
        stdout = result.get("stdout", "") if isinstance(result, dict) else None
        if not isinstance(stdout, str) or len(stdout) > 16384:
            return _failed("invalid_remote_result")
        markers = [line[len(RESULT_MARKER):] for line in stdout.splitlines() if line.startswith(RESULT_MARKER)]
        if len(markers) != 1:
            reason = result.get("reason")
            return _failed(reason if isinstance(reason, str) and reason.isidentifier() else "missing_result")
        value = json.loads(markers[0])
        if not isinstance(value, dict) or value.get("ok") is not True or result.get("ok") is not True:
            return _failed(str(value.get("reason", "check_rejected")) if isinstance(value, dict) else "check_rejected")
        out = dict(ok=True, state=str(value.get("state")), alive=value.get("alive") is True,
                   previous_runs=int(_number(value.get("previous_runs", 0))),
                   previous_bytes=int(_number(value.get("previous_bytes", 0))))
        runs = []
        for item in value.get("runs") or []:
            if not isinstance(item, dict) or not isinstance(item.get("name"), str):
                raise ValueError("run")
            runs.append(dict(name=item["name"][:64], state=item.get("state") if item.get("state") in STATES else "UNKNOWN",
                             current=item.get("current") is True,
                             **{key: _number(item.get(key, 0)) for key in ("duration_s", "frames", "dropped",
                                                                         "segments", "bytes")}))
        out["runs"] = runs[:20]
        notes = {}
        for name, note in (value.get("notes") or {}).items():
            if isinstance(name, str) and isinstance(note, dict):
                notes[name[:80]] = dict(title=str(note.get("title", ""))[:120], comment=str(note.get("comment", ""))[:2000],
                                        updated=note.get("updated") if isinstance(note.get("updated"), int) else 0)
        out["notes"] = notes
        status = value.get("status")
        if isinstance(status, dict):
            recorder = {key: _number(status[key]) for key in _NUMBERS if key in status}
            recorder["state"] = status.get("state") if status.get("state") in STATES else "UNKNOWN"
            recorder["file"] = str(status.get("file", ""))[:64]
            recorder["error"] = str(status.get("error", ""))[:120]
            out["recorder"] = recorder
        return out
    except (ValueError, TypeError, KeyError):
        return _failed("invalid_remote_result")
    except Exception:
        return _failed("local_control_error")


def summary_text(value):
    """One line for the GUI label."""
    if not value.get("ok"):
        return "전방 녹화: 확인 실패 · " + str(value.get("reason", "unknown"))
    previous = value.get("previous_runs", 0)
    tail = f" · 이전 시험 {previous}개 저장({value.get('previous_bytes', 0) / 1e9:.1f}GB)" if previous else ""
    state = value.get("state")
    if state == "NOT_INSTALLED":
        return "전방 녹화: Pi5에 녹화기 미설치" + tail
    if state == "NO_TRIAL_RUN":
        return "전방 녹화: 진행 중인 시험 없음" + tail
    if state == "NOT_STARTED":
        return "전방 녹화: 이번 시험에서 시작 안 됨" + tail
    recorder = value.get("recorder")
    if state != "CHECKED" or not recorder:
        return "전방 녹화: 상태 미확인" + tail
    label = STATES.get(recorder["state"], "상태 미확인")
    if recorder["state"] != "STOPPED" and not value.get("alive"):
        label = "녹화기 종료됨"
    minutes = recorder.get("duration_s", 0) / 60
    parts = [label, f"{minutes:.1f}분", f"{recorder.get('frames', 0):.0f}프레임",
             f"{recorder.get('bytes', 0) / 1e9:.2f}GB", f"파일 {recorder.get('segments', 0):.0f}개",
             f"여유 {recorder.get('free_gb', 0):.0f}GB"]
    if recorder.get("dropped"):
        parts.append(f"누락 {recorder['dropped']:.0f}")
    if recorder.get("error"):
        parts.append(recorder["error"])
    return "전방 녹화: " + " · ".join(parts) + tail


def _run_time(name):
    """visual_trial_20261111_203955_55181 -> 11/11 20:39:55"""
    parts = name.split("_")
    if len(parts) >= 4 and len(parts[2]) == 8 and len(parts[3]) == 6 and (parts[2] + parts[3]).isdigit():
        d, t = parts[2], parts[3]
        return f"{int(d[4:6])}/{int(d[6:8])} {t[0:2]}:{t[2:4]}:{t[4:6]}"
    return name


def runs_text(value):
    """One line per recorded trial, newest first; empty when there is nothing to list."""
    if not value.get("ok"):
        return ""
    lines = []
    for run in value.get("runs", []):
        live = run["current"] and value.get("alive")
        if run["state"] in ("RECORDING", "WAITING", "NO_IMAGES") and not live:
            state = "중단됨 (정상 종료 기록 없음)"  # status never reached STOPPED: process ended abruptly
        else:
            state = STATES.get(run["state"], "상태 미확인")
        line = (f"{_run_time(run['name'])} · {run['duration_s'] / 60:.1f}분 · {run['frames']:.0f}프레임 · "
                f"{run['bytes'] / 1e9:.2f}GB · 파일 {run['segments']:.0f}개 · {state}")
        if run["dropped"]:
            line += f" · 누락 {run['dropped']:.0f}"
        if run["current"]:
            line += " · 현재 시험"
        lines.append(line)
    return "\n".join(lines) if lines else "저장된 전방 녹화 없음"


# ---------------------------------------------------------------- download --
import os
import re
import shlex
import shutil
import subprocess
import tarfile
import tempfile
import time

RUN_NAME = re.compile(r"visual_trial_[A-Za-z0-9_-]{1,64}")


def front_video_root():
    """Laptop folder for downloaded front videos: next to the GUI recordings."""
    from session_recording import recording_directory
    return Path(recording_directory()) / "전방녹화"


def downloaded(run_name):
    return (front_video_root() / run_name).is_dir()


def _ssh_stream(host, password, remote_command):
    """Start one SSH stream to a board, following a recovered route."""
    import board_address
    target = board_address.route(host)
    state = board_address.state_dir()
    state.mkdir(mode=0o700, parents=True, exist_ok=True)
    env = os.environ.copy()
    temporary = tempfile.TemporaryDirectory(prefix="nomad-ssh-")
    if password:
        askpass = Path(temporary.name) / "askpass"
        askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n', encoding="utf-8")
        askpass.chmod(0o700)
        env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force", NOMAD_SSH_PASSWORD=password)
        env.setdefault("DISPLAY", ":0")
    common = ["-o", "ConnectTimeout=5", "-o", "ConnectionAttempts=1", "-o", "ServerAliveInterval=5",
              "-o", "ServerAliveCountMax=3", "-o", "NumberOfPasswordPrompts=1",
              "-o", "StrictHostKeyChecking=accept-new",
              "-o", 'UserKnownHostsFile="' + str(state / "known_hosts") + '"',
              "-o", "LogLevel=ERROR", "-o", "BatchMode=" + ("no" if password else "yes")]
    args = ["ssh", "-T", *common, *board_address.ssh_route_options(target, common), target["host"], remote_command]
    process = subprocess.Popen(args, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                               env=env, start_new_session=True)
    return process, temporary, target


def _jpeg_size(data):
    """(width, height) from a JPEG's SOF marker, or None."""
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            return None
        marker = data[i + 1]
        length = int.from_bytes(data[i + 2:i + 4], "big")
        if marker in (0xC0, 0xC1, 0xC2):
            return int.from_bytes(data[i + 7:i + 9], "big"), int.from_bytes(data[i + 5:i + 7], "big")
        i += 2 + length
    return None


class _AviWriter:
    """Streaming MJPEG AVI writer: JPEG frames are copied unchanged, sizes fixed at close."""

    def __init__(self, output, fps, size):
        import struct
        self.struct, self.path, self.size = struct, Path(output), size
        self.scale, self.rate = 1000, max(1, round(fps * 1000))
        self.file = open(self.path, "wb")
        self.index, self.frames, self.biggest, self.bytes = [], 0, 0, 0
        self.file.write(self._header(0, 0))
        self.movi_start = self.file.tell()  # position of the movi LIST
        self.file.write(b"LIST\0\0\0\0movi")

    def _header(self, frames, biggest):
        pack, (w, h) = self.struct.pack, self.size
        chunk = lambda tag, body: tag + pack("<I", len(body)) + body
        avih = pack("<IIIIIIIIII16x", round(1e6 * self.scale / self.rate), biggest * self.rate // self.scale,
                    0, 0x10, frames, 0, 1, biggest, w, h)
        strh = pack("<4s4sIHHIIIIIIIIhhhh", b"vids", b"MJPG", 0, 0, 0, 0, self.scale, self.rate, 0, frames,
                    biggest, 0xFFFFFFFF, 0, 0, 0, w, h)
        strf = pack("<IiiHH4sIiiII", 40, w, h, 1, 24, b"MJPG", w * h * 3, 0, 0, 0, 0)
        strl = b"strl" + chunk(b"strh", strh) + chunk(b"strf", strf)
        hdrl = b"hdrl" + chunk(b"avih", avih) + b"LIST" + pack("<I", len(strl)) + strl
        return b"RIFF\0\0\0\0AVI " + b"LIST" + pack("<I", len(hdrl)) + hdrl

    def add(self, frame):
        pack = self.struct.pack
        offset = self.file.tell() - (self.movi_start + 8)
        self.file.write(b"00dc" + pack("<I", len(frame)) + frame + (b"\0" if len(frame) % 2 else b""))
        self.index.append(pack("<4sIII", b"00dc", 0x10, offset, len(frame)))
        self.frames += 1
        self.biggest = max(self.biggest, len(frame))
        self.bytes += len(frame)

    def close(self):
        pack = self.struct.pack
        movi_end = self.file.tell()
        self.file.write(b"idx1" + pack("<I", 16 * len(self.index)) + b"".join(self.index))
        end = self.file.tell()
        self.file.seek(0)
        self.file.write(self._header(self.frames, self.biggest))
        self.file.seek(4)
        self.file.write(pack("<I", end - 8))
        self.file.seek(self.movi_start + 4)
        self.file.write(pack("<I", movi_end - self.movi_start - 8))
        self.file.close()
        return self.path


def _iter_mjpeg_frames(folder):
    """Yield JPEG frames from seg_NNNN.mjpeg (+ .csv offset/bytes), one segment in memory at a time."""
    import csv
    for video in sorted(folder.glob("seg_*.mjpeg")):
        data = video.read_bytes()
        table = video.with_suffix(".csv")
        rows = list(csv.DictReader(table.open(encoding="utf-8"))) if table.is_file() else []
        if rows and "offset" in rows[0]:
            for row in rows:
                start, length = int(row["offset"]), int(row["bytes"])
                if start + length <= len(data):
                    yield data[start:start + length]
        else:  # no table: split on JPEG start/end markers
            start = data.find(b"\xff\xd8")
            while start >= 0:
                end = data.find(b"\xff\xd9", start)
                if end < 0:
                    break
                yield data[start:end + 2]
                start = data.find(b"\xff\xd8", end + 2)


def _measured_fps(folder):
    import csv
    first = last = None
    count = 0
    for table in sorted(folder.glob("seg_*.csv")):
        for row in csv.DictReader(table.open(encoding="utf-8")):
            stamp = int(row["stamp_ns"])
            first = stamp if first is None else first
            last, count = stamp, count + 1
    if count > 1 and last > first:
        return max(1.0, min(30.0, (count - 1) / ((last - first) / 1e9)))
    return 10.0


def write_mjpeg_avi_parts(folder, part_limit=900_000_000):
    """Write 전체.avi (or 전체_1.avi, 전체_2.avi … above ~900 MB each) from the segments."""
    fps, outputs, writer = _measured_fps(folder), [], None
    for frame in _iter_mjpeg_frames(folder):
        if writer is not None and writer.bytes + len(frame) > part_limit:
            outputs.append(writer.close())
            writer = None
        if writer is None:
            writer = _AviWriter(folder / f".part{len(outputs) + 1}.avi", fps, _jpeg_size(frame) or (640, 360))
        writer.add(frame)
    if writer is not None:
        outputs.append(writer.close())
    final = []
    for i, path in enumerate(outputs):
        target = folder / ("전체.avi" if len(outputs) == 1 else f"전체_{i + 1}.avi")
        path.rename(target)
        final.append(target)
    return final


def _combine(folder):
    """Make 전체.avi (playable everywhere, frames unchanged) and move raw files to 원본/."""
    raw = folder / "원본"
    mjpeg = sorted(folder.glob("seg_*.mjpeg"))
    avi = sorted(folder.glob("seg_*.avi"))
    outputs = []
    if mjpeg:
        outputs = write_mjpeg_avi_parts(folder)
    elif avi and shutil.which("ffmpeg"):
        listing = folder / ".concat.txt"
        listing.write_text("".join(f"file '{p.name}'\n" for p in avi), encoding="utf-8")
        try:
            subprocess.run([shutil.which("ffmpeg"), "-nostdin", "-v", "error", "-y", "-f", "concat", "-safe", "0",
                            "-i", str(listing), "-c", "copy", str(folder / "전체.avi")],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=600)
        except (OSError, subprocess.SubprocessError):
            pass
        listing.unlink(missing_ok=True)
        if (folder / "전체.avi").is_file():
            outputs.append(folder / "전체.avi")
    if outputs and mjpeg:
        raw.mkdir(exist_ok=True)
        for path in list(folder.glob("seg_*")) + [folder / "status.json"]:
            if path.is_file():
                path.rename(raw / path.name)
    return outputs[0] if outputs else None


def download_run(pi_host, run_name, password=None, progress=None, cancelled=None):
    """Copy one trial's front_video folder from Pi5 to this laptop (read-only on Pi5).

    Returns {ok, folder, files, bytes, combined, reason}."""
    try:
        if not isinstance(pi_host, str) or pi_host != pi_host.strip():
            raise ValueError("address")
        host = validate_host(pi_host)
        if host.split("@", 1)[0] != "user" or not RUN_NAME.fullmatch(str(run_name)):
            raise ValueError("input")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("password")
    except (TypeError, ValueError):
        return dict(ok=False, reason="invalid_pi5_address_or_run")
    root = front_video_root()
    root.mkdir(parents=True, exist_ok=True)
    final = root / run_name
    remote = "cd ~/nomad_ws/log/" + shlex.quote(run_name) + " && exec tar cf - front_video"
    import board_address
    for attempt in range(2):
        work = Path(tempfile.mkdtemp(prefix="." + run_name + ".", dir=root))
        process, temporary, target = _ssh_stream(host, password, remote)
        copied, files, reason = 0, 0, None
        try:
            with tarfile.open(fileobj=process.stdout, mode="r|") as archive:
                for member in archive:
                    if cancelled is not None and cancelled():
                        reason = "cancelled"
                        break
                    name = Path(member.name)
                    if not member.isfile() or name.is_absolute() or ".." in name.parts or \
                            name.parts[:1] != ("front_video",) or len(name.parts) != 2:
                        continue
                    source = archive.extractfile(member)
                    with open(work / name.name, "wb") as out:
                        while True:
                            block = source.read(1 << 20)
                            if not block:
                                break
                            out.write(block)
                            copied += len(block)
                            if progress is not None:
                                progress(copied)
                    files += 1
        except (tarfile.TarError, OSError):
            reason = reason or "transfer_failed"
        finally:
            if process.poll() is None:
                try:
                    os.killpg(process.pid, 15)
                except ProcessLookupError:
                    pass
            process.wait()
            stderr = process.stderr.read().decode("utf-8", "replace").lower() if process.stderr else ""
            temporary.cleanup()
        if reason is None and (process.returncode != 0 or files == 0):
            reason = "no_such_recording" if "no such file" in stderr else (
                "ssh_unreachable" if process.returncode == 255 else "transfer_failed")
        if reason is None:
            if final.exists():
                shutil.rmtree(final)
            work.rename(final)
            note = local_notes().get(run_name)
            if note:
                _write_memo_file(run_name, note)
            board_address.learn(host, target)
            return dict(ok=True, folder=str(final), files=files, bytes=copied, combined=str(_combine(final) or ""),
                        route=target.get("method"))
        shutil.rmtree(work, ignore_errors=True)
        if reason != "ssh_unreachable" or attempt or not board_address.recover(host):
            return dict(ok=False, reason=reason)
    return dict(ok=False, reason="transfer_failed")


# ------------------------------------------------------------ notes, delete --
# Notes (title + comment) are shared through ONE file on Pi5 keyed by trial
# name. This laptop keeps its own copy (메모_모음.json, and 메모.txt inside a
# downloaded folder). Nothing here ever deletes a laptop copy or a note.
import base64
import json as _json


def _notes_cache_path():
    return front_video_root() / "메모_모음.json"


def local_notes():
    try:
        data = _json.loads(_notes_cache_path().read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def remember_notes(notes):
    """Merge notes into the laptop cache (newer wins, never removes) and refresh 메모.txt files."""
    if not notes:
        return
    cache = local_notes()
    changed = False
    for run, note in notes.items():
        if not RUN_NAME.fullmatch(run) or not isinstance(note, dict):
            continue
        old = cache.get(run, {})
        if note.get("updated", 0) >= old.get("updated", 0) and note != old:
            cache[run] = note
            changed = True
            _write_memo_file(run, note)
    if changed:
        root = front_video_root()
        root.mkdir(parents=True, exist_ok=True)
        temporary = _notes_cache_path().with_suffix(".tmp")
        temporary.write_text(_json.dumps(cache, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        os.replace(temporary, _notes_cache_path())


def _write_memo_file(run, note):
    folder = front_video_root() / run
    if folder.is_dir():
        text = f"제목: {note.get('title', '')}\n\n{note.get('comment', '')}\n"
        (folder / "메모.txt").write_text(text, encoding="utf-8")


def local_runs():
    """Trials already downloaded to this laptop (kept even if Pi5 deleted them)."""
    root = front_video_root()
    if not root.is_dir():
        return []
    return sorted((p.name for p in root.iterdir() if p.is_dir() and RUN_NAME.fullmatch(p.name)), reverse=True)


# ------------------------------------------------- team shared folder copy --
# 2026-11-12: a local GUI copy also lists/plays front videos in the shared
# 주행기록/전방녹화 and can copy (never move) a downloaded trial there.

def shared_front_root():
    """Shared 주행기록/전방녹화 when this local copy reaches the team share now, else None."""
    try:
        import local_install
        folder = local_install.reachable_shared_recordings()
    except Exception:
        return None
    if folder is None:
        return None
    root = Path(folder) / "전방녹화"
    return None if os.path.abspath(str(root)) == os.path.abspath(str(front_video_root())) else root


def shared_runs(root, timeout=5):
    """Trial folders in the shared 전방녹화 (bounded: a dropped network mount can block)."""
    if root is None:
        return []
    try:
        result = subprocess.run(["ls", "-1", "--", str(root)], stdin=subprocess.DEVNULL, capture_output=True,
                                text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return []
    if result.returncode != 0:
        return []
    return sorted((n for n in result.stdout.splitlines() if RUN_NAME.fullmatch(n)), reverse=True)


def upload_run(run_name, shared_root, progress=None, cancelled=None, chunk=1 << 20):
    """Copy ~/…/전방녹화/<run> to the shared 전방녹화 via <run>.part, then rename.
    The laptop copy stays. An existing shared folder is never overwritten."""
    if not RUN_NAME.fullmatch(str(run_name)) or shared_root is None:
        return dict(ok=False, reason="bad_run")
    source = front_video_root() / run_name
    if not source.is_dir():
        return dict(ok=False, reason="not_downloaded")
    shared_root = Path(shared_root)
    target = shared_root / run_name
    partial = shared_root / (run_name + ".part")
    try:
        if target.exists():
            return dict(ok=False, reason="already_shared")
        shared_root.mkdir(parents=True, exist_ok=True)
        if partial.exists():
            shutil.rmtree(partial)   # our own unfinished copy from an earlier attempt
        files = [p for p in sorted(source.rglob("*")) if p.is_file() and not p.is_symlink()]
        total, copied = sum(p.stat().st_size for p in files), 0
        for path in files:
            destination = partial / path.relative_to(source)
            destination.parent.mkdir(parents=True, exist_ok=True)
            with open(path, "rb") as reader, open(destination, "wb") as writer:
                while True:
                    if cancelled and cancelled():
                        writer.close()
                        shutil.rmtree(partial, ignore_errors=True)
                        return dict(ok=False, reason="cancelled")
                    block = reader.read(chunk)
                    if not block:
                        break
                    writer.write(block)
                    copied += len(block)
                    if progress:
                        progress(copied)
            shutil.copystat(path, destination)
        if target.exists():
            shutil.rmtree(partial, ignore_errors=True)
            return dict(ok=False, reason="already_shared")
        os.rename(partial, target)
    except OSError as error:
        shutil.rmtree(partial, ignore_errors=True)
        return dict(ok=False, reason="copy_failed: " + (error.strerror or str(error))[:80])
    return dict(ok=True, folder=str(target), bytes=total, files=len(files))


def _manage(pi_host, password, payload):
    try:
        host = validate_host(pi_host)
        if host.split("@", 1)[0] != "user":
            raise ValueError("account")
        if password is not None and (not isinstance(password, str) or "\n" in password or "\r" in password):
            raise ValueError("password")
    except (TypeError, ValueError):
        return dict(ok=False, reason="invalid_pi5_address")
    encoded = base64.b64encode(_json.dumps(payload, ensure_ascii=False).encode("utf-8")).decode("ascii")
    body = (BASE_DIR / "remote/front_manage.sh").read_text(encoding="utf-8")
    with tempfile.TemporaryDirectory(prefix="nomad-front-") as tmp:
        script = Path(tmp) / "front_manage.sh"
        script.write_text("export NOMAD_FRONT_PAYLOAD=" + encoded + "\n" + body, encoding="utf-8")
        result = run_remote(host, script, password, timeout_s=30)
    marker = "NOMAD_FRONT_MANAGE_RESULT="
    lines = [line[len(marker):] for line in result.get("stdout", "").splitlines() if line.startswith(marker)]
    if len(lines) != 1:
        return dict(ok=False, reason=result.get("reason") or "missing_result")
    try:
        value = _json.loads(lines[0])
    except ValueError:
        return dict(ok=False, reason="invalid_remote_result")
    return value if isinstance(value, dict) else dict(ok=False, reason="invalid_remote_result")


def set_note(pi_host, run_name, title, comment, password=None):
    if not RUN_NAME.fullmatch(str(run_name)) or not isinstance(title, str) or not isinstance(comment, str) \
            or len(title) > 120 or len(comment) > 2000:
        return dict(ok=False, reason="bad_note")
    value = _manage(pi_host, password, {"action": "note", "run": run_name, "title": title, "comment": comment})
    if value.get("ok") and isinstance(value.get("note"), dict):
        remember_notes({run_name: dict(title=value["note"].get("title", ""), comment=value["note"].get("comment", ""),
                                       updated=value["note"].get("updated", 0))})
    return value


def delete_on_pi5(pi_host, run_name, password=None):
    """Delete only the Pi5 original front_video of one trial. Laptop copies and notes stay."""
    if not RUN_NAME.fullmatch(str(run_name)):
        return dict(ok=False, reason="bad_run")
    return _manage(pi_host, password, {"action": "delete", "run": run_name})
