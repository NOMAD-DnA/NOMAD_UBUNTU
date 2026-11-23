"""2026-11-15 GUI '키보드 조종' (누름 유지형): Pi4 팀원 제어기의 수동 조종 서비스를 노트북 키보드로 쓴다.

- Pi4 준비: remote/pi4_manual_prepare.sh (팀원 linux/start.sh와 같은 ensure_gui.sh 호출, START 없음).
- 연결: SSH 터널 127.0.0.1:<빈 포트> → Pi4 127.0.0.1:8765. HTTP 규약은 Pi4 web/index.html(1afcbb1)과 같다:
  GET /api/status(토큰·서버 시각) · POST /api/commissioning {action 1 시작/2 갱신/3 닫기} · POST /api/stop · /api/reset.
- 누름 유지: 앞으로 키를 누르는 동안만 설정 ERPM(뒤로 키는 후진 ERPM, Pi4 manual_reverse_max_erpm 이하),
  떼면 속도 0 → 제어기가 브레이크 → 핸드브레이크로 세운다
  (제어기 설정을 따름, 경사 정차 보장 아님). 조향 키도 누르는 동안만 꺾이고 떼면 가운데로.
- 갱신은 최대 20 Hz로 계속 보낸다(끊기면 제어기가 스스로 출력 해제). 창 초점을 잃으면 세션을 닫는다.
- 지상 주행 사용은 사용자 결정(2026-11-15: 지하주차장 지상 시험 확인). Pi4 코드는 바꾸지 않는다.
"""
from __future__ import annotations

import http.client
import json
import os
import secrets
import shlex
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path

from control import run_remote, validate_host

BASE_DIR = Path(__file__).resolve().parent
MARKER = "NOMAD_DRIVE_RESULT="
REMOTE_PORT = 8765
HOST_HEADER = f"127.0.0.1:{REMOTE_PORT}"
STRICT = dict(fresh=0.3, request=0.6, clock=1.0)
WIFI = dict(fresh=1.5, request=1.5, clock=2.0)
UPDATE_S = 0.05
POLL_S = 0.1


def control_timing(data):
    """Same contract check as the Pi4 page's controlTiming(); None = refuse to drive."""
    if data.get("wifi_manual_profile") is True:
        ok = (data.get("control_units") == "raw" and data.get("commissioning") is True
              and data.get("continuous_manual_supported") is True and data.get("manual_timeout_s") == 2
              and data.get("command_max_age_s") == 1.5 and data.get("browser_freshness_s") == 1.5
              and data.get("browser_request_timeout_ms") == 1500)
        return WIFI if ok else None
    if data.get("wifi_manual_profile") not in (None, False):
        return None
    for key, limit in (("manual_timeout_s", .35), ("command_max_age_s", .3), ("browser_freshness_s", .3),
                       ("browser_request_timeout_ms", 600)):
        value = data.get(key)
        if value is not None and not (isinstance(value, (int, float)) and 0 < value <= limit):
            return None
    return STRICT


def servo_target(data, steer, fraction):
    """steer -1 (A/←, servo down like the Pi4 page) / 0 / +1 → servo position, None if the range is unknown."""
    lo, mid, hi = data.get("servo_min"), data.get("servo_center"), data.get("servo_max")
    if lo == 0 and hi == 1 and not isinstance(mid, (int, float)):
        mid = 0.5
    if not all(isinstance(v, (int, float)) for v in (lo, mid, hi)) or not lo < mid < hi:
        return None
    fraction = min(max(float(fraction), 0.0), 1.0)
    if steer < 0:
        return mid - fraction * (mid - lo)
    if steer > 0:
        return mid + fraction * (hi - mid)
    return mid


class ServerClock:
    """Conservative server-time estimate (Pi4 page observeServerClock/conservativeServerTime) with a rate allowance.

    2026-11-15: right after boot the Pi4 clock was 5.6 days behind and chrony slewed it (~7.5 % fast); the page's
    strict interval check then invalidated every second sample and dropped the session. Intervals may now differ by
    DRIFT × elapsed, and the estimate is lowered by DRIFT × age so it is never ahead of the true server time
    while |rate error| <= DRIFT (chrony's maximum slew is 8.3 %). A step (makestep, restart) still invalidates.
    """
    DRIFT = 0.1

    def __init__(self):
        self.samples, self.last = [], None

    def invalidate(self):
        self.samples, self.last = [], None
        return False

    def observe(self, sent, received, stamp, window):
        if not all(isinstance(v, (int, float)) for v in (sent, received, stamp)) or stamp <= 0 or received < sent \
                or (self.last is not None and received < self.last):
            return self.invalidate()
        recent = [s for s in self.samples if received - s[0] <= window]
        lower, upper = stamp - received, stamp - sent
        if any(lower > s[2] + self.DRIFT * (received - s[0]) or s[1] > upper + self.DRIFT * (received - s[0])
               for s in recent):
            return self.invalidate()
        self.samples, self.last = recent + [(received, lower, upper)], received
        return True

    def now(self, at, window):
        if self.last is None or at < self.last or at - self.last > window:
            return None
        recent = [s for s in self.samples if s[0] <= at and at - s[0] <= window]
        return at + max(s[1] - self.DRIFT * (at - s[0]) for s in recent) if recent else None


class HttpTransport:
    """HTTP to the tunnel's local port with the Host/Origin the Pi4 server requires."""

    def __init__(self, port):
        self.port, self.connection = port, None

    def __call__(self, method, path, body=None, token="", timeout=0.6):
        headers = {"Host": HOST_HEADER, "Cache-Control": "no-store"}
        payload = None
        if method == "POST":
            payload = json.dumps(body or {}).encode()
            headers.update({"Content-Type": "application/json", "Origin": "http://" + HOST_HEADER,
                            "X-Nomad-Token": token})
        for attempt in (0, 1):
            try:
                if self.connection is None:
                    self.connection = http.client.HTTPConnection("127.0.0.1", self.port, timeout=timeout)
                self.connection.timeout = timeout
                if self.connection.sock is not None:
                    self.connection.sock.settimeout(timeout)
                self.connection.request(method, path, body=payload, headers=headers)
                response = self.connection.getresponse()
                data = response.read()
                if response.status != 200:
                    raise OSError(f"HTTP {response.status}")
                return json.loads(data.decode())
            except (OSError, http.client.HTTPException, ValueError):
                if self.connection is not None:
                    self.connection.close()
                self.connection = None
                if attempt or method == "POST":
                    raise
        raise OSError("unreachable")

    def close(self):
        if self.connection is not None:
            self.connection.close()
            self.connection = None


class DriveClient:
    """One control loop thread. UI calls set_keys/request_start/stop/close_session from any thread."""

    def __init__(self, transport, clock=time.monotonic, sleep=time.sleep):
        self.transport, self.clock, self.sleep = transport, clock, sleep
        self.lock = threading.Lock()
        self.status, self.token, self.last_poll, self.poll_failed = {}, "", None, False
        self.server = ServerClock()
        self.keys = dict(forward=False, left=False, right=False, back=False)
        self.erpm, self.steer_fraction, self.reverse_erpm = 1000.0, 0.7, 600.0
        # 2026-11-15 사용자: 조향이 뒷바퀴라 A(왼쪽)를 누르면 차가 왼쪽으로 돌도록 서보 방향을 뒤집는다(기본 켬).
        self.steer_invert = True
        self.session = None
        self.want_start = self.want_stop = False
        self.message = "연결됨 · '조종 시작'을 누르세요."
        self.running = False
        self.thread = None

    # ---- UI side -----------------------------------------------------------
    def set_keys(self, forward, left, right, back=False):
        with self.lock:
            self.keys = dict(forward=bool(forward), left=bool(left), right=bool(right), back=bool(back))

    def set_limits(self, erpm, steer_fraction, reverse_erpm=None):
        with self.lock:
            self.erpm, self.steer_fraction = max(0.0, float(erpm)), min(max(float(steer_fraction), 0.0), 1.0)
            if reverse_erpm is not None:
                self.reverse_erpm = max(0.0, float(reverse_erpm))

    def set_steer_invert(self, invert):
        with self.lock:
            self.steer_invert = bool(invert)

    def request_start(self):
        with self.lock:
            self.want_start = True

    def request_stop(self):
        with self.lock:
            self.want_stop, self.want_start = True, False
            self.keys = dict(forward=False, left=False, right=False, back=False)

    def close_session(self, reason):
        with self.lock:
            self.want_start = False
            self.keys = dict(forward=False, left=False, right=False, back=False)
            session, self.session = self.session, None
        if session:
            self._close(session, reason)

    def snapshot(self):
        with self.lock:
            timing = control_timing(self.status)
            return dict(status=dict(self.status), active=bool(self.session), message=self.message,
                        fresh=self._fresh(timing), keys=dict(self.keys), erpm=self.erpm)

    def start_loop(self):
        self.running = True
        self.thread = threading.Thread(target=self._loop, name="nomad-keyboard-drive", daemon=True)
        self.thread.start()

    def shutdown(self):
        self.close_session("창을 닫아 조종을 끝냈습니다.")
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=3)
        close = getattr(self.transport, "close", None)
        if close:
            close()

    # ---- loop ----------------------------------------------------------------
    def _fresh(self, timing):
        if timing is None or self.last_poll is None:
            return False
        age = self.clock() - self.last_poll
        return 0 <= age <= timing["fresh"] and self.status.get("connected") is True \
            and self.status.get("telemetry_fresh") is True

    def _post(self, route, body, timing):
        return self.transport("POST", "/api/" + route, body, self.token, (timing or STRICT)["request"])

    def _targets(self):
        with self.lock:
            keys, erpm, fraction, data, back_erpm = dict(self.keys), self.erpm, self.steer_fraction, self.status, self.reverse_erpm
            invert = self.steer_invert
        steer = (-1 if keys["left"] else 0) + (1 if keys["right"] else 0)
        if invert:
            steer = -steer
        servo = servo_target(data, steer, fraction)
        if servo is None:
            raise ValueError("서보 범위를 모릅니다.")
        top = data.get("motor_max_erpm")
        back_top = data.get("manual_reverse_max_erpm")
        speed = 0.0
        if keys["forward"] and not keys.get("back") and isinstance(top, (int, float)) and top > 0:
            speed = min(erpm, top)
        elif keys.get("back") and not keys["forward"] and isinstance(back_top, (int, float)) and back_top > 0:
            # 2026-11-15: 후진은 Pi4가 manual_reverse_max_erpm(>0)을 알릴 때만. 방향 전환 정지는 제어기가 처리.
            speed = -min(back_erpm, back_top)
        return dict(axis=3, motor_erpm=round(speed, 1), servo_position=round(servo, 4), duration_s=0)

    def _poll(self):
        timing = control_timing(self.status) or STRICT
        sent = self.clock()
        try:
            data = self.transport("GET", "/api/status", None, "", timing["request"])
        except Exception:
            with self.lock:
                self.poll_failed = True
                self.status = dict(self.status, connected=False)
            return
        received = self.clock()
        timing = control_timing(data)
        valid = timing is not None and self.server.observe(sent, received, data.get("server_time_s"), timing["clock"]) \
            and (not self.token or data.get("token") == self.token)
        with self.lock:
            self.status, self.token, self.last_poll, self.poll_failed = data, data.get("token", ""), received, False
            session = self.session if not valid else None
            if session:
                self.session = None
                if not valid:
                    self.server.invalidate()
        if session:
            self._close(session, "서버 시각 또는 연결이 바뀌어 조종을 해제했습니다. 다시 '조종 시작'을 누르세요.")

    def _close(self, session, reason):
        timing = control_timing(self.status)
        session["sequence"] += 1
        try:
            self._post("commissioning", dict(action=3, session=session["id"], sequence=session["sequence"]), timing)
        except Exception:
            reason += " (닫기 응답 없음 · 갱신이 끊기면 제어기가 스스로 해제)"
        with self.lock:
            self.message = reason

    def _start(self):
        timing = control_timing(self.status)
        data = self.status
        why = None
        if timing is None:
            why = "제어기 통신 규약이 이 GUI와 다릅니다(시간 설정)."
        elif not self._fresh(timing):
            why = "제어기 상태가 최신이 아닙니다(연결·VESC 확인)."
        elif data.get("control_units") != "raw" or data.get("commissioning_setup_acknowledged") is not True:
            why = "Pi4가 조종 가능 모드가 아닙니다. 'Pi4 연결'을 다시 누르세요."
        elif data.get("driver_ready") is not True:
            why = "VESC 드라이버가 준비되지 않았습니다."
        elif data.get("stationary") is not True:
            why = "바퀴가 완전히 멈춘 뒤 시작할 수 있습니다."
        if why:
            with self.lock:
                self.message = why
            return
        try:
            if data.get("state") != "IDLE":
                result = self._post("reset", {}, timing)
                if result.get("accepted") is not True:
                    raise ValueError(result.get("reason") or "정지 조건을 확인하세요.")
            with self.lock:
                self.keys = dict(forward=False, left=False, right=False, back=False)
            server_now = self.server.now(self.clock(), timing["clock"])
            if server_now is None:
                raise ValueError("통신 시각 확인 중입니다. 잠시 뒤 다시 누르세요.")
            session = dict(id=secrets.token_hex(16), sequence=1)
            result = self._post("commissioning", dict(action=1, session=session["id"], sequence=1,
                                                      server_time_s=server_now, **self._targets()), timing)
            if result.get("accepted") is not True:
                raise ValueError(result.get("reason") or "시작 요청이 거절됐습니다.")
            with self.lock:
                self.session = session
                self.message = "조종 중 · W/↑ 전진 · S/↓ 후진 · A/D·←/→ 조향 · Space/Esc 정지"
        except Exception as error:
            with self.lock:
                self.message = "시작 실패: " + str(error)[:160]

    def _update(self):
        timing = control_timing(self.status)
        with self.lock:
            session = self.session
        if not session:
            return
        try:
            if not self._fresh(timing):
                raise ValueError("상태 갱신이 끊겨 조종을 해제합니다.")
            server_now = self.server.now(self.clock(), timing["clock"])
            if server_now is None:
                raise ValueError("유효한 서버 시각이 없어 조종을 해제합니다.")
            session["sequence"] += 1
            result = self._post("commissioning", dict(action=2, session=session["id"], sequence=session["sequence"],
                                                      server_time_s=server_now, **self._targets()), timing)
            if result.get("accepted") is not True:
                raise ValueError(result.get("reason") or "제어기가 목표를 거절했습니다.")
        except Exception as error:
            with self.lock:
                still = self.session is session
                if still:
                    self.session = None
            if still:
                self._close(session, str(error)[:200])

    def _stop(self):
        timing = control_timing(self.status)
        with self.lock:
            session, self.session, self.want_stop = self.session, None, False
        if session:
            self._close(session, "STOP")
        try:
            result = self._post("stop", {}, timing)
            text = "STOP 확인 · 제어기 정지 출력(브레이크→핸드브레이크)" if result.get("accepted") is True \
                else "STOP 거절: " + str(result.get("reason", ""))[:120]
        except Exception as error:
            text = "STOP 응답 없음 · 차량을 눈으로 확인하세요 (" + str(error)[:80] + ")"
        with self.lock:
            self.message = text

    def _loop(self):
        next_poll = 0.0
        while self.running:
            started = self.clock()
            with self.lock:
                want_stop, want_start = self.want_stop, self.want_start
                self.want_start = False
            if want_stop:
                self._stop()
            if started >= next_poll:
                next_poll = started + POLL_S
                self._poll()
            if want_start and not want_stop:
                self._start()
            self._update()
            self.sleep(max(0.0, UPDATE_S - (self.clock() - started)))


def _free_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class Tunnel:
    """ssh -N -L 127.0.0.1:<port>:127.0.0.1:8765 to Pi4, same auth/known_hosts as control.run_remote."""

    def __init__(self, host, password):
        self.host, self.password, self.process, self.port, self.folder = validate_host(host), password, None, None, None

    def open(self, wait_s=8.0):
        target = dict(host=self.host)
        try:
            import board_address
            target = board_address.route(self.host)
        except Exception:
            pass
        state_home = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial"
        state_home.mkdir(mode=0o700, parents=True, exist_ok=True)
        env = os.environ.copy()
        options = ["-o", "ConnectTimeout=5", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
                   "-o", "NumberOfPasswordPrompts=1", "-o", "StrictHostKeyChecking=accept-new",
                   "-o", 'UserKnownHostsFile="' + str(state_home / "known_hosts") + '"', "-o", "LogLevel=ERROR",
                   "-o", "ExitOnForwardFailure=yes", "-o", "BatchMode=" + ("no" if self.password else "yes")]
        if self.password:
            self.folder = tempfile.TemporaryDirectory(prefix="nomad-drive-")
            askpass = Path(self.folder.name) / "askpass"
            askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_SSH_PASSWORD"\n', encoding="utf-8")
            askpass.chmod(0o700)
            env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force", NOMAD_SSH_PASSWORD=self.password)
            env.setdefault("DISPLAY", ":0")
        if target.get("via"):
            hop_options = [part for part in options if part != "ExitOnForwardFailure=yes"]
            hop_options = [part for i, part in enumerate(hop_options)
                           if not (part == "-o" and i + 1 < len(hop_options) and hop_options[i + 1] == "-o")]
            hop = " ".join(shlex.quote(part) for part in ["ssh", *hop_options, "-W", "%h:%p", target["via"]])
            options += ["-o", "ProxyCommand=" + hop]
        if target.get("alias"):
            options += ["-o", "HostKeyAlias=" + target["alias"]]
        self.port = _free_port()
        self.process = subprocess.Popen(["ssh", "-N", "-T", *options, "-L",
                                         f"127.0.0.1:{self.port}:127.0.0.1:{REMOTE_PORT}", target["host"]],
                                        stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                        stderr=subprocess.DEVNULL, env=env, start_new_session=True)
        end = time.monotonic() + wait_s
        while time.monotonic() < end:
            if self.process.poll() is not None:
                break
            try:
                with socket.create_connection(("127.0.0.1", self.port), timeout=0.3):
                    return True
            except OSError:
                time.sleep(0.2)
        self.close()
        return False

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None
        if self.folder is not None:
            self.folder.cleanup()
            self.folder = None


def _pi4_script(pi4, script, password, timeout_s):
    try:
        host = validate_host(pi4)
        if host.split("@", 1)[0] != "user":
            raise ValueError("Pi4 account")
    except ValueError:
        return dict(ok=False, reason="invalid_pi4_address")
    result = run_remote(host, BASE_DIR / "remote" / script, password, timeout_s=timeout_s)
    stdout = result.get("stdout", "") if isinstance(result, dict) else ""
    lines = [line[len(MARKER):] for line in stdout.splitlines() if line.startswith(MARKER)]
    if not lines:
        return dict(ok=False, reason=(result or {}).get("reason") or "missing_result")
    try:
        value = json.loads(lines[-1])
        return value if isinstance(value, dict) else dict(ok=False, reason="invalid_remote_result")
    except ValueError:
        return dict(ok=False, reason="invalid_remote_result")


def stop_service(pi4, password, timeout_s=40):
    """2026-11-15 사용자 요청: '연결 끊기'는 Pi4 수동 조종 서비스(driver·controller·gui)도 끈다(commission_session/stop.sh)."""
    return _pi4_script(pi4, "pi4_manual_stop.sh", password, timeout_s)


def prepare(pi4, password, timeout_s=75):
    """Run remote/pi4_manual_prepare.sh. Returns {ok, reason, message}."""
    return _pi4_script(pi4, "pi4_manual_prepare.sh", password, timeout_s)
