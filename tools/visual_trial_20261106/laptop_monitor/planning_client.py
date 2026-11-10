"""Laptop-only operator connection to the Pi4 C++ planner.

Goal selection never arms output. Arm/disarm are separate explicit operator
actions; direct throttle, steering and VESC APIs are deliberately absent.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import sys
import tempfile
from operator_timing import OperatorJournal

from PyQt5 import QtCore
from control import validate_host

MAX_LINE = 512 * 1024
MAX_PATH_POINTS = 600
DEFAULT_PLANNING_COMMAND = "bash /home/user/nomad_cpp_ws/src/nomad_planning_cpp/scripts/run_planning_output.sh --stdio"


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def goal_command(x, y, frame):
    if not finite(x) or not finite(y) or frame != "odom":
        raise ValueError("목표는 odom 좌표계의 유한한 좌표여야 합니다.")
    return {"cmd": "goal", "x": float(x), "y": float(y), "yaw": 0., "frame": frame}


def clean_path(value):
    if not isinstance(value, list) or len(value) > 20000:
        raise ValueError("Invalid path")
    points = []
    for item in value:
        if isinstance(item, dict):
            point = [item.get("x"), item.get("y"), item.get("yaw", 0.)]
        elif isinstance(item, list) and len(item) == 3:
            point = item
        else:
            raise ValueError("Invalid point")
        if not all(finite(v) for v in point):
            raise ValueError("Non-finite point")
        points.append([float(v) for v in point])
    # Display only: include both ends without changing the planner's ROS path.
    if len(points) > MAX_PATH_POINTS:
        points = [points[round(i * (len(points) - 1) / (MAX_PATH_POINTS - 1))]
                  for i in range(MAX_PATH_POINTS)]
    return points


def output_summary(event):
    """Describe reported command permission, never infer physical movement."""
    armed = event.get("armed") if isinstance(event.get("armed"), bool) else None
    enabled = event.get("actuator_enabled") if isinstance(event.get("actuator_enabled"), bool) else None
    mode = str(event.get("output_mode", "미수신"))
    if event.get("plan_only", True):
        permission = "경로 확인 · 출력 차단"
    elif armed is None:
        permission = "출력 허가 상태 미수신"
    elif not armed:
        permission = "출력 잠금"
    elif enabled is None:
        permission = "현재 구동 상태 미수신"
    elif not enabled:
        permission = "출력 허용 유지 · 현재 정지/대기"
    elif event.get("state") not in ("DRIVING", "TRACKING", "MOVING") or event.get("output_inhibit"):
        permission = "출력 허용 · 현재 정지/대기"
    else:
        permission = "출력 허용 · 구동 명령 가능"
    inhibit = str(event.get("output_inhibit", ""))
    source = str(event.get("feedback_source", "미수신"))
    prototype = " · 임시 환산: 0.25m/s·±0.30rad 기준, 실측 검증 전" if event.get("prototype_output") else ""
    return (f"{permission} | armed={(str(armed).lower() if armed is not None else 'unknown')} · enabled={(str(enabled).lower() if enabled is not None else 'unknown')}"
            f" · 모드 {mode} · 피드백 출처 {source}" + (f" · 차단: {inhibit}" if inhibit else "") + prototype)


def planning_for_display(event, age):
    """Retain explicit diagnostic paths on a disconnected link, never permission."""
    if age is not None and age <= 3:
        return event
    if "path_valid" not in event:
        return {}  # Old senders did not distinguish a retained path from a plan.
    return {**event, "path_valid": False, "executable": False,
            "gpp_state": "STALE", "gpp_reason": "플래너 상태 수신 지연 · 이전 경로 참고용",
            "lpp_state": "STALE", "lpp_reason": "최신 실행 가능 상태 확인 불가",
            "_display_stale": True}


def stage_summary(event):
    if not event:
        return "GPP · 상태 미수신", "LPP · 상태 미수신"
    gpp = f"GPP · {event.get('gpp_state', '상태 미제공')}"
    lpp = f"LPP · {event.get('lpp_state', '상태 미제공')} · {event.get('lpp_reason', '')}"
    path_stamp = event.get("gpp_stamp_ns")
    if isinstance(path_stamp, int) and not isinstance(path_stamp, bool) and path_stamp > 0 and finite(event.get("t")):
        path_age = max(0., event["t"]-path_stamp/1e9)
        gpp += f" · 경로 기준 시각 {path_age:.1f}초 전"
    gpp += f" · {event.get('gpp_reason', '')}"
    if event.get("path") and event.get("path_valid") is not True:
        gpp += " · 이전/무효 경로 참고용"
    lpp += " · 실행 후보 유효" if event.get("executable") is True else " · 실행 후보 없음"
    return gpp, lpp


def display_event(value):
    """Bound/validate the C++ stdio packet before UI or spectator fan-out."""
    stamp = value.get("t")
    if not finite(stamp) or stamp <= 0 or value.get("frame", "odom") != "odom":
        raise ValueError("Invalid planning timestamp/frame")
    event = {"kind": "telemetry", "source": "planning", "t": float(stamp),
             "frame": "odom", "state": str(value.get("state", "WAIT"))[:80],
             "reason": str(value.get("reason", value.get("detail", "")))[:400],
             "plan_only": value.get("plan_only") is not False,
             "armed": value.get("armed") if isinstance(value.get("armed"), bool) else None,
             "actuator_enabled": value.get("actuator_enabled") if isinstance(value.get("actuator_enabled"), bool) else None,
             "prototype_output": value.get("prototype_output") is True,
             "output_mode": str(value.get("output_mode", "미수신"))[:80],
             "output_inhibit": str(value.get("output_inhibit", ""))[:300],
             "feedback_source": str(value.get("feedback_source", "미수신"))[:100],
             "path": clean_path(value.get("path", [])),
             "trajectory": clean_path(value.get("trajectory", value.get("local_path", [])))}
    for field in ("gpp_state", "gpp_reason", "lpp_state", "lpp_reason"):
        if field in value:
            if not isinstance(value[field], str):
                raise ValueError("Invalid planning stage")
            event[field] = value[field][:400 if field.endswith("reason") else 80]
    for field in ("path_valid", "executable"):
        if field in value:
            if not isinstance(value[field], bool):
                raise ValueError("Invalid planning stage validity")
            event[field] = value[field]
    measured = value.get("stamp_ns", 0)
    if not isinstance(measured, int) or isinstance(measured, bool) or measured < 0:
        raise ValueError("Invalid source timestamp")
    event["stamp_ns"] = measured
    if "gpp_stamp_ns" in value:
        path_stamp = value["gpp_stamp_ns"]
        if isinstance(path_stamp, bool) or not isinstance(path_stamp, int) or path_stamp < 0:
            raise ValueError("Invalid GPP source timestamp")
        event["gpp_stamp_ns"] = path_stamp
    output_stamp = value.get("output_stamp_ns", 0)
    if not isinstance(output_stamp, int) or isinstance(output_stamp, bool) or output_stamp < 0:
        raise ValueError("Invalid output timestamp")
    event["output_stamp_ns"] = output_stamp
    if value.get("goal") is not None:
        goal = clean_path([value["goal"]])
        event["goal"] = goal[0]
    if len(json.dumps(event, allow_nan=False)) > 65536:
        raise ValueError("Planning display too large")
    return event


class PlanningClient(QtCore.QObject):
    packet = QtCore.pyqtSignal(dict)
    progress = QtCore.pyqtSignal(str)
    closed = QtCore.pyqtSignal()

    def __init__(self, host, command, runtime_dir, parent=None):
        super().__init__(parent)
        self.host = validate_host(host)
        if self.host.split("@", 1)[0] != "user":
            raise ValueError("Pi4 계정은 user@주소로 입력해 주세요.")
        self.command = command
        self.runtime_dir = Path(runtime_dir)
        self.process = QtCore.QProcess(self)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.readyReadStandardError.connect(self.drain_error)
        self.process.finished.connect(self.finished)
        self.process.errorOccurred.connect(self.process_error)
        self.clock_process = QtCore.QProcess(self)
        self.clock_process.finished.connect(self.clock_finished)
        self.clock_process.errorOccurred.connect(self.clock_error)
        self.buffer = bytearray()
        self.operator_journal = OperatorJournal()
        self.temporary = None
        self.stopping = False
        self.ready = False
        self.did_close = False
        self.force_timer = QtCore.QTimer(self)
        self.force_timer.setSingleShot(True)
        self.force_timer.timeout.connect(self.force_close)

    def start(self):
        self.clock_process.setProcessEnvironment(QtCore.QProcessEnvironment.systemEnvironment())
        self.progress.emit("Pi4 시각 맞춤 · 플래너 시작 전")
        self.clock_process.start(sys.executable, [str(Path(__file__).with_name("clock_sync.py")), "--host", self.host])

    def clock_finished(self, code, _status):
        self.clock_process.readAllStandardOutput()
        self.clock_process.readAllStandardError()
        if self.did_close:
            return
        if self.stopping:
            self.finished(0, QtCore.QProcess.NormalExit)
        elif code == 0:
            self.start_planner()
        else:
            self.finished(code, QtCore.QProcess.NormalExit)
            self.progress.emit("Pi4 시각 맞춤 실패 · 주소/비밀번호 확인")

    def clock_error(self, _error):
        self.finished(-1, QtCore.QProcess.CrashExit)
        self.progress.emit("Pi4 시각 맞춤 실행 실패")

    def start_planner(self):
        if self.stopping:
            return
        env = QtCore.QProcessEnvironment.systemEnvironment()
        password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")
        self.temporary = tempfile.TemporaryDirectory(prefix=".planning-ssh-", dir=self.runtime_dir)
        if password:
            askpass = Path(self.temporary.name) / "askpass"
            askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_BOARD_PASSWORD1"\n')
            askpass.chmod(0o700)
            env.insert("SSH_ASKPASS", str(askpass))
            env.insert("SSH_ASKPASS_REQUIRE", "force")
            if not env.contains("DISPLAY"):
                env.insert("DISPLAY", ":0")
        self.process.setProcessEnvironment(env)
        args = ["-T", "-o", "ConnectTimeout=6", "-o", "ServerAliveInterval=3",
                "-o", "ServerAliveCountMax=1", "-o", "NumberOfPasswordPrompts=1",
                "-o", "StrictHostKeyChecking=accept-new", "-o", "LogLevel=ERROR",
                "-o", 'UserKnownHostsFile="' + str(self.runtime_dir / "known_hosts") + '"',
                "-o", "BatchMode=" + ("no" if password else "yes"), self.host, self.command]
        self.progress.emit("Pi4 플래너 연결 중 · 출력 잠금 상태로 시작")
        self.process.start("ssh", args)

    def send(self, command, *, timing=None, origin="command_send"):
        if command.get("cmd") == "goal":
            command = goal_command(command.get("x"), command.get("y"), command.get("frame"))
        elif command not in ({"cmd": "cancel"}, {"cmd": "arm"}, {"cmd": "disarm"}):
            raise ValueError("Only goal/cancel/arm/disarm commands are supported")
        if (self.stopping or self.process.state() != QtCore.QProcess.Running or
                (not self.ready and command["cmd"] in ("goal", "arm"))):
            return False
        body = (json.dumps(command, allow_nan=False, separators=(",", ":")) + "\n").encode()
        sent = self.process.write(body) == len(body)
        self.operator_journal.request(command["cmd"], timing=timing, origin=origin, sent=sent)
        if self.operator_journal.error:
            self.progress.emit(self.operator_journal.error)
        return sent

    def read_output(self):
        self.buffer.extend(bytes(self.process.readAllStandardOutput()))
        while b"\n" in self.buffer:
            raw, _, remainder = self.buffer.partition(b"\n")
            self.buffer = bytearray(remainder)
            if len(raw) > MAX_LINE:
                self.stop()
                return
            try:
                value = json.loads(raw)
                if not isinstance(value, dict):
                    continue
                if value.get("type") == "ack":
                    command = value.get("cmd", value.get("command", "goal"))
                    self.operator_journal.acknowledge(command, value.get("ok"))
                    title = {"arm": "출력 허용 요청", "disarm": "출력 정지 요청", "cancel": "목표 취소 요청"}.get(command, "목표 요청")
                    self.progress.emit(title + (" 접수" if value.get("ok") is True else " 거부") +
                                       " · " + str(value.get("detail", ""))[:300])
                    continue
                if value.get("type") != "planning":
                    continue
                event = display_event(value)
            except (ValueError, TypeError, UnicodeDecodeError, OverflowError):
                continue
            self.ready = True
            self.packet.emit(event)
        if len(self.buffer) > MAX_LINE:
            self.stop()

    def drain_error(self):
        # Do not retain/relay SSH diagnostics containing board-specific details.
        chunk = bytes(self.process.readAllStandardError()).decode("utf-8", errors="replace")
        self.operator_journal.observe_stderr(chunk)

    def process_error(self, _error):
        self.ready = False
        self.progress.emit("Pi4 플래너 SSH 연결 실패 · 주소/접속/실행 파일 확인")
        if self.process.state() == QtCore.QProcess.NotRunning:
            self.finished(-1, QtCore.QProcess.CrashExit)

    def stop(self):
        if self.stopping:
            return
        if self.clock_process.state() != QtCore.QProcess.NotRunning:
            self.stopping = True
            self.clock_process.kill()
            return
        self.send({"cmd": "disarm"}, origin="session_close")
        self.send({"cmd": "cancel"})
        self.stopping = True
        self.ready = False
        # The native planner treats EOF/SSH loss as disarm and cancels its work.
        self.process.closeWriteChannel()
        if self.process.state() == QtCore.QProcess.NotRunning:
            self.finished(0, QtCore.QProcess.NormalExit)
        else:
            self.force_timer.start(4000)

    def force_close(self):
        if self.process.state() != QtCore.QProcess.NotRunning:
            self.process.kill()

    def finished(self, _code, _status):
        if self.did_close:
            return
        self.did_close = True
        self.drain_error()
        self.operator_journal.close()
        self.force_timer.stop()
        self.ready = False
        if self.temporary is not None:
            self.temporary.cleanup()
            self.temporary = None
        self.progress.emit("Pi4 플래너 연결 종료 · 센서/보드 전원 유지")
        self.closed.emit()
