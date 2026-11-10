#!/usr/bin/env python3
"""Linux laptop entry point. Board runtimes remain the existing native programs."""
from __future__ import annotations

import argparse
from datetime import datetime
import ipaddress
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

BASE_DIR = Path(__file__).resolve().parent
STATE_ROOT = (Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial").expanduser().resolve()
FINDER = BASE_DIR / "scripts/find_ssh_boards.sh"
DEFAULT_RELAY_PORT = 8768


def validate_relay_address(text):
    value = text.strip()
    if not value or value.count(":") > 1:
        raise ValueError("메인 PC의 IPv4 또는 호스트명:포트를 입력해 주세요.")
    host, separator, port_text = value.partition(":")
    try:
        address = ipaddress.ip_address(host)
        if address.version != 4 or address.is_unspecified or address.is_multicast:
            raise ValueError()
        host = str(address)
    except ValueError:
        if re.fullmatch(r"[0-9.]+", host) or not re.fullmatch(
                r"[A-Za-z0-9](?:[A-Za-z0-9.-]{0,251}[A-Za-z0-9])?", host):
            raise ValueError("메인 PC 주소를 확인해 주세요.") from None
    try:
        port = int(port_text) if separator else DEFAULT_RELAY_PORT
        if not 1 <= port <= 65535:
            raise ValueError()
    except ValueError:
        raise ValueError("중계 포트는 1~65535 사이 숫자여야 합니다.") from None
    return f"{host}:{port}"


def validate_targets(pi_host, jetson_host):
    # The backend owns the common SSH validation rules. Account restrictions
    # here also prevent a discovered Pi4 being selected as the Pi5 by accident.
    from control import validate_host
    pi_host, jetson_host = validate_host(pi_host.strip()), validate_host(jetson_host.strip())
    if pi_host.split("@", 1)[0] != "user":
        raise ValueError("Pi5 계정은 user@주소 형식으로 입력해 주세요.")
    if jetson_host.split("@", 1)[0] != "nvidia":
        raise ValueError("Jetson 계정은 nvidia@주소 형식으로 입력해 주세요.")
    if pi_host.split("@", 1)[1].lower() == jetson_host.split("@", 1)[1].lower():
        raise ValueError("Pi5와 Jetson에 서로 다른 보드 주소를 입력해 주세요.")
    return pi_host, jetson_host


def parse_discovery(text):
    found = {"pi": [], "jetson": []}
    for line in text.splitlines():
        match = re.fullmatch(r"(Pi 5|Jetson):\s+ssh\s+(\S+)", line.strip())
        if match:
            board = "pi" if match[1] == "Pi 5" else "jetson"
            if match[2] not in found[board]:
                found[board].append(match[2])
    return found


def private_directory(path):
    path.mkdir(parents=True, exist_ok=True, mode=0o700)
    path.chmod(0o700)
    return path


class Launcher(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NOMAD 시험 연결")
        self.resize(860, 740)
        self.setMinimumSize(760, 680)
        self.process = None
        self.process_kind = None
        self.cancel_reason = None
        self.output = ""
        self.pending_text = ""
        self.viewer = None
        self.viewer_kind = None
        self.run_dir = None
        self.pending_targets = None
        private_directory(STATE_ROOT)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(26, 22, 26, 22)
        layout.setSpacing(13)
        title = QtWidgets.QLabel("NOMAD  /  실시간 시험")
        title.setObjectName("title")
        layout.addWidget(title)
        subtitle = QtWidgets.QLabel("보드 주소를 찾거나 직접 입력한 뒤 실시간 화면에 연결합니다.")
        subtitle.setObjectName("muted")
        self.subtitle = subtitle
        layout.addWidget(subtitle)

        role_row = QtWidgets.QHBoxLayout()
        self.connection_mode = QtWidgets.QComboBox()
        self.connection_mode.addItem("운영 · 보드 연결 / 관전 중계", "operator")
        self.connection_mode.addItem("관전 · 메인 PC에서 수신", "spectator")
        self.connection_mode.setAccessibleName("운영 또는 관전 연결 방식")
        role_row.addWidget(self.connection_mode)
        self.relay_port = QtWidgets.QSpinBox()
        self.relay_port.setRange(1, 65535)
        self.relay_port.setValue(DEFAULT_RELAY_PORT)
        self.relay_port.setPrefix("중계 포트 ")
        role_row.addWidget(self.relay_port)
        self.relay_host = QtWidgets.QLineEdit()
        self.relay_host.setPlaceholderText("관전: 메인 PC 주소:8768")
        self.relay_host.setAccessibleName("관전 대상 메인 PC 주소")
        role_row.addWidget(self.relay_host, 1)
        layout.addLayout(role_row)
        self.role_note = QtWidgets.QLabel()
        self.role_note.setObjectName("muted")
        self.role_note.setWordWrap(True)
        layout.addWidget(self.role_note)

        search_row = QtWidgets.QHBoxLayout()
        self.find_button = QtWidgets.QPushButton("보드 찾기")
        self.find_button.clicked.connect(self.discover)
        self.subnet = QtWidgets.QLineEdit()
        self.subnet.setPlaceholderText("검색 대역 · 비워 두면 현재 LAN (예: 192.168.1.0/24)")
        self.subnet.setAccessibleName("선택 검색 대역")
        self.cancel_button = QtWidgets.QPushButton("검색 취소")
        self.cancel_button.clicked.connect(self.cancel_search)
        self.cancel_button.hide()
        search_row.addWidget(self.find_button)
        search_row.addWidget(self.subnet, 1)
        search_row.addWidget(self.cancel_button)
        layout.addLayout(search_row)

        self.hosts = {}
        board_row = QtWidgets.QHBoxLayout()
        for key, title_text, purpose, placeholder in (
                ("pi", "01  Raspberry Pi 5", "VIO · GNSS 융합 / 위치추정", "user@보드주소"),
                ("jetson", "02  Jetson", "카메라 · 라이다 · 인지 · 지도", "nvidia@보드주소")):
            group = QtWidgets.QGroupBox(title_text)
            group_layout = QtWidgets.QVBoxLayout(group)
            purpose_label = QtWidgets.QLabel(purpose)
            purpose_label.setObjectName("muted")
            group_layout.addWidget(purpose_label)
            combo = QtWidgets.QComboBox()
            combo.setEditable(True)
            combo.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
            combo.lineEdit().setPlaceholderText(placeholder)
            combo.setAccessibleName(title_text + " SSH 주소")
            self.hosts[key] = combo
            group_layout.addWidget(combo)
            board_row.addWidget(group)
        layout.addLayout(board_row)

        password_row = QtWidgets.QHBoxLayout()
        password_row.addWidget(QtWidgets.QLabel("SSH 비밀번호"))
        self.password = QtWidgets.QLineEdit(os.environ.get("NOMAD_BOARD_PASSWORD1", ""))
        self.password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.password.setPlaceholderText("두 보드 공통 · 화면 연결만 할 때는 SSH 키도 사용 가능")
        password_row.addWidget(self.password, 1)
        self.reveal = QtWidgets.QCheckBox("표시")
        self.reveal.toggled.connect(lambda value: self.password.setEchoMode(
            QtWidgets.QLineEdit.Normal if value else QtWidgets.QLineEdit.Password))
        password_row.addWidget(self.reveal)
        layout.addLayout(password_row)
        help_text = QtWidgets.QLabel(
            "비밀번호는 저장하지 않습니다. 보드 찾기도 입력한 SSH 비밀번호를 사용합니다.")
        help_text.setObjectName("muted")
        help_text.setWordWrap(True)
        layout.addWidget(help_text)

        action_row = QtWidgets.QHBoxLayout()
        self.start_button = QtWidgets.QPushButton("시험 시작")
        self.start_button.setObjectName("primary")
        self.start_button.clicked.connect(self.start_trial)
        self.connect_button = QtWidgets.QPushButton("화면 연결")
        self.connect_button.clicked.connect(self.connect_viewer)
        self.stop_button = QtWidgets.QPushButton("시험 종료")
        self.stop_button.setObjectName("stop")
        self.stop_button.clicked.connect(self.stop_trial)
        self.replay_button = QtWidgets.QPushButton("기록 열기")
        self.replay_button.clicked.connect(self.open_recording)
        action_row.addWidget(self.start_button)
        action_row.addWidget(self.connect_button)
        action_row.addWidget(self.stop_button)
        action_row.addWidget(self.replay_button)
        layout.addLayout(action_row)
        note = QtWidgets.QLabel(
            "시험 시작: 센서·위치추정·인지·지도를 기동하고 화면을 엽니다.\n"
            "화면 연결: 이미 실행 중인 모듈의 화면만 엽니다. 모터 제어는 포함하지 않습니다.\n"
            "기록 열기: 보드 연결 없이 저장한 시험 데이터를 재생합니다.")
        note.setWordWrap(True)
        note.setObjectName("muted")
        layout.addWidget(note)
        self.status = QtWidgets.QLabel("보드 주소를 입력하거나 ‘보드 찾기’를 눌러 주세요.")
        self.status.setWordWrap(True)
        self.status.setObjectName("status")
        layout.addWidget(self.status)
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(800)
        self.log.setPlaceholderText("검색 결과와 실행 진행 상황")
        layout.addWidget(self.log, 1)
        footer = QtWidgets.QLabel("‘시험 종료’는 두 보드의 센서·모듈을 종료합니다. 보드 전원과 결과 기록은 유지합니다.")
        footer.setObjectName("muted")
        footer.setWordWrap(True)
        layout.addWidget(footer)
        self.setStyleSheet("""
            QMainWindow { background: #f3f5f9; }
            QWidget { font-size: 13px; color: #17263b; }
            QLabel#title { font-size: 23px; font-weight: 700; }
            QLabel#muted { color: #586b82; font-size: 12px; }
            QLabel#status { background: #e4edf8; padding: 10px; border-radius: 5px; }
            QGroupBox { background: white; border: 1px solid #d5deea; border-radius: 6px;
                        font-weight: 600; margin-top: 10px; padding: 12px 8px 8px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; }
            QLineEdit, QComboBox, QPlainTextEdit { background: white; border: 1px solid #bdcbdb;
                border-radius: 4px; padding: 7px; selection-background-color: #2563a9; }
            QPlainTextEdit { font-family: monospace; font-size: 11px; }
            QPushButton { border: 1px solid #aebed2; border-radius: 5px; background: white;
                          padding: 10px 17px; font-weight: 600; }
            QPushButton:hover { background: #e6edf6; }
            QPushButton#primary { background: #1c4e83; color: white; border: 1px solid #1c4e83; }
            QPushButton#primary:hover { background: #286198; }
            QPushButton#stop { color: #a52d36; border-color: #cda9ad; }
            QPushButton#stop:hover { background: #f9e9eb; }
            QPushButton:disabled { background: #e3e7ed; color: #8390a1; border-color: #d2d9e2; }
            QPushButton#primary:disabled, QPushButton#stop:disabled {
                background: #e3e7ed; color: #8390a1; border-color: #d2d9e2; }
        """)
        self.load_hosts()
        self.deadline = QtCore.QTimer(self)
        self.deadline.setSingleShot(True)
        self.deadline.timeout.connect(self.discovery_timeout)
        self.viewer_timer = QtCore.QTimer(self)
        self.viewer_timer.timeout.connect(self.check_viewer)
        self.viewer_timer.start(500)
        self.connection_mode.currentIndexChanged.connect(self.mode_changed)
        self.update_buttons()

    def mode_changed(self):
        self.update_buttons()
        self.subtitle.setText("메인 PC에서 전송하는 실시간 화면을 읽기 전용으로 받습니다." if self.is_spectator() else
                              "보드 주소를 찾거나 직접 입력한 뒤 실시간 화면에 연결합니다.")
        self.status.setText("메인 PC 주소를 입력하고 ‘관전 연결’을 눌러 주세요." if self.is_spectator() else
                            "보드 주소를 입력하거나 ‘보드 찾기’를 눌러 주세요.")

    def is_spectator(self):
        return self.connection_mode.currentData() == "spectator"

    def relay_port_available(self):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as test_socket:
                test_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                test_socket.bind(("0.0.0.0", self.relay_port.value()))
        except OSError:
            self.status.setText("이 PC의 중계 포트를 이미 사용 중입니다. 관전 모드로 연결하거나 포트를 바꿔 주세요.")
            return False
        return True

    def append_log(self, message):
        # Protect against a backend inadvertently echoing the input password.
        secret = self.password.text()
        if secret and len(secret) >= 4:
            message = message.replace(secret, "***")
        elif secret and message.strip() == secret:
            message = "***"
        self.log.appendPlainText(message.rstrip())

    def load_hosts(self):
        try:
            saved = json.loads((STATE_ROOT / "boards.json").read_text())
            pi, jetson = validate_targets(saved["pi"], saved["jetson"])
            self.hosts["pi"].addItem(pi)
            self.hosts["jetson"].addItem(jetson)
        except (OSError, KeyError, ValueError, ImportError, TypeError):
            pass

    def targets(self):
        try:
            return validate_targets(self.hosts["pi"].currentText(), self.hosts["jetson"].currentText())
        except (ValueError, ImportError) as error:
            self.status.setText(str(error))
            return None

    def save_hosts(self, targets):
        path = STATE_ROOT / "boards.json"
        temporary = path.with_suffix(".tmp")
        with temporary.open("w", encoding="utf-8") as stream:
            json.dump({"pi": targets[0], "jetson": targets[1]}, stream, ensure_ascii=False)
            stream.write("\n")
        temporary.chmod(0o600)
        temporary.replace(path)

    def update_buttons(self):
        busy = self.process is not None
        viewer_running = self.viewer is not None and self.viewer.poll() is None
        spectator = self.is_spectator()
        available = not busy and not viewer_running
        self.connection_mode.setEnabled(available)
        self.relay_port.setEnabled(available and not spectator)
        self.relay_host.setEnabled(available and spectator)
        self.relay_port.setVisible(not spectator)
        self.relay_host.setVisible(spectator)
        self.find_button.setEnabled(available and not spectator)
        self.subnet.setEnabled(available and not spectator)
        self.start_button.setEnabled(available and not spectator)
        self.connect_button.setEnabled(not busy and not viewer_running)
        self.connect_button.setText("관전 연결" if spectator else "화면 연결 · 중계")
        self.stop_button.setEnabled(not busy and not spectator and self.viewer_kind != "replay")
        self.replay_button.setEnabled(available)
        for combo in self.hosts.values():
            combo.setEnabled(available and not spectator)
        self.password.setEnabled(available and not spectator)
        self.reveal.setEnabled(available and not spectator)
        self.role_note.setText("관전 PC는 메인 PC 주소만 입력합니다. 보드 접속·모듈 제어는 하지 않습니다." if spectator else
                               "운영 PC 한 대만 보드에 연결합니다. 다른 PC는 실시간 창에 표시되는 중계 주소로 관전합니다.")
        self.cancel_button.setVisible(self.process_kind == "discover")

    def begin_process(self, kind, program, arguments, environment=None):
        self.process_kind = kind
        self.cancel_reason = None
        self.output = ""
        self.pending_text = ""
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        if environment is not None:
            env = QtCore.QProcessEnvironment()
            for name, value in environment.items():
                env.insert(name, str(value))
            self.process.setProcessEnvironment(env)
        self.process.readyReadStandardOutput.connect(self.read_output)
        self.process.finished.connect(self.process_finished)
        self.process.errorOccurred.connect(self.process_error)
        self.process.start(program, arguments)
        self.update_buttons()

    def discover(self):
        if self.process is not None or self.is_spectator():
            return
        if not FINDER.is_file():
            self.status.setText("보드 찾기 스크립트가 없습니다. scripts/find_ssh_boards.sh를 확인해 주세요.")
            return
        args = ["bash", str(FINDER)]
        subnet = self.subnet.text().strip()
        if subnet:
            try:
                network = ipaddress.ip_network(subnet, strict=False)
                if network.version != 4 or network.num_addresses > 4096:
                    raise ValueError()
            except ValueError:
                self.status.setText("IPv4 검색 대역을 /20보다 작거나 같게 입력해 주세요. 예: 192.168.1.0/24")
                return
            args += ["--subnet", str(network)]
        setsid = shutil.which("setsid")
        if not setsid:
            self.status.setText("setsid가 필요합니다. util-linux 설치를 확인해 주세요.")
            return
        self.status.setText("보드를 찾고 있습니다. 최대 30초 후 종료되며 취소할 수 있습니다.")
        self.append_log("[검색] 현재 네트워크의 SSH와 실제 보드 모델을 확인합니다.")
        discovery_env = os.environ.copy()
        discovery_env["NOMAD_BOARD_PASSWORD"] = self.password.text()
        self.begin_process("discover", setsid, args, environment=discovery_env)
        self.deadline.start(30000)

    def read_output(self):
        if self.process is None:
            return
        chunk = bytes(self.process.readAllStandardOutput()).decode("utf-8", errors="replace")
        self.output = (self.output + chunk)[-65536:]
        self.pending_text += chunk
        lines = self.pending_text.split("\n")
        self.pending_text = lines.pop()
        for line in lines:
            if line.strip():
                self.append_log(line)

    def stop_search_process(self, reason):
        if self.process is None or self.process_kind != "discover":
            return
        self.cancel_reason = reason
        process = self.process
        try:
            pid = int(process.processId())
            if pid <= 0:
                raise ValueError("Process is still starting")
            os.killpg(pid, signal.SIGTERM)
        except (OSError, ValueError):
            process.terminate()
        QtCore.QTimer.singleShot(800, lambda: self.force_search_stop(process))

    def force_search_stop(self, process):
        if self.process is process and process.state() != QtCore.QProcess.NotRunning:
            try:
                pid = int(process.processId())
                if pid <= 0:
                    raise ValueError("Process has not started")
                os.killpg(pid, signal.SIGKILL)
            except (OSError, ValueError):
                process.kill()

    def cancel_search(self):
        self.stop_search_process("검색을 취소했습니다. 주소를 직접 입력할 수도 있습니다.")

    def discovery_timeout(self):
        self.stop_search_process("30초 검색을 마쳤습니다. 찾은 주소를 확인하거나 직접 입력해 주세요.")

    def process_error(self, error):
        if error == QtCore.QProcess.FailedToStart:
            self.status.setText("실행 파일을 열 수 없습니다. Python과 앱 파일 구성을 확인해 주세요.")
            self.process_finished(-1, QtCore.QProcess.CrashExit)

    def process_finished(self, exit_code, exit_status):
        if self.process is None:
            return
        self.read_output()
        if self.pending_text.strip():
            self.append_log(self.pending_text)
        kind, output = self.process_kind, self.output
        process = self.process
        self.process = None
        self.process_kind = None
        self.deadline.stop()
        process.deleteLater()
        if kind == "discover":
            found = parse_discovery(output)
            for board, values in found.items():
                if values:
                    previous = self.hosts[board].currentText()
                    self.hosts[board].clear()
                    self.hosts[board].addItems(values)
                    if previous in values:
                        self.hosts[board].setCurrentText(previous)
                    elif len(values) > 1:
                        self.hosts[board].setCurrentIndex(-1)
                        self.hosts[board].lineEdit().clear()
            count = sum(len(values) for values in found.values())
            if self.cancel_reason:
                self.status.setText(self.cancel_reason)
            elif found["pi"] and found["jetson"]:
                self.status.setText("같은 종류의 보드가 여러 개입니다. 사용할 주소를 선택해 주세요." if
                                    any(len(values) > 1 for values in found.values()) else
                                    "Pi5와 Jetson을 찾았습니다. 주소를 확인하고 화면을 연결해 주세요.")
            elif count:
                self.status.setText("일부 보드만 찾았습니다. 누락된 보드 주소를 직접 입력해 주세요.")
            else:
                self.status.setText("보드를 찾지 못했습니다. 같은 네트워크·SSH·보드 전원을 확인해 주세요.")
        elif kind == "start":
            if exit_code == 0 and exit_status == QtCore.QProcess.NormalExit:
                self.append_log("[시작] 기동 명령 완료. 실제 데이터 수신 상태는 실시간 화면에서 확인합니다.")
                self.launch_viewer(self.pending_targets, self.run_dir)
            else:
                self.status.setText("시험 시작이 완료되지 않았습니다. 일부 모듈은 실행 중일 수 있으므로 ‘시험 종료’로 정리할 수 있습니다.")
        elif kind == "stop":
            result = None
            for line in reversed(output.splitlines()):
                try:
                    candidate = json.loads(line)
                except ValueError:
                    continue
                if isinstance(candidate, dict) and isinstance(candidate.get("boards"), dict):
                    result = candidate
                    break
            boards = result.get("boards", {}) if result else {}
            reasons = {"authentication_failed": "인증 실패", "ssh_unreachable": "접속 실패",
                       "timeout": "응답 시간 초과", "remote_cleanup_incomplete": "일부 모듈 잔류",
                       "host_key_check_failed": "SSH 호스트 키 확인 필요"}
            messages = []
            all_stopped = exit_code == 0 and exit_status == QtCore.QProcess.NormalExit
            for key, name in (("pi5", "Pi5"), ("jetson", "Jetson")):
                board = boards.get(key, {})
                stopped = board.get("ok") is True and board.get("state") == "STOPPED"
                all_stopped = all_stopped and stopped
                if stopped:
                    messages.append(name + " 종료 확인")
                else:
                    reason = reasons.get(board.get("reason"), "종료 확인 불가")
                    messages.append(name + " 미확인(" + reason + ")")
            if all_stopped:
                self.status.setText(" · ".join(messages) + ". 보드 전원과 기록은 유지합니다.")
            else:
                self.status.setText(" · ".join(messages) + ". 주소와 연결을 확인한 뒤 종료를 다시 시도해 주세요.")
        self.update_buttons()

    def new_run(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return private_directory(STATE_ROOT / stamp)

    def child_environment(self, run_dir, *, replay=False):
        env = os.environ.copy()
        if self.password.text() and not self.is_spectator() and not replay:
            env["NOMAD_BOARD_PASSWORD1"] = self.password.text()
        else:
            env.pop("NOMAD_BOARD_PASSWORD1", None)
        if self.is_spectator():
            env.pop("NOMAD_BOARD_PASSWORD", None)
        env["NOMAD_TRIAL_STATE_DIR"] = str(run_dir)
        env["NOMAD_VIEWER_STATE_DIR"] = str(private_directory(STATE_ROOT / "viewer"))
        env["MPLCONFIGDIR"] = str(STATE_ROOT / "matplotlib")
        if replay:
            # Playback has no remote access. Do not pass inherited credentials
            # or SSH agent/askpass access into the offline viewer process.
            for name in tuple(env):
                upper = name.upper()
                if (any(part in upper for part in ("PASSWORD", "TOKEN", "SECRET", "CREDENTIAL", "API_KEY", "ACCESS_KEY"))
                        or upper in {"SSH_AUTH_SOCK", "SSH_ASKPASS", "SSH_ASKPASS_REQUIRE"}):
                    env.pop(name, None)
        return env

    def start_trial(self):
        if self.is_spectator():
            return
        targets = self.targets()
        if not targets or self.process is not None or self.viewer is not None:
            return
        if not self.password.text():
            self.status.setText("시험 시작에는 LAN 설정을 위한 보드 비밀번호가 필요합니다. 비밀번호를 입력해 주세요.")
            return
        script = BASE_DIR / "startup.py"
        if not script.is_file():
            self.status.setText("시험 시작 파일이 없습니다. 이미 켜진 모듈은 ‘화면 연결’로 볼 수 있습니다.")
            return
        if not self.relay_port_available():
            return
        self.pending_targets = targets
        self.run_dir = self.new_run()
        self.save_hosts(targets)
        self.status.setText("센서와 모듈을 순서대로 켜고 있습니다. 진행이 끝나면 화면이 열립니다.")
        self.append_log("[시작] " + " / ".join(targets))
        self.begin_process("start", sys.executable,
                           [str(script), "--pi-host", targets[0], "--jetson-host", targets[1]],
                           self.child_environment(self.run_dir))

    def connect_viewer(self):
        if self.is_spectator():
            if self.process is not None or self.viewer is not None:
                return
            try:
                validate_relay_address(self.relay_host.text())
            except ValueError as error:
                self.status.setText(str(error))
                return
            self.launch_viewer(None, self.new_run())
            return
        targets = self.targets()
        if not targets or self.process is not None or self.viewer is not None:
            return
        if not self.relay_port_available():
            return
        self.save_hosts(targets)
        self.launch_viewer(targets, self.new_run())

    def stop_trial(self):
        if self.process is not None or self.is_spectator() or self.viewer_kind == "replay":
            return
        targets = self.targets()
        if not targets:
            return
        script = BASE_DIR / "control.py"
        if not script.is_file():
            self.status.setText("종료 제어 파일이 없습니다. 앱의 control.py 구성을 확인해 주세요.")
            return
        run_dir = self.run_dir or self.new_run()
        self.run_dir = run_dir
        self.save_hosts(targets)
        self.status.setText("두 보드의 센서·모듈을 종료하고 있습니다. 보드 전원은 유지합니다.")
        self.append_log("[종료] " + " / ".join(targets))
        self.begin_process("stop", sys.executable,
                           [str(script), "stop", "--pi-host", targets[0], "--jetson-host", targets[1],
                            "--state-dir", str(run_dir)], self.child_environment(run_dir))

    def monitor_arguments(self, targets, run_dir):
        if self.is_spectator():
            return [sys.executable, str(BASE_DIR / "monitor.py"),
                    "--relay-host", validate_relay_address(self.relay_host.text()),
                    "--preview-heading-calibration", str(BASE_DIR.parent / "config/oak_basalt_calib.json"),
                    "--state-file", str(run_dir / "live_state.json")]
        return [sys.executable, str(BASE_DIR / "monitor.py"),
                "--relay-listen", str(self.relay_port.value()),
                "--pi-host", targets[0], "--jetson-host", targets[1],
                "--pi-command", "bash /home/user/nomad_ws/tools/visual_trial_20261106/run_monitor.sh",
                "--jetson-command", "bash /home/nvidia/nomad_ws/tools/visual_trial_20261106/run_monitor.sh",
                "--preview-heading-calibration", str(BASE_DIR.parent / "config/oak_basalt_calib.json"),
                "--enable-trial-control", "--state-file", str(run_dir / "live_state.json"),
                "--ssh-progress-file", str(run_dir / "live_ssh.json")]

    def open_recording(self):
        if self.process is not None or self.viewer is not None:
            return
        filename, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "NOMAD 시험 기록 열기", str(STATE_ROOT), "NOMAD 시험 기록 (*.nomad)")
        if filename:
            self.launch_replay(Path(filename))

    def launch_replay(self, recording):
        if self.process is not None or self.viewer is not None:
            return
        try:
            recording = Path(recording).expanduser().resolve(strict=True)
            if not recording.is_file():
                raise ValueError("시험 기록 파일을 선택해 주세요.")
            run_dir = self.new_run()
            arguments = [sys.executable, str(BASE_DIR / "monitor.py"), "--replay", str(recording)]
            with (run_dir / "monitor.log").open("a", encoding="utf-8") as log:
                self.viewer = subprocess.Popen(
                    arguments, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                    env=self.child_environment(run_dir, replay=True), start_new_session=True,
                    cwd=str(BASE_DIR))
            self.viewer_kind = "replay"
            self.run_dir = run_dir
            self.status.setText("저장한 시험 기록을 재생합니다. 보드 연결이나 구동 명령은 보내지 않습니다.")
            self.append_log("[재생] " + str(recording))
            self.append_log("[재생] 실행 기록: " + str(run_dir))
        except (OSError, ValueError) as error:
            self.viewer = None
            self.viewer_kind = None
            self.status.setText("시험 기록을 열지 못했습니다: " + str(error))
        self.update_buttons()

    def launch_viewer(self, targets, run_dir):
        calibration = BASE_DIR.parent / "config/oak_basalt_calib.json"
        if not calibration.is_file():
            self.status.setText("설정 파일이 없습니다. 앱 상위 config/oak_basalt_calib.json을 확인해 주세요.")
            return
        try:
            with (run_dir / "monitor.log").open("a", encoding="utf-8") as log:
                self.viewer = subprocess.Popen(self.monitor_arguments(targets, run_dir),
                                               stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                                               env=self.child_environment(run_dir), start_new_session=True,
                                               cwd=str(BASE_DIR))
            self.viewer_kind = "live"
            self.run_dir = run_dir
            self.status.setText("관전 창을 열었습니다. 보드 제어 없이 메인 PC에서 화면을 받습니다." if self.is_spectator() else
                                "운영 창을 열었습니다. 다른 PC에는 실시간 창의 중계 주소를 알려 주세요.")
            self.append_log("[화면] 기록: " + str(run_dir))
        except OSError as error:
            self.viewer = None
            self.viewer_kind = None
            self.status.setText("실시간 창을 열지 못했습니다: " + str(error))
        self.update_buttons()

    def check_viewer(self):
        if self.viewer is not None and self.viewer.poll() is not None:
            code = self.viewer.returncode
            replay = self.viewer_kind == "replay"
            self.viewer = None
            self.viewer_kind = None
            if replay:
                self.status.setText("재생 창이 닫혔습니다. 다른 기록을 열 수 있습니다." if code == 0 else
                                    "재생 창이 종료되었습니다. 기록 폴더의 monitor.log를 확인해 주세요.")
            else:
                self.status.setText("실시간 창이 닫혔습니다. 다시 연결할 수 있습니다." if code == 0 else
                                    "실시간 창이 종료되었습니다. 기록 폴더의 monitor.log를 확인해 주세요.")
            self.update_buttons()

    def closeEvent(self, event):
        if self.process_kind in ("start", "stop") and self.process is not None:
            action = "시작" if self.process_kind == "start" else "종료"
            self.status.setText("시험 " + action + " 작업이 진행 중입니다. 완료 후 창을 닫아 주세요.")
            event.ignore()
            return
        if self.process_kind == "discover":
            self.cancel_search()
            if self.process is not None:
                self.process.waitForFinished(1300)
                if self.process is not None:
                    self.force_search_stop(self.process)
                    self.process.waitForFinished(500)
        # Closing the launcher leaves the separate monitor and board modules on.
        event.accept()


def main():
    parser = argparse.ArgumentParser(description="NOMAD Linux 시험 연결 앱")
    parser.add_argument("--quit-after-s", type=float, default=0.0, help="로컬 UI 검사 전용; 보드에 연결하지 않음")
    parser.add_argument("--snapshot", type=Path, help="로컬 UI 검사 화면 저장")
    args = parser.parse_args()
    app = QtWidgets.QApplication(sys.argv[:1])
    app.setApplicationName("NOMAD Trial")
    window = Launcher()
    window.show()
    if args.snapshot:
        def capture():
            args.snapshot.parent.mkdir(parents=True, exist_ok=True)
            window.grab().save(str(args.snapshot))
        QtCore.QTimer.singleShot(300, capture)
    if args.quit_after_s > 0:
        QtCore.QTimer.singleShot(int(args.quit_after_s * 1000), app.quit)
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
