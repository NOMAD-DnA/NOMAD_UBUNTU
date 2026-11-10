"""Laptop-only packet recording and offline playback controls for the same UI."""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from session_recording import SessionReader, SessionRecorder


def timestamp_label(seconds):
    seconds = max(0., float(seconds))
    minutes, seconds = divmod(seconds, 60.)
    hours, minutes = divmod(int(minutes), 60)
    return f"{hours:02}:{minutes:02}:{seconds:04.1f}"


class RecordingMixin:
    """All playback input uses the existing validation/display path, never SSH."""

    def init_recording(self, args):
        self.replay_mode = bool(getattr(args, "replay", None))
        self.recorder = None
        self.recording_result = None
        self._record_started = 0.
        self._packet_cache = {}
        self._last_view_json = None
        self._replay_windows = []
        self.reader = None
        self._replay_position = 0.
        self._replay_seq = 0
        self._replay_playing = False
        self._replay_seeking = False
        self._replay_seek_target = 0.
        self._replay_error = None
        self._replay_restore_windows = []
        self._replay_resume_after_seek = False
        self._replay_wall_anchor = time.monotonic()
        self._replay_position_anchor = 0.
        if self.replay_mode:
            # Enforce this here as well as in parse_args: direct callers and
            # developer windows cannot accidentally construct a board client.
            args.no_ssh = True
            args.relay_host = None
            args.relay_listen = None
            args.enable_trial_control = False
            args.preview_heading_calibration = None
            self.reader = SessionReader(Path(args.replay).expanduser().resolve())

    def now(self):
        return 1_000_000. + self._replay_position if self.replay_mode else time.monotonic()

    def wall_now(self):
        if self.replay_mode:
            return float(self.reader.metadata.get("started_wall_s", 0.)) + self._replay_position
        return time.time()

    def setup_recording_bar(self, layout):
        bar = QtWidgets.QHBoxLayout()
        if not self.replay_mode:
            self.record_button = QtWidgets.QPushButton("기록 시작")
            self.record_button.setToolTip("이미 받은 영상·지도·궤적·상태·로그를 이 PC에 저장합니다.")
            self.record_button.clicked.connect(self.toggle_recording)
            bar.addWidget(self.record_button)
            self.open_record_button = QtWidgets.QPushButton("기록 열기")
            self.open_record_button.clicked.connect(self.open_recording_file)
            bar.addWidget(self.open_record_button)
            folder_button = QtWidgets.QPushButton("기록 폴더")
            folder_button.clicked.connect(self.open_recording_folder)
            bar.addWidget(folder_button)
            self.record_label = QtWidgets.QLabel("기록 꺼짐 · 이 PC에 수신된 데이터 저장")
            self.record_label.setWordWrap(True)
            bar.addWidget(self.record_label, 1)
        else:
            self.replay_play = QtWidgets.QPushButton("재생")
            self.replay_play.clicked.connect(self.toggle_replay)
            bar.addWidget(self.replay_play)
            reset = QtWidgets.QPushButton("처음으로")
            reset.clicked.connect(lambda: self.seek_replay(0.))
            bar.addWidget(reset)
            self.replay_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
            self.replay_slider.setRange(0, min(2_147_483_647, math.ceil(self.reader.duration * 1000)))
            self.replay_slider.sliderPressed.connect(self.pause_replay)
            self.replay_slider.sliderReleased.connect(lambda: self.seek_replay(self.replay_slider.value() / 1000.))
            self.replay_slider.sliderMoved.connect(lambda value: self.replay_time.setText(
                timestamp_label(value / 1000.) + " / " + timestamp_label(self.reader.duration)))
            self.replay_slider.setAccessibleName("기록 재생 시각")
            bar.addWidget(self.replay_slider, 1)
            self.replay_time = QtWidgets.QLabel()
            bar.addWidget(self.replay_time)
            self.replay_speed = QtWidgets.QComboBox()
            for speed in (.5, 1., 2., 4.):
                self.replay_speed.addItem(f"{speed:g}배", speed)
            self.replay_speed.setCurrentIndex(1)
            self.replay_speed.currentIndexChanged.connect(self.reset_replay_anchor)
            bar.addWidget(self.replay_speed)
            self.record_label = QtWidgets.QLabel()
            self.record_label.setWordWrap(True)
        self.record_label.setTextFormat(QtCore.Qt.PlainText)
        self.record_label.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Preferred)
        layout.addLayout(bar)
        if self.replay_mode:
            layout.addWidget(self.record_label)

    def _view_event(self):
        # An explicit allowlist: no Namespace, environment, passwords, commands,
        # or process settings are ever included in the recording metadata.
        cpu_history = {}
        for mode, values in getattr(self, "slope_cpu_history", {}).items():
            entry = dict(values)
            received = entry.pop("received", entry.pop("received_monotonic", None))
            if isinstance(received, (int, float)) and math.isfinite(received):
                entry["_record_received_offset_s"] = received - self._record_started
            cpu_history[mode] = entry
        return {"kind": "view", "source": "gui", "selected_goal": self.selected_goal,
                "active_model": self.active_model, "model_switch_result": self.model_switch_result,
                "active_cloud_source": getattr(self, "active_cloud_source", None),
                "active_slope_mode": getattr(self, "active_slope_mode", None),
                "slope_cpu_history": cpu_history,
                "trial_stop_result": self.trial_stop_result, "ssh": self.ssh_progress,
                "heading_reset_result": getattr(self, "heading_reset_result", None),
                "vio_restart_result": getattr(self, "vio_restart_result", None)}

    def record_view(self):
        if self.recorder is None or self.replay_mode:
            return
        event = self._view_event()
        encoded = json.dumps(event, sort_keys=True, ensure_ascii=False)
        if encoded != self._last_view_json:
            self._last_view_json = encoded
            self.recorder.append("gui", event, max(0., time.monotonic() - self._record_started))

    def record_packet(self, board, event, received_at):
        if self.replay_mode:
            return
        raw = dict(event)
        raw["_record_receive_age_s"] = max(0., self.now() - received_at)
        kind, source = raw["kind"], raw["source"]
        if kind != "log":
            self._packet_cache[(board, kind, source)] = raw
        if self.recorder is not None:
            self.recorder.append(board, raw, max(0., time.monotonic() - self._record_started))

    def toggle_recording(self):
        if self.replay_mode:
            return
        if self.recorder is not None:
            self.stop_recording()
            return
        root = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial/recordings"
        destination = root / (time.strftime("%Y%m%d_%H%M%S") + f"_{time.time_ns() % 1_000_000:06d}.nomad")
        self._record_started = time.monotonic()
        try:
            recorder = SessionRecorder(destination, metadata={
                "started_wall_s": time.time(), "forward_axis": list(self.forward_axis),
                "heading_note": self.heading_note, "ui_hz": self.args.ui_hz,
                "active_model": self.active_model,
                "active_cloud_source": getattr(self, "active_cloud_source", None),
                "active_slope_mode": getattr(self, "active_slope_mode", None),
                "scope": "accepted_gui_packets_after_ingress_coalescing",
                "initial_history": "bounded history already visible at recording start; offset zero",
                "streams": "images,poses,status,rates,telemetry,module logs,selected goal",
            })
        except (OSError, ValueError, RuntimeError) as error:
            self.record_label.setText("기록 시작 실패 · " + str(error))
            self.record_label.setStyleSheet("color:#fca5a5;")
            return
        self.recorder = recorder
        self.recording_result = None
        self._last_view_json = None
        # Preserve the currently visible bounded trajectory once at time zero.
        # This is a baseline, not an invented chronology before recording start.
        owners = {"vio": "pi", "fusion": "pi", "public": "pi"}
        for source, history in self.histories.items():
            cached = self._packet_cache.get(("pi", "pose", source), {})
            age = self.age("pose", source) or 0.
            for pose in history:
                event = {**cached, **pose, "kind": "pose", "source": source,
                         "_record_receive_age_s": age}
                recorder.append(owners[source], event, 0.)
        for (board, kind, source), cached in self._packet_cache.items():
            if kind == "pose":
                continue
            recorder.append(board, {**cached, "_record_receive_age_s": self.age(kind, source) or 0.}, 0.)
        for history in self.module_logs.values():
            for event in history:
                source = event["source"]
                recorder.append(event["board"], {**event, "_record_receive_age_s": self.age("log", source) or 0.}, 0.)
        self.record_view()
        self.record_button.setText("기록 정지")
        self.update_recording_bar()

    def stop_recording(self):
        if self.recorder is None:
            return
        self.record_view()
        self.recording_result = self.recorder.close(duration_s=max(0., time.monotonic() - self._record_started))
        self.recorder = None
        self.record_button.setText("기록 시작")
        self.update_recording_bar()

    def update_recording_bar(self):
        if self.replay_mode:
            if self._replay_error:
                self.record_label.setText("기록 재생 오류 · " + self._replay_error)
                self.record_label.setStyleSheet("color:#fca5a5;")
                return
            if not self.replay_slider.isSliderDown():
                self.replay_slider.setValue(round(self._replay_position * 1000))
                self.replay_time.setText(timestamp_label(self._replay_position) + " / " + timestamp_label(self.reader.duration))
            self.replay_play.setText("일시정지" if self._replay_playing else "재생")
            incomplete = not self.reader.metadata.get("_recording", {}).get("complete", False)
            state = "시각 이동 중…" if self._replay_seeking else ("재생 중" if self._replay_playing else "일시정지")
            self.record_label.setText(f"기록 재생 · {state} · {Path(self.args.replay).name}" +
                                      (" · 미완료 기록: 저장된 부분까지 재생" if incomplete else ""))
            self.record_label.setStyleSheet("color:#7dd3fc;")
            return
        status = self.recorder.status() if self.recorder is not None else self.recording_result
        if not status:
            return
        failed = status.get("state") == "FAILED" or bool(status.get("error"))
        label = "기록 오류 · 누락 없는 저장 미확인" if failed else ("기록 중" if self.recorder else "저장 완료")
        self.record_label.setText(f"{label} · {status.get('events', 0):,}개 · {status.get('path', '')}" +
                                  (" · " + str(status["error"]) if failed else ""))
        self.record_label.setToolTip(str(status.get("path", "")))
        self.record_label.setStyleSheet("color:#fca5a5;" if failed else "color:#7dd3fc;")

    def open_recording_file(self):
        root = str(Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial/recordings")
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "주행 기록 열기", root, "NOMAD 주행 기록 (*.nomad)")
        if not path:
            return
        args = argparse.Namespace(**vars(self.args))
        args.replay = path
        args.state_file = args.ssh_progress_file = args.snapshot = None
        args.quit_after_s = 0.
        args.open_developer_window = []
        try:
            window = type(self)(args)
        except (OSError, ValueError, RuntimeError) as error:
            QtWidgets.QMessageBox.warning(self, "기록 열기 실패", str(error))
            return
        self._replay_windows.append(window)
        window.show()

    def open_recording_folder(self):
        status = self.recorder.status() if self.recorder is not None else self.recording_result
        path = Path(status["path"]).parent if status else (
            Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial/recordings")
        path.mkdir(parents=True, exist_ok=True)
        QtGui.QDesktopServices.openUrl(QtCore.QUrl.fromLocalFile(str(path)))

    def reset_replay_anchor(self, *_):
        self._replay_wall_anchor = time.monotonic()
        self._replay_position_anchor = self._replay_position

    def pause_replay(self):
        self._replay_playing = False
        self._replay_resume_after_seek = False
        self.update_recording_bar()

    def toggle_replay(self):
        if self._replay_seeking:
            return
        if self._replay_position >= self.reader.duration and not self._replay_playing:
            self.seek_replay(0., resume=True)
            return
        self._replay_playing = not self._replay_playing
        self.reset_replay_anchor()
        self.update_recording_bar()

    def _apply_replay_record(self, record):
        self._replay_seq = record["seq"]
        self._replay_position = record["offset_s"]
        event = dict(record["event"])
        if record["board"] == "gui" and event.get("kind") == "view":
            goal = event.get("selected_goal")
            self.selected_goal = tuple(goal) if isinstance(goal, list) and len(goal) == 3 else None
            self.active_model = event.get("active_model")
            self.model_switch_result = event.get("model_switch_result")
            self.trial_stop_result = event.get("trial_stop_result")
            self.active_cloud_source = event.get("active_cloud_source")
            self.active_slope_mode = event.get("active_slope_mode")
            self.slope_cpu_history = {}
            for mode, values in event.get("slope_cpu_history", {}).items():
                entry = dict(values)
                offset = entry.pop("_record_received_offset_s", None)
                if isinstance(offset, (int, float)) and math.isfinite(offset):
                    entry["received"] = 1_000_000. + offset
                self.slope_cpu_history[mode] = entry
            self.heading_reset_result = event.get("heading_reset_result")
            restart_result = event.get("vio_restart_result")
            if (isinstance(restart_result, dict) and restart_result.get("ok") is True and
                    restart_result != self.vio_restart_result):
                self.clear_raw_vio_history()
            self.vio_restart_result = restart_result
            ssh = event.get("ssh")
            if isinstance(ssh, dict):
                self.ssh_progress = ssh
            for panel in self.panels.values():
                panel.set_planning(self.telemetry.get("planning", {}), self.selected_goal)
            return
        age = event.get("_record_receive_age_s", 0.)
        if not isinstance(age, (int, float)) or not math.isfinite(age) or age < 0:
            raise ValueError("기록의 수신 시각이 유효하지 않습니다")
        event["_local_received_monotonic"] = self.now() - age
        event["_relay_age_s"] = age
        self.on_packet(record["board"], event)
        if event.get("kind") == "image":
            # A later malformed/unlocated preview must retain the earlier valid
            # frame just as live refresh did, even when seeking skips paints.
            self.decode_images((event.get("source"),))

    def seek_replay(self, seconds, resume=False):
        if self._replay_error:
            return
        self.pause_replay()
        self._replay_resume_after_seek = resume
        self._replay_seek_target = max(0., min(self.reader.duration, float(seconds)))
        if not self._replay_seeking:
            self._replay_restore_windows = [(part, window.isVisible(), window.saveGeometry())
                                            for part, window in self.developer_windows.items()]
            for window in self.developer_windows.values():
                window.close()
                window.deleteLater()
            self.developer_windows.clear()
        self._replay_seeking = True
        self._replay_seq = 0
        self._replay_position = 0.
        self.reset_replay_data()
        self.update_recording_bar()
        self.advance_replay()

    def reset_replay_data(self):
        from relay import CaptureOrder
        from module_logs import LogHistory
        from render_utils import HistorySampler
        for history in self.histories.values():
            history.clear()
        self._plot_samplers = {key: HistorySampler(4000) for key in self.histories}
        self._plot_signatures.clear()
        self._fit_initialized = False
        self._decoded_payloads.clear()
        self.received.clear()
        self.statuses.clear()
        self.rates.clear()
        self.images.clear()
        self.latest_qimages.clear()
        self.telemetry.clear()
        self._comparison_epoch = None
        self._raw_vio_epoch = None
        self.heading_reset_result = None
        self.vio_restart_result = None
        self.module_logs = {part: LogHistory(128) for part in self.module_logs}
        self.image_dirty.clear()
        self.capture_order = CaptureOrder()
        self.accepted_events = self.invalid_events = 0
        self.selected_goal = None
        self.active_model = self.reader.metadata.get("active_model")
        self.active_cloud_source = self.reader.metadata.get("active_cloud_source")
        self.active_slope_mode = self.reader.metadata.get("active_slope_mode")
        self.cloud_switch_result = self.slope_switch_result = None
        self.slope_cpu_history = {}
        self.trial_stop_result = self.model_switch_result = None
        self.ssh_progress = {board: {"phase": "recording"} for board in ("pi", "jetson")}
        self.planning_message.setText("기록 재생 · 이 시각의 플래너 데이터 대기")
        self.plot_dirty = True
        for source in self.lines:
            self.lines[source].set_data_3d([], [], [])
            self.markers[source].set_data_3d([], [], [])
        for panel in self.panels.values():
            panel.clear_data()

    def advance_replay(self):
        if not self.replay_mode or self._replay_error:
            return
        target = self._replay_seek_target if self._replay_seeking else self._replay_position
        if self._replay_playing:
            speed = float(self.replay_speed.currentData())
            target = min(self.reader.duration, self._replay_position_anchor +
                         (time.monotonic() - self._replay_wall_anchor) * speed)
        try:
            # Bounded chunks keep seeking responsive and never queue one Qt
            # signal per frame. Images update retained buffers; painting remains
            # on the existing UI timer, including during a long seek.
            deadline = time.monotonic() + .012
            while True:
                rows = self.reader.events_until(target, after_seq=self._replay_seq, limit=128)
                for record in rows:
                    self._apply_replay_record(record)
                if not rows:
                    self._replay_position = target
                    if self._replay_seeking:
                        self._replay_seeking = False
                        for part, visible, geometry in self._replay_restore_windows:
                            if visible:
                                self.open_developer_window(part)
                                self.developer_windows[part].restoreGeometry(geometry)
                        self._replay_restore_windows = []
                        if self._replay_resume_after_seek:
                            self._replay_playing = True
                            self._replay_resume_after_seek = False
                            self.reset_replay_anchor()
                    if target >= self.reader.duration:
                        self._replay_playing = False
                    break
                if time.monotonic() >= deadline:
                    break
        except (OSError, ValueError, RuntimeError) as error:
            self._replay_playing = self._replay_seeking = False
            self._replay_error = str(error)
            self.record_label.setText("기록 재생 오류 · " + str(error))
            self.record_label.setStyleSheet("color:#fca5a5;")
            self.replay_play.setEnabled(False)
            self.replay_slider.setEnabled(False)
            return
        self.update_recording_bar()

    def close_recording(self):
        self.stop_recording()
        if self.reader is not None:
            self.reader.close()

