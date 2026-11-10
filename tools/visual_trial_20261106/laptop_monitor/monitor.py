#!/usr/bin/env python3
"""Laptop-only viewer for real JSONL output from two read-only collectors."""
from __future__ import annotations

import argparse
import base64
from collections import deque
import json
import math
import os
from pathlib import Path
from session_ui import RecordingMixin
import selectors
import signal
import socket
import subprocess
import sys
import tempfile
import time
import numpy as np

BASE_DIR = Path(__file__).resolve().parent
RUNTIME_DIR = Path(os.environ.get("NOMAD_VIEWER_STATE_DIR", str(BASE_DIR))).expanduser().resolve()
RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("MPLCONFIGDIR", str(RUNTIME_DIR / ".mplconfig"))

from PyQt5 import QtCore, QtGui, QtNetwork, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from matplotlib.transforms import Affine2D
from heading_indicator import HeadingIndicator
from localization_state import localization_summary, comparison_samples, flatten as localization_fields
from relay import CaptureOrder, LatestPacketBuffer, owns_stream
from module_logs import LogHistory
from render_utils import HistorySampler, history_signature, fit_axes_3d, stable_metadata


POSE_SOURCES = ("vio", "fusion", "public")
IMAGE_SOURCES = ("rgb", "semantic", "slope", "costmap", "vio_tracking")
TELEMETRY_SOURCES = ("gnss", "fusion", "vio_features", "magnetometer", "planning", "slope_cpu")
MAX_POSES = 20000
MAX_LINE_BYTES = 16 * 1024 * 1024


def finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def relay_endpoint(value):
    """Parse the relay backend's IPv4/hostname TCP address."""
    value = str(value).strip()
    host, separator, port = value.rpartition(":")
    if ":" in host or "[" in host or "]" in host:
        raise ValueError("use an IPv4 address or hostname")
    if not separator or not host or any(c.isspace() for c in host):
        raise ValueError("use HOST:PORT")
    try:
        port = int(port)
    except ValueError:
        raise ValueError("port must be an integer") from None
    if not 1 <= port <= 65535:
        raise ValueError("port must be between 1 and 65535")
    return host, port


def atomic_json(path, value):
    path = Path(path).expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(temporary, path)


class Collector(QtCore.QThread):
    progress = QtCore.pyqtSignal(str, dict)

    def __init__(self, board, host, command, parent=None):
        super().__init__(parent)
        self.board, self.host, self.command = board, host, command
        self.process = None
        self.inbox = LatestPacketBuffer()
        self.stats = {"phase": "pending", "bytes": 0, "lines": 0,
                      "json_events": 0, "invalid_lines": 0, "exit_code": None,
                      "error_code": None, "last_event_wall_s": None}

    def emit_progress(self, phase=None):
        if phase is not None:
            self.stats["phase"] = phase
        self.progress.emit(self.board, dict(self.stats))

    def stop(self):
        self.requestInterruption()

    def run(self):
        # Only the local SSH process group is stopped. No board module controls
        # or remote kill commands are issued by this viewer.
        runtime = tempfile.TemporaryDirectory(prefix=".ssh-runtime-", dir=RUNTIME_DIR)
        selector = selectors.DefaultSelector()
        buffer = b""
        oversized = False
        stderr_tail = b""
        try:
            env = os.environ.copy()
            password_present = bool(env.get("NOMAD_BOARD_PASSWORD1"))
            if password_present:
                askpass = Path(runtime.name) / "askpass"
                askpass.write_text('#!/bin/sh\nprintf \'%s\\n\' "$NOMAD_BOARD_PASSWORD1"\n', encoding="utf-8")
                askpass.chmod(0o700)
                env.update(SSH_ASKPASS=str(askpass), SSH_ASKPASS_REQUIRE="force")
                env.setdefault("DISPLAY", ":0")
            args = ["ssh", "-T", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=5",
                    "-o", "ServerAliveCountMax=2", "-o", "NumberOfPasswordPrompts=1",
                    "-o", "StrictHostKeyChecking=accept-new", "-o",
                    'UserKnownHostsFile="' + str(RUNTIME_DIR / "known_hosts") + '"',
                    "-o", "LogLevel=ERROR", "-o", "BatchMode=" + ("no" if password_present else "yes"),
                    self.host, self.command]
            self.emit_progress("connecting")
            # monitor_probe treats stdin EOF as a request to leave rclcpp::spin.
            # Keep the pipe's write end alive for this collector's whole life.
            self.process = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                            stderr=subprocess.PIPE, env=env, start_new_session=True)
            selector.register(self.process.stdout, selectors.EVENT_READ, "stdout")
            selector.register(self.process.stderr, selectors.EVENT_READ, "stderr")
            last_progress = 0.0
            while selector.get_map() and not self.isInterruptionRequested():
                for key, _ in selector.select(timeout=0.15):
                    chunk = os.read(key.fileobj.fileno(), 65536)
                    if not chunk:
                        selector.unregister(key.fileobj)
                        continue
                    if key.data == "stderr":
                        # Raw stderr, remote commands and environment values are
                        # deliberately absent from diagnostics and saved state.
                        stderr_tail = (stderr_tail + chunk)[-4096:]
                        continue
                    self.stats["bytes"] += len(chunk)
                    self.stats["phase"] = "receiving"
                    buffer += chunk
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        self.stats["lines"] += 1
                        if oversized or len(line) > MAX_LINE_BYTES:
                            self.stats["invalid_lines"] += 1
                            oversized = False
                            continue
                        try:
                            event = json.loads(line)
                            if not isinstance(event, dict) or event.get("kind") not in ("pose", "image", "status", "rates", "telemetry"):
                                raise ValueError("Unsupported event")
                        except (ValueError, UnicodeDecodeError):
                            self.stats["invalid_lines"] += 1
                            continue
                        self.stats["json_events"] += 1
                        self.stats["last_event_wall_s"] = time.time()
                        self.inbox.put(self.board, event)
                    if len(buffer) > MAX_LINE_BYTES:
                        buffer = b""
                        oversized = True
                if time.monotonic() - last_progress >= 0.5:
                    self.emit_progress()
                    last_progress = time.monotonic()
            if self.isInterruptionRequested():
                self.emit_progress("stopping_collector_ssh")
            else:
                self.stats["exit_code"] = self.process.poll()
                lowered = stderr_tail.lower()
                if b"permission denied" in lowered:
                    self.stats["error_code"] = "authentication_failed"
                elif b"host key verification failed" in lowered or b"host identification has changed" in lowered:
                    self.stats["error_code"] = "host_key_check_failed"
                elif b"connection timed out" in lowered or b"no route to host" in lowered:
                    self.stats["error_code"] = "network_unavailable"
                elif b"not found" in lowered or b"no such file" in lowered:
                    self.stats["error_code"] = "collector_not_found"
                elif stderr_tail:
                    self.stats["error_code"] = "ssh_or_collector_error"
        except (OSError, ValueError):
            self.stats["error_code"] = "local_ssh_start_or_read_failed"
        finally:
            if self.process is not None:
                if self.process.stdin and not self.process.stdin.closed:
                    self.process.stdin.close()
                if self.process.poll() is None:
                    try:
                        self.process.wait(timeout=0.8)
                    except subprocess.TimeoutExpired:
                        try:
                            os.killpg(self.process.pid, signal.SIGTERM)
                            self.process.wait(timeout=1.5)
                        except subprocess.TimeoutExpired:
                            os.killpg(self.process.pid, signal.SIGKILL)
                            self.process.wait(timeout=1)
                        except ProcessLookupError:
                            pass
                    except ProcessLookupError:
                        pass
                self.stats["exit_code"] = self.process.returncode
                for stream in (self.process.stdout, self.process.stderr):
                    if stream:
                        stream.close()
            selector.close()
            runtime.cleanup()
            self.emit_progress("closed" if self.isInterruptionRequested() else "ended")


class RelayReceiver(QtCore.QThread):
    """Spectator TCP receiver; it never opens SSH or controls a board."""
    progress = QtCore.pyqtSignal(dict)

    def __init__(self, host, port, parent=None):
        super().__init__(parent)
        from relay import RelayClient
        self.inbox = LatestPacketBuffer()
        self.client = RelayClient(host, port, self.inbox.put, self._progress)

    def _progress(self, event):
        if event.get("state") == "connected":
            self.inbox.new_session()
        self.progress.emit(event)

    def stop(self):
        self.requestInterruption()
        self.client.stop()

    def run(self):
        if self.isInterruptionRequested():
            return
        try:
            self.client.run()
        except Exception:
            # Do not include a raw exception, address credentials or environment.
            self.progress.emit({"state": "disconnected", "error": "relay_receiver_failed"})


class TrialStopWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str)
    result = QtCore.pyqtSignal(dict)

    def __init__(self, pi_host, jetson_host, password, parent=None):
        super().__init__(parent)
        self.pi_host, self.jetson_host, self.password = pi_host, jetson_host, password

    def run(self):
        try:
            from control import stop_boards
            def progress(board, result):
                name = "Pi5" if board == "pi5" else "Jetson"
                self.progress.emit(name + (" 종료 확인" if result.get("ok") else " 종료 미확인: " + str(result.get("reason", ""))))
            value = stop_boards(self.pi_host, self.jetson_host, self.password,
                                progress_callback=progress)
        except Exception as error:
            value = {"ok": False, "state": "FAILED", "reason": type(error).__name__, "boards": {}}
        finally:
            self.password = ""
        self.result.emit(value)


class ModelSwitchWorker(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, jetson_host, model, password, parent=None):
        super().__init__(parent)
        self.jetson_host, self.model, self.password = jetson_host, model, password

    def run(self):
        try:
            from model_control import switch_model
            value = switch_model(self.jetson_host, self.model, self.password)
        except Exception:
            value = {"ok": False, "state": "UNCONFIRMED", "model": None, "reason": "local_control_error"}
        finally:
            self.password = ""
        self.result.emit(value)


class CloudSwitchWorker(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, jetson_host, source, password, parent=None):
        super().__init__(parent)
        self.jetson_host, self.source, self.password = jetson_host, source, password

    def run(self):
        try:
            from cloud_control import switch_cloud_source
            value = switch_cloud_source(self.jetson_host, self.source, self.password)
        except Exception:
            value = {"ok": False, "state": "UNCONFIRMED", "source": None, "reason": "local_control_error"}
        finally:
            self.password = ""
        self.result.emit(value)


class SlopeSwitchWorker(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, host, mode, password, parent=None):
        super().__init__(parent)
        self.host, self.mode, self.password = host, mode, password

    def run(self):
        try:
            from slope_control import switch_slope_mode
            value = switch_slope_mode(self.host, self.mode, self.password)
        except Exception:
            value = {"ok": False, "state": "UNCONFIRMED", "mode": None, "reason": "local_control_error"}
        finally:
            self.password = ""
        self.result.emit(value)


class HeadingResetWorker(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, host, password, parent=None):
        super().__init__(parent)
        self.host, self.password = host, password

    def run(self):
        try:
            from heading_control import reset_heading
            value = reset_heading(self.host, self.password)
        except Exception:
            value = {"ok": False, "state": "UNCONFIRMED", "reason": "local_control_error"}
        finally:
            self.password = ""
        self.result.emit(value)


class VioRestartWorker(QtCore.QThread):
    result = QtCore.pyqtSignal(dict)

    def __init__(self, host, password, parent=None):
        super().__init__(parent)
        self.host, self.password = host, password

    def run(self):
        try:
            from vio_control import restart_vio
            value = restart_vio(self.host, self.password)
        except Exception:
            value = {"ok": False, "state": "VIO_RESTART_FAILED", "reason": "local_control_error"}
        finally:
            self.password = ""
        self.result.emit(value)


class BufferedImageLabel(QtWidgets.QLabel):
    """Paint the retained frame without QLabel pixmap size-hint relayouts."""
    def __init__(self, text=""):
        super().__init__(text)
        self._retained_pixmap = None
        self.setAttribute(QtCore.Qt.WA_OpaquePaintEvent, True)

    def setPixmap(self, pixmap):
        if pixmap is None or pixmap.isNull():
            return
        if self._retained_pixmap is not None and self._retained_pixmap.cacheKey() == pixmap.cacheKey():
            return
        self._retained_pixmap = pixmap
        self.update()

    def pixmap(self):
        return self._retained_pixmap

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor("#0b1220"))
        pixmap = self._retained_pixmap
        if pixmap is not None and not pixmap.isNull():
            painter.drawPixmap((self.width()-pixmap.width())//2,
                               (self.height()-pixmap.height())//2, pixmap)
        else:
            painter.setPen(QtGui.QColor("#94a3b8"))
            painter.drawText(self.rect().adjusted(8, 8, -8, -8),
                             QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.text())
        painter.setPen(QtGui.QColor("#334155"))
        painter.drawRect(self.rect().adjusted(0, 0, -1, -1))


class ImagePanel(QtWidgets.QWidget):
    def __init__(self, title):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet("font-weight:600; color:#e2e8f0;")
        self.display = BufferedImageLabel("WAITING — no image received")
        self.display.setAlignment(QtCore.Qt.AlignCenter)
        self.display.setMinimumSize(140, 110)
        self.display.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.display.setStyleSheet("background:#0b1220; border:1px solid #334155; color:#94a3b8;")
        self.meta = QtWidgets.QLabel("last receive: —")
        self.meta.setStyleSheet("color:#94a3b8; font-size:11px;")
        stable_metadata(self.meta)
        self.pixmap = None
        self._image_key = None
        self._scaled_key = None
        layout.addWidget(self.title)
        layout.addWidget(self.display, 1)
        layout.addWidget(self.meta)

    def set_image(self, image):
        if image.isNull() or image.cacheKey() == self._image_key:
            return
        self._image_key = image.cacheKey()
        self.pixmap = QtGui.QPixmap.fromImage(image)
        self.redraw()

    def redraw(self):
        if self.pixmap is not None:
            key = (self.pixmap.cacheKey(), self.display.width(), self.display.height())
            if key == self._scaled_key:
                return
            self.display.setPixmap(self.pixmap.scaled(self.display.size(), QtCore.Qt.KeepAspectRatio,
                                                       QtCore.Qt.SmoothTransformation))
            self._scaled_key = key

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.redraw()


def grid_geometry(event):
    """Validate actual ROS grid geometry; never infer it from image dimensions."""
    if event.get("georeferenced") is not True:
        return None
    g = event.get("grid")
    if not isinstance(g, dict) or g.get("image_y") != "high_to_low":
        return None
    o = g.get("origin", {})
    if not isinstance(g.get("frame_id"), str) or not g["frame_id"]:
        return None
    if not all(finite_number(g.get(k)) and g[k] > 0 for k in ("resolution", "width", "height")):
        return None
    if not all(finite_number(o.get(k)) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")):
        return None
    q = [o[k] for k in ("qx", "qy", "qz", "qw")]
    if abs(sum(v * v for v in q) - 1) > .03 or abs(q[0]) > .001 or abs(q[1]) > .001:
        return None
    yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 1 - 2 * (q[1] ** 2 + q[2] ** 2))
    return g, yaw


def rotate_vector(pose, vector):
    """Rotate a physical axis using the full attitude, including roll/pitch."""
    x, y, z, w = (pose[k] for k in ("qx", "qy", "qz", "qw"))
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if norm < 1e-9:
        raise ValueError("Invalid attitude")
    x, y, z, w = (v / norm for v in (x, y, z, w))
    vx, vy, vz = vector
    return ((1-2*(y*y+z*z))*vx + 2*(x*y-z*w)*vy + 2*(x*z+y*w)*vz,
            2*(x*y+z*w)*vx + (1-2*(x*x+z*z))*vy + 2*(y*z-x*w)*vz,
            2*(x*z-y*w)*vx + 2*(y*z+x*w)*vy + (1-2*(x*x+y*y))*vz)


class MapPanel(QtWidgets.QWidget):
    goal_selected = QtCore.pyqtSignal(float, float, str)

    """Georeferenced XY display. Pixel interpolation stays nearest-neighbour."""
    def __init__(self, title, slope=False, forward_axis=(1., 0., 0.), heading_note="body +X"):
        super().__init__()
        self.slope = slope
        self.forward_axis = forward_axis
        self.event = {}
        self.pixels = None
        self.history = []
        self.pose_age = None
        self.dirty = True
        self.goal_selection_enabled = False
        self.selected_goal = None
        self.planning = {}
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet("font-weight:600; color:#e2e8f0;")
        layout.addWidget(self.title)
        self.figure = Figure(facecolor="#111827")
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.canvas.setMinimumSize(220, 190)
        self.ax = self.figure.add_subplot(111, facecolor="#0b1220")
        self.canvas.mpl_connect("button_press_event", self.select_goal)
        self.figure.subplots_adjust(left=.13, right=.97, bottom=.16, top=.92)
        layout.addWidget(self.canvas, 1)
        legend = ("Slope | green 0° · yellow 15° · red ≥30° | gray: unknown"
                  if slope else "Cost | green 0 · yellow 50 · red 100 | gray: unknown")
        key = QtWidgets.QLabel(legend + "\nCyan: position / traveled path · arrow: " + heading_note)
        key.setWordWrap(True)
        key.setStyleSheet("color:#cbd5e1; font-size:11px;")
        layout.addWidget(key)
        self.meta = QtWidgets.QLabel("WAITING — grid geometry")
        self.meta.setStyleSheet("color:#94a3b8; font-size:11px;")
        layout.addWidget(self.meta)
        # Allocate artists once. Incoming pose/status updates must not clear the
        # axes or recreate the map, legend, text and vehicle collections.
        ax = self.ax
        ax.tick_params(colors="#94a3b8", labelsize=8)
        for spine in ax.spines.values():
            spine.set_color("#475569")
        ax.set_xlabel("X (m)", color="#cbd5e1", fontsize=9)
        ax.set_ylabel("Y (m)", color="#cbd5e1", fontsize=9)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(color="#64748b", alpha=.22, lw=.6)
        ax.set_xlim(-2, 2)
        ax.set_ylim(-2, 2)
        self._map_artist = ax.imshow(np.zeros((1, 1, 3), dtype=np.uint8), origin="upper",
                                     extent=(0, 1, 0, 1), interpolation="nearest", zorder=1,
                                     visible=False)
        self._boundary_artist, = ax.plot([], [], color="#64748b", lw=.6, zorder=2)
        self._planning_artists = {}
        for name, color, label in (("path", "#c084fc", "GPP"), ("trajectory", "#fbbf24", "RPP")):
            self._planning_artists[name], = ax.plot([], [], color=color, lw=2, label=label, zorder=5)
        self._planning_legend = ax.legend(loc="upper right", facecolor="#1e293b", edgecolor="#475569",
                                          labelcolor="#e2e8f0", fontsize=8)
        self._planning_legend.set_visible(False)
        self._history_artist, = ax.plot([], [], color="#22d3ee", lw=1.4, zorder=4)
        self._goal_artist = ax.scatter([], [], marker="*", s=160, color="#fbbf24",
                                       edgecolors="white", linewidths=.7, zorder=9)
        self._vehicle_artist = ax.scatter([], [], s=65, facecolors="#22d3ee",
                                          edgecolors="white", linewidths=1, zorder=6)
        self._heading_artist = ax.annotate("", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color="#22d3ee", lw=2), zorder=7)
        self._heading_artist.set_visible(False)
        self._pose_label = ax.annotate("", (0, 0), xytext=(6, 8), textcoords="offset points",
            color="white", fontsize=8, zorder=8,
            bbox=dict(facecolor="#111827", alpha=.7, edgecolor="none", pad=2))
        self._pose_label.set_visible(False)
        self._waiting_map = ax.text(.5, .5, "WAITING FOR MAP COORDINATES", transform=ax.transAxes,
                                   color="#94a3b8", ha="center", va="center", fontsize=10)
        self._waiting_pose = ax.text(.02, .97, "", transform=ax.transAxes,
                                    color="#fbbf24", va="top", fontsize=8, zorder=8, visible=False)
        self._last_pixels = None
        self._geometry_key = self._planning_key = self._pose_key = self._goal_key = None
        self._view_bounds = None
        self._image_cache_key = None
        self._pose_input_key = None
        self._pose_sampler = HistorySampler(1800)
        stable_metadata(self.meta)

    def clear_data(self):
        """A backward replay seek must never retain a future map or vehicle."""
        self.event, self.planning = {}, {}
        self.pixels = self._last_pixels = None
        self.history, self.pose_age, self.selected_goal = [], None, None
        self._geometry_key = self._planning_key = self._pose_key = self._goal_key = None
        self._image_cache_key = self._pose_input_key = None
        self._pose_sampler = HistorySampler(1800)
        self._view_bounds = None
        self._map_artist.set_visible(False)
        self._boundary_artist.set_data([], [])
        self._history_artist.set_data([], [])
        for artist in self._planning_artists.values():
            artist.set_data([], [])
        self._planning_legend.set_visible(False)
        for artist in (self._vehicle_artist, self._goal_artist):
            artist.set_offsets(np.empty((0, 2)))
        self._heading_artist.set_visible(False)
        self._pose_label.set_visible(False)
        self._waiting_pose.set_visible(False)
        self._waiting_map.set_visible(True)
        self.ax.set_title("")
        self.meta.setText("WAITING — grid geometry")
        self.dirty = True
        self.canvas.draw_idle()

    def select_goal(self, event):
        geometry = grid_geometry(self.event)
        if (not self.goal_selection_enabled or event.button != 1 or event.inaxes is not self.ax or
                geometry is None or self.pixels is None or
                not finite_number(event.xdata) or not finite_number(event.ydata)):
            return
        g, yaw = geometry
        # The displayed RGB map has no numerical occupancy array. The C++
        # planner validates traversability; outside-map goals stay unknown.
        self.goal_selected.emit(event.xdata, event.ydata, g["frame_id"])

    def set_planning(self, event, goal=None):
        self.planning = event
        self.selected_goal = goal
        self.dirty = True

    def set_image(self, image, event):
        if grid_geometry(event) is None:
            # Keep the last located map visible; do not place an unpaired image
            # using another capture's origin. The status line explains the wait.
            return
        cache_key = image.cacheKey()
        geometry_changed = grid_geometry(event) != grid_geometry(self.event)
        self.event = event
        if cache_key == self._image_cache_key:
            self.dirty |= geometry_changed
            return
        self._image_cache_key = cache_key
        rgb = image.convertToFormat(QtGui.QImage.Format_RGB888)
        bits = rgb.bits()
        bits.setsize(rgb.byteCount())
        pixels = np.frombuffer(bits, dtype=np.uint8).reshape(rgb.height(), rgb.bytesPerLine())[:, :rgb.width() * 3].reshape(rgb.height(), rgb.width(), 3)
        if self.pixels is None or not np.array_equal(self.pixels, pixels):
            self.pixels = pixels.copy()
            self.dirty = True
        self.dirty |= geometry_changed

    def set_pose(self, history, age):
        self.history = history
        self.pose_age = age
        key = (history_signature(history), age is not None and age <= 1)
        if key != self._pose_input_key:
            self._pose_input_key = key
            self.dirty = True

    def redraw(self):
        if not self.dirty:
            return
        self.dirty = False
        ax = self.ax
        geometry = grid_geometry(self.event)
        if geometry is None or self.pixels is None:
            if not self._waiting_map.get_visible():
                self._waiting_map.set_visible(True)
                self.canvas.draw_idle()
            return
        g, yaw = geometry
        o = g["origin"]
        width, height = g["width"] * g["resolution"], g["height"] * g["resolution"]
        geometry_key = (g["frame_id"], g["resolution"], width, height, o["x"], o["y"], yaw)
        geometry_changed = geometry_key != self._geometry_key
        changed = geometry_changed or self.pixels is not self._last_pixels
        if changed:
            self._waiting_map.set_visible(False)
            self._map_artist.set_visible(True)
            if self.pixels is not self._last_pixels:
                self._map_artist.set_data(self.pixels)
                self._last_pixels = self.pixels
            if geometry_changed:
                transform = Affine2D().rotate(yaw).translate(o["x"], o["y"])
                self._map_artist.set_extent((0, width, 0, height))
                self._map_artist.set_transform(transform + ax.transData)
                self._map_corners = transform.transform(np.array([[0, 0], [width, 0], [width, height], [0, height]]))
                boundary = np.vstack([self._map_corners, self._map_corners[0]])
                self._boundary_artist.set_data(boundary[:, 0], boundary[:, 1])
                ax.set_title(f"XY top view · {g['frame_id']} · {g['resolution']:.2f} m/cell",
                             color="#cbd5e1", fontsize=9, pad=6)
                self._geometry_key = geometry_key
        corners = self._map_corners
        xs, ys = list(corners[:, 0]), list(corners[:, 1])
        planning = self.planning if self.planning.get("frame") == g["frame_id"] else {}
        paths = tuple(tuple(tuple(p) for p in planning.get(name, [])) for name in ("path", "trajectory"))
        planning_key = (paths, planning.get("path_valid"), planning.get("executable"))
        if planning_key != self._planning_key:
            for name in ("path", "trajectory"):
                path = planning.get(name, [])
                self._planning_artists[name].set_data([p[0] for p in path], [p[1] for p in path])
                valid = planning.get("path_valid", True) if name == "path" else planning.get("executable", True)
                self._planning_artists[name].set_linestyle("-" if valid else "--")
                self._planning_artists[name].set_alpha(1. if valid else .45)
            self._planning_legend.set_visible(any(paths))
            self._planning_key = planning_key
            changed = True
        goal = self.selected_goal
        if goal is None and self.planning.get("goal") and self.planning.get("frame") == g["frame_id"]:
            goal = (*self.planning["goal"][:2], g["frame_id"])
        goal_key = tuple(goal) if goal and goal[2] == g["frame_id"] else None
        if goal_key != self._goal_key:
            self._goal_artist.set_offsets(np.asarray([[goal[0], goal[1]]]) if goal_key else np.empty((0, 2)))
            self._goal_key = goal_key
            changed = True
        poses = self._pose_sampler.sample(self.history, g["frame_id"])
        color = "#22d3ee" if self.pose_age is not None and self.pose_age <= 1 else "#94a3b8"
        pose_key = (g["frame_id"], len(poses), tuple(poses[0].items()) if poses else (),
                    tuple(poses[-1].items()) if poses else (), color, width, height, tuple(self.forward_axis))
        if poses:
            p = poses[-1]
            xs.append(p["x"])
            ys.append(p["y"])
        if pose_key != self._pose_key:
            if poses:
                points = poses
                self._history_artist.set_data([p["x"] for p in points], [p["y"] for p in points])
                self._vehicle_artist.set_offsets([[p["x"], p["y"]]])
                self._vehicle_artist.set_facecolor(color)
                forward = rotate_vector(p, self.forward_axis)
                heading = math.atan2(forward[1], forward[0])
                length = max(.25, min(.65, max(width, height) * .08))
                self._heading_artist.xy = (p["x"] + length * math.cos(heading), p["y"] + length * math.sin(heading))
                self._heading_artist.set_position((p["x"], p["y"]))
                self._heading_artist.arrow_patch.set_color(color)
                self._heading_artist.set_visible(math.hypot(forward[0], forward[1]) > .05)
                self._pose_label.xy = (p["x"], p["y"])
                self._pose_label.set_text(f"  ({p['x']:.2f}, {p['y']:.2f})")
                self._pose_label.set_visible(True)
                self._waiting_pose.set_visible(False)
            else:
                self._history_artist.set_data([], [])
                self._vehicle_artist.set_offsets(np.empty((0, 2)))
                self._heading_artist.set_visible(False)
                self._pose_label.set_visible(False)
                self._waiting_pose.set_text("WAITING: vehicle pose in " + g["frame_id"])
                self._waiting_pose.set_visible(True)
            self._pose_key = pose_key
            changed = True
        span = max(3., max(xs) - min(xs), max(ys) - min(ys)) * 1.12
        cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
        bounds = (cx - span / 2, cx + span / 2, cy - span / 2, cy + span / 2)
        current = self._view_bounds
        inside = current and current[0] <= min(xs) and max(xs) <= current[1] and current[2] <= min(ys) and max(ys) <= current[3]
        if not inside and bounds != current:
            ax.set_xlim(bounds[:2])
            ax.set_ylim(bounds[2:])
            self._view_bounds = bounds
            changed = True
        if changed:
            self.canvas.draw_idle()


class MonitorWindow(RecordingMixin, QtWidgets.QMainWindow):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.init_recording(args)
        self.spectator_mode = bool(getattr(args, "relay_host", None))
        self.relay_server = None
        self.relay_receiver = None
        self.relay_progress = {"state": "pending" if self.spectator_mode else "disabled"}
        self.relay_error = None
        self.relay_local_addresses = []
        self.relay_address = relay_endpoint(args.relay_host) if self.spectator_mode else None
        if self.spectator_mode and getattr(args, "relay_listen", None) is not None:
            raise ValueError("relay listening and spectator modes are mutually exclusive")
        self.started = time.time()
        if self.replay_mode:
            self.started = float(self.reader.metadata.get("started_wall_s", self.started))
        self.histories = {source: deque(maxlen=MAX_POSES) for source in (*POSE_SOURCES, "gnss", "vio_compare")}
        self._comparison_epoch = None
        self._raw_vio_epoch = None
        self.heading_worker = None
        self.heading_reset_in_progress = False
        self.heading_reset_result = None
        self.vio_restart_in_progress = False
        self.vio_restart_result = None
        self.vio_restart_worker = None
        self._plot_samplers = {source: HistorySampler(4000) for source in POSE_SOURCES}
        self._plot_signatures = {}
        self._fit_initialized = False
        self._decoded_payloads = {}
        self.received = {}
        self.statuses = {}
        self.rates = {}
        self.images = {}
        self.latest_qimages = {}
        self.telemetry = {}
        self.developer_windows = {}
        self.module_logs = {part: LogHistory(maxlen=128) for part in
                            ("localization", "perception", "traversability", "planning")}
        self.log_workers = {}
        self.image_dirty = set()
        self.plot_dirty = False
        self.invalid_events = 0
        self.accepted_events = 0
        self.capture_order = CaptureOrder()
        self.workers = []
        self.planning_client = None
        self.planning_stale_disarmed = False
        self.selected_goal = None
        self.stop_worker = None
        self.trial_stop_result = None
        self.model_worker = None
        self.model_switch_in_progress = False
        self.model_switch_result = None
        self.active_model = None
        self.cloud_worker = None
        self.cloud_switch_in_progress = False
        self.cloud_switch_result = None
        self.active_cloud_source = None
        self.slope_worker = None
        self.slope_switch_in_progress = False
        self.slope_switch_result = None
        self.active_slope_mode = None
        self.slope_cpu_client = None
        self.slope_cpu_history = {}
        self.ssh_progress = {board: {"phase": "via_relay" if self.spectator_mode else
                                     ("disabled" if args.no_ssh else "command_unset")}
                             for board in ("pi", "jetson")}
        self.last_state_write = 0.0
        self.setWindowTitle("NOMAD | 기록 재생" if self.replay_mode else "NOMAD | Live localization + perception")
        self.resize(1440, 950)
        self.setMinimumSize(1000, 700)
        self.setStyleSheet("QMainWindow,QWidget {background:#111827;color:#e2e8f0;} "
                           "QLabel {font-size:12px;} QCheckBox {padding:3px;} "
                           "QPushButton {padding:5px 12px;background:#334155;border-radius:4px;}")
        central = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(central)
        layout.setSpacing(4)
        self.setup_recording_bar(layout)
        if args.enable_trial_control:
            toolbar = QtWidgets.QHBoxLayout()
            toolbar.addWidget(QtWidgets.QLabel("관전 모드 · 보드 SSH 연결 없음" if self.spectator_mode else
                                              f"Pi5: {args.pi_host}  |  Jetson: {args.jetson_host}"), 1)
            self.model_combo = QtWidgets.QComboBox()
            self.model_combo.addItem("B2 · 기존", "b2")
            self.model_combo.addItem("FCN · 실내", "fcn")
            self.model_combo.setToolTip("인지 모델만 전환합니다. 카메라·VIO·지도·화면 수신은 유지합니다.")
            toolbar.addWidget(self.model_combo)
            self.model_button = QtWidgets.QPushButton("모델 적용")
            self.model_button.clicked.connect(self.apply_model)
            toolbar.addWidget(self.model_button)
            self.stop_button = QtWidgets.QPushButton("시험 종료")
            self.stop_button.setToolTip("센서·VIO·융합·지도·화면 수신을 종료합니다. 보드 전원은 유지합니다.")
            self.stop_button.setStyleSheet("QPushButton {background:#991b1b;color:white;font-weight:600;padding:9px 22px;} QPushButton:disabled {background:#475569;}")
            self.stop_button.clicked.connect(self.stop_trial)
            toolbar.addWidget(self.stop_button)
            layout.addLayout(toolbar)
            self.stop_message = QtWidgets.QLabel("시험 종료: 실행 모듈 종료 · 보드 전원 유지")
            self.stop_message.setWordWrap(True)
            self.stop_message.setStyleSheet("color:#cbd5e1;padding:3px;")
            layout.addWidget(self.stop_message)
            self.model_message = QtWidgets.QLabel("인지 모델: 미확인 · 선택 후 적용")
            self.model_message.setWordWrap(True)
            self.model_message.setStyleSheet("color:#cbd5e1;padding:3px;")
            layout.addWidget(self.model_message)
            cloud_bar = QtWidgets.QHBoxLayout()
            cloud_bar.addWidget(QtWidgets.QLabel("OAK 점군"))
            self.cloud_combo = QtWidgets.QComboBox()
            self.cloud_combo.addItem("OpenCV StereoSGBM", "opencv")
            self.cloud_combo.addItem("OAK 내부 StereoDepth", "device")
            cloud_bar.addWidget(self.cloud_combo)
            self.cloud_button = QtWidgets.QPushButton("점군 방식 적용")
            self.cloud_button.clicked.connect(self.apply_cloud_source)
            cloud_bar.addWidget(self.cloud_button)
            self.cloud_status_button = QtWidgets.QPushButton("현재 방식 확인")
            self.cloud_status_button.clicked.connect(lambda: self.apply_cloud_source(query=True))
            cloud_bar.addWidget(self.cloud_status_button)
            cloud_bar.addStretch(1)
            layout.addLayout(cloud_bar)
            self.cloud_message = QtWidgets.QLabel("OAK 점군: 미확인. 기존 지도는 유지되며, 방식 비교는 새 지도에서 진행하세요.")
            self.cloud_message.setWordWrap(True)
            layout.addWidget(self.cloud_message)
            slope_bar = QtWidgets.QHBoxLayout()
            slope_bar.addWidget(QtWidgets.QLabel("경사 계산"))
            self.slope_combo = QtWidgets.QComboBox()
            self.slope_combo.addItem("기존 방식 (1스레드)", "sequential")
            self.slope_combo.addItem("병렬 방식 (3스레드)", "parallel")
            self.slope_combo.setToolTip("기존에 고정한 경사는 유지합니다. 병렬 방식도 재계산할 셀이 적으면 순차 처리합니다.")
            slope_bar.addWidget(self.slope_combo)
            self.slope_button = QtWidgets.QPushButton("경사 방식 적용")
            self.slope_button.clicked.connect(self.apply_slope_mode)
            slope_bar.addWidget(self.slope_button)
            self.slope_status_button = QtWidgets.QPushButton("현재 방식 확인")
            self.slope_status_button.clicked.connect(lambda: self.apply_slope_mode(query=True))
            slope_bar.addWidget(self.slope_status_button)
            slope_bar.addStretch(1)
            layout.addLayout(slope_bar)
            self.slope_message = QtWidgets.QLabel("경사 방식: 미확인. CPU는 개발 창 > 인지에서 확인하세요.")
            self.slope_message.setWordWrap(True)
            layout.addWidget(self.slope_message)
            self.set_slope_controls_enabled(not args.no_ssh and not self.spectator_mode)
            self.set_cloud_controls_enabled(not args.no_ssh and not self.spectator_mode)
            if args.no_ssh or self.spectator_mode:
                self.stop_button.setEnabled(False)
                self.model_combo.setEnabled(False)
                self.model_button.setEnabled(False)
                self.model_message.setText("인지 모델: 미확인 · 관전 모드는 읽기 전용입니다." if self.spectator_mode else
                                           "인지 모델: 미확인 · SSH 없는 화면에서는 전환하지 않습니다.")
                if self.spectator_mode:
                    self.stop_message.setText("관전 모드 · 시험 종료와 모델 변경은 운용 PC에서 진행합니다.")
        developer_bar = QtWidgets.QHBoxLayout()
        developer_bar.addWidget(QtWidgets.QLabel("개발 창"))
        for part, label in (("localization", "위치추정"), ("perception", "인지"),
                            ("traversability", "주행가능성"), ("planning", "플래닝"),
                            ("control", "제어")):
            button = QtWidgets.QPushButton(label)
            button.clicked.connect(lambda checked=False, key=part: self.open_developer_window(key))
            developer_bar.addWidget(button)
        developer_bar.addStretch(1)
        developer_bar.addWidget(QtWidgets.QLabel("통합 화면과 같은 수신 데이터 사용"))
        layout.addLayout(developer_bar)
        self.status_label = QtWidgets.QLabel("Localization: WAITING | Perception: WAITING")
        self.status_label.setWordWrap(True)
        self.status_label.setStyleSheet("font-size:14px; font-weight:600; padding:5px;")
        self.rate_label = QtWidgets.QLabel("Pi rates: WAITING | Jetson rates: WAITING")
        self.rate_label.setWordWrap(True)
        self.ssh_label = QtWidgets.QLabel()
        self.ssh_label.setStyleSheet("color:#94a3b8; font-size:11px;")
        layout.addWidget(self.status_label)
        self.localization_mode_label = QtWidgets.QLabel("위치 모드 · 입력 대기")
        self.localization_mode_label.setWordWrap(True)
        layout.addWidget(self.localization_mode_label)
        layout.addWidget(self.rate_label)
        layout.addWidget(self.ssh_label)
        self.relay_label = QtWidgets.QLabel()
        self.relay_label.setWordWrap(True)
        self.relay_label.setStyleSheet("color:#7dd3fc; font-size:12px; padding:3px;")
        layout.addWidget(self.relay_label)
        planning_bar = QtWidgets.QHBoxLayout()
        planning_bar.addWidget(QtWidgets.QLabel("Pi4 · 플래닝"))
        self.planning_host = QtWidgets.QLineEdit(getattr(args, "planning_host", "") or "user@lpp.local")
        self.planning_host.setMaximumWidth(210)
        self.planning_host.setToolTip("C++ 플래너가 설치된 Pi4 SSH 주소")
        planning_bar.addWidget(self.planning_host)
        self.planning_connect = QtWidgets.QPushButton("플래너 연결")
        self.planning_connect.clicked.connect(self.connect_planner)
        planning_bar.addWidget(self.planning_connect)
        self.goal_send = QtWidgets.QPushButton("선택 목표 보내기")
        self.goal_send.clicked.connect(self.send_goal)
        self.goal_send.setEnabled(False)
        planning_bar.addWidget(self.goal_send)
        self.goal_cancel = QtWidgets.QPushButton("목표 취소")
        self.goal_cancel.clicked.connect(self.cancel_goal)
        self.goal_cancel.setEnabled(False)
        planning_bar.addWidget(self.goal_cancel)
        self.output_arm = QtWidgets.QPushButton("출력 허용")
        self.output_arm.setToolTip("목표 전송과 별도로 구동 명령을 허용합니다. 실측 검증 전 임시 환산을 사용합니다.")
        self.output_arm.clicked.connect(self.arm_output)
        self.output_arm.setEnabled(False)
        planning_bar.addWidget(self.output_arm)
        self.output_disarm = QtWidgets.QPushButton("출력 정지")
        self.output_disarm.setStyleSheet("QPushButton {background:#991b1b;color:white;}")
        self.output_disarm.clicked.connect(self.disarm_output)
        self.output_disarm.setEnabled(False)
        planning_bar.addWidget(self.output_disarm)
        planning_bar.addStretch(1)
        layout.addLayout(planning_bar)
        self.planning_message = QtWidgets.QLabel("목표 선택 → 목표 보내기 · 출력은 별도 ‘출력 허용’ 클릭 전까지 잠금")
        self.planning_message.setWordWrap(True)
        self.planning_message.setStyleSheet("color:#c4b5fd;font-size:11px;")
        layout.addWidget(self.planning_message)
        if self.spectator_mode or args.no_ssh:
            self.planning_connect.setEnabled(False)
            self.planning_host.setEnabled(False)
            if self.spectator_mode:
                self.planning_message.setText("관전 모드 · 운용 PC의 계획 경로 표시 / 목표 전송 불가")
        upper = QtWidgets.QHBoxLayout()
        plot_box = QtWidgets.QWidget()
        plot_layout = QtWidgets.QVBoxLayout(plot_box)
        plot_layout.setContentsMargins(0, 0, 0, 0)
        controls = QtWidgets.QHBoxLayout()
        controls.addWidget(QtWidgets.QLabel("3D trajectory | drag to rotate"))
        self.auto_fit = QtWidgets.QCheckBox("Auto fit")
        self.auto_fit.setChecked(True)
        controls.addWidget(self.auto_fit)
        reset = QtWidgets.QPushButton("Reset view")
        reset.clicked.connect(self.reset_view)
        controls.addWidget(reset)
        controls.addStretch(1)
        plot_layout.addLayout(controls)
        self.figure = Figure(facecolor="#111827")
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111, projection="3d", facecolor="#111827")
        self.ax.set_xlabel("x (m)", color="#cbd5e1")
        self.ax.set_ylabel("y (m)", color="#cbd5e1")
        self.ax.set_zlabel("z (m)", color="#cbd5e1")
        self.ax.tick_params(colors="#94a3b8", labelsize=8)
        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            axis.set_pane_color((0.07, 0.11, 0.18, 1.0))
        self.lines, self.markers = {}, {}
        for source, color in zip(POSE_SOURCES, ("#38bdf8", "#fb923c", "#4ade80")):
            self.lines[source], = self.ax.plot([], [], [], color=color, lw=1.3, label=source.upper())
            self.markers[source], = self.ax.plot([], [], [], color=color, marker="o", markersize=4)
        legend = self.ax.legend(loc="upper left", facecolor="#1e293b", edgecolor="#334155")
        for text in legend.get_texts():
            text.set_color("#e2e8f0")
        self.figure.subplots_adjust(left=0.01, right=0.90, bottom=0.06, top=0.98)
        self.pose_label = QtWidgets.QLabel("VIO: WAITING | FUSION: WAITING | PUBLIC: WAITING")
        self.pose_label.setStyleSheet("color:#94a3b8; font-size:11px;")
        plot_canvas_box = QtWidgets.QWidget()
        canvas_layout = QtWidgets.QGridLayout(plot_canvas_box)
        canvas_layout.setContentsMargins(0, 0, 0, 0)
        canvas_layout.addWidget(self.canvas, 0, 0)
        self.heading_indicator = HeadingIndicator()
        self.heading_indicator.setAttribute(QtCore.Qt.WA_TransparentForMouseEvents)
        canvas_layout.addWidget(self.heading_indicator, 0, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignRight)
        plot_layout.addWidget(plot_canvas_box, 1)
        plot_layout.addWidget(self.pose_label)
        # Camera previews belong to developer windows. Their latest decoded
        # images stay in the shared cache for those windows and spectators.
        self.panels = {}
        forward_axis, heading_note = (1., 0., 0.), "body +X"
        if args.preview_heading_calibration:
            calibration = json.loads(Path(args.preview_heading_calibration).read_text())
            forward_axis = rotate_vector(calibration["value0"]["T_imu_cam"][0], (0., 0., 1.))
            heading_note = "OAK camera front (IMU=base preview)"
        if self.replay_mode:
            axis = self.reader.metadata.get("forward_axis", (1., 0., 0.))
            if (isinstance(axis, (list, tuple)) and len(axis) == 3 and
                    all(finite_number(value) for value in axis)):
                forward_axis = tuple(axis)
            heading_note = str(self.reader.metadata.get("heading_note", "recorded heading"))[:160]
        self.forward_axis = forward_axis
        self.heading_note = heading_note
        self.panels["costmap"] = MapPanel("COSTMAP | XY TOP VIEW", forward_axis=forward_axis, heading_note=heading_note)
        self.panels["slope"] = MapPanel("SLOPE | XY TOP VIEW", slope=True, forward_axis=forward_axis, heading_note=heading_note)
        self.panels["costmap"].goal_selected.connect(self.select_goal)
        for source in ("costmap", "slope"):
            upper.addWidget(self.panels[source], 1)
        upper.addWidget(plot_box, 1)
        layout.addLayout(upper, 1)
        self.footer = QtWidgets.QLabel("창 닫기: 화면 수신·Pi4 플래너 종료 | 시험 종료: 센서·위치추정·인지·지도도 종료 / 보드 전원 유지"
                                      if args.enable_trial_control else "창 닫기: 화면 수신과 이 창에서 시작한 플래너만 종료 · 센서/보드 전원 유지")
        self.footer.setStyleSheet("color:#94a3b8; font-size:11px;")
        if self.spectator_mode:
            self.footer.setText("읽기 전용 관전 · 창 닫기: 이 PC의 중계 수신만 종료 · 보드와 운용 PC는 유지")
        if self.replay_mode:
            self.footer.setText("기록 재생 · 개발 창에서 같은 시점의 데이터 확인 · 보드 연결·목표/구동 명령 없음")
        layout.addWidget(self.footer)
        self.setCentralWidget(central)
        # Bind synchronously before any board collector can start. A port conflict
        # must fail locally rather than silently create another set of observers.
        if getattr(args, "relay_listen", None) is not None:
            from relay import RelayServer
            server = RelayServer(host="0.0.0.0", port=args.relay_listen)
            try:
                server.start()
            except Exception:
                server.close()
                raise
            self.relay_server = server
            # Enumerate only once; the 5 Hz paint loop should not query network
            # interfaces repeatedly. This is local and opens no board sockets.
            self.relay_local_addresses = sorted({address.toString() for address in QtNetwork.QNetworkInterface.allAddresses()
                                                 if address.protocol() == QtNetwork.QAbstractSocket.IPv4Protocol and
                                                 not address.isLoopback()})
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(round(1000 / args.ui_hz))
        self.ingest_timer = QtCore.QTimer(self)
        self.ingest_timer.timeout.connect(self.drain_receivers)
        self.ingest_timer.start(33)
        if self.spectator_mode:
            self.relay_receiver = RelayReceiver(*self.relay_address, parent=self)
            self.relay_receiver.progress.connect(self.on_relay_progress)
            self.relay_receiver.start()
        elif not args.no_ssh:
            for board, host, command in (("pi", args.pi_host, args.pi_command),
                                          ("jetson", args.jetson_host, args.jetson_command)):
                if not command:
                    continue
                worker = Collector(board, host, command, self)
                worker.progress.connect(self.on_progress)
                self.workers.append(worker)
                worker.start()
                self.ensure_log_reader(board, host)
        if args.snapshot:
            QtCore.QTimer.singleShot(round(args.snapshot_after_s * 1000), self.save_snapshot)
            if args.snapshot_interval_s > 0:
                self.snapshot_timer = QtCore.QTimer(self)
                self.snapshot_timer.timeout.connect(self.save_snapshot)
                self.snapshot_timer.start(round(args.snapshot_interval_s * 1000))
        if args.quit_after_s > 0:
            QtCore.QTimer.singleShot(round(args.quit_after_s * 1000), self.close)
        if self.replay_mode:
            self.seek_replay(0.)
        self.refresh()
        for part in getattr(args, "open_developer_window", ()):
            self.open_developer_window(part)

    def developer_snapshot(self):
        """One shared receiver; child windows only read the latest UI snapshot."""
        return {"histories": self.histories, "images": self.latest_qimages,
                "image_meta": self.images, "received": self.received,
                "statuses": self.statuses, "rates": self.rates, "telemetry": self.telemetry,
                "module_logs": self.module_logs,
                "ssh": self.ssh_progress, "now": self.now(), "wall_now": self.wall_now(),
                "replay": self.replay_mode, "heading_ui": self.heading_ui(), "vio_ui": self.vio_ui(),
                "active_model": self.active_model, "active_cloud_source": self.active_cloud_source,
                "slope_cpu_history": self.slope_cpu_history, "forward_axis": self.forward_axis,
                "heading_note": self.heading_note, "trial_stop_result": self.trial_stop_result,
                "planning_ui": {"host": self.planning_host.text(),
                                "message": self.planning_message.text(),
                                "selected_goal": self.selected_goal,
                                "spectator": self.spectator_mode,
                                "no_ssh": self.args.no_ssh,
                                "client_active": self.planning_client is not None,
                                "send_enabled": self.goal_send.isEnabled(),
                                "cancel_enabled": self.goal_cancel.isEnabled(),
                                "arm_enabled": self.output_arm.isEnabled(),
                                "disarm_enabled": self.output_disarm.isEnabled()}}

    def open_developer_window(self, part):
        if part == "perception":
            self.ensure_slope_cpu_reader()
        if part in ("planning", "control"):
            self.ensure_log_reader("pi4", self.planning_host.text())
        window = self.developer_windows.get(part)
        if window is None:
            if part == "localization":
                from developer_localization import LocalizationWindow as Window
            elif part == "perception":
                from developer_perception import PerceptionWindow as Window
            elif part == "traversability":
                from developer_traversability import TraversabilityWindow as Window
            elif part == "planning":
                from developer_planning import PlanningWindow as Window
            elif part == "control":
                from developer_control import ControlWindow as Window
            else:
                raise ValueError("Unknown developer window")
            window = Window(parent=self, components={"ImagePanel": ImagePanel, "MapPanel": MapPanel,
                                                     "grid_geometry": grid_geometry,
                                                     "rotate_vector": rotate_vector})
            if part == "localization":
                window.heading_reset_requested.connect(self.reset_heading_from_magnetometer)
                window.vio_restart_requested.connect(self.restart_vio)
            self.developer_windows[part] = window
        window.update_snapshot(self.developer_snapshot())
        window.show()
        window.raise_()
        window.activateWindow()

    def heading_ui(self):
        snapshot = {"now": self.now(), "received": self.received,
                    "telemetry": self.telemetry, "statuses": self.statuses, "replay": self.replay_mode}
        summary = localization_summary(snapshot)
        reason = summary["reset_reason"]
        live = not self.replay_mode and not self.spectator_mode and not self.args.no_ssh
        if self.replay_mode:
            reason = "기록 재생은 읽기 전용입니다."
        elif self.spectator_mode:
            reason = "관전 PC는 읽기 전용입니다. 운용 PC에서 초기화하세요."
        elif self.args.no_ssh:
            reason = "보드 제어 연결이 없습니다."
        elif self.trial_stop_result is not None:
            reason = "시험을 다시 시작한 뒤 초기화하세요."
        elif self.heading_reset_in_progress:
            reason = "Pi5 방향 초기화 결과 확인 중입니다."
        elif self.vio_restart_in_progress:
            reason = "VIO 재실행 결과 확인 중입니다."
        return {"enabled": bool(live and self.trial_stop_result is None and
                                 not self.heading_reset_in_progress and not self.vio_restart_in_progress and summary["can_reset_heading"]),
                "busy": self.heading_reset_in_progress, "result": self.heading_reset_result,
                "reason": reason, "spectator": self.spectator_mode, "no_ssh": self.args.no_ssh}

    def vio_ui(self):
        live = not self.replay_mode and not self.spectator_mode and not self.args.no_ssh
        enabled = bool(live and self.trial_stop_result is None and not self.vio_restart_in_progress
                       and not self.heading_reset_in_progress)
        snapshot = {"now": self.now(), "received": self.received,
                    "telemetry": self.telemetry, "statuses": self.statuses}
        summary = localization_summary(snapshot)
        reason = "융합기가 살아 있으면 마지막 유효 위치·방향에 VIO를 이어 붙입니다."
        if not live:
            reason = "관전·기록 재생·SSH 없는 화면은 읽기 전용입니다."
        elif self.trial_stop_result is not None:
            reason = "시험을 시작한 뒤 VIO를 재실행하세요."
        elif self.heading_reset_in_progress:
            reason = "방향 초기화 결과를 확인 중입니다."
        elif self.vio_restart_in_progress:
            reason = "VIO 재실행 결과를 확인 중입니다."
        return {"enabled": enabled, "busy": self.vio_restart_in_progress,
                "result": self.vio_restart_result, "reason": reason,
                "spectator": self.spectator_mode, "no_ssh": self.args.no_ssh,
                "fusion_continuity": summary["diagnostics_fresh"] or summary["status_fresh"]}

    def restart_vio(self):
        if not self.vio_ui()["enabled"]:
            return
        password = os.environ.get("NOMAD_BOARD_PASSWORD1") or os.environ.get("NOMAD_BOARD_PASSWORD")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(
                self, "VIO 재실행", "Pi5 SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted or not password:
                return
        self.vio_restart_in_progress = True
        self.vio_restart_result = None
        worker = VioRestartWorker(self.args.pi_host, password, self)
        self.vio_restart_worker = worker
        worker.result.connect(self.on_vio_restarted)
        worker.finished.connect(self.on_vio_restart_finished)
        worker.start()

    @QtCore.pyqtSlot(dict)
    def on_vio_restarted(self, value):
        self.vio_restart_result = {**value, "completed_wall_s": self.wall_now()}
        # VIO native coordinates start a new trajectory. The living fusion is
        # untouched; its new epoch reattaches sensor poses at its last state.
        if value.get("ok") is True:
            self.clear_raw_vio_history()
        self.write_state()

    def clear_raw_vio_history(self):
        self.histories["vio"].clear()
        self.received.pop("pose:vio", None)
        self._plot_signatures.pop("vio", None)
        self.lines["vio"].set_data_3d([], [], [])
        self.markers["vio"].set_data_3d([], [], [])
        self.plot_dirty = True

    @QtCore.pyqtSlot()
    def on_vio_restart_finished(self):
        self.vio_restart_in_progress = False
        if self.vio_restart_result is None:
            self.vio_restart_result = {"ok": False, "state": "VIO_RESTART_FAILED",
                                       "reason": "worker_ended_without_result"}
        self.write_state()

    def reset_heading_from_magnetometer(self):
        # Only a user click in the live operator window can create this worker.
        ui = self.heading_ui()
        if not ui["enabled"]:
            return
        password = os.environ.get("NOMAD_BOARD_PASSWORD1") or os.environ.get("NOMAD_BOARD_PASSWORD")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(
                self, "Pi5 방향 초기화", "Pi5 SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted or not password:
                return
        self.heading_reset_in_progress = True
        self.heading_reset_result = None
        worker = HeadingResetWorker(self.args.pi_host, password, self)
        self.heading_worker = worker
        worker.result.connect(self.on_heading_reset_result)
        worker.finished.connect(self.on_heading_reset_finished)
        worker.start()

    @QtCore.pyqtSlot(dict)
    def on_heading_reset_result(self, value):
        # The remote module decides whether an independent MAG heading is usable.
        # Never rotate received poses in the GUI as a substitute for its response.
        self.heading_reset_result = {**value, "completed_wall_s": self.wall_now()}
        self.write_state()

    @QtCore.pyqtSlot()
    def on_heading_reset_finished(self):
        self.heading_reset_in_progress = False
        if self.heading_reset_result is None:
            self.heading_reset_result = {"ok": False, "state": "UNCONFIRMED",
                                         "reason": "worker_ended_without_result"}
        self.write_state()

    def update_comparison_samples(self, event, received_at):
        values = localization_fields(event)
        raw_epoch = values.get("vio_epoch")
        try:
            if not isinstance(raw_epoch, bool) and raw_epoch is not None:
                raw_epoch = int(raw_epoch)
                if raw_epoch >= 0:
                    key = (event.get("_source_session", "legacy"), raw_epoch)
                    if self._raw_vio_epoch is not None and key != self._raw_vio_epoch:
                        self.clear_raw_vio_history()
                    self._raw_vio_epoch = key
        except (ValueError, TypeError, OverflowError):
            pass
        epoch = values.get("viewer_epoch")
        if isinstance(epoch, bool) or not isinstance(values.get("viewer_frame"), str) or not values["viewer_frame"].strip():
            return
        try:
            epoch = int(epoch)
            if epoch < 0:
                return
        except (ValueError, TypeError, OverflowError):
            return
        session = event.get("_source_session", "legacy")
        key = (session, epoch, values.get("viewer_frame"))
        if key != self._comparison_epoch:
            for source in ("gnss", "vio_compare"):
                self.histories[source].clear()
                self.received.pop("pose:" + source, None)
            self._comparison_epoch = key
        for sample in comparison_samples(event):
            source = sample["source"]
            history = self.histories[source]
            if history and sample["t"] <= history[-1]["t"]:
                continue
            # Collector time is preserved. Relaying cached diagnostics must not
            # make an old GNSS measurement appear to arrive at the current time.
            age = max(0., float(event["t"]) - sample["t"])
            measured_age = event.get("measurement_age_s", 0.)
            if finite_number(measured_age):
                age += max(0., measured_age)
            if sample["t"] > float(event["t"]) + .1:
                continue
            history.append(sample)
            self.received["pose:" + source] = received_at - age

    def ensure_log_reader(self, board, host):
        # One low-rate external log reader per board, only on the operator PC.
        # Spectators receive its bounded text through the existing relay.
        if self.spectator_mode or self.args.no_ssh or not host:
            return
        previous = self.log_workers.get(board)
        if previous is not None and previous.host == host:
            return
        if previous is not None:
            previous.stop()
        from module_log_client import ModuleLogClient
        try:
            worker = ModuleLogClient(board, host, RUNTIME_DIR, self)
        except ValueError:
            return
        self.log_workers[board] = worker
        worker.start()

    def reset_view(self):
        self.ax.view_init(elev=24, azim=-60)
        self.plot_dirty = True

    def connect_planner(self):
        if self.spectator_mode or self.args.no_ssh:
            return
        if self.planning_client is not None:
            self.planning_client.stop()
            return
        from planning_client import DEFAULT_PLANNING_COMMAND, PlanningClient
        try:
            client = PlanningClient(self.planning_host.text(),
                                    getattr(self.args, "planning_command", DEFAULT_PLANNING_COMMAND),
                                    RUNTIME_DIR, self)
        except ValueError as error:
            self.planning_message.setText(str(error))
            return
        self.planning_client = client
        self.ensure_log_reader("pi4", self.planning_host.text())
        self.planning_stale_disarmed = False
        client.packet.connect(lambda value: self.on_packet("pi4", value))
        client.progress.connect(self.planning_message.setText)
        client.closed.connect(lambda: self.on_planner_closed(client))
        self.planning_host.setEnabled(False)
        self.planning_connect.setText("플래너 종료")
        client.start()

    def on_planner_closed(self, client):
        if self.planning_client is not client:
            return
        self.planning_client = None
        self.planning_host.setEnabled(True)
        self.planning_connect.setText("플래너 연결")
        self.goal_send.setEnabled(False)
        self.goal_cancel.setEnabled(False)
        self.output_arm.setEnabled(False)
        self.output_disarm.setEnabled(False)
        self.selected_goal = None
        event = {"kind": "telemetry", "source": "planning", "t": time.time(), "frame": "odom",
                 "state": "DISCONNECTED", "reason": "플래너 연결 종료", "plan_only": True,
                 "armed": False, "actuator_enabled": False, "output_mode": "disconnected",
                 "output_inhibit": "연결 종료", "feedback_source": "unmeasured",
                 "path": [], "trajectory": []}
        self.on_packet("pi4", event)
        client.deleteLater()

    def select_goal(self, x, y, frame):
        if self.spectator_mode or self.args.no_ssh:
            return
        geometry = grid_geometry(self.panels["costmap"].event)
        if geometry is None or geometry[0]["frame_id"] != frame:
            self.planning_message.setText("목표 선택 대기 · 좌표가 확인된 지도가 필요합니다.")
            return
        from planning_client import goal_command
        try:
            goal_command(x, y, frame)
        except ValueError as error:
            self.planning_message.setText(str(error))
            return
        self.selected_goal = (x, y, frame)
        age = self.age("image", "costmap")
        suffix = "보내기 전" if self.goal_map_fresh() else (
            f"지도 갱신 지연 {age:.1f}초 · 목표 전송 가능 · 주행은 유효 입력 대기" if age is not None else "지도 미수신 · 목표 전송 가능 · 주행은 유효 입력 대기")
        self.planning_message.setText(f"선택 목표 ({x:.2f}, {y:.2f}) m · {frame} · {suffix}")
        for key in ("costmap", "slope"):
            self.panels[key].set_planning(self.telemetry.get("planning", {}), self.selected_goal)

    def goal_map_fresh(self):
        age = self.age("image", "costmap")
        event = self.images.get("costmap", {})
        return bool(age is not None and age <= 2 and not event.get("decode_error")
                    and grid_geometry(event) is not None)

    def send_goal(self):
        if self.spectator_mode or self.args.no_ssh or self.planning_client is None or self.selected_goal is None:
            return
        from planning_client import goal_command
        if self.planning_client.send(goal_command(*self.selected_goal)):
            self.planning_message.setText("목표 전송 · 지도 갱신 지연 · 주행은 플래너의 유효 입력 확인 대기"
                                          if not self.goal_map_fresh() else
                                          "목표 전송 · 지도/위치 확인 중 · 출력 허용 상태는 Pi4 보고값으로 표시")

    def cancel_goal(self):
        if self.spectator_mode or self.args.no_ssh or self.planning_client is None:
            return
        self.planning_client.send({"cmd": "disarm"})
        if self.planning_client.send({"cmd": "cancel"}):
            self.selected_goal = None
            self.planning_message.setText("출력 정지·목표 취소 요청 · Pi4 응답 대기")

    def arm_output(self):
        click_timing = (time.time_ns(), time.monotonic_ns())
        if self.spectator_mode or self.args.no_ssh or self.planning_client is None:
            return
        age = self.age("telemetry", "planning")
        report = self.telemetry.get("planning", {})
        if age is None or age > 3 or report.get("plan_only", True):
            self.planning_message.setText("출력 허용 불가 · 최신 출력 모드 상태를 먼저 확인해 주세요.")
            return
        if self.planning_client.send({"cmd": "arm"}, timing=click_timing, origin="output_allow_button"):
            self.planning_message.setText("출력 허용 요청 · Pi4 응답 대기 · 실제 모터 회전 확인값 아님")

    def disarm_output(self):
        click_timing = (time.time_ns(), time.monotonic_ns())
        if self.spectator_mode or self.args.no_ssh or self.planning_client is None:
            return
        if self.planning_client.send({"cmd": "disarm"}, timing=click_timing, origin="output_stop_button"):
            self.planning_message.setText("출력 정지 요청 · Pi4 응답 대기")

    def stop_trial(self):
        if self.spectator_mode or self.args.no_ssh or not self.args.enable_trial_control:
            return
        if self.cloud_switch_in_progress or self.slope_switch_in_progress:
            self.stop_message.setText("계산 또는 점군 방식 전환을 확인 중입니다. 완료 후 시험을 종료할 수 있습니다.")
            return
        if self.model_switch_in_progress:
            self.stop_message.setText("모델 전환 결과를 확인 중입니다. 완료 후 시험을 종료할 수 있습니다.")
            return
        if self.stop_worker is not None and self.stop_worker.isRunning():
            return
        if self.planning_client is not None:
            self.planning_client.stop()
        password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(self, "보드 접속", "SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted:
                return
        self.trial_stop_result = None
        self.stop_button.setEnabled(False)
        self.model_combo.setEnabled(False)
        self.model_button.setEnabled(False)
        self.set_cloud_controls_enabled(False)
        self.set_slope_controls_enabled(False)
        self.stop_button.setText("종료 중…")
        self.stop_message.setText("Pi5·Jetson의 실행 모듈을 종료하는 중… 보드 전원은 유지합니다.")
        self.stop_worker = TrialStopWorker(self.args.pi_host, self.args.jetson_host, password, self)
        self.stop_worker.progress.connect(self.stop_message.setText)
        self.stop_worker.result.connect(self.on_trial_stopped)
        self.stop_worker.start()

    def apply_model(self):
        if self.spectator_mode or self.args.no_ssh or not self.args.enable_trial_control or self.model_switch_in_progress or self.cloud_switch_in_progress or self.slope_switch_in_progress:
            return
        # A failed/partial stop also leaves the trial's running state uncertain.
        if self.trial_stop_result is not None or (self.stop_worker is not None and self.stop_worker.isRunning()):
            return
        password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(self, "보드 접속", "SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted:
                return
        model = self.model_combo.currentData()
        self.model_switch_in_progress = True
        self.model_switch_result = None
        self.active_model = None
        self.model_combo.setEnabled(False)
        self.model_button.setEnabled(False)
        self.model_button.setText("전환 중…")
        self.set_cloud_controls_enabled(False)
        self.set_slope_controls_enabled(False)
        self.stop_button.setEnabled(False)
        self.model_message.setText("인지 모델 전환 중… 카메라·VIO·지도·화면 수신은 유지합니다.")
        self.model_worker = ModelSwitchWorker(self.args.jetson_host, model, password, self)
        self.model_worker.result.connect(self.on_model_switched)
        self.model_worker.finished.connect(self.on_model_switch_finished)
        self.model_worker.start()

    @QtCore.pyqtSlot(dict)
    def on_model_switched(self, value):
        self.model_switch_result = value
        model = value.get("model")
        if value.get("ok") is True and model in ("b2", "fcn"):
            self.active_model = model
            self.model_message.setText("인지 모델: " + ("B2 · 기존" if model == "b2" else "FCN · 실내") + " · 전환 확인")
        else:
            self.active_model = None
            self.model_message.setText("모델 전환 미확인 · 현재 모델 미확인 (" + str(value.get("reason", "model_switch_failed")) + ")")
        self.write_state()

    @QtCore.pyqtSlot()
    def on_model_switch_finished(self):
        self.model_switch_in_progress = False
        if self.model_switch_result is None:
            self.on_model_switched({"ok": False, "state": "UNCONFIRMED", "model": None, "reason": "worker_ended_without_result"})
        can_switch = not self.spectator_mode and not self.args.no_ssh and self.trial_stop_result is None
        self.model_combo.setEnabled(can_switch)
        self.model_button.setEnabled(can_switch)
        self.model_button.setText("모델 적용")
        self.set_cloud_controls_enabled(can_switch and not self.cloud_switch_in_progress and not self.slope_switch_in_progress)
        self.set_slope_controls_enabled(can_switch and not self.cloud_switch_in_progress)
        self.stop_button.setEnabled(not self.spectator_mode and not self.args.no_ssh and
                                    not bool(self.trial_stop_result and self.trial_stop_result.get("ok")))
        self.write_state()

    def set_cloud_controls_enabled(self, enabled):
        if hasattr(self, "cloud_combo"):
            for widget in (self.cloud_combo, self.cloud_button, self.cloud_status_button):
                widget.setEnabled(enabled)

    def apply_cloud_source(self, checked=False, *, query=False):
        if (self.spectator_mode or self.args.no_ssh or not self.args.enable_trial_control or
                self.model_switch_in_progress or self.cloud_switch_in_progress or self.slope_switch_in_progress or
                self.trial_stop_result is not None or
                (self.stop_worker is not None and self.stop_worker.isRunning())):
            return
        password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(self, "보드 접속", "SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted:
                return
        source = "status" if query else self.cloud_combo.currentData()
        self.cloud_switch_in_progress = True
        self.cloud_switch_result = None
        self.active_cloud_source = None
        self.set_cloud_controls_enabled(False)
        self.set_slope_controls_enabled(False)
        self.model_combo.setEnabled(False)
        self.model_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.cloud_message.setText("선택한 방식의 새 점군을 확인하는 중… 카메라와 VIO 및 기존 지도는 유지합니다.")
        self.cloud_worker = CloudSwitchWorker(self.args.jetson_host, source, password, self)
        self.cloud_worker.result.connect(self.on_cloud_switched)
        self.cloud_worker.finished.connect(self.on_cloud_switch_finished)
        self.cloud_worker.start()

    @QtCore.pyqtSlot(dict)
    def on_cloud_switched(self, value):
        from cloud_control import SOURCE_NAMES
        self.cloud_switch_result = value
        source = value.get("source")
        confirmed = (value.get("ok") is True or value.get("rollback_confirmed") is True) and source in SOURCE_NAMES
        self.active_cloud_source = source if confirmed else None
        if value.get("ok") is True and confirmed:
            self.cloud_combo.setCurrentIndex(self.cloud_combo.findData(source))
            self.cloud_message.setText("OAK 점군: " + SOURCE_NAMES[source] + " 적용 확인. 기존 지도는 유지됩니다. 비교는 새 지도에서 진행하세요.")
        elif value.get("rollback_confirmed") is True and confirmed:
            self.cloud_message.setText("점군 전환 실패. " + SOURCE_NAMES[source] + " 복원 확인 (" + str(value.get("reason", "")) + ")")
        else:
            self.cloud_message.setText("OAK 점군 방식 미확인 (" + str(value.get("reason", "cloud_switch_failed")) + ")")
        self.write_state()

    @QtCore.pyqtSlot()
    def on_cloud_switch_finished(self):
        self.cloud_switch_in_progress = False
        if self.cloud_switch_result is None:
            self.on_cloud_switched({"ok": False, "state": "UNCONFIRMED", "source": None, "reason": "worker_ended_without_result"})
        allowed = not self.spectator_mode and not self.args.no_ssh and self.trial_stop_result is None
        self.set_cloud_controls_enabled(allowed and not self.model_switch_in_progress and not self.slope_switch_in_progress)
        self.set_slope_controls_enabled(allowed and not self.model_switch_in_progress)
        self.model_combo.setEnabled(allowed)
        self.model_button.setEnabled(allowed)
        self.stop_button.setEnabled(allowed)
        self.write_state()

    def ensure_slope_cpu_reader(self, password=None):
        if self.spectator_mode or self.args.no_ssh:
            return
        if self.slope_cpu_client is not None:
            thread = self.slope_cpu_client._thread
            if password is None and thread is not None and thread.is_alive():
                return
            if not self.slope_cpu_client.stop():
                return
        from slope_cpu_client import SlopeCpuClient
        self.slope_cpu_client = SlopeCpuClient(self.args.jetson_host, RUNTIME_DIR, password, self)
        self.slope_cpu_client.start()

    def set_slope_controls_enabled(self, enabled):
        if hasattr(self, "slope_combo"):
            for widget in (self.slope_combo, self.slope_button, self.slope_status_button):
                widget.setEnabled(enabled)

    def apply_slope_mode(self, checked=False, *, query=False):
        if (self.spectator_mode or self.args.no_ssh or not self.args.enable_trial_control or
                self.model_switch_in_progress or self.cloud_switch_in_progress or self.slope_switch_in_progress or
                self.trial_stop_result is not None or
                (self.stop_worker is not None and self.stop_worker.isRunning())):
            return
        password = os.environ.get("NOMAD_BOARD_PASSWORD1", "")
        if not password:
            password, accepted = QtWidgets.QInputDialog.getText(self, "보드 접속", "SSH 비밀번호", QtWidgets.QLineEdit.Password)
            if not accepted:
                return
        mode = "status" if query else self.slope_combo.currentData()
        self.slope_switch_in_progress = True
        self.slope_switch_result = None
        self.active_slope_mode = None
        self.set_slope_controls_enabled(False)
        self.set_cloud_controls_enabled(False)
        self.model_combo.setEnabled(False)
        self.model_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.slope_message.setText("경사 방식 확인 중. 계산 묶음 사이에 적용하며 기존 지도는 유지합니다.")
        self.ensure_slope_cpu_reader(password)
        self.slope_worker = SlopeSwitchWorker(self.args.jetson_host, mode, password, self)
        self.slope_worker.result.connect(self.on_slope_switched)
        self.slope_worker.finished.connect(self.on_slope_switch_finished)
        self.slope_worker.start()

    @QtCore.pyqtSlot(dict)
    def on_slope_switched(self, value):
        from slope_control import MODES
        self.slope_switch_result = value
        mode = value.get("mode")
        if value.get("ok") is True and value.get("state") == "SAVED_FOR_NEXT_START" and mode in MODES:
            self.active_slope_mode = None
            self.slope_combo.setCurrentIndex(self.slope_combo.findData(mode))
            self.slope_message.setText("경사 계산: " + MODES[mode] + " 저장. 다음 지도 시작부터 적용합니다.")
        elif value.get("ok") is True and value.get("state") == "MODE_CONFIRMED" and mode in MODES:
            self.active_slope_mode = mode
            self.slope_combo.setCurrentIndex(self.slope_combo.findData(mode))
            self.slope_message.setText("경사 계산: " + MODES[mode] + " 적용 확인. 기존 지도 유지. CPU는 개발 창 > 인지.")
        elif value.get("ok") is True and value.get("running") is False:
            self.active_slope_mode = None
            selected = value.get("selected_mode")
            if selected in MODES: self.slope_combo.setCurrentIndex(self.slope_combo.findData(selected))
            self.slope_message.setText("경사 worker 미실행. 저장된 방식: " + MODES.get(selected, "미확인"))
        else:
            self.active_slope_mode = mode if value.get("rollback_confirmed") is True else None
            self.slope_message.setText(("이전 경사 방식 복원 확인. " if self.active_slope_mode else "경사 방식 미확인. ") +
                                       str(value.get("reason", "worker_unconfirmed")))
        self.write_state()

    @QtCore.pyqtSlot()
    def on_slope_switch_finished(self):
        self.slope_switch_in_progress = False
        if self.slope_switch_result is None:
            self.on_slope_switched({"ok": False, "reason": "worker_ended_without_result"})
        allowed = not self.spectator_mode and not self.args.no_ssh and self.trial_stop_result is None
        self.set_slope_controls_enabled(allowed)
        self.set_cloud_controls_enabled(allowed)
        self.model_combo.setEnabled(allowed)
        self.model_button.setEnabled(allowed)
        self.stop_button.setEnabled(allowed)
        self.write_state()

    @QtCore.pyqtSlot(dict)
    def on_trial_stopped(self, value):
        self.trial_stop_result = value
        for worker in self.workers:
            worker.stop()
        success = value.get("ok") is True
        self.stop_button.setEnabled(not self.spectator_mode and not self.args.no_ssh and not success)
        self.stop_button.setText("종료 완료" if success else "종료 재시도")
        details = []
        for board, result in value.get("boards", {}).items():
            details.append(f"{board}: " + ("종료 확인" if result.get("ok") else str(result.get("reason") or "종료 미확인")))
        if success:
            self.stop_message.setText("시험 종료 완료 · Pi5·Jetson 보드 전원 유지")
        else:
            self.stop_message.setText("일부 보드 종료를 확인하지 못했습니다. " + " | ".join(details) +
                                     (" | " + str(value["reason"]) if value.get("reason") else ""))
        self.write_state()

    @QtCore.pyqtSlot(str, dict)
    def on_progress(self, board, value):
        self.ssh_progress[board] = value

    @QtCore.pyqtSlot(dict)
    def on_relay_progress(self, value):
        self.relay_progress = value

    def drain_receivers(self):
        if self.replay_mode:
            self.advance_replay()
            return
        # Network workers keep bounded per-stream mailboxes. This is the only
        # Qt ingestion wakeup: no per-frame queued signal or JPEG decode work.
        receivers = self.workers + ([self.relay_receiver] if self.relay_receiver is not None else [])
        for receiver in receivers:
            for board, event in receiver.inbox.drain():
                self.on_packet(board, event)

    @QtCore.pyqtSlot(str, dict)
    def on_packet(self, board, event):
        kind, source = event.get("kind"), event.get("source")
        if kind == "log":
            event = {**event, "_source_session": event.get("session", "legacy")}
        try:
            if not owns_stream(board, event):
                raise ValueError()
            relay_age = event.get("_relay_age_s", 0.)
            if not finite_number(relay_age) or relay_age < 0:
                raise ValueError()
            now = self.now()
            received_at = event.get("_local_received_monotonic", now - relay_age)
            if not finite_number(received_at) or received_at > now:
                raise ValueError()
            stream_key = (board, kind, source)
            if not self.capture_order.accept(stream_key, event, commit=False):
                raise ValueError()
            if kind == "pose":
                keys = ("t", "x", "y", "z", "qx", "qy", "qz", "qw")
                if source not in self.histories or not all(finite_number(event.get(k)) for k in keys):
                    raise ValueError()
                pose = {key: float(event[key]) for key in keys}
                pose["frame_id"] = str(event.get("frame_id", ""))
                pose["child_frame_id"] = str(event.get("child_frame_id", ""))
                norm = math.sqrt(sum(pose[key] ** 2 for key in ("qx", "qy", "qz", "qw")))
                if abs(norm - 1.0) > 0.02 or pose["t"] <= 0:
                    raise ValueError()
                previous = self.capture_order.latest.get(stream_key)
                if previous and previous[0] != event.get("_source_session", "legacy"):
                    self.histories[source].clear()
                self.histories[source].append(pose)
                if source == "public":
                    for key in ("costmap", "slope"):
                        self.panels[key].set_pose(self.histories["public"], relay_age)
                self.plot_dirty = True
            elif kind == "image":
                if source not in IMAGE_SOURCES or event.get("encoding") not in ("jpeg", "png"):
                    raise ValueError()
                if not finite_number(event.get("t")) or not isinstance(event.get("data"), str):
                    raise ValueError()
                self.images[source] = {k: event.get(k) for k in ("t", "encoding", "data", "width", "height", "grid", "georeferenced", "georeference_reason")}
                self.image_dirty.add(source)
            elif kind == "log":
                from relay import valid_log_event
                if not valid_log_event(board, event):
                    raise ValueError()
                self.module_logs[source].append({**{key: value for key, value in event.items()
                                                   if not key.startswith("_")}, "board": board})
            elif kind == "status":
                if source not in ("localization", "perception") or not finite_number(event.get("t")):
                    raise ValueError()
                self.statuses[source] = {"state": str(event.get("state", "UNKNOWN"))[:80],
                                         "detail": str(event.get("detail", ""))[:300], "t": event["t"]}
            elif kind == "rates":
                if source not in ("pi", "jetson") or not isinstance(event.get("rates"), dict):
                    raise ValueError()
                values = event["rates"]
                if not all(isinstance(k, str) and finite_number(v) and v >= 0 for k, v in values.items()):
                    raise ValueError()
                self.rates[source] = dict(values)
            elif kind == "telemetry":
                if source not in TELEMETRY_SOURCES or not finite_number(event.get("t")):
                    raise ValueError()
                # Small diagnostics only; do not retain image buffers or non-finite JSON values here.
                encoded = json.dumps(event, allow_nan=False)
                if len(encoded) > 65536:
                    raise ValueError()
                if source == "planning":
                    from planning_client import display_event, output_summary, stage_summary
                    clean = display_event(event)
                    self.telemetry[source] = clean
                    self.planning_stale_disarmed = False
                    for key in ("costmap", "slope"):
                        self.panels[key].set_planning(clean, self.selected_goal)
                    gpp, lpp = stage_summary(clean)
                    self.planning_message.setText(gpp + " | " + lpp + " | " + output_summary(clean))
                else:
                    self.telemetry[source] = json.loads(encoded)
                    if source == "fusion":
                        self.update_comparison_samples(event, received_at)
                    if source == "slope_cpu":
                        cpu = event.get("cpu", {})
                        mode = event.get("active_mode")
                        if (mode in ("sequential", "parallel") and cpu.get("state") == "live" and
                                finite_number(cpu.get("percent_one_core"))):
                            self.slope_cpu_history[mode] = {"percent_one_core": cpu["percent_one_core"],
                                                            "t": event["t"], "received": received_at}
            else:
                raise ValueError()
        except (ValueError, OverflowError, TypeError):
            self.invalid_events += 1
            return
        self.capture_order.accept(stream_key, event)
        self.received[kind + ":" + source] = received_at
        self.accepted_events += 1
        self.record_packet(board, event, received_at)
        if self.relay_server is not None:
            try:
                # Fan out the validated raw packet once, not a rendered snapshot.
                # Capture t is untouched; the backend adds cache residence age.
                forwarded = dict(event)
                if "_local_received_monotonic" in forwarded:
                    forwarded.pop("_local_received_monotonic")
                    forwarded["_relay_age_s"] = max(0., time.monotonic() - received_at)
                self.relay_server.publish(board, forwarded)
            except Exception:
                self.relay_error = "relay_publish_failed"

    def age(self, kind, source):
        received = self.received.get(kind + ":" + source)
        return None if received is None else max(0.0, self.now() - received)

    def age_text(self, kind, source):
        age = self.age(kind, source)
        return "WAITING" if age is None else f"age {age:.1f}s" + (" STALE" if age > 1.0 else "")

    def update_plot(self):
        all_points = []
        changed = False
        for source in POSE_SOURCES:
            history = self.histories[source]
            if not history:
                continue
            # Keep every received pose; only drawing is decimated at high counts.
            key = history_signature(history)
            points = self._plot_samplers[source].sample(history)
            if key != self._plot_signatures.get(source):
                xyz = [[p[key] for p in points] for key in ("x", "y", "z")]
                self.lines[source].set_data_3d(*xyz)
                last = history[-1]
                self.markers[source].set_data_3d([last["x"]], [last["y"]], [last["z"]])
                self._plot_signatures[source] = key
                changed = True
            all_points.extend(points)
        if all_points and self.auto_fit.isChecked():
            self._fit_initialized = fit_axes_3d(self.ax, all_points, initialized=self._fit_initialized)
        if changed:
            self.canvas.draw_idle()
        self.plot_dirty = False

    def decode_images(self, sources=None):
        """Preserve the last valid preview, including across replay time jumps."""
        for source in tuple(self.image_dirty if sources is None else sources):
            if source not in self.image_dirty:
                continue
            event = self.images[source]
            try:
                payload = (event["encoding"], event["data"])
                if self._decoded_payloads.get(source) == payload:
                    image = self.latest_qimages[source]
                else:
                    data = base64.b64decode(event["data"], validate=True)
                    image = QtGui.QImage.fromData(data, event["encoding"].upper().encode("ascii"))
                if image.isNull():
                    raise ValueError()
                if source in ("costmap", "slope"):
                    self.panels[source].set_image(image, event)
                elif source in self.panels:
                    self.panels[source].set_image(image)
                self.latest_qimages[source] = image
                self._decoded_payloads[source] = payload
                event["decoded_width"], event["decoded_height"] = image.width(), image.height()
                event.pop("decode_error", None)
            except (ValueError, TypeError):
                event["decode_error"] = True
                self.invalid_events += 1
                if source in self.panels:
                    self.panels[source].meta.setText("INVALID image payload — last valid image retained")
            self.image_dirty.discard(source)

    def refresh(self):
        self.record_view()
        self.update_recording_bar()
        if self.slope_cpu_client is not None:
            for board, event in self.slope_cpu_client.drain():
                self.on_packet(board, event)
        for worker in self.log_workers.values():
            for board, event in worker.drain():
                self.on_packet(board, event)
        planning_age = self.age("telemetry", "planning")
        client = self.planning_client
        connected = bool(client is not None and client.ready and not client.stopping and
                         planning_age is not None and planning_age <= 3)
        controllable = not self.spectator_mode and not self.args.no_ssh
        map_fresh = self.goal_map_fresh()
        goal_connected = bool(client is not None and client.ready and not client.stopping)
        self.goal_send.setEnabled(goal_connected and self.selected_goal is not None and controllable)
        self.goal_send.setToolTip("지도 갱신 지연 · 목표 전송은 가능하며 실제 주행은 플래너가 입력 유효성을 확인합니다."
                                 if not map_fresh else "선택한 odom 목표를 Pi4 플래너로 전송합니다.")
        self.goal_cancel.setEnabled(connected and controllable)
        report = self.telemetry.get("planning", {})
        self.output_arm.setEnabled(connected and controllable and not report.get("plan_only", True)
                                   and report.get("armed") is False)
        self.output_disarm.setEnabled(bool(controllable and client is not None and not client.stopping))
        self.panels["costmap"].goal_selection_enabled = not self.spectator_mode and not self.args.no_ssh
        if planning_age is not None and planning_age > 3:
            if controllable and client is not None and not self.planning_stale_disarmed:
                client.send({"cmd": "disarm"})
                self.planning_stale_disarmed = True
            self.planning_message.setText("플래너 상태 수신 지연 · 출력 정지 요청 · 마지막 허용 상태는 현재 상태가 아님")
            for key in ("costmap", "slope"):
                if self.panels[key].planning:
                    from planning_client import planning_for_display
                    self.panels[key].set_planning(planning_for_display(report, planning_age), self.selected_goal)
            self.planning_message.setText("플래너 수신 오래됨 · 출력 정지 요청 · 남은 경로는 참고용")
        summaries = []
        for source in ("localization", "perception"):
            status = self.statuses.get(source)
            state = "WAITING" if status is None else status["state"]
            summaries.append(f"{source.upper()}: {state} ({self.age_text('status', source)})")
        self.status_label.setText("  |  ".join(summaries))
        if self.trial_stop_result is not None:
            self.status_label.setText("시험 종료 완료 · 화면에 마지막 결과 유지" if self.trial_stop_result.get("ok") else
                                      "시험 종료 일부 미확인 · 종료 재시도 가능")
        self.status_label.setToolTip("\n".join(f"{k}: {v['detail']}" for k, v in self.statuses.items()))
        mode = localization_summary({"now": self.now(), "received": self.received,
                                     "statuses": self.statuses, "telemetry": self.telemetry})
        self.localization_mode_label.setText("위치 모드 · " + mode["label"] + "  |  " + mode["orientation_text"])
        self.localization_mode_label.setStyleSheet("font-size:13px; font-weight:600; color:" + mode["color"] + ";")
        self.localization_mode_label.setToolTip(mode["detail"])
        summaries = []
        for board in ("pi", "jetson"):
            rates = self.rates.get(board)
            rates_text = "WAITING" if rates is None else "  ".join(f"{k} {v:.1f}" for k, v in rates.items())
            summaries.append(f"{board.upper()} Hz: {rates_text} ({self.age_text('rates', board)})")
        self.rate_label.setText("  |  ".join(summaries))
        self.ssh_label.setText("  |  ".join(f"{board} SSH: {v.get('phase', 'pending')}"
                                            + (f" [{v['error_code']}]" if v.get("error_code") else "")
                                            for board, v in self.ssh_progress.items()))
        if self.replay_mode:
            self.relay_label.setText("기록 재생 · 보드 접속 없음")
        elif self.spectator_mode:
            host, port = self.relay_address
            state = self.relay_progress.get("state", "pending")
            self.relay_label.setText(f"읽기 전용 관전 | 운용 PC {host}:{port} | 중계 {state}")
        elif self.relay_server is not None:
            status = self.relay_server.status()
            port = status.get("port", getattr(self.args, "relay_listen", "?"))
            address_text = " / ".join(f"{host}:{port}" for host in self.relay_local_addresses) or f"{socket.gethostname()}:{port}"
            self.relay_label.setText(f"운용 PC 중계 | {address_text} | 관전 PC {status.get('client_count', 0)}대" +
                                     (" | 중계 전송 오류" if self.relay_error else ""))
        else:
            self.relay_label.setText("직접 연결 · 중계 꺼짐")
        self.pose_label.setText("  |  ".join(f"{s.upper()} {len(self.histories[s])} poses / {self.age_text('pose', s)}"
                                             for s in POSE_SOURCES))
        heading_sources = ("public", "fusion", "vio")
        heading_source = next((s for s in heading_sources if self.histories[s] and
                               self.age("pose", s) is not None and self.age("pose", s) <= .5), None)
        if heading_source is None:
            heading_source = next((s for s in heading_sources if self.histories[s]), "public")
        heading_pose = self.histories[heading_source][-1] if self.histories[heading_source] else None
        self.heading_indicator.set_data(heading_pose, self.forward_axis, self.age("pose", heading_source),
                                        diagnostics=self.telemetry.get("fusion"),
                                        diagnostics_age_s=self.age("telemetry", "fusion"), source=heading_source)
        if self.plot_dirty:
            self.update_plot()
        self.decode_images()
        for source, panel in self.panels.items():
            image = self.images.get(source)
            if image and image.get("decode_error"):
                panel.meta.setText("INVALID image payload | " + self.age_text("image", source))
                continue
            dims = "" if image is None else f" | {image.get('decoded_width', '?')}×{image.get('decoded_height', '?')}"
            panel.meta.setText(self.age_text("image", source) + dims)
            if source in ("costmap", "slope"):
                was_fresh = panel.pose_age is not None and panel.pose_age <= 1
                panel.pose_age = self.age("pose", "public")
                if was_fresh != (panel.pose_age is not None and panel.pose_age <= 1):
                    panel.dirty = True
                if image and not grid_geometry(image):
                    panel.meta.setText(self.age_text("image", source) + " | " + str(image.get("georeference_reason") or "waiting for actual map geometry"))
                panel.redraw()
        snapshot = self.developer_snapshot()
        for window in self.developer_windows.values():
            if window.isVisible():
                window.update_snapshot(snapshot)
        if time.monotonic() - self.last_state_write >= 1.0:
            self.write_state()
            self.last_state_write = time.monotonic()

    def state(self):
        return {"schema": "nomad_laptop_monitor_v1", "real_data_only": True,
                "recording": self.recorder.status() if self.recorder else self.recording_result,
                "playback": {"enabled": self.replay_mode, "position_s": self._replay_position,
                             "file": str(self.args.replay) if self.replay_mode else None},
                "ingress": {**{worker.board: worker.inbox.status() for worker in self.workers},
                            **({"relay": self.relay_receiver.inbox.status()} if self.relay_receiver else {})},
                "started_wall_s": self.started, "saved_wall_s": time.time(),
                "accepted_events": self.accepted_events, "invalid_events": self.invalid_events,
                "max_poses_per_source": MAX_POSES, "ui_hz": self.args.ui_hz,
                "heading_display": {"forward_axis_in_pose_frame": self.forward_axis, "note": self.heading_note,
                                    "compass": self.heading_indicator.state()},
                "trial_control": {"enabled": self.args.enable_trial_control and not self.spectator_mode and not self.replay_mode,
                                  "stop_result": self.trial_stop_result},
                "relay": {"mode": "spectator" if self.spectator_mode else
                          ("operator" if self.relay_server is not None else "direct"),
                          "read_only": self.spectator_mode,
                          "connection": dict(self.relay_progress) if self.spectator_mode else
                          (self.relay_server.status() if self.relay_server is not None else None),
                          "error": self.relay_error},
                "slope_compute_mode": {"active": self.active_slope_mode, "switching": self.slope_switch_in_progress,
                                       "last_switch_result": self.slope_switch_result},
                "oak_cloud_source": {"active": self.active_cloud_source, "switching": self.cloud_switch_in_progress,
                                     "last_switch_result": self.cloud_switch_result},
                "semantic_model": {"active": self.active_model, "switching": self.model_switch_in_progress,
                                   "last_switch_result": self.model_switch_result},
                "ssh": self.ssh_progress,
                "poses": {source: {"count": len(history), "latest": history[-1] if history else None,
                                    "receive_age_s": self.age("pose", source)}
                          for source, history in self.histories.items()},
                "images": {source: {**{k: v for k, v in self.images.get(source, {}).items() if k != "data"},
                                     "receive_age_s": self.age("image", source)} for source in IMAGE_SOURCES},
                "statuses": {k: {"state": v["state"], "t": v["t"],
                                  "receive_age_s": self.age("status", k)} for k, v in self.statuses.items()},
                "rates": self.rates,
                "telemetry": {key: {**value, "receive_age_s": self.age("telemetry", key)}
                              for key, value in self.telemetry.items()},
                "developer_windows": {key: window.isVisible() for key, window in self.developer_windows.items()}}

    def write_state(self):
        try:
            if self.args.state_file:
                atomic_json(self.args.state_file, self.state())
            if self.args.ssh_progress_file:
                atomic_json(self.args.ssh_progress_file,
                            {"saved_wall_s": time.time(), "boards": self.ssh_progress})
        except OSError:
            self.footer.setText("State save failed; check output path permissions.")

    def save_snapshot(self):
        self.refresh()
        destination = Path(self.args.snapshot).expanduser().resolve()
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            self.canvas.draw()
            if not self.grab().save(str(destination)):
                raise OSError()
            self.write_state()
        except OSError:
            self.footer.setText("Snapshot save failed; check output path permissions.")

    def closeEvent(self, event):
        if self.planning_client is not None:
            self.planning_client.stop()
            self.footer.setText("Pi4 플래너 연결 종료 중…")
            QtCore.QTimer.singleShot(150, self.close)
            event.ignore()
            return
        if self.heading_reset_in_progress or self.vio_restart_in_progress:
            self.footer.setText("Pi5 위치추정 제어 응답을 확인 중입니다. 잠시 후 닫아주세요.")
            QtCore.QTimer.singleShot(150, self.close)
            event.ignore()
            return
        if self.cloud_switch_in_progress or self.slope_switch_in_progress:
            self.stop_message.setText("계산 또는 점군 방식 전환을 확인 중입니다. 완료 후 창을 닫을 수 있습니다.")
            event.ignore()
            return
        if self.model_switch_in_progress:
            self.model_message.setText("모델 전환 결과를 확인 중입니다. 완료 후 창을 닫을 수 있습니다.")
            event.ignore()
            return
        if self.stop_worker is not None and self.stop_worker.isRunning():
            self.stop_message.setText("보드 종료를 처리 중입니다. 완료 후 창을 닫을 수 있습니다.")
            event.ignore()
            return
        self.timer.stop()
        if self.slope_cpu_client is not None:
            self.slope_cpu_client.stop()
        self.ingest_timer.stop()
        for worker in self.log_workers.values():
            worker.stop()
        self.ingest_timer.stop()
        if self.relay_receiver is not None:
            self.relay_receiver.stop()
            if not self.relay_receiver.wait(2500):
                # Never destroy a running QThread. Closing the client should
                # unblock it promptly; allow a short retry if the OS is delayed.
                self.footer.setText("중계 수신 연결을 종료하는 중…")
                QtCore.QTimer.singleShot(100, self.close)
                event.ignore()
                return
        if self.relay_server is not None:
            self.relay_server.close()
        for window in self.developer_windows.values():
            window.close()
        for worker in self.workers:
            worker.stop()
        for worker in self.workers:
            worker.wait(4500)
        self.close_recording()
        self.write_state()
        event.accept()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi-host", default="user@192.168.3.144")
    parser.add_argument("--jetson-host", default="nvidia@192.168.1.152")
    parser.add_argument("--pi-command", default="", help="Read-only C++ monitor_probe command supplied by operator")
    parser.add_argument("--jetson-command", default="", help="Read-only C++ monitor_probe command supplied by operator")
    parser.add_argument("--planning-host", default="", help="Pi4 user@address; connects only after the planner button is pressed")
    from planning_client import DEFAULT_PLANNING_COMMAND
    parser.add_argument("--planning-command", default=DEFAULT_PLANNING_COMMAND)
    parser.add_argument("--ui-hz", type=float, default=5.0)
    parser.add_argument("--state-file", help="Optional JSON state file, updated once a second")
    parser.add_argument("--ssh-progress-file", help="Optional SSH phase/counter JSON; no raw commands, stderr or credentials")
    parser.add_argument("--snapshot", help="Save current whole-window PNG once after startup")
    parser.add_argument("--snapshot-after-s", type=float, default=2.0)
    parser.add_argument("--snapshot-interval-s", type=float, default=0.0,
                        help="Optionally refresh the snapshot PNG periodically")
    parser.add_argument("--quit-after-s", type=float, default=0.0, help="Local UI initialization check only")
    parser.add_argument("--no-ssh", action="store_true", help="Initialize empty waiting UI without starting SSH")
    parser.add_argument("--replay", metavar="FILE.nomad", help="Replay a local recording; never connect to boards or send commands")
    relay = parser.add_mutually_exclusive_group()
    relay.add_argument("--relay-listen", type=int, metavar="PORT", help="Operator PC: share accepted packets on all interfaces")
    relay.add_argument("--relay-host", metavar="HOST:PORT", help="Spectator: read-only TCP connection to the operator PC; no board SSH")
    parser.add_argument("--open-developer-window", action="append", default=[],
                        choices=("localization", "perception", "traversability", "planning", "control"),
                        help="Open an independent developer window using the existing collectors; repeatable")
    parser.add_argument("--preview-heading-calibration", help="Explicit IMU=base preview only: use saved Basalt T_imu_cam[0] optical +Z for the front arrow; does not change ROS poses or TF")
    parser.add_argument("--enable-trial-control", action="store_true", help="Show the explicit remote trial stop button; board power stays on")
    args = parser.parse_args()
    if args.replay:
        args.no_ssh = True
        args.relay_listen = args.relay_host = None
        args.enable_trial_control = False
        args.preview_heading_calibration = None
    if args.relay_listen is not None and not 1 <= args.relay_listen <= 65535:
        parser.error("--relay-listen must be between 1 and 65535")
    if args.relay_host:
        try:
            relay_endpoint(args.relay_host)
        except ValueError as error:
            parser.error("--relay-host: " + str(error))
    if not math.isfinite(args.ui_hz) or not 1.0 <= args.ui_hz <= 10.0:
        parser.error("--ui-hz must be between 1 and 10")
    if not math.isfinite(args.snapshot_after_s) or args.snapshot_after_s < 0:
        parser.error("--snapshot-after-s must be finite and nonnegative")
    if not math.isfinite(args.snapshot_interval_s) or args.snapshot_interval_s < 0:
        parser.error("--snapshot-interval-s must be finite and nonnegative")
    if not math.isfinite(args.quit_after_s) or args.quit_after_s < 0:
        parser.error("--quit-after-s must be finite and nonnegative")
    return args


def main():
    args = parse_args()
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("NOMAD live monitor")
    try:
        window = MonitorWindow(args)
    except (OSError, ValueError, RuntimeError) as error:
        if not args.replay:
            raise
        QtWidgets.QMessageBox.critical(None, "기록 열기 실패", str(error))
        return 1
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
