"""Small built-in player for downloaded front videos (no codec or app install).

Front videos are MJPEG: every frame is a plain JPEG, which Qt decodes itself.
Frames are read on demand from 원본/seg_NNNN.mjpeg (+ .csv offsets and stamps)
or, when only an AVI exists, from its '00dc' chunks.
"""
from __future__ import annotations

import csv
import shutil
import struct
import subprocess
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets

VIDEO_TYPES = ("video/x-msvideo", "video/avi")


def frame_index(folder):
    """[(path, offset, length, stamp_ns or None)] for one downloaded trial folder."""
    folder = Path(folder)
    raw = folder / "원본" if (folder / "원본").is_dir() else folder
    frames = []
    for video in sorted(raw.glob("seg_*.mjpeg")):
        table = video.with_suffix(".csv")
        rows = list(csv.DictReader(table.open(encoding="utf-8"))) if table.is_file() else []
        size = video.stat().st_size
        if rows and "offset" in rows[0]:
            frames += [(video, int(r["offset"]), int(r["bytes"]), int(r["stamp_ns"])) for r in rows
                       if int(r["offset"]) + int(r["bytes"]) <= size]
    if frames:
        return frames
    for video in sorted(folder.glob("전체*.avi")) or sorted(raw.glob("seg_*.avi")):
        frames += avi_frames(video)
    return frames


def avi_frames(path):
    """JPEG chunk positions inside an MJPEG AVI (walks the RIFF tree)."""
    frames = []
    with open(path, "rb") as stream:
        def walk(start, end):
            position = start
            while position + 8 <= end:
                stream.seek(position)
                header = stream.read(12)
                if len(header) < 8:
                    return
                tag, size = header[:4], struct.unpack("<I", header[4:8])[0]
                if tag in (b"RIFF", b"LIST"):
                    walk(position + 12, min(end, position + 8 + size))
                elif tag[2:] in (b"dc", b"db") and size > 4:
                    frames.append((Path(path), position + 8, size, None))
                position += 8 + size + (size & 1)
        stream.seek(0, 2)
        walk(0, stream.tell())
    return frames


def system_player_available():
    """True when the desktop has an app registered for AVI (xdg-open would work)."""
    if not shutil.which("xdg-open"):
        return False
    for mime in VIDEO_TYPES:
        try:
            out = subprocess.run(["xdg-mime", "query", "default", mime], stdout=subprocess.PIPE,
                                 stderr=subprocess.DEVNULL, timeout=3).stdout.decode().strip()
        except (OSError, subprocess.SubprocessError):
            out = ""
        if out:
            return True
    return False


class PlayerDialog(QtWidgets.QDialog):
    def __init__(self, folder, title="", parent=None):
        super().__init__(parent)
        self.frames = frame_index(folder)
        self.position, self.cache_path, self.cache_file = 0, None, None
        self.setWindowTitle("전방 영상 · " + (title or Path(folder).name))
        self.resize(720, 500)
        layout = QtWidgets.QVBoxLayout(self)
        self.view = QtWidgets.QLabel("영상 없음")
        self.view.setAlignment(QtCore.Qt.AlignCenter)
        self.view.setMinimumSize(320, 180)
        self.view.setStyleSheet("background:#111;color:#ccc;")
        layout.addWidget(self.view, 1)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, max(0, len(self.frames) - 1))
        self.slider.valueChanged.connect(self.seek)
        layout.addWidget(self.slider)
        row = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("▶ 재생")
        self.play_button.clicked.connect(self.toggle)
        self.speed = QtWidgets.QComboBox()
        for label, value in (("0.5배", 0.5), ("1배", 1.0), ("2배", 2.0), ("4배", 4.0)):
            self.speed.addItem(label, value)
        self.speed.setCurrentIndex(1)
        self.time = QtWidgets.QLabel("")
        row.addWidget(self.play_button)
        row.addWidget(self.speed)
        row.addWidget(self.time, 1)
        layout.addLayout(row)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.next_frame)
        stamps = [f[3] for f in self.frames if f[3] is not None]
        self.fps = 10.0
        if len(stamps) > 1 and stamps[-1] > stamps[0]:
            self.fps = max(1.0, min(30.0, (len(stamps) - 1) / ((stamps[-1] - stamps[0]) / 1e9)))
        if self.frames:
            self.seek(0)
        else:
            self.play_button.setEnabled(False)

    def read(self, index):
        path, offset, length, _ = self.frames[index]
        if self.cache_path != path:
            if self.cache_file:
                self.cache_file.close()
            self.cache_file, self.cache_path = open(path, "rb"), path
        self.cache_file.seek(offset)
        return self.cache_file.read(length)

    def seek(self, index):
        if not self.frames:
            return
        self.position = max(0, min(index, len(self.frames) - 1))
        image = QtGui.QImage.fromData(self.read(self.position), "JPG")
        if not image.isNull():
            pixmap = QtGui.QPixmap.fromImage(image).scaled(self.view.size(), QtCore.Qt.KeepAspectRatio,
                                                           QtCore.Qt.SmoothTransformation)
            self.view.setPixmap(pixmap)
        seconds = self.position / self.fps
        total = (len(self.frames) - 1) / self.fps
        self.time.setText(f"{int(seconds // 60)}:{seconds % 60:04.1f} / {int(total // 60)}:{total % 60:04.1f}"
                          f" · {self.position + 1}/{len(self.frames)}")
        if self.slider.value() != self.position:
            self.slider.blockSignals(True)
            self.slider.setValue(self.position)
            self.slider.blockSignals(False)

    def toggle(self):
        if self.timer.isActive():
            self.timer.stop()
            self.play_button.setText("▶ 재생")
        else:
            if self.position >= len(self.frames) - 1:
                self.seek(0)
            self.timer.start(max(5, round(1000 / (self.fps * self.speed.currentData()))))
            self.play_button.setText("❚❚ 멈춤")

    def next_frame(self):
        if self.position >= len(self.frames) - 1:
            self.toggle()
            return
        self.timer.setInterval(max(5, round(1000 / (self.fps * self.speed.currentData()))))
        self.seek(self.position + 1)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.frames:
            self.seek(self.position)

    def closeEvent(self, event):
        self.timer.stop()
        if self.cache_file:
            self.cache_file.close()
        event.accept()


class InstallVlcDialog(QtWidgets.QDialog):
    """Installs VLC with the system's own admin password prompt (pkexec)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("VLC 설치")
        self.resize(520, 300)
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("VLC 영상 플레이어를 설치합니다. 관리자 비밀번호 창이 뜹니다.\n"
                                          "인터넷 연결이 필요하며 1~3분 걸릴 수 있습니다."))
        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log, 1)
        self.close_button = QtWidgets.QPushButton("닫기")
        self.close_button.clicked.connect(self.close)
        layout.addWidget(self.close_button)
        self.ok = False
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(
            lambda: self.log.appendPlainText(bytes(self.process.readAllStandardOutput()).decode("utf-8", "replace").rstrip()))
        self.process.finished.connect(self.done_install)
        self.close_button.setEnabled(False)
        self.process.start("pkexec", ["env", "DEBIAN_FRONTEND=noninteractive", "apt-get", "install", "-y", "vlc"])

    def done_install(self, code, _status):
        self.ok = code == 0 and bool(shutil.which("vlc"))
        self.log.appendPlainText("\n설치 완료" if self.ok else f"\n설치되지 않았습니다 (코드 {code}). "
                                 "비밀번호 창을 취소했거나 인터넷 연결을 확인해 주세요.")
        self.close_button.setEnabled(True)

    def closeEvent(self, event):
        if self.process.state() != QtCore.QProcess.NotRunning:
            event.ignore()
            return
        event.accept()


def play(folder, title="", parent=None):
    """재생: system player if one is registered; otherwise ask (install VLC or play here)."""
    folder = Path(folder)
    videos = sorted(folder.glob("전체*.avi")) or sorted((folder / "원본").glob("seg_*.avi")) or \
        sorted(folder.glob("seg_*.avi"))
    if not videos and not frame_index(folder):
        QtWidgets.QMessageBox.information(parent, "재생", "재생할 영상이 없습니다. 목록에서 다시 받아 주세요.")
        return None
    if videos and system_player_available():
        subprocess.Popen(["xdg-open", str(videos[0])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return None
    if videos and shutil.which("vlc"):
        subprocess.Popen(["vlc", str(videos[0])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return None
    box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Question, "영상 플레이어 없음",
                                "이 노트북에는 AVI 영상을 열 플레이어가 설정돼 있지 않습니다.\n"
                                "VLC를 설치할까요? (관리자 비밀번호 필요)\n\n"
                                "설치 없이 GUI 안에서 바로 볼 수도 있습니다.", parent=parent)
    install = box.addButton("VLC 설치 후 재생", QtWidgets.QMessageBox.AcceptRole)
    inside = box.addButton("GUI 안에서 바로 보기", QtWidgets.QMessageBox.ActionRole)
    box.addButton("취소", QtWidgets.QMessageBox.RejectRole)
    box.setDefaultButton(inside)
    box.exec_()
    if box.clickedButton() is install:
        if not shutil.which("pkexec"):
            QtWidgets.QMessageBox.warning(parent, "VLC 설치", "관리자 권한 도구(pkexec)가 없습니다. "
                                          "터미널에서 'sudo apt install vlc'로 설치해 주세요.")
            return None
        dialog = InstallVlcDialog(parent)
        dialog.exec_()
        if dialog.ok and videos:
            subprocess.Popen(["vlc", str(videos[0])], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return None
    if box.clickedButton() is inside:
        player = PlayerDialog(folder, title, parent)
        player.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
        player.show()
        return player
    return None
