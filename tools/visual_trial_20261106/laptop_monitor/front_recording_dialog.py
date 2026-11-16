"""Front video list window (opened from the launcher "전방 녹화" and the main
window "목록·받기").

- Lists Pi5 trials with front video, plus trials already on this laptop
  (kept even after the Pi5 original is deleted).
- 이름·메모: title and comment shared through one notes file on Pi5, keyed by
  trial name; this laptop keeps its own copy. Notes are never deleted.
- Pi5에서 삭제: removes only the Pi5 original front_video after confirmation.
  Laptop copies are never deleted from the GUI.
- 노트북으로 받기 / 재생 / 받은 폴더 열기.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

from PyQt5 import QtCore, QtWidgets


class _Task(QtCore.QThread):
    """Runs one blocking call off the UI thread."""
    progress = QtCore.pyqtSignal(int)
    result = QtCore.pyqtSignal(dict)

    def __init__(self, function, parent=None, with_progress=False):
        super().__init__(parent)
        self.function, self.with_progress, self._cancel = function, with_progress, False

    def cancel(self):
        self._cancel = True

    def run(self):
        try:
            if self.with_progress:
                value = self.function(self.progress.emit, lambda: self._cancel)
            else:
                value = self.function()
        except Exception:
            value = {"ok": False, "reason": "local_control_error"}
        self.result.emit(value if isinstance(value, dict) else {"ok": False, "reason": "invalid_result"})


class NoteDialog(QtWidgets.QDialog):
    def __init__(self, run_label, title, comment, parent=None):
        super().__init__(parent)
        self.setWindowTitle("이름·메모 · " + run_label)
        layout = QtWidgets.QFormLayout(self)
        self.title = QtWidgets.QLineEdit(title)
        self.title.setMaxLength(120)
        self.title.setPlaceholderText("예: 우면산 S자 2회차 성공")
        self.comment = QtWidgets.QPlainTextEdit(comment)
        self.comment.setPlaceholderText("예: 중간에 사람 지나감, 30초쯤 VIO 흔들림")
        layout.addRow("이름", self.title)
        layout.addRow("메모", self.comment)
        layout.addRow(QtWidgets.QLabel("팀원 모두에게 보입니다 (Pi5 메모 파일에 저장). 영상을 지워도 메모는 남습니다."))
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)
        self.resize(460, 260)

    def values(self):
        return self.title.text().strip(), self.comment.toPlainText().strip()[:2000]


class FrontRecordingDialog(QtWidgets.QDialog):
    COLUMNS = ("이름", "시작 시각", "길이", "크기", "Pi5", "노트북", "공유폴더", "메모")

    def __init__(self, pi_host, password=None, parent=None):
        super().__init__(parent)
        self.pi_host, self.password = pi_host, password or None
        self.rows, self.notes, self.task, self.task_kind = [], {}, None, None
        self.shared_root, self.shared = None, set()
        self.setWindowTitle("전방 녹화 목록 · Pi5")
        self.resize(980, 460)
        layout = QtWidgets.QVBoxLayout(self)
        self.summary = QtWidgets.QLabel(f"Pi5 {pi_host} 의 시험별 전방 영상과 이 노트북에 받은 영상입니다. "
                                        "목록·받기·메모·삭제는 Pi5가 켜져 있어야 합니다.")
        self.summary.setWordWrap(True)
        layout.addWidget(self.summary)
        self.table = QtWidgets.QTableWidget(0, len(self.COLUMNS))
        self.table.setHorizontalHeaderLabels(self.COLUMNS)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.itemSelectionChanged.connect(self.update_buttons)
        self.table.doubleClicked.connect(lambda _index: self.edit_note())
        layout.addWidget(self.table, 1)
        self.status = QtWidgets.QLabel("")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        row = QtWidgets.QHBoxLayout()
        self.refresh_button = QtWidgets.QPushButton("새로고침")
        self.download_button = QtWidgets.QPushButton("노트북으로 받기")
        self.cancel_button = QtWidgets.QPushButton("받기·올리기 취소")
        self.play_button = QtWidgets.QPushButton("재생")
        self.note_button = QtWidgets.QPushButton("이름·메모")
        self.delete_button = QtWidgets.QPushButton("Pi5에서 삭제")
        self.delete_button.setToolTip("Pi5 원본만 지웁니다. 노트북에 받은 영상과 메모는 지우지 않습니다.")
        self.open_button = QtWidgets.QPushButton("받은 폴더 열기")
        self.upload_button = QtWidgets.QPushButton("공유폴더로 올리기")
        self.upload_button.setToolTip("이 노트북에 받은 영상을 팀 공유폴더 주행기록/전방녹화로 복사합니다. 노트북 사본은 남습니다.")
        self.shared_button = QtWidgets.QPushButton("공유폴더 열기")
        close = QtWidgets.QPushButton("닫기")
        for button, slot in ((self.refresh_button, self.refresh), (self.download_button, self.download),
                             (self.cancel_button, self.cancel_download), (self.play_button, self.play),
                             (self.note_button, self.edit_note), (self.delete_button, self.delete),
                             (self.open_button, self.open_folder), (self.upload_button, self.upload),
                             (self.shared_button, self.open_shared), (close, self.close)):
            button.clicked.connect(slot)
        for button in (self.refresh_button, self.download_button, self.cancel_button, self.play_button,
                       self.note_button, self.delete_button, self.open_button, self.upload_button,
                       self.shared_button):
            row.addWidget(button)
        row.addStretch(1)
        row.addWidget(close)
        layout.addLayout(row)
        self.update_buttons()
        QtCore.QTimer.singleShot(0, self.refresh)

    # -------------------------------------------------------------- state --
    def busy(self):
        return self.task is not None and self.task.isRunning()

    def selected(self):
        rows = self.table.selectionModel().selectedRows() if self.table.selectionModel() else []
        return self.rows[rows[0].row()] if rows and rows[0].row() < len(self.rows) else None

    def update_buttons(self):
        from front_recording import downloaded
        row, busy = self.selected(), self.busy()
        on_pi5 = row is not None and row.get("pi5") is not None
        self.refresh_button.setEnabled(not busy)
        self.download_button.setEnabled(not busy and on_pi5)
        self.cancel_button.setEnabled(busy and self.task_kind in ("download", "upload"))
        self.play_button.setEnabled(row is not None and (downloaded(row["name"]) or row["name"] in self.shared))
        self.upload_button.setEnabled(not busy and row is not None and self.shared_root is not None
                                      and downloaded(row["name"]) and row["name"] not in self.shared)
        self.shared_button.setEnabled(self.shared_root is not None)
        self.note_button.setEnabled(not busy and row is not None)
        self.delete_button.setEnabled(not busy and on_pi5 and not row["pi5"].get("current_live"))

    def start(self, kind, function, on_result, with_progress=False):
        self.task_kind = kind
        self.task = _Task(function, self, with_progress)
        self.task.result.connect(on_result)
        self.task.finished.connect(self.update_buttons)
        self.task.start()
        self.update_buttons()

    # ------------------------------------------------------------ listing --
    def refresh(self):
        if self.busy():
            return
        from front_recording import check_front_recording, shared_front_root, shared_runs
        self.status.setText("Pi5 녹화 목록을 읽는 중…")

        def check():
            value = check_front_recording(self.pi_host, self.password)
            root = shared_front_root()
            return dict(value, shared_root=str(root) if root else "", shared_runs=shared_runs(root))
        self.start("check", check, self.show_runs)

    @QtCore.pyqtSlot(dict)
    def show_runs(self, value):
        from front_recording import local_notes, local_runs, remember_notes, summary_text
        if value.get("ok"):
            remember_notes(value.get("notes", {}))
        self.status.setText(summary_text(value))
        self.notes = {**local_notes(), **(value.get("notes", {}) if value.get("ok") else {})}
        pi5 = {run["name"]: dict(run, current_live=run.get("current") and run.get("state") == "RECORDING"
                                 and value.get("alive")) for run in (value.get("runs", []) if value.get("ok") else [])}
        self.shared_root = Path(value["shared_root"]) if value.get("shared_root") else None
        self.shared = set(value.get("shared_runs", []))
        if self.shared_root is not None:
            self.summary.setText(f"Pi5 {self.pi_host}, 이 노트북, 팀 공유폴더({self.shared_root})의 전방 영상입니다. "
                                 "목록·받기·메모·삭제는 Pi5가 켜져 있어야 합니다.")
        names = sorted(set(pi5) | set(local_runs()) | self.shared, reverse=True)
        self.rows = [{"name": name, "pi5": pi5.get(name)} for name in names]
        self.render()

    def render(self):
        from front_recording import STATES, _run_time, downloaded
        selected = self.selected()
        self.table.setRowCount(len(self.rows))
        for index, row in enumerate(self.rows):
            run, note = row["pi5"], self.notes.get(row["name"], {})
            if run is not None:
                state = STATES.get(run["state"], "상태 미확인")
                pi5 = "녹화 중" if run.get("current_live") else ("있음 · " + state)
                length, size = f"{run['duration_s'] / 60:.1f}분", f"{run['bytes'] / 1e9:.2f}GB"
            else:
                pi5, length, size = "없음 (삭제됨/연결 안 됨)", "", ""
            cells = (note.get("title", ""), _run_time(row["name"]), length, size, pi5,
                     "받음" if downloaded(row["name"]) else "", "있음" if row["name"] in self.shared else "",
                     note.get("comment", "").replace("\n", " "))
            for column, text in enumerate(cells):
                item = QtWidgets.QTableWidgetItem(text)
                item.setToolTip(row["name"] + (("\n\n" + note["comment"]) if note.get("comment") else ""))
                self.table.setItem(index, column, item)
        self.table.resizeColumnsToContents()
        if selected is not None:
            for index, row in enumerate(self.rows):
                if row["name"] == selected["name"]:
                    self.table.selectRow(index)
        self.update_buttons()

    # ----------------------------------------------------------- download --
    def download(self):
        row = self.selected()
        if row is None or row["pi5"] is None or self.busy():
            return
        from front_recording import download_run
        total = max(1, int(row["pi5"]["bytes"]))
        self.status.setText(f"{row['name']} 받는 중… 0%")
        self.start("download", lambda progress, cancelled: download_run(
            self.pi_host, row["name"], self.password, progress=progress, cancelled=cancelled), self.download_done,
            with_progress=True)
        self.task.progress.connect(lambda copied: self.status.setText(
            f"{row['name']} 받는 중… {min(99, copied * 100 // total)}% ({copied / 1e6:.0f}/{total / 1e6:.0f}MB)"))

    def cancel_download(self):
        if self.busy() and self.task_kind in ("download", "upload"):
            self.task.cancel()

    # ------------------------------------------------------ shared folder --
    def upload(self):
        from front_recording import downloaded, front_video_root, upload_run
        row = self.selected()
        if row is None or self.busy() or self.shared_root is None or not downloaded(row["name"]):
            return
        total = max(1, sum(p.stat().st_size for p in (front_video_root() / row["name"]).rglob("*") if p.is_file()))
        root = self.shared_root
        self.status.setText(f"{row['name']} 공유폴더로 올리는 중… 0%")
        self.start("upload", lambda progress, cancelled: upload_run(row["name"], root, progress, cancelled),
                   self.upload_done, with_progress=True)
        self.task.progress.connect(lambda copied: self.status.setText(
            f"{row['name']} 공유폴더로 올리는 중… {min(99, copied * 100 // total)}% "
            f"({copied / 1e6:.0f}/{total / 1e6:.0f}MB)"))

    @QtCore.pyqtSlot(dict)
    def upload_done(self, value):
        if value.get("ok"):
            self.shared.add(Path(value["folder"]).name)
            self.status.setText(f"공유폴더에 올림: {value['folder']} · {value['bytes'] / 1e6:.0f}MB (노트북 사본 유지)")
        else:
            reason = {"already_shared": "공유폴더에 이미 있습니다.", "cancelled": "취소했습니다.",
                      "not_downloaded": "먼저 노트북으로 받아 주세요."}.get(value.get("reason"), str(value.get("reason")))
            self.status.setText("올리기 안 됨: " + reason)
        self.render()

    def open_shared(self):
        if self.shared_root is None:
            return
        row = self.selected()
        folder = self.shared_root / row["name"] if row is not None and row["name"] in self.shared else self.shared_root
        self._open(folder)

    @QtCore.pyqtSlot(dict)
    def download_done(self, value):
        if value.get("ok"):
            video = (" · 영상: " + Path(value["combined"]).name) if value.get("combined") else ""
            self.status.setText(f"받기 완료: {value['folder']}{video} · {value['bytes'] / 1e6:.0f}MB")
            self.last_folder = value["folder"]
        else:
            self.status.setText("받기 실패: " + str(value.get("reason")))
        self.render()

    # --------------------------------------------------------- note/delete --
    def edit_note(self):
        row = self.selected()
        if row is None or self.busy():
            return
        from front_recording import _run_time, set_note
        note = self.notes.get(row["name"], {})
        dialog = NoteDialog(_run_time(row["name"]), note.get("title", ""), note.get("comment", ""), self)
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        title, comment = dialog.values()
        self.status.setText("메모를 Pi5에 저장하는 중…")
        self.start("note", lambda: set_note(self.pi_host, row["name"], title, comment, self.password), self.note_done)

    @QtCore.pyqtSlot(dict)
    def note_done(self, value):
        if value.get("ok"):
            from front_recording import local_notes
            self.notes.update(local_notes())
            self.status.setText("메모 저장 완료 (팀 공유)")
            self.render()
        else:
            self.status.setText("메모 저장 실패: " + str(value.get("reason")) + " · Pi5 연결을 확인해 주세요.")

    def delete(self):
        row = self.selected()
        if row is None or row["pi5"] is None or self.busy():
            return
        from front_recording import _run_time, delete_on_pi5, downloaded
        note = self.notes.get(row["name"], {})
        label = (note.get("title") + " · " if note.get("title") else "") + _run_time(row["name"])
        laptop = "이 노트북에 받은 사본은 그대로 남습니다." if downloaded(row["name"]) else \
            "이 노트북에는 받은 사본이 없습니다. 지우면 영상이 남지 않습니다."
        box = QtWidgets.QMessageBox(QtWidgets.QMessageBox.Warning, "Pi5 원본 삭제",
                                    f"{label}\nPi5의 전방 영상 원본 {row['pi5']['bytes'] / 1e6:.0f}MB를 지웁니다.\n"
                                    f"되돌릴 수 없습니다.\n\n{laptop}\n메모는 지우지 않습니다.", parent=self)
        delete = box.addButton("Pi5 원본 삭제", QtWidgets.QMessageBox.DestructiveRole)
        box.addButton("취소", QtWidgets.QMessageBox.RejectRole)
        box.setDefaultButton(box.buttons()[-1])
        box.exec_()
        if box.clickedButton() is not delete:
            return
        self.status.setText("Pi5 원본을 지우는 중…")
        self.start("delete", lambda: delete_on_pi5(self.pi_host, row["name"], self.password), self.delete_done)

    @QtCore.pyqtSlot(dict)
    def delete_done(self, value):
        if value.get("ok"):
            self.status.setText(f"Pi5 원본 삭제 완료 ({value.get('bytes', 0) / 1e6:.0f}MB). 메모·노트북 사본은 유지.")
            QtCore.QTimer.singleShot(0, self.refresh)
        else:
            reason = {"recording_now": "지금 녹화 중인 시험은 지울 수 없습니다."}.get(value.get("reason"),
                                                                               str(value.get("reason")))
            self.status.setText("삭제 안 됨: " + reason)

    # -------------------------------------------------------------- files --
    def _open(self, path):
        try:
            subprocess.Popen(["xdg-open", str(path)], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except OSError:
            self.status.setText("열 수 없습니다: " + str(path))

    def play(self):
        from front_recording import front_video_root
        from front_player import play
        row = self.selected()
        if row is None:
            return
        title = self.notes.get(row["name"], {}).get("title", "")
        folder = front_video_root() / row["name"]
        if not folder.is_dir() and self.shared_root is not None and row["name"] in self.shared:
            folder = self.shared_root / row["name"]   # not on this laptop: play from the shared folder
        self.player = play(folder, title, self)

    def open_folder(self):
        from front_recording import front_video_root
        row = self.selected()
        folder = front_video_root() / row["name"] if row is not None and (front_video_root() / row["name"]).is_dir() \
            else Path(getattr(self, "last_folder", "") or front_video_root())
        folder.mkdir(parents=True, exist_ok=True)
        self._open(folder)

    def closeEvent(self, event):
        if self.busy():
            if self.task_kind in ("download", "upload"):
                self.task.cancel()
            self.status.setText("작업을 마무리하는 중입니다. 잠시 후 닫아 주세요.")
            event.ignore()
            return
        event.accept()


def open_dialog(pi_host, password=None, parent=None):
    """Open a non-modal list window (kept alive by its parent)."""
    dialog = FrontRecordingDialog(pi_host, password, parent)
    dialog.setAttribute(QtCore.Qt.WA_DeleteOnClose, True)
    dialog.show()
    return dialog
