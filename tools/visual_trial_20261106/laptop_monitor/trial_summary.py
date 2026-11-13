"""Required operator notes for a completed GUI trial; no board communication."""
import sqlite3
from PyQt5 import QtCore, QtWidgets


class TrialSummaryDialog(QtWidgets.QDialog):
    def __init__(self, parent, save_summary, previous=None, save_copy=None):
        super().__init__(parent)
        self._save_summary = save_summary
        self._save_copy = save_copy
        self._ready = False
        self.setWindowTitle("시험 종료 · 시험 기록 작성")
        self.setWindowModality(QtCore.Qt.WindowModal)
        self.setMinimumWidth(580)
        layout = QtWidgets.QVBoxLayout(self)
        heading = QtWidgets.QLabel("시험 이름과 내용을 모두 입력해 주세요.")
        heading.setStyleSheet("font-size:18px;font-weight:bold;")
        layout.addWidget(heading)
        detail = QtWidgets.QLabel("모듈 종료는 이미 요청했습니다. 입력한 내용은 해당 주행 기록(.nomad)에 함께 저장됩니다.")
        detail.setWordWrap(True)
        layout.addWidget(detail)
        form = QtWidgets.QFormLayout()
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText("예: 우회전 경로 추종 시험 1차")
        self.name_edit.setMaxLength(200)
        form.addRow("시험 이름 (필수)", self.name_edit)
        self.description_edit = QtWidgets.QPlainTextEdit()
        self.description_edit.setPlaceholderText("시험 목적, 조건, 실제 동작과 이상 증상을 적어 주세요.")
        self.description_edit.setMinimumHeight(180)
        form.addRow("시험 내용 (필수)", self.description_edit)
        layout.addLayout(form)
        self.state_label = QtWidgets.QLabel()
        self.state_label.setWordWrap(True)
        self.state_label.setTextFormat(QtCore.Qt.PlainText)
        layout.addWidget(self.state_label)
        self.error_label = QtWidgets.QLabel()
        self.error_label.setTextFormat(QtCore.Qt.PlainText)
        self.error_label.setStyleSheet("color:#fca5a5;")
        self.error_label.setWordWrap(True)
        layout.addWidget(self.error_label)
        self.save_button = QtWidgets.QPushButton("시험 기록 저장")
        self.save_button.clicked.connect(self.accept)
        layout.addWidget(self.save_button)
        self.copy_button = QtWidgets.QPushButton("메모만 다른 위치에 저장…")
        self.copy_button.clicked.connect(self.save_copy)
        self.copy_button.hide()
        layout.addWidget(self.copy_button)
        self.name_edit.textChanged.connect(self._validate)
        self.description_edit.textChanged.connect(self._validate)
        if previous:
            self.name_edit.setText(previous.get("name", ""))
            self.description_edit.setPlainText(previous.get("description", ""))
        self._validate()

    def set_recording_state(self, ready, message):
        self._ready = ready
        self.state_label.setText(message)
        self._validate()

    def _validate(self):
        self.save_button.setEnabled(bool(self._ready and self.name_edit.text().strip()
                                         and self.description_edit.toPlainText().strip()))

    def accept(self):
        if not self._ready:
            self.error_label.setText("모듈 종료 결과와 기록 저장을 확인하는 중입니다.")
            return
        name = self.name_edit.text().strip()
        description = self.description_edit.toPlainText().strip()
        if not name or not description:
            self.error_label.setText("시험 이름과 내용을 모두 입력해 주세요.")
            return
        try:
            self._save_summary(name, description)
        except (OSError, ValueError, RuntimeError, sqlite3.Error) as error:
            self.error_label.setText("저장 실패 · 입력 내용은 유지됩니다. " + str(error))
            self.copy_button.setVisible(self._save_copy is not None)
            return
        self.error_label.clear()
        super().accept()

    def save_copy(self):
        name = self.name_edit.text().strip()
        description = self.description_edit.toPlainText().strip()
        if not self._ready or not name or not description or self._save_copy is None:
            self.error_label.setText("시험 이름과 내용을 모두 입력해 주세요.")
            return
        try:
            saved = self._save_copy(name, description)
        except (OSError, ValueError, RuntimeError) as error:
            self.error_label.setText("메모 저장 실패 · " + str(error))
            return
        if saved:
            super().accept()

    def reject(self):
        # Escape and the window close button cannot discard required notes.
        self.error_label.setText("시험 이름과 내용을 입력하고 ‘시험 기록 저장’을 눌러 주세요.")
