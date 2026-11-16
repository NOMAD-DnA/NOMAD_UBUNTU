"""Ordered carrot distances; editing never starts or changes a running planner."""
import json

from PyQt5 import QtWidgets
from module_settings import check_value


class CandidateEditor(QtWidgets.QWidget):
    def __init__(self, doc, values):
        super().__init__()
        self.doc = doc
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.array = QtWidgets.QLineEdit()
        self.array.setToolTip("검사 순서대로 입력하세요. 예: [0.8, 1.0, 0.6, 1.2, 1.5]")
        layout.addWidget(self.array)
        row = QtWidgets.QHBoxLayout()
        self.add_button = QtWidgets.QPushButton("+ 추가")
        self.remove_button = QtWidgets.QPushButton("− 제거")
        self.add_button.clicked.connect(self.add_candidate)
        self.remove_button.clicked.connect(self.remove_candidate)
        row.addWidget(self.add_button)
        row.addWidget(self.remove_button)
        layout.addLayout(row)
        self.setValue(values)

    def value(self):
        try:
            values = json.loads(self.array.text())
        except ValueError as exc:
            raise ValueError("캐럿 후보는 [0.8, 1.0, 0.6] 형식으로 입력하세요") from exc
        values = check_value([], self.doc, values)
        if not values:
            raise ValueError("캐럿 후보를 최소 1개 남겨 주세요")
        return values

    def setValue(self, values):
        self.array.setText(json.dumps(values, allow_nan=False))

    def _current(self):
        try:
            return self.value()
        except ValueError as exc:
            QtWidgets.QMessageBox.warning(self, "캐럿 후보", str(exc))
            return None

    def add_candidate(self):
        values = self._current()
        if values is None:
            return
        value, ok = QtWidgets.QInputDialog.getDouble(
            self, "캐럿 후보 추가", "끝에 추가할 거리 (m)", 0.8,
            self.doc["min"], self.doc["max"], 4)
        if ok:
            try:
                self.setValue(check_value([], self.doc, values + [value]))
            except ValueError as exc:
                QtWidgets.QMessageBox.warning(self, "캐럿 후보", str(exc))

    def remove_candidate(self):
        values = self._current()
        if values is None:
            return
        if len(values) == 1:
            QtWidgets.QMessageBox.warning(self, "캐럿 후보", "캐럿 후보를 최소 1개 남겨 주세요")
            return
        options = [f"{index + 1}번째: {value:g} m" for index, value in enumerate(values)]
        selected, ok = QtWidgets.QInputDialog.getItem(
            self, "캐럿 후보 제거", "제거할 후보", options, 0, False)
        if ok:
            values.pop(options.index(selected))
            self.setValue(values)
