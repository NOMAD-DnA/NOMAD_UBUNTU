"""Generic per-part settings with Korean descriptions (shared by mapper2 and planner, 2026-11-13).

A SettingsSpec points to a folder with one JSON file per part (defaults) and a matching
<part>_doc.json (key -> title, description, unit, min, max, default, options, safety).
Without a board store the user's edits are saved per laptop in the GUI state folder; with one
(board_settings.BoardStore, 2026-11-13) the dialog reads and saves the Jetson's values and presets.
Values outside the documented range are refused. SettingsDialog shows one tab per part as a table
(항목 · 값 · 단위·범위 · 설명).
"""
from __future__ import annotations

import copy
import json
import math
import os
from pathlib import Path


def leaves(cfg: dict, prefix: str = "") -> dict:
    out = {}
    for key, value in cfg.items():
        if key.startswith("_"):
            continue
        name = prefix + key
        if isinstance(value, dict):
            out.update(leaves(value, name + "."))
        else:
            out[name] = value
    return out


def get_dotted(cfg: dict, key: str):
    for part in key.split("."):
        cfg = cfg[part]
    return cfg


def set_dotted(cfg: dict, key: str, value) -> None:
    parts = key.split(".")
    for part in parts[:-1]:
        cfg = cfg[part]
    cfg[parts[-1]] = value


def check_value(default, doc: dict, value):
    """Return the value converted to the default's type, or raise ValueError with a Korean reason."""
    title = doc.get("title", "")
    if isinstance(default, list):
        if not isinstance(value, list) or len(value) > doc.get("max_items", 32):
            raise ValueError(f"{title}: 숫자 배열이어야 합니다 (최대 {doc.get('max_items', 32)}개)")
        result = []
        for item in value:
            if isinstance(item, bool) or not isinstance(item, (int, float)) or not math.isfinite(item):
                raise ValueError(f"{title}: 유한한 숫자만 입력하세요")
            result.append(check_value(0.0, doc, item))
        if len(result) != len(set(result)):
            raise ValueError(f"{title}: 같은 거리를 중복해서 넣을 수 없습니다")
        return result
    if isinstance(default, bool):
        if not isinstance(value, bool):
            raise ValueError(f"{title}: 켬/끔이어야 합니다")
        return value
    if default is None and value is None:
        return None
    if isinstance(default, (int, float)) or default is None:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{title}: 숫자여야 합니다")
        value = int(value) if isinstance(default, int) and not isinstance(default, bool) else float(value)
        low, high = doc.get("min"), doc.get("max")
        if (low is not None and value < low) or (high is not None and value > high):
            raise ValueError(f"{title}: {low} ~ {high} 범위여야 합니다 (입력 {value})")
        return value
    if isinstance(default, str):
        if not isinstance(value, str):
            raise ValueError(f"{title}: 글자여야 합니다")
        options = doc.get("options")
        if options and value not in options:
            raise ValueError(f"{title}: {', '.join(options)} 중 하나여야 합니다")
        return value
    raise ValueError(f"{title}: 지원하지 않는 값 형식")


class SettingsSpec:
    def __init__(self, config_dir, parts, state_file: str, label: str):
        self.config_dir = Path(config_dir)
        self.parts = tuple(parts)          # ((file_stem, tab label), ...)
        self.state_file = state_file
        self.label = label

    def defaults(self) -> dict:
        return {p: json.loads((self.config_dir / f"{p}.json").read_text(encoding="utf-8")) for p, _ in self.parts}

    def docs(self) -> dict:
        return {p: json.loads((self.config_dir / f"{p}_doc.json").read_text(encoding="utf-8")) for p, _ in self.parts}

    def validate(self, configs: dict) -> dict:
        base, notes = self.defaults(), self.docs()
        out = copy.deepcopy(base)
        for part, _ in self.parts:
            given = leaves(configs.get(part, {})) if isinstance(configs.get(part), dict) else {}
            for key, default in leaves(base[part]).items():
                if key in given:
                    set_dotted(out[part], key, check_value(default, notes[part].get(key, {}), given[key]))
        return out

    def path(self, state_root) -> Path:
        return Path(state_root) / self.state_file

    def load(self, state_root) -> dict:
        try:
            return self.validate(json.loads(self.path(state_root).read_text(encoding="utf-8")))
        except (OSError, ValueError, TypeError, KeyError):
            return self.defaults()

    def save(self, configs: dict, state_root) -> dict:
        configs = self.validate(configs)
        target = self.path(state_root)
        target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        temporary = target.with_suffix(".tmp")
        temporary.write_text(json.dumps(configs, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
        temporary.chmod(0o600)
        os.replace(temporary, target)
        return configs

    def changed_keys(self, configs: dict) -> list[str]:
        base = self.defaults()
        out = []
        for part, label in self.parts:
            defaults = leaves(base[part])
            for key, value in leaves(configs[part]).items():
                if defaults.get(key) != value:
                    out.append(f"{label}: {key}")
        return out

    def summary(self, configs: dict) -> str:
        keys = self.changed_keys(configs)
        return f"{self.label}: 기본값" if not keys else f"{self.label}: {len(keys)}개 바꿈"


class SettingsDialog:
    """Qt dialog: one tab per part, rows = setting, value widget, unit/range, description."""

    def __init__(self, parent, state_root, spec: SettingsSpec, title: str, intro: str, board=None):
        """board: optional board_settings.BoardStore (2026-11-13). With it the dialog shows the
        Jetson's current values, saves to the Jetson and offers named presets stored there."""
        from PyQt5 import QtCore, QtGui, QtWidgets
        self.QtWidgets, self.QtCore, self.QtGui = QtWidgets, QtCore, QtGui
        self.spec = spec
        self.state_root = state_root
        self.notes = spec.docs()
        self.board = board
        # 2026-11-16: Pi5 modules (모듈 세부 설정) say "Pi5" instead of "Jetson".
        self.where = getattr(getattr(board, "module", None), "board_label", "Jetson")
        self.board_ok = False
        self.presets: dict = {}
        self.base = spec.defaults()
        self.dialog = QtWidgets.QDialog(parent)
        self.dialog.setWindowTitle(title)
        self.dialog.resize(1000, 700)
        layout = QtWidgets.QVBoxLayout(self.dialog)
        note = QtWidgets.QLabel(intro)
        note.setWordWrap(True)
        layout.addWidget(note)
        configs = spec.load(state_root)
        if board is not None:
            layout.addLayout(self._board_row())
            configs = self._read_board(configs)
        self.tabs = QtWidgets.QTabWidget()
        layout.addWidget(self.tabs)
        self.widgets: dict[tuple[str, str], object] = {}
        # 2026-11-13: exact value behind each number box. The box shows 4 decimals; an untouched
        # box must save this value, not the rounded display (0.6108652 -> 0.6109 broke the 35 deg limit).
        self.exact: dict[tuple[str, str], object] = {}
        self.error = QtWidgets.QLabel("")
        self.error.setWordWrap(True)
        self.error.setStyleSheet("color: #c0392b")
        layout.addWidget(self.error)
        for part, label in spec.parts:
            self.tabs.addTab(self._table(part, configs[part]), label)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Save | QtWidgets.QDialogButtonBox.Cancel |
                                             QtWidgets.QDialogButtonBox.RestoreDefaults)
        buttons.button(QtWidgets.QDialogButtonBox.RestoreDefaults).setText("이 탭 기본값으로")
        buttons.button(QtWidgets.QDialogButtonBox.RestoreDefaults).clicked.connect(self.restore_tab)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.dialog.reject)
        if board is not None:
            buttons.button(QtWidgets.QDialogButtonBox.Save).setText(f"{self.where}에 저장")
            buttons.button(QtWidgets.QDialogButtonBox.RestoreDefaults).setText("이 탭 보드 기본값으로")
        layout.addWidget(buttons)

    # -- Jetson board values and presets (2026-11-13) -----------------------
    def _board_row(self):
        QtWidgets = self.QtWidgets
        box = QtWidgets.QVBoxLayout()
        self.board_status = QtWidgets.QLabel("")
        self.board_status.setWordWrap(True)
        box.addWidget(self.board_status)
        row = QtWidgets.QHBoxLayout()
        self.reload_button = QtWidgets.QPushButton(f"{self.where}에서 다시 읽기")
        self.reload_button.clicked.connect(self.reload_board)
        row.addWidget(self.reload_button)
        row.addSpacing(16)
        row.addWidget(QtWidgets.QLabel("프리셋"))
        self.preset_combo = QtWidgets.QComboBox()
        self.preset_combo.setMinimumWidth(240)
        row.addWidget(self.preset_combo, 1)
        self.preset_load_button = QtWidgets.QPushButton("표에 불러오기")
        self.preset_load_button.setToolTip(f"프리셋 값을 표에 채웁니다. '{self.where}에 저장'을 눌러야 보드에 적용됩니다.")
        self.preset_load_button.clicked.connect(self.load_preset)
        row.addWidget(self.preset_load_button)
        self.preset_save_button = QtWidgets.QPushButton("지금 표를 프리셋으로 저장…")
        self.preset_save_button.setToolTip(f"지금 표의 값을 이름을 붙여 {self.where}에 보관합니다 (잘 되던 값 저장).")
        self.preset_save_button.clicked.connect(self.save_preset)
        row.addWidget(self.preset_save_button)
        box.addLayout(row)
        return box

    def _busy(self, call):
        """Run a board call (ssh, up to its timeout) off the GUI thread.

        2026-11-16: the call used to run on the GUI thread, so an unreachable board froze the
        whole app for the ssh timeout. Now it runs in a worker thread while Qt events keep being
        processed; this dialog and its parent are disabled meanwhile so nothing is clicked twice."""
        import threading
        app = self.QtWidgets.QApplication.instance()
        if app is None:
            return call()
        outcome: dict = {}
        done = threading.Event()

        def work():
            try:
                outcome["value"] = call()
            except BaseException as exc:  # re-raised on the GUI thread below
                outcome["error"] = exc
            finally:
                done.set()

        blocked = [w for w in (self.dialog, self.dialog.parentWidget()) if w is not None and w.isEnabled()]
        for widget in blocked:
            widget.setEnabled(False)
        app.setOverrideCursor(self.QtCore.Qt.WaitCursor)
        try:
            threading.Thread(target=work, name="board-settings", daemon=True).start()
            while not done.wait(0.03):
                app.processEvents(self.QtCore.QEventLoop.AllEvents, 30)
        finally:
            app.restoreOverrideCursor()
            for widget in blocked:
                widget.setEnabled(True)
        if "error" in outcome:
            raise outcome["error"]
        return outcome.get("value")

    def _read_board(self, fallback):
        from board_settings import BoardError
        self.board_status.setStyleSheet("")
        self.board_status.setText(f"{self.where}에서 값을 읽는 중… (보드가 없으면 최대 수십 초)")
        try:
            data = self._busy(self.board.read)
        except BoardError as exc:
            self.board_ok = False
            self.presets = {}
            self._show_presets()
            self.board_status.setStyleSheet("color: #c0392b")
            self.board_status.setText(f"{self.where} 값을 읽지 못했습니다: {exc}. 아래 표는 이 노트북이 마지막으로 본 값입니다. "
                                      f"보드에 연결된 뒤 '{self.where}에서 다시 읽기'를 눌러야 저장할 수 있습니다.")
            return fallback
        self.board_ok = True
        self.base = data["defaults"]
        self.presets = data["presets"]
        self._show_presets()
        source = (f"{self.where}에 저장된 현재 값" if data["source"] == "current" else
                  f"{self.where}에 아직 저장한 값이 없어 보드 기본값")
        text = f"{source}을 보여 줍니다 ({data['read_at']} 읽음). 저장하면 {self.where}에 바로 기록되고, 다음 시작부터 적용됩니다."
        if not data["board_default"]:
            text += " (보드 기본값 파일을 찾지 못해 기본값은 GUI 사본 기준)"
        self.board_status.setStyleSheet("")
        if data["problems"]:
            self.board_status.setStyleSheet("color: #b9770e")
            text += "\n표에 넣지 못한 보드 값 (범위 밖 등, 저장하면 표의 값으로 바뀝니다): " + "; ".join(data["problems"][:6])
        self.board_status.setText(text)
        return data["values"]

    def _show_presets(self):
        self.preset_combo.clear()
        for name in sorted(self.presets):
            preset = self.presets[name]
            saved = preset.get("saved_at", "") if isinstance(preset, dict) else ""
            self.preset_combo.addItem(f"{name}  ({saved})" if saved else name, name)
        enabled = self.board_ok
        self.preset_load_button.setEnabled(enabled and bool(self.presets))
        self.preset_save_button.setEnabled(enabled)

    def set_values(self, configs, parts=None):
        for (part, key), (widget, default) in self.widgets.items():
            if parts is not None and part not in parts:
                continue
            value = get_dotted(configs[part], key)
            self.exact[(part, key)] = value
            if isinstance(default, bool):
                widget.setChecked(bool(value))
            elif isinstance(default, str):
                if hasattr(widget, "setCurrentText"):
                    widget.setCurrentText(value)
                else:
                    widget.setText(value)
            elif default is None:
                widget.setValue(widget.minimum() if value is None else float(value))
            else:
                widget.setValue(value)

    def reload_board(self):
        configs = self._read_board(self.values())
        self.set_values(configs)
        self.error.setText("")

    def load_preset(self):
        from board_settings import BoardError
        name = self.preset_combo.currentData()
        if not name:
            return
        try:
            configs, problems = self.board.preset_values(self.presets[name], self.base)
        except BoardError as exc:
            self.error.setText(str(exc))
            return
        self.set_values(configs)
        self.error.setText(f"프리셋 '{name}'을 표에 불러왔습니다. '{self.where}에 저장'을 눌러야 보드에 적용됩니다." +
                           (" 범위 밖이라 빠진 값: " + "; ".join(problems[:4]) if problems else ""))

    def save_preset(self):
        from board_settings import BoardError
        QtWidgets = self.QtWidgets
        try:
            configs = self.spec.validate(self.values())
        except ValueError as exc:
            self.error.setText("프리셋으로 저장하지 못했습니다: " + str(exc))
            return
        name, ok = QtWidgets.QInputDialog.getText(self.dialog, "프리셋 저장", "프리셋 이름 (예: 1113 야지 잘됨)")
        if not ok or not name.strip():
            return
        name = name.strip()
        note, _ = QtWidgets.QInputDialog.getText(self.dialog, "프리셋 메모", "메모 (선택, 날씨·장소·결과 등)")
        try:
            try:
                self._busy(lambda: self.board.save_preset(name, configs, note))
            except FileExistsError:
                answer = QtWidgets.QMessageBox.question(self.dialog, "프리셋 덮어쓰기",
                                                        f"'{name}' 프리셋이 이미 있습니다. 덮어쓸까요?")
                if answer != QtWidgets.QMessageBox.Yes:
                    return
                self._busy(lambda: self.board.save_preset(name, configs, note, overwrite=True))
        except BoardError as exc:
            self.error.setText("프리셋 저장 실패: " + str(exc))
            return
        self.presets[name] = {"name": name, "saved_at": "방금", "configs": configs}
        self._show_presets()
        self.preset_combo.setCurrentIndex(self.preset_combo.findData(name))
        self.error.setText(f"프리셋 '{name}'을 {self.where}에 저장했습니다.")

    def _widget(self, default, doc, value):
        QtWidgets = self.QtWidgets
        if isinstance(default, bool):
            w = QtWidgets.QCheckBox("켬")
            w.setChecked(bool(value))
            return w
        if isinstance(default, str):
            options = doc.get("options")
            if options:
                w = QtWidgets.QComboBox()
                w.addItems(options)
                w.setCurrentText(value)
                return w
            return QtWidgets.QLineEdit(value)
        if isinstance(default, int) and not isinstance(default, bool):
            w = QtWidgets.QSpinBox()
            w.setRange(int(doc.get("min", -10**6)), int(doc.get("max", 10**6)))
            w.setValue(int(value))
            return w
        w = QtWidgets.QDoubleSpinBox()
        low, high = float(doc.get("min", -1e6)), float(doc.get("max", 1e6))
        w.setDecimals(4)
        w.setSingleStep(max((high - low) / 100.0, 0.0001))
        if default is None:   # "use the legacy value" when at the minimum
            w.setRange(low - 1.0, high)
            w.setSpecialValueText("기본(기존 설정 따름)")
            w.setValue(low - 1.0 if value is None else float(value))
        else:
            w.setRange(low, high)
            w.setValue(float(value))
        return w

    def _table(self, part, cfg):
        QtWidgets, QtGui = self.QtWidgets, self.QtGui
        table = QtWidgets.QTableWidget()
        rows = leaves(self.spec.defaults()[part])
        table.setColumnCount(4)
        table.setHorizontalHeaderLabels(["항목", "값", "단위 · 범위", "설명"])
        table.setRowCount(len(rows))
        table.verticalHeader().setVisible(False)
        table.setWordWrap(True)
        for row, (key, default) in enumerate(rows.items()):
            doc = self.notes[part].get(key, {})
            safety = bool(doc.get("safety"))
            name = QtWidgets.QTableWidgetItem(("[안전] " if safety else "") + doc.get("title", key))
            name.setToolTip(key)
            if safety:
                name.setForeground(QtGui.QBrush(QtGui.QColor("#b03a2e")))
            table.setItem(row, 0, name)
            widget = self._widget(default, doc, get_dotted(cfg, key))
            widget.setMaximumHeight(30)
            self.widgets[(part, key)] = (widget, default)
            self.exact[(part, key)] = get_dotted(cfg, key)
            holder = QtWidgets.QWidget()
            box = QtWidgets.QVBoxLayout(holder)
            box.setContentsMargins(4, 2, 4, 2)
            box.addWidget(widget)
            table.setCellWidget(row, 1, holder)
            unit = "켬/끔" if isinstance(default, bool) else doc.get("unit", "")
            if doc.get("min") is not None:
                unit += f"  ({doc['min']} ~ {doc['max']})"
            base_value = get_dotted(self.base[part], key)   # board default when read from the Jetson
            shown = ("켬" if base_value else "끔") if isinstance(default, bool) else (base_value if base_value is not None else "기존 설정")
            unit += f"\n기본 {shown}"
            table.setItem(row, 2, QtWidgets.QTableWidgetItem(unit))
            table.setItem(row, 3, QtWidgets.QTableWidgetItem(doc.get("description", "")))
        table.setColumnWidth(0, 180)
        table.setColumnWidth(1, 170)
        table.setColumnWidth(2, 150)
        table.horizontalHeader().setStretchLastSection(True)
        # Re-fit row heights whenever the description column width changes (wrapped text must not be cut).
        table.horizontalHeader().sectionResized.connect(lambda *_: table.resizeRowsToContents())
        table.resizeRowsToContents()
        return table

    def values(self) -> dict:
        configs = self.spec.defaults()
        for (part, key), (widget, default) in self.widgets.items():
            if isinstance(default, bool):
                value = widget.isChecked()
            elif isinstance(default, str):
                value = widget.currentText() if hasattr(widget, "currentText") else widget.text()
            elif isinstance(default, int):
                value = widget.value()
            else:
                value = widget.value()
                if default is None and value <= widget.minimum():
                    value = None
                else:
                    exact = self.exact.get((part, key))
                    if (isinstance(exact, (int, float)) and not isinstance(exact, bool)
                            and abs(value - round(float(exact), widget.decimals())) < 10 ** -(widget.decimals() + 2)):
                        value = exact   # untouched box: keep the full-precision value
            set_dotted(configs[part], key, value)
        return configs

    def restore_tab(self):
        part = self.spec.parts[self.tabs.currentIndex()][0]
        self.set_values(self.base, parts={part})

    def accept(self):
        if self.board is not None:
            from board_settings import BoardError
            if not self.board_ok:
                self.error.setText(f"{self.where} 값을 읽지 못한 상태라 저장하지 않았습니다. 연결 후 '{self.where}에서 다시 읽기'를 눌러 주세요.")
                return
            try:
                configs = self.spec.validate(self.values())
                self._busy(lambda: self.board.save(configs))
            except (ValueError, BoardError) as exc:
                self.error.setText(f"{self.where}에 저장하지 못했습니다: " + str(exc))
                return
            self.dialog.accept()
            return
        try:
            self.spec.save(self.values(), self.state_root)
        except (ValueError, OSError) as exc:
            self.error.setText("저장하지 못했습니다: " + str(exc))
            return
        self.dialog.accept()

    def exec_(self) -> int:
        self.QtCore.QTimer.singleShot(0, self.fit_rows)
        return self.dialog.exec_()

    def fit_rows(self):
        for index in range(self.tabs.count()):
            self.tabs.widget(index).resizeRowsToContents()
