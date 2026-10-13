"""Small script launcher: menu definitions contain commands, GUI owns only its runs."""

import importlib.util
import json
import os
from pathlib import Path
import shlex
import signal
import sys
import tempfile
from copy import deepcopy
from options import OptionsDialog, defaults, overrides
from log_actions import LogActions
from datetime import datetime, timezone

from PyQt5.QtCore import QProcess, QProcessEnvironment, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

ROOT = Path(__file__).resolve().parents[3]


def load_source(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(menu, values, extra=""):
    script = ROOT / menu["script"]
    if not script.is_file():
        raise ValueError(f"실행 파일을 찾을 수 없습니다: {script}")
    return (
        [str(script)]
        + [arg.format(**values) for arg in menu["args"]]
        + shlex.split(extra)
    )


class Launcher(LogActions, QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("NOMAD · 프로젝트 실행")
        self.resize(1060, 740)
        self.config = json.loads(Path(__file__).with_name("menu.json").read_text())
        self.menus = self.config["menus"]
        maps = load_source(
            "nomad_gui_maps", ROOT / "src/nomad_gazebo/scripts/select_map.py"
        )
        planners = load_source(
            "nomad_gui_planners",
            ROOT / "src/nomad_path_planning/scripts/select_algorithms.py",
        )
        self.sources = {
            "maps": [
                (f"{i}. {m['name']}", m["id"])
                for i, m in enumerate(maps.discover(ROOT / "src/nomad_gazebo/maps"), 1)
            ],
            "planners": [(p[0], (p[1], p[2])) for p in planners.PAIRS],
        }
        self.runs = {}
        self.counter = 0
        self.fields = {}
        self.option_base = defaults()
        self.option_state = deepcopy(self.option_base)
        self.option_dirs = []
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        title = QLabel("NOMAD 실행 메뉴")
        title.setObjectName("title")
        layout.addWidget(title)
        layout.addWidget(
            QLabel(
                "기존 스크립트를 선택해서 실행합니다. 창을 열어도 시뮬레이션은 시작되지 않습니다."
            )
        )
        split = QSplitter()
        layout.addWidget(split, 1)
        left = QWidget()
        left_layout = QVBoxLayout(left)
        split.addWidget(left)
        self.menu = QComboBox()
        for m in self.menus:
            self.menu.addItem(m["title"])
        left_layout.addWidget(QLabel("실행할 기능"))
        left_layout.addWidget(self.menu)
        self.hint = QLabel()
        self.hint.setWordWrap(True)
        left_layout.addWidget(self.hint)
        self.form = QFormLayout()
        left_layout.addLayout(self.form)
        for key, spec in self.config["fields"].items():
            kind = spec["kind"]
            if kind == "choice":
                widget = QComboBox()
                for label, value in self.sources.get(
                    spec.get("source"), spec.get("choices", [])
                ):
                    widget.addItem(label, value)
                if "default" in spec:
                    index = widget.findData(spec["default"])
                    if index >= 0:
                        widget.setCurrentIndex(index)
                elif "default_index" in spec:
                    widget.setCurrentIndex(spec["default_index"])
                widget.currentIndexChanged.connect(self.preview)
            elif kind == "bool":
                widget = QCheckBox()
                widget.setChecked(spec["default"])
                widget.toggled.connect(self.preview)
            elif kind == "number":
                widget = QDoubleSpinBox()
                widget.setDecimals(2)
                widget.setRange(spec["min"], spec["max"])
                widget.setValue(spec["default"])
                widget.setSingleStep(0.1)
                widget.valueChanged.connect(self.preview)
            else:
                widget = QLineEdit(str(spec.get("default", "")))
                widget.textChanged.connect(self.preview)
            self.fields[key] = widget
            self.form.addRow(spec["label"], widget)
        advanced = QGroupBox("상세 옵션")
        advanced.setCheckable(True)
        advanced.setChecked(False)
        advanced_layout = QVBoxLayout(advanced)
        self.advanced_body = QWidget()
        advanced_form = QFormLayout(self.advanced_body)
        self.domain = QSpinBox()
        self.domain.setRange(0, 232)
        self.domain.setValue(42)
        self.settings_button = QPushButton("세부 설정 열기…")
        self.settings_button.clicked.connect(self.open_options)
        advanced_form.addRow("ROS domain", self.domain)
        advanced_form.addRow(self.settings_button)
        advanced_layout.addWidget(self.advanced_body)
        self.advanced_body.hide()
        advanced.toggled.connect(self.advanced_body.setVisible)
        self.domain.valueChanged.connect(self.preview)
        left_layout.addWidget(advanced)
        self.command_text = QTextEdit()
        self.command_text.setReadOnly(True)
        self.command_text.setMaximumHeight(120)
        left_layout.addWidget(QLabel("실행할 명령"))
        left_layout.addWidget(self.command_text)
        buttons = QHBoxLayout()
        self.start = QPushButton("실행")
        self.start.setObjectName("start")
        stop = QPushButton("선택 종료")
        stop_all = QPushButton("GUI 실행 모두 종료")
        self.start.clicked.connect(self.launch)
        stop.clicked.connect(self.stop_selected)
        stop_all.clicked.connect(self.stop_all)
        for b in [self.start, stop, stop_all]:
            buttons.addWidget(b)
        left_layout.addLayout(buttons)
        self.init_logs(ROOT, left_layout)
        left_layout.addStretch()
        right = QWidget()
        right_layout = QVBoxLayout(right)
        split.addWidget(right)
        right_layout.addWidget(QLabel("이 창에서 실행한 프로세스"))
        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["기능", "상태", "PID"])
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        self.table.setMaximumHeight(190)
        self.table.horizontalHeader().setStretchLastSection(True)
        right_layout.addWidget(self.table)
        right_layout.addWidget(QLabel("실행 로그"))
        self.init_live_filters(right_layout)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.document().setMaximumBlockCount(1500)
        self.log.setObjectName("log")
        right_layout.addWidget(self.log, 1)
        split.setSizes([450, 610])
        self.statusBar().showMessage(
            "준비됨 · 외부에서 실행한 프로세스는 종료하지 않습니다."
        )
        self.setStyleSheet(
            """QWidget {font-size: 13px;} QLabel#title {font-size: 23px; font-weight: 600;}
            QComboBox,QLineEdit,QSpinBox,QDoubleSpinBox {padding: 5px;} QPushButton {padding: 9px;}
            QPushButton#start {background: #247565; color: white; border-radius: 4px;}
            QTextEdit#log {background: #20262d; color: #e2e8ed; font-family: monospace;}
            QGroupBox {margin-top: 10px;}"""
        )
        self.menu.currentIndexChanged.connect(self.select_menu)
        self.select_menu()

    def values(self):
        result = {}
        for key, widget in self.fields.items():
            if isinstance(widget, QComboBox):
                result[key] = widget.currentData()
            elif isinstance(widget, QCheckBox):
                result[key] = str(widget.isChecked()).lower()
            elif isinstance(widget, QDoubleSpinBox):
                result[key] = str(widget.value())
            else:
                result[key] = widget.text()
        result["gpp"], result["lpp"] = result["planner"]
        return result

    def effective_options(self):
        state = deepcopy(self.option_state)
        menu_id = self.menus[self.menu.currentIndex()]["id"]
        values = self.values()
        state["visualization"]["rviz"] = values["rviz"] == "true"
        state["simulation"].update(map=values["map"], vio=values["vio"], headless=values["headless"] == "true")
        for block in state["configs"]["planning"].values():
            parameters = block["ros__parameters"]
            if "gpp_algorithm" in parameters:
                parameters["gpp_algorithm"] = values["gpp"]
            if "lpp_algorithm" in parameters:
                parameters["lpp_algorithm"] = values["lpp"]
        if menu_id == "map":
            state["configs"]["terrain"]["live_terrain"]["ros__parameters"].update(
                resolution=float(values["resolution"]),
                obstacle_cost_radius=float(values["radius"]),
            )
        elif menu_id == "semantic":
            state["semantic"].update(
                backend=values["backend"],
                rate=float(values["rate"]),
                view=values["overlay"] == "true",
            )
        return state

    def open_options(self):
        menu = self.menus[self.menu.currentIndex()]
        dialog = OptionsDialog(self, menu["id"], self.effective_options())
        if dialog.exec_() != dialog.Accepted:
            return
        self.option_state = dialog.result_state
        self.fields["rviz"].setChecked(self.option_state["visualization"]["rviz"])
        self.fields["map"].setCurrentIndex(self.fields["map"].findData(self.option_state["simulation"]["map"]))
        self.fields["headless"].setChecked(self.option_state["simulation"]["headless"])
        terrain = self.option_state["configs"]["terrain"]["live_terrain"][
            "ros__parameters"
        ]
        self.fields["resolution"].setValue(terrain["resolution"])
        self.fields["radius"].setValue(terrain["obstacle_cost_radius"])
        self.fields["backend"].setCurrentIndex(
            self.fields["backend"].findData(self.option_state["semantic"]["backend"])
        )
        self.fields["rate"].setValue(self.option_state["semantic"]["rate"])
        self.fields["overlay"].setChecked(self.option_state["semantic"]["view"])
        self.preview()

    def select_menu(self):
        menu = self.menus[self.menu.currentIndex()]
        for key, widget in self.fields.items():
            enabled = key in menu["fields"]
            widget.setVisible(enabled)
            self.form.labelForField(widget).setVisible(enabled)
        self.hint.setText(menu["hint"])
        self.preview()

    def preview(self, *_):
        if not hasattr(self, "command_text"):
            return
        try:
            menu = self.menus[self.menu.currentIndex()]
            args = command(menu, self.values()) + overrides(
                menu["id"], self.effective_options(), self.option_base
            )
            env = f"NOMAD_ROS_DOMAIN_ID={self.domain.value()} NOMAD_MAP={self.values()['map']}"
            self.command_text.setPlainText(env + "\n" + shlex.join(args))
        except (ValueError, KeyError) as error:
            self.command_text.setPlainText(str(error))

    def launch(self):
        menu = deepcopy(self.menus[self.menu.currentIndex()])
        if menu["id"] == "all" and self.effective_options()["visualization"]["initial_teleop"]:
            menu["resources"].append("teleop")
        domain = self.domain.value()
        for run in self.runs.values():
            if run["domain"] == domain and set(menu["resources"]) & set(
                run["menu"]["resources"]
            ):
                QMessageBox.information(
                    self,
                    "이미 실행 중",
                    "같은 domain에서 이 기능을 이미 실행했습니다. 먼저 해당 실행을 종료하세요.",
                )
                return
            if run["domain"] == domain and {menu["id"], run["menu"]["id"]} == {
                "control",
                "teleop",
            }:
                QMessageBox.information(
                    self,
                    "조종 종료 필요",
                    "수동 조종과 차량 제어를 동시에 실행하지 않습니다.",
                )
                return
        try:
            directory = tempfile.TemporaryDirectory(prefix="nomad-gui-options-")
            args = command(menu, self.values()) + overrides(
                menu["id"], self.effective_options(), self.option_base, directory.name
            )
            runtime_directory=Path(self.log_spool.name)/f"run_{self.counter+1:03d}"
            runtime_directory.mkdir()
            if menu["id"]=="all":args.extend([f"record_directory:={runtime_directory/'map'}",f"vio_log_directory:={runtime_directory/'vio'}"])
            elif menu["id"]=="localization":args.append(f"vio_log_directory:={runtime_directory/'vio'}")
            elif menu["id"]=="map":args.extend(["-p",f"directory:={runtime_directory/'map'}"])
            self.option_dirs.append(directory)
        except (ValueError, KeyError) as error:
            QMessageBox.warning(self, "설정 확인", str(error))
            return
        self.counter += 1
        key = self.counter
        row = self.table.rowCount()
        self.table.insertRow(row)
        for col, value in enumerate([menu["title"], "시작 중", ""]):
            self.table.setItem(row, col, QTableWidgetItem(value))
        process = QProcess(self)
        process.setWorkingDirectory(str(ROOT))
        process.setProcessChannelMode(QProcess.MergedChannels)
        env = QProcessEnvironment.systemEnvironment()
        env.insert("NOMAD_ROS_DOMAIN_ID", str(domain))
        env.insert("NOMAD_RUN_LOG_DIR", str(runtime_directory))
        env.insert("NOMAD_MAP", str(self.values()["map"]))
        env.insert("NOMAD_PLANNER_PAIR", str(self.fields["planner"].currentIndex() + 1))
        if menu["id"] == "semantic":
            env.insert("QT_QPA_PLATFORM", "xcb")
        process.setProcessEnvironment(env)
        record = {"module": menu["id"], "title": menu["title"], "domain": domain,
                  "command": args, "requested_at": datetime.now(timezone.utc).isoformat(),
                  "settings": self.effective_options(), "options_directory": directory.name, "output_directory": str(runtime_directory)}
        self.run_history.append(record)
        self.runs[key] = {
            "process": process,
            "menu": menu,
            "row": row,
            "domain": domain,
            "record": record,
        }
        process.started.connect(lambda: self.started(key))
        process.readyReadStandardOutput.connect(lambda: self.read_log(key))
        process.finished.connect(lambda code, _: self.finished(key, code))
        process.errorOccurred.connect(lambda error: self.process_error(key, error))
        self.append_log(f"[{menu['title']}] 시작\n" + shlex.join(args))
        process.start("/usr/bin/setsid", ["bash"] + args)

    def process_error(self, key, error):
        if key not in self.runs:
            return
        run = self.runs[key]
        self.append_log(f"[{run['menu']['title']}] {run['process'].errorString()}")
        if error == QProcess.FailedToStart:
            self.finished(key, -1)

    def started(self, key):
        run = self.runs[key]
        run["record"]["started_at"] = datetime.now(timezone.utc).isoformat()
        run["record"]["pid"] = int(run["process"].processId())
        self.table.item(run["row"], 1).setText("실행 중")
        self.table.item(run["row"], 2).setText(str(run["process"].processId()))

    def read_log(self, key):
        if key not in self.runs:
            return
        run = self.runs[key]
        text = bytes(run["process"].readAllStandardOutput()).decode(errors="replace")
        buffered=run.get("log_buffer", "")+text
        lines=buffered.split("\n")
        run["log_buffer"]=lines.pop()
        if lines:self.append_log("\n".join(lines),source=run["menu"]["title"])

    def finished(self, key, code):
        if key not in self.runs:
            return
        self.read_log(key)
        run = self.runs.pop(key)
        if run.get("log_buffer"):
            self.append_log(run["log_buffer"],source=run["menu"]["title"])
        run["record"]["ended_at"] = datetime.now(timezone.utc).isoformat()
        run["record"]["exit_code"] = code
        self.table.item(run["row"], 1).setText(f"종료 ({code})")
        self.append_log(f"[{run['menu']['title']}] 종료 코드 {code}")
        run["process"].deleteLater()

    def stop(self, key):
        if key not in self.runs:
            return
        run = self.runs[key]
        process = run["process"]
        pid = int(process.processId())
        if pid:
            try:
                os.killpg(pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        self.table.item(run["row"], 1).setText("종료 요청")
        QTimer.singleShot(12000, lambda: self.terminate(key))

    def terminate(self, key):
        if key in self.runs:
            self.runs[key]["process"].terminate()

    def stop_selected(self):
        row = self.table.currentRow()
        for key, run in list(self.runs.items()):
            if run["row"] == row:
                self.stop(key)

    def stop_all(self):
        for key in list(self.runs):
            self.stop(key)

    def closeEvent(self, event):
        if self.runs:
            QMessageBox.information(
                self,
                "실행 종료",
                "먼저 GUI 실행 모두 종료를 누르고 프로세스가 종료된 뒤 창을 닫으세요.",
            )
            event.ignore()
        else:
            for directory in self.option_dirs:
                directory.cleanup()
            self.tail_timer.stop()
            self.log_spool.cleanup()
            event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = Launcher()
    window.show()
    sys.exit(app.exec_())
