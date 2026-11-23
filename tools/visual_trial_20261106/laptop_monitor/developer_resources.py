"""Read-only resource window, fed by the monitor's existing snapshot."""
from PyQt5 import QtCore, QtWidgets
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from resource_metrics import SOURCES, BOARD_TITLES, brief, STALE_S, group_cpu
import developer_debug as dbg

WIRE_NAMES = {"pi": "pi5"}  # Pi5 GUI board key is "pi"; show "pi5" where raw names appear
METRICS = {"cpu": ("Board CPU (%)", 100.), "memory": ("RAM used (MiB)", None), "gpu": ("GPU load (%)", 100.)}


class ResourcesWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD · 보드 CPU / 메모리 / GPU")
        self.resize(1150, 880)
        central = QtWidgets.QWidget(); layout = QtWidgets.QVBoxLayout(central)
        self.labels = {}
        for board, title in BOARD_TITLES.items():
            label = QtWidgets.QLabel(title+" · 미수신"); label.setWordWrap(True)
            self.labels[board] = label; layout.addWidget(label)
        note = QtWidgets.QLabel("보드/코어 CPU: 0~100% · 프로세스 CPU: 100%=한 코어 · GPP/LPP는 같은 프로세스이면 합산. "
                               "기록 시작을 누르면 주행 기록과 함께 저장됩니다. 그래프는 최근 최대 10분, 표는 최신 표본입니다.")
        note.setWordWrap(True); layout.addWidget(note)
        export_bar = QtWidgets.QHBoxLayout()
        self.export_button = QtWidgets.QPushButton("저장된 기록 분석 · CSV/그래프")
        self.export_button.clicked.connect(self.export_recording)
        export_bar.addWidget(self.export_button)
        self.export_status = QtWidgets.QLabel("기존 .nomad 파일에서 오프라인 분석")
        self.export_status.setWordWrap(True); export_bar.addWidget(self.export_status, 1)
        layout.addLayout(export_bar)
        filters = QtWidgets.QHBoxLayout()
        filters.addWidget(QtWidgets.QLabel("그래프"))
        self.metric = QtWidgets.QComboBox()
        for key, title in (("cpu", "보드 CPU"), ("memory", "메모리 사용"), ("gpu", "GPU 부하")):
            self.metric.addItem(title, key)
        self.metric.currentIndexChanged.connect(self._filters_changed)
        filters.addWidget(self.metric)
        filters.addSpacing(18)
        filters.addWidget(QtWidgets.QLabel("표 보드"))
        self.board_filter = QtWidgets.QComboBox()
        for key, title in (("", "전체"), *BOARD_TITLES.items()):
            self.board_filter.addItem(title, key)
        self.board_filter.currentIndexChanged.connect(self._filters_changed)
        filters.addWidget(self.board_filter)
        self.nomad_only = QtWidgets.QCheckBox("NOMAD 작업만")
        self.nomad_only.setToolTip("'기타'로 분류된 프로세스를 숨깁니다.")
        self.nomad_only.toggled.connect(self._filters_changed)
        filters.addWidget(self.nomad_only)
        self.search = QtWidgets.QLineEdit()
        self.search.setPlaceholderText("프로세스·작업 이름 검색")
        self.search.textChanged.connect(self._filters_changed)
        filters.addWidget(self.search, 1)
        layout.addLayout(filters)
        self.export_process = None
        self.figure = Figure(figsize=(10, 3), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.ax = self.figure.add_subplot(111)
        layout.addWidget(self.canvas, 1)
        self.cores = QtWidgets.QLabel(); self.cores.setWordWrap(True); layout.addWidget(self.cores)
        self.table = QtWidgets.QTableWidget(0, 7)
        self.table.setHorizontalHeaderLabels(["보드", "작업", "PID", "프로세스", "CPU (한 코어=100%)", "RSS MiB", "스레드"])
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.table, 1)
        self.details = QtWidgets.QLabel(); self.details.setWordWrap(True); layout.addWidget(self.details)
        self.setCentralWidget(central)
        self.signature = None
        self._last_snapshot = None
        self.debug = dbg.DebugTools(self, "resources", "NOMAD 보드 부하", (
            {"key": "telemetry:resources_jetson", "label": "젯슨 부하 계측", "limit": STALE_S},
            {"key": "telemetry:resources_pi4", "label": "라파4 부하 계측", "limit": STALE_S},
            {"key": "telemetry:resources_pi5", "label": "라파5 부하 계측", "limit": STALE_S},
        ))
        self.debug.clear_hooks.append(lambda: setattr(self, "signature", None))

    def _filters_changed(self, *_):
        self.signature = None
        if self._last_snapshot is not None and not self.debug.frozen:
            self.update_snapshot(self._last_snapshot)

    def _process_visible(self, board, process):
        if self.board_filter.currentData() and board != self.board_filter.currentData():
            return False
        if self.nomad_only.isChecked() and process.get("group") == "other":
            return False
        query = self.search.text().strip().casefold()
        if query:
            label = dbg.GROUP_LABELS.get(process.get("group"), str(process.get("group")))
            return query in (str(process.get("name", "")) + " " + label + " " + str(process.get("group"))).casefold()
        return True

    def _recent_rows(self, snapshot, board):
        now = snapshot.get("now")
        rows = snapshot.get("resource_history", {}).get(board, [])
        return [r for r in rows if dbg.finite(now) and dbg.finite(r.get("received")) and 0 <= now - r["received"] <= dbg.WINDOW_S]

    def _debug_checks(self, snapshot):
        telemetry = snapshot.get("telemetry", {})
        checks = []
        for source, board in SOURCES.items():
            title = BOARD_TITLES.get(board, board)
            event = telemetry.get(source, {})
            age = dbg.age(snapshot, "telemetry:" + source)
            checks.append(dbg.freshness(title + " 부하 계측 수신", age, STALE_S, 10., missing="unknown",
                                        note="" if event else "계측 도구 실행 확인"))
            if not event:
                continue
            cpu = event.get("cpu", {})
            total = cpu.get("total_percent")
            if cpu.get("state") != "live" or not dbg.finite(total):
                checks.append((title + " CPU", "unknown", "첫 측정 구간 준비 중"))
            else:
                checks.append((title + " CPU", "ok" if total < 85 else "warn" if total < 95 else "fail",
                               f"{total:.1f}% (85% 이상 주의 · 95% 이상 실패)"))
            memory = event.get("memory", {})
            available, capacity = memory.get("available_mib"), memory.get("total_mib")
            if dbg.finite(available) and dbg.finite(capacity) and capacity > 0:
                ratio = available / capacity
                checks.append((title + " 남은 메모리", "ok" if ratio >= .15 else "warn" if ratio >= .05 else "fail",
                               f"{available:.0f} / {capacity:.0f} MiB 남음 ({ratio*100:.0f}% · 15% 미만 주의)"))
            swap = memory.get("swap_used_mib")
            if dbg.finite(swap):
                checks.append((title + " 스왑 사용", "ok" if swap < 64 else "warn", f"{swap:.0f} MiB (64 MiB 이상 주의 · 느려질 수 있음)"))
            temps = [t for t in event.get("temperatures", []) if dbg.finite(t.get("celsius"))]
            if temps:
                hottest = max(temps, key=lambda t: t["celsius"])
                c = hottest["celsius"]
                checks.append((title + " 최고 온도", "ok" if c < 80 else "warn" if c < 90 else "fail",
                               f"{hottest['name']} {c:.1f}°C (80°C 이상 주의 · 90°C 이상 실패)"))
            failures = event.get("proc_read_failures", 0)
            checks.append((title + " 프로세스 읽기", "ok" if not failures else "warn",
                           f"읽기 실패 {failures}개 · 표시 외 {event.get('processes_omitted', 0)}개 / 전체 {event.get('processes_total', '—')}개"))
        return checks

    def _debug_sections(self, snapshot):
        telemetry = snapshot.get("telemetry", {})
        summary, spread, groups = [], [], []
        columns = []
        for source, board in SOURCES.items():
            event = telemetry.get(source, {})
            memory, gpu, cpu = event.get("memory", {}), event.get("gpu", {}), event.get("cpu", {})
            temps = [t for t in event.get("temperatures", []) if dbg.finite(t.get("celsius"))]
            columns.append([dbg.plain(event.get("model")), dbg.plain(event.get("kernel")),
                            f"{dbg.number_text(cpu.get('total_percent'), 1, '%')} · 코어 {cpu.get('online_cores', '—')}개 · 표본 {dbg.number_text(cpu.get('interval_s'), 2, 's')}",
                            f"{dbg.number_text(memory.get('used_mib'), 0)} / {dbg.number_text(memory.get('total_mib'), 0)} MiB",
                            f"{dbg.number_text(memory.get('swap_used_mib'), 0)} / {dbg.number_text(memory.get('swap_total_mib'), 0)} MiB",
                            f"평균 {dbg.number_text(gpu.get('load_percent'), 1, '%')} · 최대 {dbg.number_text(gpu.get('max_percent'), 0, '%')} ({gpu.get('reads', '—')}회) @ {dbg.number_text(gpu.get('frequency_mhz'), 0, 'MHz')}",
                            ", ".join(f"{t['name']} {t['celsius']:.1f}°C" for t in temps) or "—",
                            f"{event.get('processes_total', '—')}개 (표시 외 {event.get('processes_omitted', '—')})",
                            dbg.number_text(event.get("scan_ms"), 2, " ms"),
                            dbg.plain(event.get("seq"))] if event else ["미수신"] * 10)
            rows = self._recent_rows(snapshot, board)
            title = BOARD_TITLES.get(board, board)
            spread.append([title, dbg.stats_text([r.get("cpu") for r in rows], 1, "%"),
                           dbg.stats_text([r.get("memory") for r in rows], 0, " MiB"),
                           dbg.stats_text([r.get("gpu") for r in rows], 1, "%"), f"{len(rows)}개"])
            names = sorted({k for r in rows for k in r.get("groups", {})} | set(group_cpu(event)))
            current = group_cpu(event) if event else {}
            for name in names:
                values = [r.get("groups", {}).get(name, 0.) for r in rows]
                groups.append([title, dbg.GROUP_LABELS.get(name, name), dbg.number_text(current.get(name), 1, "%"),
                               dbg.stats_text(values, 1, "%")])
        names = ("모델", "커널", "CPU", "메모리 사용 / 전체", "스왑 사용 / 전체", "GPU", "온도", "프로세스", "계측 시간", "표본 번호")
        summary = [[n, *v] for n, *v in zip(names, *columns)]
        groups.sort(key=lambda row: (row[0], row[1] == "기타"))
        return {
            "보드 요약 · 최신 표본": (["항목", *(BOARD_TITLES.get(b, b) for b in SOURCES.values())], summary, ""),
            "최근 60초 통계": (["보드", "CPU 최소/평균/최대", "메모리 최소/평균/최대", "GPU 최소/평균/최대", "표본"], spread,
                         "보드 계측 도구가 보낸 표본만 셉니다."),
            "작업 그룹 CPU · 최근 60초": (["보드", "작업 그룹", "현재", "최소 / 평균 / 최대"], groups,
                                   "100% = CPU 한 코어. 같은 그룹의 프로세스 합계입니다. 표본에 없던 그룹은 0으로 셉니다."),
        }

    def update_snapshot(self, snapshot):
        self.debug.observe(snapshot)
        if self.debug.frozen:
            return
        self._last_snapshot = snapshot
        self._update_view(snapshot)
        self.debug.publish(snapshot, self._debug_checks(snapshot), self._debug_sections(snapshot))

    def _update_view(self, snapshot):
        now = snapshot["now"]; telemetry = snapshot.get("telemetry", {})
        signature = []
        events = []
        for source, board in SOURCES.items():
            event = telemetry.get(source, {})
            received = snapshot.get("received", {}).get("telemetry:"+source)
            age = None if received is None else max(0., now-received)
            title = BOARD_TITLES.get(board, board)
            self.labels[board].setText(title+" · "+brief(event, age))
            signature.append((event.get("session"), event.get("seq"), age is None or age > STALE_S))
            if event:
                events.append((board, event, age))
        if tuple(signature) == self.signature:
            return
        self.signature = tuple(signature)
        self.ax.clear()
        span = 60.
        metric = self.metric.currentData() or "cpu"
        ylabel, top = METRICS[metric]
        for board, color in (("jetson", "#2563eb"), ("pi4", "#ea580c"), ("pi", "#16a34a")):
            rows = snapshot.get("resource_history", {}).get(board, [])
            rows = [r for r in rows if 0 <= now-r["received"] <= 600]
            if rows:
                span = max(span, min(600., now-rows[0]["received"]))
                # NaN breaks the line across missing/null measurements.
                xs, ys = [], []
                previous = None
                for r in rows:
                    if previous is not None and r["received"]-previous > STALE_S:
                        xs.append(r["received"]-now); ys.append(float("nan"))
                    value = r.get(metric)
                    xs.append(r["received"]-now); ys.append(value if value is not None else float("nan"))
                    previous = r["received"]
                self.ax.plot(xs, ys, color=color, label=WIRE_NAMES.get(board, board))
        self.ax.set(xlim=(-span, 0), xlabel="Seconds before current view", ylabel=ylabel)
        if top is not None:
            self.ax.set_ylim(0, top)
        else:
            self.ax.set_ylim(bottom=0)
        self.ax.grid(alpha=.2)
        if self.ax.lines:
            self.ax.legend(loc="upper left")
        self.canvas.draw_idle()
        core_lines, details, processes = [], [], []
        for board, event, age in events:
            live = age is not None and age <= STALE_S and event["cpu"]["state"] == "live"
            state = "현재" if live else "이전/준비 값"
            cores = "  ".join(c["id"]+": "+(f'{c["percent"]:.0f}%' if c["percent"] is not None else "—")+
                              (f' @{c["frequency_mhz"]:.0f}MHz' if c["frequency_mhz"] is not None else "")
                              for c in event["cpu"]["cores"])
            core_lines.append(WIRE_NAMES.get(board, board)+" ["+state+"] "+cores)
            temperatures = ", ".join(f'{t["name"]} {t["celsius"]:.1f}°C' for t in event["temperatures"])
            groups = ", ".join(f"{k} {v:.1f}%" for k, v in group_cpu(event).items() if k != "other")
            swap = event["memory"].get("swap_used_mib")
            details.append(f'{WIRE_NAMES.get(board, board)}: {event["model"]} · 표본 {event["cpu"]["interval_s"]}s · '
                           f'계측 {event["scan_ms"]:.2f}ms · 스왑 {swap}MiB · 표시 외 {event["processes_omitted"]}개\n'
                           +temperatures+"\n"+groups)
            for p in event["processes"]:
                if self._process_visible(board, p):
                    processes.append((board, p, live))
        self.cores.setText("\n".join(core_lines)); self.details.setText("\n".join(details))
        processes.sort(key=lambda item: (item[1]["group"] == "other", item[0],
                       -(item[1]["cpu_percent_one_core"] or 0.)))
        self.table.setRowCount(len(processes))
        for row, (board, p, live) in enumerate(processes):
            cpu = p["cpu_percent_one_core"]
            label = {"planning_gpp_lpp": "플래닝 GPP+LPP", "control": "제어", "vesc_driver": "VESC 드라이버",
                     "mapping": "지도", "semantic": "의미분할", "stereo_cloud": "점군 생성", "camera": "카메라",
                     "lidar": "라이다", "slope": "경사 계산", "measurement": "계측/전달", "other_nomad": "기타 NOMAD", "other": "기타"}
            values = [WIRE_NAMES.get(board, board), label.get(p["group"], p["group"]), str(p["pid"]), p["name"],
                      (f"{cpu:.1f}%" if cpu is not None else "준비 중")+("" if live else " (이전/준비)"),
                      f'{p["rss_mib"]:.1f}', str(p["threads"])]
            for col, value in enumerate(values):
                item = QtWidgets.QTableWidgetItem(value);item.setFlags(item.flags() & ~QtCore.Qt.ItemIsEditable)
                self.table.setItem(row, col, item)

    def export_recording(self):
        from session_ui import choose_recording
        from pathlib import Path
        import sys
        filename = choose_recording(self)
        if not filename:
            return
        process = QtCore.QProcess(self)
        self.export_process = process
        self.export_button.setEnabled(False)
        self.export_status.setText("분석 중 · 원본 기록은 유지됩니다")
        process.setProgram(sys.executable)
        process.setArguments([str(Path(__file__).with_name("resource_report.py")), filename])
        process.finished.connect(self.export_finished)
        process.errorOccurred.connect(self.export_failed)
        process.start()

    def export_finished(self, code, _status):
        process = self.export_process
        if process is None:
            return
        output = bytes(process.readAllStandardOutput()).decode("utf-8", errors="replace").strip()
        error = bytes(process.readAllStandardError()).decode("utf-8", errors="replace").strip()
        self.export_status.setText("분석 저장: " + output if code == 0 else "분석 실패: " + error[-400:])
        self.export_button.setEnabled(True)
        process.deleteLater(); self.export_process = None

    def export_failed(self, _error):
        self.export_status.setText("분석 실행 실패 · 파일 및 실행 환경 확인")
        self.export_button.setEnabled(True)
