#!/usr/bin/env python3
"""2026-11-17 NOMAD VIO 시험기 — VIO·융합 야외 시험용 서브 GUI (노트북 전용, 바탕화면 "NOMAD VIO 시험기 (로컬)").

- 운영 GUI(공유폴더 laptop_monitor)가 시험을 시작해 센서·VIO·융합을 띄운 상태에서 옆에 띄워 쓴다.
- 위치추정 보기: 운영 GUI의 관전 중계(기본 127.0.0.1:8768)에 관전자로 접속(읽기 전용, 새 보드 구독 없음).
- 키보드 주행·센서 녹화: 공유 GUI의 '조종·녹화' 창(drive_record_dialog)을 수정 없이 그대로 연다.
- 융합기 전환: remote/pi5_fusion_test_{on,off,status}.sh 를 control.run_remote로 Pi5에서 실행(Pi5 ~/nomad_fusion_test_kit 필요).
- 업데이트: 로컬 사본이면 켤 때 공유폴더 새 버전을 확인(local_install.offer, 운영 GUI와 같은 방식).
비밀번호는 이 창의 입력칸에서만 받고 저장하지 않는다.
"""
from __future__ import annotations

import math
import os
import sys
import threading
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import numpy as np
from PyQt5 import QtCore, QtWidgets
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
import matplotlib
matplotlib.rcParams["font.family"] = "Noto Sans CJK JP"; matplotlib.rcParams["axes.unicode_minus"] = False

import vio_tester_view as fv     # State, relay_thread, bag_thread
import drive_record_dialog       # 운영 GUI의 조종·녹화 창(수정하지 않고 사용)
from control import run_remote

BLUE, GRAY, ORANGE, GREEN = "#2a78d6", "#b9b8b3", "#eb6834", "#1baf7a"


def board_roles():
    """운영 GUI가 보드 찾기·시험 시작 때 저장한 주소(board_routes.json)를 지금 경로로 풀어서 돌려준다."""
    try:
        import board_address
        roles = dict(board_address._load().get("roles", {}))
        return {role: board_address.route(host).get("host") or host for role, host in roles.items()}
    except Exception:
        return {}


class Tester(QtWidgets.QMainWindow):
    remote_done = QtCore.pyqtSignal(str)

    def __init__(self, relay=None, bag=None, speed=1.0):
        super().__init__()
        self.setWindowTitle("NOMAD VIO 시험기 — 위치추정 보기 · 키보드 주행")
        self.resize(1500, 950)
        self.state = fv.State(); self.drive = None
        roles = board_roles()
        w = QtWidgets.QWidget(); self.setCentralWidget(w); v = QtWidgets.QVBoxLayout(w)
        top = QtWidgets.QGridLayout(); v.addLayout(top)
        self.pi4 = QtWidgets.QLineEdit(roles.get("pi4", "")); self.jetson = QtWidgets.QLineEdit(roles.get("jetson", ""))
        self.pi5 = QtWidgets.QLineEdit(roles.get("pi5", "")); self.password = QtWidgets.QLineEdit(); self.password.setEchoMode(QtWidgets.QLineEdit.Password)
        self.relay = QtWidgets.QLineEdit(relay or "127.0.0.1:8768")
        for i, (lab, wid) in enumerate([("Pi4", self.pi4), ("Jetson", self.jetson), ("Pi5", self.pi5), ("비밀번호(저장 안 함)", self.password), ("관전 중계", self.relay)]):
            top.addWidget(QtWidgets.QLabel(lab), 0, 2 * i); top.addWidget(wid, 0, 2 * i + 1)
        # 보드 주소는 운영 GUI(보드 찾기·시험 시작)가 저장한 값을 2초마다 따라간다. 직접 고친 칸은 그대로 둔다.
        self.edited = {"pi4": False, "jetson": False, "pi5": False}
        for role, wid in (("pi4", self.pi4), ("jetson", self.jetson), ("pi5", self.pi5)):
            wid.setToolTip("운영 GUI가 찾은 주소를 자동으로 따라갑니다. 직접 고치면 그 값을 씁니다(비우면 다시 자동).")
            wid.textEdited.connect(lambda text, r=role: self.edited.__setitem__(r, bool(text.strip())))
        self.addr_timer = QtCore.QTimer(self); self.addr_timer.timeout.connect(self.follow_boards); self.addr_timer.start(2000)
        row = QtWidgets.QHBoxLayout(); v.addLayout(row)
        self.b_relay = QtWidgets.QPushButton("중계 연결(보기 시작)"); self.b_relay.clicked.connect(self.start_relay)
        self.b_drive = QtWidgets.QPushButton("조종·녹화 창 열기 (키보드 주행)"); self.b_drive.clicked.connect(self.open_drive)
        self.b_on = QtWidgets.QPushButton("Pi5 융합기 → 시험판(window_xy)"); self.b_on.clicked.connect(lambda: self.pi5_swap("on"))
        self.b_off = QtWidgets.QPushButton("Pi5 융합기 → 원래대로"); self.b_off.clicked.connect(lambda: self.pi5_swap("off"))
        self.b_stat = QtWidgets.QPushButton("융합기 상태"); self.b_stat.clicked.connect(lambda: self.pi5_swap("status"))
        self.b_clear = QtWidgets.QPushButton("궤적 지우기"); self.b_clear.clicked.connect(self.state.reset)
        for b in (self.b_relay, self.b_drive, self.b_on, self.b_off, self.b_stat, self.b_clear): row.addWidget(b)
        self.share_label = QtWidgets.QLabel("공유폴더: —"); v.addWidget(self.share_label)
        self.msg = QtWidgets.QLabel("운영 GUI에서 시험을 시작한 뒤 '중계 연결'을 누르세요. 융합기 전환은 시험 시작 후에만 됩니다."); self.msg.setWordWrap(True); v.addWidget(self.msg)
        self.fig = Figure(figsize=(14, 8)); self.canvas = FigureCanvasQTAgg(self.fig); v.addWidget(self.canvas, 1)
        gs = self.fig.add_gridspec(2, 2, width_ratios=[1.6, 1], height_ratios=[2.2, 1], hspace=.32, wspace=.18, left=.05, right=.98, top=.95, bottom=.07)
        self.axM = self.fig.add_subplot(gs[:, 0]); self.axT = self.fig.add_subplot(gs[0, 1]); self.axH = self.fig.add_subplot(gs[1, 1])
        self.remote_done.connect(self.msg.setText)
        self.timer = QtCore.QTimer(self); self.timer.timeout.connect(self.draw); self.timer.start(500)
        if bag:
            threading.Thread(target=fv.bag_thread, args=(self.state, bag, speed), daemon=True).start()
            self.b_relay.setEnabled(False)

    # ---- 동작
    def follow_boards(self):
        roles = board_roles()
        for role, wid in (("pi4", self.pi4), ("jetson", self.jetson), ("pi5", self.pi5)):
            if not self.edited[role] and roles.get(role) and wid.text() != roles[role] and not wid.hasFocus():
                wid.setText(roles[role])

    def start_relay(self):
        try:
            host, port = self.relay.text().strip().rsplit(":", 1); port = int(port)
        except ValueError:
            self.msg.setText("중계 주소는 host:port 형식입니다"); return
        threading.Thread(target=fv.relay_thread, args=(self.state, host, port), daemon=True).start()
        self.b_relay.setEnabled(False); self.msg.setText(f"중계 {host}:{port}에 관전자로 연결 중…")

    def open_drive(self):
        self.drive = drive_record_dialog.open_dialog(self.pi4.text().strip(), self.jetson.text().strip(), self.pi5.text().strip(),
                                                     self.password.text() or None, self, self.drive)

    def pi5_swap(self, mode):
        host = self.pi5.text().strip(); script = str(HERE / "remote" / f"pi5_fusion_test_{mode}.sh"); pw = self.password.text() or None
        if mode == "on" and QtWidgets.QMessageBox.question(self, "융합기 전환", "Pi5의 융합기를 시험판(window_xy)으로 바꿉니다.\n자율주행 중이면 하지 마세요. 진행할까요?") != QtWidgets.QMessageBox.Yes:
            return
        self.msg.setText(f"Pi5 융합기 {mode} 실행 중…")
        def work():
            r = run_remote(host, script, pw, 40)
            out = (r.get("stdout") or r.get("output") or "").strip() if isinstance(r, dict) else str(r)
            self.remote_done.emit(f"Pi5 융합기 {mode}: {'성공' if r.get('ok', r.get('returncode') == 0) else '실패'} — {out[-400:] or r}")
        threading.Thread(target=work, daemon=True).start()

    def closeEvent(self, e):
        if self.drive is not None:
            self.drive.close()
        super().closeEvent(e)

    # ---- 그리기 (windowxy_field_view와 같은 내용)
    def draw(self):
        st = self.state
        with st.lock:
            fus = np.array(st.fus) if st.fus else np.zeros((0, 3)); gn = list(st.gnss); v = dict(st.vals); g = dict(st.gtel)
            hist = np.array(st.hist) if st.hist else np.zeros((0, 3)); yaw = st.yaw; mode = st.mode; t0 = st.t0
        axM, axT, axH = self.axM, self.axT, self.axH
        axM.cla(); axM.set_aspect("equal", adjustable="datalim"); axM.grid(alpha=.3)
        if len(fus): axM.plot(fus[:, 1], fus[:, 2], color=BLUE, lw=2, label="융합 위치 (차량이 쓰는 값, VIO 좌표)")
        pre = [x for x in gn if not x[4]]; post = [x for x in gn if x[4]]
        if pre: p = np.array(pre); axM.scatter(p[:, 1], p[:, 2], s=8, color=GRAY, label="GNSS (정렬 전, 회전 안 됨)")
        if post: p = np.array(post); axM.scatter(p[:, 1], p[:, 2], s=14, c=np.clip(p[:, 3], 0, 5), cmap="autumn_r", vmin=0, vmax=5, label="GNSS (정렬 후, VIO 좌표로 변환)", zorder=3)
        if len(fus) and yaw is not None:
            x, y = fus[-1, 1], fus[-1, 2]
            axM.annotate("", xy=(x + 1.2 * math.cos(yaw), y + 1.2 * math.sin(yaw)), xytext=(x, y), arrowprops=dict(arrowstyle="-|>", color=BLUE, lw=2.5))
            axM.plot(fus[0, 1], fus[0, 2], "ks", ms=8, label="출발")
        axM.legend(loc="upper left", fontsize=9); axM.set_xlabel("x (m)"); axM.set_ylabel("y (m)")
        axM.set_title("지도 — 차량 출력 좌표(VIO 기준). GNSS 점 색 = hAcc (노랑 작음 → 빨강 큼, 0~5 m)", fontsize=10)
        def f(k, fmt="{:.2f}"):
            try:
                x = float(v.get(k, "nan")); return fmt.format(x) if math.isfinite(x) else "—"
            except ValueError: return v.get(k, "—")
        aligned = v.get("viewer_wxy_aligned") == "true"
        lines = [f"연결: {st.conn}", "",
                 f"융합 상태 : {mode}" + ("   ⚠ VIO 실패" if v.get("vio_failed") == "true" else ""),
                 f"융합 모드 : {v.get('gnss_fusion_mode', '?')}" + ("" if v.get("gnss_fusion_mode") == "window_xy" else "   (시험판 아님)"), "",
                 f"GNSS      : {fv.FIX_NAMES.get(g.get('fix_type'), g.get('fix_type') if g.get('fix_type') is not None else '—')} · hAcc {g.get('h_acc_m') if g.get('h_acc_m') is not None else '—'} m · 위성 {g.get('satellites', '—')}",
                 f"융합에 쓴 σ: {f('viewer_wxy_gnss_sigma_m')} m", "",
                 "초기 정렬 " + ("✔ 완료" if aligned else "… 진행 중"),
                 f"  쌍 개수      : {v.get('viewer_wxy_pairs', '—')} (최소 10)",
                 f"  직진 범위    : {f('viewer_wxy_fit_span_m', '{:.1f}')} m (최소 5)",
                 f"  정렬 각 오차 : {f('viewer_wxy_fit_sigma_yaw_deg', '{:.1f}')}° (3° 이하면 확정)",
                 f"  정렬 각      : {f('viewer_wxy_fit_yaw_deg', '{:.1f}')}°", "",
                 f"창 안 GNSS 프레임: {v.get('windowxy_frames_with_gnss', '—')} · 풀이 {v.get('solves', '—')}회",
                 f"VIO epoch(재시작 시 바뀜): {v.get('vio_epoch', '—')}"]
        axT.cla(); axT.axis("off"); axT.text(0, 1, "\n".join(lines), va="top", fontsize=11, transform=axT.transAxes)
        axH.cla(); axH.grid(alpha=.3)
        if len(hist) and t0 is not None:
            tt = hist[:, 0] - t0; axH.plot(tt, hist[:, 1], color=ORANGE, lw=1.4, label="GNSS σ (hAcc) m"); axH.plot(tt, hist[:, 2], color=GREEN, lw=1.4, label="정렬 각 오차 °")
            axH.axhline(3, color=GREEN, ls=":", lw=1); axH.set_ylim(0, 12); axH.legend(loc="upper right", fontsize=8)
        axH.set_xlabel("시간 s"); axH.set_title("GNSS 정확도와 정렬 각 오차", fontsize=10)
        self.canvas.draw_idle()


def main():
    import argparse
    ap = argparse.ArgumentParser(); ap.add_argument("--relay"); ap.add_argument("--bag"); ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--screenshot", help="점검용: SHOT_AFTER_S초 뒤 창을 그림으로 저장하고 종료"); ap.add_argument("--connect", action="store_true", help="켜자마자 중계 연결"); a = ap.parse_args()
    app = QtWidgets.QApplication(sys.argv); w = Tester(a.relay, a.bag, a.speed); w.show()
    if a.connect and not a.bag:
        w.start_relay()
    if not a.bag and not a.screenshot:
        try:
            import local_install
            QtCore.QTimer.singleShot(400, lambda: local_install.offer(w, runner="run_vio_tester.sh"))
        except Exception as error:
            w.share_label.setText(f"공유폴더: 업데이트 확인 생략 ({error})")
    if a.screenshot:
        QtCore.QTimer.singleShot(int(float(os.environ.get("SHOT_AFTER_S", "20")) * 1000), lambda: (w.draw(), w.grab().save(a.screenshot), app.quit()))
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
