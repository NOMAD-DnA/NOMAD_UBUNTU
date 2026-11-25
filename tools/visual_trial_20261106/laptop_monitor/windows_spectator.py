"""Windows spectator entry point. Reuse the operator's existing TCP relay."""
from __future__ import annotations

import os
from pathlib import Path
import sys


def spectator_args(endpoint):
    # Build a fresh argument list: never forward operator/control CLI options.
    from launcher import validate_relay_address
    import monitor
    previous = sys.argv
    try:
        sys.argv = [previous[0], "--relay-host", validate_relay_address(endpoint), "--no-ssh"]
        return monitor.parse_args()
    finally:
        sys.argv = previous


def main():
    state = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "NOMAD-Spectator" / "state"
    state.mkdir(parents=True, exist_ok=True)
    os.environ["NOMAD_VIEWER_STATE_DIR"] = str(state)
    os.environ["MPLCONFIGDIR"] = str(state / "matplotlib")
    os.environ["XDG_STATE_HOME"] = str(state)
    from PyQt5 import QtCore, QtWidgets
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("NOMAD Spectator")
    settings = QtCore.QSettings(str(state / "connection.ini"), QtCore.QSettings.IniFormat)
    address = str(settings.value("relay", ""))
    while True:
        address, accepted = QtWidgets.QInputDialog.getText(
            None, "NOMAD Windows 관전", "우분투 운용 PC의 중계 주소를 입력하세요.\n예: 192.168.3.100:8768\n공유폴더 주소가 아니라 운용 PC 주소입니다.", text=address)
        if not accepted:
            return 0
        try:
            args = spectator_args(address)
            break
        except ValueError as error:
            QtWidgets.QMessageBox.warning(None, "주소 확인", str(error))
    settings.setValue("relay", args.relay_host)
    import monitor
    try:
        window = monitor.MonitorWindow(args)
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, "관전 창 열기 실패", str(error))
        return 1
    window.setWindowTitle("NOMAD Windows 관전 - " + args.relay_host)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
