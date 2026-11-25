"""Windows spectator entry point. Reuse the operator's existing TCP relay."""
from __future__ import annotations

import argparse
import configparser
import os
from pathlib import Path
import sys

DEMO_RELAY = "10.42.0.1:8768"


def demo_config_path():
    return Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "NOMAD-Spectator" / "demo.ini"


def ensure_demo_config(path=None):
    path = Path(path) if path is not None else demo_config_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        # Exclusive creation preserves edits across reinstall and app upgrades.
        with path.open("x", encoding="utf-8", newline="\n") as handle:
            handle.write("; Save this file, then reopen NOMAD Demo to apply.\n"
                         "[connection]\nrelay = " + DEMO_RELAY + "\n")
    except FileExistsError:
        pass
    return path


def load_demo_relay(path=None):
    path = ensure_demo_config(path)
    config = configparser.ConfigParser(interpolation=None)
    try:
        config.read_string(path.read_text(encoding="utf-8-sig"))
        from launcher import validate_relay_address
        return validate_relay_address(config.get("connection", "relay").strip())
    except (configparser.Error, ValueError, UnicodeError) as error:
        raise ValueError("시연 주소 설정을 확인하세요.\n" + str(path) +
                         "\n\n[connection]\nrelay = 10.42.0.1:8768") from error


def connection_args(settings, *, demo=False, config_path=None):
    if demo:
        return spectator_args(load_demo_relay(config_path))
    from PyQt5 import QtWidgets
    address = str(settings.value("relay", ""))
    while True:
        address, accepted = QtWidgets.QInputDialog.getText(
            None, "NOMAD Windows 관전", "우분투 운용 PC의 중계 주소를 입력하세요.\n예: 192.168.3.100:8768\n공유폴더 주소가 아니라 운용 PC 주소입니다.", text=address)
        if not accepted:
            return None
        try:
            args = spectator_args(address)
            settings.setValue("relay", args.relay_host)
            return args
        except ValueError as error:
            QtWidgets.QMessageBox.warning(None, "주소 확인", str(error))


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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--demo", action="store_true", help="Open the demo hotspot relay without an address dialog")
    parser.add_argument("--init-demo-config", action="store_true", help="Create the user demo config if missing, preserving existing settings")
    options = parser.parse_args()
    if options.init_demo_config:
        print(ensure_demo_config())
        return 0
    state = Path(os.environ.get("LOCALAPPDATA", str(Path.home()))) / "NOMAD-Spectator" / "state"
    state.mkdir(parents=True, exist_ok=True)
    os.environ["NOMAD_VIEWER_STATE_DIR"] = str(state)
    os.environ["MPLCONFIGDIR"] = str(state / "matplotlib")
    os.environ["XDG_STATE_HOME"] = str(state)
    from PyQt5 import QtCore, QtWidgets
    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("NOMAD Spectator")
    settings = QtCore.QSettings(str(state / "connection.ini"), QtCore.QSettings.IniFormat)
    try:
        args = connection_args(settings, demo=options.demo)
    except (OSError, ValueError) as error:
        QtWidgets.QMessageBox.critical(None, "시연 설정 오류", str(error))
        return 1
    if args is None:
        return 0
    import monitor
    try:
        window = monitor.MonitorWindow(args)
    except Exception as error:
        QtWidgets.QMessageBox.critical(None, "관전 창 열기 실패", str(error))
        return 1
    window.setWindowTitle(("NOMAD Demo 관전 - " if options.demo else "NOMAD Windows 관전 - ") + args.relay_host)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
