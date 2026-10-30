"""Offscreen local ROS test; never calls board startup or publishes drive commands."""
import json
import os
import time
from argparse import Namespace
import pytest


def test_record_buttons_receive_rosout_and_close(tmp_path, monkeypatch):
    pytest.importorskip('rclpy')
    pytest.importorskip('PyQt5')
    os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
    monkeypatch.setenv('NOMAD_OPERATOR_GUI_LOG_DIR', str(tmp_path / 'logs'))
    from PyQt5.QtWidgets import QApplication, QDialog
    import rclpy
    from rcl_interfaces.msg import Log
    from nomad_operator_gui.app import OperatorWindow
    app = QApplication.instance() or QApplication([])
    topics = Namespace(image='/log_test/image', image_fallback='', grid='/log_test/grid',
                       odom='/log_test/odom', goal='/log_test/goal')
    window = OperatorWindow(topics)
    node = None
    try:
        window.show()
        end = time.monotonic() + 8
        while not rclpy.ok() and time.monotonic() < end:
            app.processEvents(); time.sleep(.02)
        assert rclpy.ok()
        monkeypatch.setattr(QDialog, 'exec_', lambda self: QDialog.Accepted)
        window.record_button.click()
        first = window.recorder.directory
        assert window.recorder.active and not window.record_button.isEnabled()
        node = rclpy.create_node('logging_probe', enable_rosout=False)
        publisher = node.create_publisher(Log, '/rosout', 10)
        message = Log(name='nomad_path_planning', msg='local logging integration probe', level=20)
        found = False
        while time.monotonic() < end:
            publisher.publish(message)
            app.processEvents(); time.sleep(.05)
            window.recorder.flush()
            if 'local logging integration probe' in (first / 'events.jsonl').read_text():
                found = True; break
        assert found
        window.end_record_button.click()
        assert not window.recorder.active
        window.record_button.click()
        second = window.recorder.directory
        assert second != first
        window.on_board_complete({'owned': {}, 'error': 'simulated start failure'})
        assert not window.recorder.active
        assert json.loads((second / 'manifest.json').read_text())['stop_reason'] == 'board_failure'
        window.record_button.click()
        third = window.recorder.directory
        node.destroy_node(); node = None
        window.close()
        assert json.loads((third / 'manifest.json').read_text())['stop_reason'] == 'gui_closed'
    finally:
        if node is not None: node.destroy_node()
        window.close()
        app.processEvents()
