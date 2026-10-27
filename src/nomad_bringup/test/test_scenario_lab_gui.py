"""GUI regression: selected results must never show another run's plot."""
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from unittest.mock import Mock
import pytest
from PyQt5.QtWidgets import QApplication
from nomad_bringup.scenario_lab.core import save
from nomad_bringup.scenario_lab.gui import Lab


@pytest.fixture(scope='module')
def app():
    return QApplication.instance() or QApplication([])


def test_switch_scenario_redraws_even_when_row_number_unchanged(app, tmp_path):
    for number in (1, 2):
        save(tmp_path/'runs'/f'run_{number}'/'outcome.json',
             dict(scenario=number, result=f'result_{number}', cpu={}))
    window = Lab(tmp_path)
    try:
        assert 'result_1' in window.result_note.text()
        window.result_select.setCurrentIndex(1)
        assert window.results.currentRow() == 0
        assert 'result_2' in window.result_note.text()
        assert 'run_2' in window.result_note.text()
    finally:
        window.close()


def test_immediate_stop_does_not_steal_runners_exclusive_output_directory(app, tmp_path):
    window = Lab(tmp_path)
    directory = tmp_path/'runs'/'pending'
    process = Mock()
    process.poll.return_value = None
    window.process = process
    window.current = dict(directory=str(directory), scenario=1)
    try:
        window.stop()
        assert not directory.exists()
        directory.mkdir(parents=True)
        window.poll()
        assert (directory/'STOP').exists()
        assert not window.running
    finally:
        window.process = window.current = None
        window.close()
