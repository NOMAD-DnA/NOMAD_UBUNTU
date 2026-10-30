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


def test_slope_is_captured_per_queue_item_and_forwarded_to_runner(app,tmp_path,monkeypatch):
    window=Lab(tmp_path)
    process=Mock();process.poll.return_value=None
    launch=Mock(return_value=process)
    monkeypatch.setattr('nomad_bringup.scenario_lab.gui.subprocess.Popen',launch)
    try:
        assert not window.slope.isEnabled()
        window.select.setCurrentIndex(2)
        assert window.slope.isEnabled()
        window.slope.setValue(7.5);window.add()
        window.slope.setValue(15.);window.add()
        assert [j['slope_deg'] for j in window.jobs]==[7.5,15.]
        assert window.queue.item(0,5).text()=='7.5°'
        window.start()
        command=launch.call_args.args[0]
        assert command[command.index('--slope-deg')+1]=='7.5'
    finally:
        window.process=window.current=None
        if window.log:window.log.close();window.log=None
        window.close()


def test_startup_failure_shows_reason_and_pauses_queue(app,tmp_path):
    from unittest.mock import patch
    window=Lab(tmp_path)
    try:
        window.add();window.add()
        process=Mock();process.poll.return_value=0;process.returncode=0
        with patch('nomad_bringup.scenario_lab.gui.subprocess.Popen',return_value=process):
            window.start()
        folder=window.current['directory']
        from pathlib import Path
        save(Path(folder)/'outcome.json',dict(scenario=1,result="startup_error: No module named 'nomad_vehicle_model'",cleanup_ok=True))
        window.poll()
        assert not window.running
        assert window.jobs[1]['state']=='queued'
        assert 'nomad_vehicle_model' in window.live.text()
        assert '.runner.log' in window.live.text()
    finally:window.close()
