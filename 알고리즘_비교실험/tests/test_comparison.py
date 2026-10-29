import os
os.environ.setdefault('QT_QPA_PLATFORM','offscreen')
from unittest.mock import patch,Mock
import pytest
import yaml
from PyQt5.QtWidgets import QApplication
from algorithm_comparison.model import ALGORITHMS,comparison
from algorithm_comparison.gui import ComparisonLab
from nomad_bringup.scenario_lab.runner import algorithm_config
from nomad_bringup.scenario_lab.core import PACKAGE,strategy_timings,resolved_config


@pytest.fixture(scope='module')
def app():return QApplication.instance() or QApplication([])


@pytest.mark.parametrize('key',ALGORITHMS)
def test_both_ros_nodes_receive_pair_and_all_other_settings_preserved(key):
    config=algorithm_config(key);original=resolved_config()
    gpp,lpp=key.split('+')
    for name in ('dstar_lite_gpp','ackermann_rollout_lpp'):
        assert config[name]['ros__parameters']['gpp_algorithm']==gpp
        assert config[name]['ros__parameters']['lpp_algorithm']==lpp
        config[name]['ros__parameters'].update(gpp_algorithm='hybrid_astar',lpp_algorithm='rpp')
    assert config==original


def test_unknown_algorithm_rejected():
    with pytest.raises(ValueError):algorithm_config('hybrid_astar+stanley')


def test_failed_fast_trial_not_used_for_performance():
    base=dict(algorithm='hybrid_astar+rpp',cleanup_ok=True,cpu={})
    rows=comparison([dict(base,result='arrived',completed=True,expected_result_met=True,elapsed_sim_s=100),
                     dict(base,result='timeout',expected_result_met=False,elapsed_sim_s=1)])
    assert rows[0][1:5]==[2,1,1,1]
    assert rows[0][7]==100
    assert rows[0][5] is None
    assert rows[1][1]==0 and rows[1][7] is None


def test_safe_stop_is_expected_but_not_arrival():
    row=comparison([dict(algorithm='hybrid_astar+rpp',cleanup_ok=True,
        result='safe_stop',completed=False,expected_result_met=True)])[0]
    assert row[1:5]==[1,0,1,0]


def test_matrix_and_selected_algorithm_survive_queue_reload_and_launch(app,tmp_path):
    window=ComparisonLab(tmp_path)
    window.repeat.setValue(2);window.add_matrix()
    assert len(window.jobs)==60
    assert all(sum(j['scenario']==n and j['algorithm']==key for j in window.jobs)==2
               for n in range(1,7) for key in ALGORITHMS)
    window.close();window=ComparisonLab(tmp_path)
    try:
        assert len(window.jobs)==60
        window.jobs=window.jobs[1:2]
        process=Mock();process.poll.return_value=None
        with patch('nomad_bringup.scenario_lab.gui.subprocess.Popen',return_value=process) as popen:
            window.start()
            command=popen.call_args.args[0]
            assert command[command.index('--algorithm')+1]=='hybrid_astar+rollout'
        window.log.close();window.process=None;window.current=None
    finally:window.close()


def test_timing_ignores_other_stages_and_keeps_missing_null(tmp_path):
    (tmp_path/'timing').mkdir()
    (tmp_path/'timing'/'lpp.csv').write_text('stage,elapsed_wall_ns\nlpp_strategy,1000000\nlpp_strategy,3000000\nother,990000000\n')
    timing=strategy_timings(tmp_path)
    assert timing['lpp_strategy']['p95_ms']==pytest.approx(2.9)
    assert timing['gpp_strategy']['p95_ms'] is None
