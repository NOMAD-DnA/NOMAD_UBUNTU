"""Initialization errors must be recorded before any simulator starts."""
from unittest.mock import patch
from nomad_bringup.scenario_lab.runner import run
from nomad_bringup.scenario_lab.core import read


def test_missing_vehicle_dependency_is_recorded(tmp_path,monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID','219')
    monkeypatch.setenv('GZ_PARTITION','nomad_startup_regression')
    folder=tmp_path/'trial'
    with patch('nomad_bringup.scenario_lab.runner.check_conflicts'), \
         patch('nomad_bringup.scenario_lab.runner.algorithm_config',side_effect=ModuleNotFoundError("No module named 'nomad_vehicle_model'")), \
         patch('nomad_bringup.scenario_lab.runner.subprocess.Popen') as launch:
        result=run(1,folder)
    launch.assert_not_called()
    assert result['result'].startswith('startup_error:')
    assert result['cleanup_ok'] and not result['completed']
    assert read(folder/'outcome.json')['result']==result['result']
    assert 'ModuleNotFoundError' in (folder/'error.log').read_text()
