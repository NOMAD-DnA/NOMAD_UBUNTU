"""Sensor profiles must keep estimator geometry and simulator geometry consistent."""
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
path = ROOT / 'src/nomad_bringup/gui/sensor_profile.py'
spec = importlib.util.spec_from_file_location('gui_sensor_profile_test', path)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def sensors():
    return yaml.safe_load((ROOT / 'src/nomad_gazebo/config/sensors.yaml').read_text())


def test_calibrations_match_changed_sim_geometry(tmp_path):
    config = sensors()
    config['camera'].update(width=800, height=600, baseline_m=.1)
    config['imu']['hz'] = 100.
    parameters = module.generator(ROOT).sensor_parameters(config)
    for method in ('basalt', 'openvins'):
        target = module.calibration(ROOT, config, tmp_path, method)
        if method == 'basalt':
            value = json.loads((target / 'nomad_calib.json').read_text())['value0']
            assert value['resolution'] == [[800, 600], [800, 600]]
            assert value['imu_update_rate'] == 100.
            assert value['intrinsics'][0]['intrinsics']['fx'] == parameters['intrinsics']['mono']['fx']
            assert value['T_imu_cam'][1]['py'] == -.05
        else:
            value = yaml.safe_load((target / 'nomad_gazebo/kalibr_imucam_chain.yaml').read_text())
            assert value['cam0']['resolution'] == [800, 600]
            assert value['cam0']['intrinsics'][0] == parameters['intrinsics']['mono']['fx']
            assert value['cam1']['T_imu_cam'][1][3] == -.05


@pytest.mark.parametrize('map_id', ['01_forest', '02_fork2_gentle'])
def test_runtime_world_keeps_vehicle_physics(tmp_path, map_id):
    map_spec = importlib.util.spec_from_file_location('test_map_selector', ROOT / 'src/nomad_gazebo/scripts/select_map.py')
    selector = importlib.util.module_from_spec(map_spec)
    map_spec.loader.exec_module(selector)
    maps = selector.discover(ROOT / 'src/nomad_gazebo/maps')
    # Select actual first map identifier; names may change without altering the contract.
    selected = maps[0] if map_id == '01_forest' else next(m for m in maps if m['id'] == map_id)
    original = selected['world']
    source = ET.parse(original.parent.parent / 'models/nomad_vehicle/model.sdf')
    before = original.read_bytes()
    config = sensors()
    config['camera']['fps'] = 12.
    world, resource, robot = module.runtime_world(ROOT, config, {'map': selected['id']}, tmp_path)
    model = ET.parse(tmp_path / 'vehicle.sdf')
    # Mechanical layout intentionally changes; total mass and sensor geometry do not.
    assert sum(float(x.text) for x in source.findall('.//inertial/mass')) == pytest.approx(sum(float(x.text) for x in model.findall('.//inertial/mass')))
    assert model.find(".//joint[@name='rear_left_steering_joint']") is not None
    assert not any('AckermannSteering' in p.get('name','') for p in model.findall('.//plugin'))
    assert model.find('.//sensor/update_rate').text == '12'
    assert original.read_bytes() == before
    assert resource == original.parent.parent and robot.is_file() and world.is_file()


def test_invalid_sensor_geometry_rejected():
    config = sensors()
    config['camera']['baseline_m'] = -1.
    with pytest.raises(ValueError):
        module.validate(ROOT, config)
