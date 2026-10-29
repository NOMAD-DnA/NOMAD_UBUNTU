"""Build a coherent disposable simulator/calibration profile; preserve map physics."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET
import yaml


def generator(root):
    spec = importlib.util.spec_from_file_location('nomad_gui_vehicle', root / 'src/nomad_gazebo/scripts/generate_vehicle.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def validate(root, sensors):
    generator(root).sensor_parameters(sensors)
    for section, settings in sensors.items():
        for key, value in settings.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError(f'{section}.{key}: 유한한 숫자가 필요합니다.')
                if ('noise' in key or key in ('fps', 'hz')) and value < 0:
                    raise ValueError(f'{section}.{key}: 음수는 사용할 수 없습니다.')
    gps = sensors['gps']
    if gps['hz'] <= 0 or not -90 <= gps['latitude_deg'] <= 90 or not -180 <= gps['longitude_deg'] <= 180:
        raise ValueError('GPS 주기 또는 원점 범위를 확인하세요.')


def write_yaml(path, data):
    path.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))


def calibration(root, sensors, directory, method):
    source = root / 'src/nomad_vio/config/vio' / method
    target = directory / method
    shutil.copytree(source, target, dirs_exist_ok=True)
    p = generator(root).sensor_parameters(sensors)
    k = p['intrinsics']['mono']
    c = sensors['camera']
    if method == 'basalt':
        path = target / 'nomad_calib.json'
        data = json.loads(path.read_text())
        values = data['value0']
        for i in range(2):
            values['intrinsics'][i]['intrinsics'] = {key: k[key] for key in ('fx', 'fy', 'cx', 'cy')}
            values['resolution'][i] = [c['width'], c['height']]
            values['T_imu_cam'][i]['py'] = c['baseline_m'] / 2 * (1 if i == 0 else -1)
        values['imu_update_rate'] = sensors['imu']['hz']
        # Existing Basalt schema calls this imu_update_rate, verified in source config.
        path.write_text(json.dumps(data, indent=2) + '\n')
    else:
        path = target / 'nomad_gazebo/kalibr_imucam_chain.yaml'
        data = yaml.safe_load(path.read_text().replace('%YAML:1.0', ''))
        for i in range(2):
            cam = data[f'cam{i}']
            cam['intrinsics'] = [k[key] for key in ('fx', 'fy', 'cx', 'cy')]
            cam['resolution'] = [c['width'], c['height']]
            cam['T_imu_cam'][1][3] = c['baseline_m'] / 2 * (1 if i == 0 else -1)
        write_yaml(path, data)
        path = target / 'nomad_gazebo/kalibr_imu_chain.yaml'
        data = yaml.safe_load(path.read_text().replace('%YAML:1.0', ''))
        for block in data.values():
            if isinstance(block, dict) and 'update_rate' in block:
                block['update_rate'] = sensors['imu']['hz']
        write_yaml(path, data)
    return target


def runtime_world(root, sensors, simulation, directory):
    # Preserve the selected map's links, inertias, contacts, drivetrain, spawn and scenery.
    spec = importlib.util.spec_from_file_location('nomad_gui_map', root / 'src/nomad_gazebo/scripts/select_map.py')
    maps = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(maps)
    original = maps.choose(maps.discover(root / 'src/nomad_gazebo/maps'), simulation['map'], False)['world']
    world, robot = generator(root).prepare_sensor_world(original, sensors, directory, yaml.safe_load((root/'src/nomad_bringup/config/vehicle.yaml').read_text())['/**']['ros__parameters'])
    return world, original.parent.parent, robot


def profile(root, menu, state, directory):
    validate(root, state['sensors'])
    method = state['simulation'].get('vio', 'basalt')
    args = []
    if directory is None:
        sensor_file, world, resource, robot, vio, adapter, fusion = ('<이번 실행/' + name + '>' for name in ('sensors.yaml', 'world.sdf', '원본맵', 'nomad_vehicle.urdf', method, 'adapter.yaml', 'fusion.yaml'))
    else:
        directory = Path(directory)
        sensor_file = directory / 'sensors.yaml'
        write_yaml(sensor_file, state['sensors'])
        if menu in ('all', 'sim'):
            world, resource, robot = runtime_world(root, state['sensors'], state['simulation'], directory)
        vio = calibration(root, state['sensors'], directory, method)
        c, gps = state['sensors']['camera'], state['sensors']['gps']
        mount = [c[f'mount_{axis}_m'] for axis in 'xyz']
        adapter_data = yaml.safe_load((root / 'src/nomad_vio/config/fusion_adapter.yaml').read_text())
        adapter_data['nomad_fusion_adapter']['ros__parameters']['imu_in_base'] = mount
        adapter = directory / 'adapter.yaml'
        write_yaml(adapter, adapter_data)
        data = deepcopy(state['configs']['fusion'])
        for block in data.values():
            parameters = block['ros__parameters']
            parameters['datum'] = [gps['latitude_deg'], gps['longitude_deg'], gps['elevation_m']]
            parameters['antenna_in_imu'] = [gps[f'mount_{axis}_m'] - mount[i] for i, axis in enumerate('xyz')]
        fusion = directory / 'fusion.yaml'
        write_yaml(fusion, data)
    if menu in ('all', 'sim'):
        args += [f'sensor_config:={sensor_file}', f'world_file:={world}', f'world_resource_root:={resource}', f'robot_description_file:={robot}']
    if menu in ('all', 'localization'):
        args += [f'vio_config_directory:={vio}', f'adapter_config:={adapter}', f'{"fusion_config" if menu == "all" else "config"}:={fusion}']
    return args
