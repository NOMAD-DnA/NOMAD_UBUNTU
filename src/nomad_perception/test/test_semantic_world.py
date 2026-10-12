from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from nomad_perception.semantic_world import instrument_world, load_world, canonical


def _source_root():
    # Use the runtime's own source copy when installed in a workspace. The
    # NOMAD_SOURCE_ROOT override lets standalone audit tests use a chosen copy.
    import os
    override = os.environ.get('NOMAD_SOURCE_ROOT')
    if override:
        return Path(override)
    return Path(__file__).resolve().parents[2] / 'nomad_gazebo'


@pytest.mark.parametrize('map_name,variant', [('03_vehicle_incline_test', 'forest.sdf'),
                                           ('01_original', 'forest_bare.sdf'),
                                           ('01_original', 'forest.sdf')])
def test_instrumentation_preserves_physics_and_authors_classes(map_name, variant, tmp_path):
    root = _source_root()
    source = root / 'maps' / map_name / 'worlds' / variant
    model = root / 'models' / 'nomad_vehicle' / 'model.sdf'
    before = load_world(source, {}, model).getroot().find('world')
    target, bridges = instrument_world(source, model, tmp_path)
    after = ET.parse(target).getroot().find('world')
    for tag in ('collision', 'inertial', 'physics', 'joint'):
        assert [canonical(item) for item in before.iter(tag)] == [canonical(item) for item in after.iter(tag)]
    existing_sensors = [canonical(item) for item in before.iter('sensor')]
    copied_sensors = [canonical(item) for item in after.iter('sensor')
                      if item.get('name') != 'goose_semantic_oak_rgb']
    assert existing_sensors == copied_sensors
    camera = after.find("model[@name='nomad_vehicle']/link/sensor[@name='goose_semantic_oak_rgb']")
    assert camera is not None and camera.get('type') == 'segmentation'
    assert camera.findtext('camera/image/width') == '640'
    assert camera.findtext('camera/lens/intrinsics/fx') == '465.602889175182'
    assert len(bridges) == 2
    assert bridges[0]['ros_topic_name'] == '/nomad/semantic_gt/oak_rgb/labels'
    assert bridges[0]['gz_topic_name'] == '/nomad/semantic_gt/oak_rgb/labels_map'
    ids = {int(item.text) for item in after.findall('.//visual/plugin/label')}
    assert 8 in ids
    if map_name == '03_vehicle_incline_test':
        assert 23 in ids  # User-selected grey flat/ramp substitute: asphalt.
    else:
        assert 31 in ids  # User-selected entire soil/leaves photographed terrain.
    if variant == 'forest.sdf' and map_name == '01_original':
        assert {27, 28, 40, 51}.issubset(ids)
    with pytest.raises(FileExistsError):
        instrument_world(source, model, tmp_path)


def test_default_model_uri_resolves_using_copied_vehicle(tmp_path):
    root = _source_root()
    target, _ = instrument_world(root / 'worlds' / 'forest_bare.sdf',
                                root / 'models' / 'nomad_vehicle' / 'model.sdf', tmp_path)
    world = ET.parse(target).getroot().find('world')
    assert not world.findall('include')
    assert world.find("model[@name='nomad_vehicle']") is not None
