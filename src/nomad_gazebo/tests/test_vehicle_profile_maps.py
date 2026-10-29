import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest,yaml
from nomad_vehicle_model import profile
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('profile_generator',R/'src/nomad_gazebo/scripts/generate_vehicle.py')
gen=importlib.util.module_from_spec(spec);spec.loader.exec_module(gen)
maps=sorted((R/'src/nomad_gazebo/maps').glob('*/worlds/forest.sdf'))

@pytest.mark.parametrize('world',maps,ids=lambda p:p.parent.parent.name)
def test_all_map_profiles_keep_spawn_sensors_and_one_drivetrain(world,tmp_path):
    config=yaml.safe_load((R/'src/nomad_gazebo/config/sensors.yaml').read_text())
    values=yaml.safe_load((R/'src/nomad_bringup/config/vehicle.yaml').read_text())['/**']['ros__parameters']
    before=world.read_bytes()
    path,urdf=gen.prepare_sensor_world(world,config,tmp_path,values)
    root=ET.parse(path).find('world');model=root.find("model[@name='nomad_vehicle']")
    if model is None:model=ET.parse(tmp_path/'vehicle.sdf').find('model')
    assert len(model.findall('link'))==7
    assert len(model.findall('joint'))==6
    assert sum(float(e.text) for e in model.findall('link/inertial/mass'))==pytest.approx(34.6)
    assert model.find("joint[@name='rear_left_steering_joint']") is not None
    assert model.find("joint[@name='front_left_steering_joint']") is None
    assert len(model.findall("plugin[@name='gz::sim::systems::JointController']"))==2
    assert len(model.findall("plugin[@name='gz::sim::systems::JointPositionController']"))==2
    assert len(model.findall('link/sensor'))==5
    assert len(ET.parse(urdf).findall("joint[@type='revolute']"))==2
    assert world.read_bytes()==before


def test_geometry_edit_changes_sdf_urdf_and_metadata_together(tmp_path):
    cfg=R/'src/nomad_gazebo/config/sensors.yaml'
    values=dict(drive_layout='front_drive_rear_steer',wheelbase=.8,reference_offset=-.4,
                track=.65,wheel_radius=.17,joint_limit=.5,max_steer=.35)
    m=gen.generate_vehicle(tmp_path,cfg,vehicle_config=values)
    sdf=ET.parse(tmp_path/'models/nomad_vehicle/model.sdf')
    urdf=ET.parse(tmp_path/'urdf/nomad_vehicle.urdf')
    pose=list(map(float,sdf.findtext("model/link[@name='front_left_wheel']/pose").split()))
    assert pose[:3]==pytest.approx([.4,.325,.17])
    assert float(sdf.findtext("model/link[@name='front_left_wheel']/collision/geometry/cylinder/radius"))==pytest.approx(.17)
    assert list(map(float,urdf.find("joint[@name='front_left_wheel_joint']/origin").get('xyz').split()))==pytest.approx([.4,.325,.17])
    assert m['wheelbase_m']==.8 and m['wheel_radius_m']==.17
