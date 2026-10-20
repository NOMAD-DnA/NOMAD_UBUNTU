import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml, pytest
R=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('vehicle',R/'src/nomad_gazebo/scripts/generate_vehicle.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
@pytest.mark.parametrize('inline',[True,False])
def test_overlay_preserves_contacts_physics_and_source(tmp_path,inline):
 c=yaml.safe_load((R/'src/nomad_gazebo/config/sensors_oak_d_lite.yaml').read_text())
 m.generate_vehicle(tmp_path/'old',c)
 source=tmp_path/'old/models/nomad_vehicle/model.sdf'
 vt=ET.parse(source); model=vt.find('model'); base=model.find("link[@name='base_link']")
 ET.SubElement(base,'sensor',name='trial_contacts',type='contact')
 vt.write(source)
 root=ET.Element('sdf',version='1.10');world=ET.SubElement(root,'world',name='test')
 if inline:world.append(model)
 else:
  inc=ET.SubElement(world,'include');ET.SubElement(inc,'name').text='nomad_vehicle';ET.SubElement(inc,'uri').text=str(source)
 path=tmp_path/'original.sdf';ET.ElementTree(root).write(path);before=path.read_bytes()
 new=yaml.safe_load((R/'src/nomad_gazebo/config/sensors.yaml').read_text())
 result,robot=m.prepare_sensor_world(path,new,tmp_path/'runtime')
 updated=ET.parse(result).find('world/model') if inline else ET.parse(tmp_path/'runtime/vehicle.sdf').find('model')
 assert updated.find("link/sensor[@name='trial_contacts']") is not None
 assert [ET.tostring(x) for x in model.findall('.//inertial')]==[ET.tostring(x) for x in updated.findall('.//inertial')]
 assert [ET.tostring(x) for x in model.findall('plugin')]==[ET.tostring(x) for x in updated.findall('plugin')]
 params=m.sensor_parameters(new)
 assert float(updated.findtext("link/sensor[@name='oak_rgb']/camera/lens/intrinsics/fx"))==pytest.approx(params['intrinsics']['rgb']['fx'])
 poses=[updated.findtext(f"link/sensor[@name='oak_{side}']/pose").split() for side in ('left','right')]
 assert float(poses[0][1])-float(poses[1][1])==pytest.approx(.12)
 assert path.read_bytes()==before
 assert robot.exists()
