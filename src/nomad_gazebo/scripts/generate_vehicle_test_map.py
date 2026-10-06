#!/usr/bin/env python3
"""Generate an independent grey incline test world from editable authoring inputs."""
import argparse,copy,json,math
from pathlib import Path
import xml.etree.ElementTree as ET

def node(parent,tag,text=None,**attrs):
 e=ET.SubElement(parent,tag,attrs)
 if text is not None:e.text=str(text)
 return e

def material(visual,grey):
 m=node(visual,'material');node(m,'ambient',f'{grey} {grey} {grey} 1');node(m,'diffuse',f'{grey} {grey} {grey} 1');node(m,'specular','0.05 0.05 0.05 1')

def friction(collision,mu):
 surface=node(collision,'surface');ode=node(node(surface,'friction'),'ode');node(ode,'mu',mu);node(ode,'mu2',mu)

def save_xml(root,path):
 ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

def mesh(path,length,width,height):
 verts=[(0,-width/2,0),(2*length,-width/2,0),(length,-width/2,height),(0,width/2,0),(2*length,width/2,0),(length,width/2,height)]
 faces=[(0,1,2),(3,5,4),(0,3,4),(0,4,1),(0,2,5),(0,5,3),(1,4,5),(1,5,2)]
 normals=[]
 for a,b,c in faces:
  u=[verts[b][j]-verts[a][j] for j in range(3)];v=[verts[c][j]-verts[a][j] for j in range(3)]
  cross=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]];norm=math.sqrt(sum(x*x for x in cross));normals.append([x/norm for x in cross])
 (path.parent/'ramp_grey.mtl').write_text('newmtl grey\nKa 0.64 0.64 0.64\nKd 0.64 0.64 0.64\nKs 0.05 0.05 0.05\n')
 text='# Closed convex triangular prism; metres\nmtllib ramp_grey.mtl\no '+path.stem+'\nusemtl grey\n'
 text+=''.join('v '+' '.join(f'{x:.10f}' for x in v)+'\n' for v in verts)
 text+=''.join('vn '+' '.join(f'{x:.10f}' for x in n)+'\n' for n in normals)
 text+=''.join('f '+' '.join(f'{v+1}//{i+1}' for v in f)+'\n' for i,f in enumerate(faces))
 path.write_text(text)

def generate(package):
 bundle=package/'maps/03_vehicle_incline_test';cfg=json.loads((bundle/'assets/authoring/vehicle_test.json').read_text())
 angles=cfg['incline_degrees'];length=float(cfg['ascent_horizontal_m']);width=float(cfg['lane_width_m']);spacing=float(cfg['lane_spacing_m']);mu=float(cfg['friction_mu']);payload=float(cfg['payload_kg'])
 assert angles and all(0<a<60 for a in angles) and len(set(angles))==len(angles)
 assert length>0 and spacing>width>0 and mu>=0 and payload>=0
 ground=cfg['ground_size_m'];assert len(ground)==2 and all(x>0 for x in ground)
 start=float(cfg['ramp_start_x_m']);ys=[(i-(len(angles)-1)/2)*spacing for i in range(len(angles))]
 assert abs(start)<ground[0]/2 and start+2*length<ground[0]/2
 assert max(abs(y) for y in ys)+width/2<ground[1]/2
 for folder in ['assets/geometry','worlds','models/nomad_vehicle_test']:(bundle/folder).mkdir(parents=True,exist_ok=True)
 # Only common world systems and GUI; forest geometry never enters this bundle.
 template=ET.parse(package/'worlds/forest.sdf').getroot().find('world');sdf=ET.Element('sdf',version='1.10');world=node(sdf,'world',name='nomad_forest')
 for tag in ['gravity','physics','plugin','scene','light','gui']:
  for e in template.findall(tag):world.append(copy.deepcopy(e))
 world.find('scene/background').text='0.65 0.65 0.65 1'
 view=world.find("gui/plugin[@filename='MinimalScene']")
 if view is not None:
  view.find('background_color').text='0.65 0.65 0.65';view.find('camera_pose').text='-30 -48 48 0 0.7 0.9'
 groundmodel=node(world,'model',name='test_flat_ground');node(groundmodel,'static','true');link=node(groundmodel,'link',name='ground');node(link,'pose','0 0 -0.1 0 0 0')
 for kind in ['visual','collision']:
  e=node(link,kind,name=kind);node(node(node(e,'geometry'),'box'),'size',f'{ground[0]} {ground[1]} 0.2')
  if kind=='visual':material(e,.42)
  else:friction(e,mu)
 lanes=[]
 for a,y in zip(angles,ys):
  h=length*math.tan(math.radians(a));name=f'ramp_{a:02d}_deg';meshfile=bundle/f'assets/geometry/{name}.obj';mesh(meshfile,length,width,h)
  model=node(world,'model',name=name);node(model,'static','true');node(model,'pose',f'{start} {y} 0 0 0 0');link=node(model,'link',name='ramp')
  for kind in ['visual','collision']:
   e=node(link,kind,name=kind);g=node(node(e,'geometry'),'mesh');node(g,'uri',f'../assets/geometry/{name}.obj');node(g,'scale','1 1 1')
   if kind=='visual':material(e,.64)
   else:friction(e,mu)
  lanes.append({'angle_deg':a,'center_y_m':y,'start_x_m':start,'apex_x_m':start+length,'end_x_m':start+2*length,'height_m':h,'ascent_surface_length_m':length/math.cos(math.radians(a))})
 vehicle=ET.parse(package/'models/nomad_vehicle/model.sdf').getroot();model=vehicle.find('model');inertial=model.find("link[@name='base_link']/inertial")
 # Payload shares the original chassis CG; both mass and inertia are updated.
 mass=inertial.find('mass');mass.text=str(float(mass.text)+payload)
 sx,sy,sz=cfg['payload_size_m'];assert all(x>0 for x in [sx,sy,sz])
 for key,addition in [('ixx',payload*(sy*sy+sz*sz)/12),('iyy',payload*(sx*sx+sz*sz)/12),('izz',payload*(sx*sx+sy*sy)/12)]:
  e=inertial.find('inertia/'+key);e.text=str(float(e.text)+addition)
 for visual in model.findall('link/visual'):
  old=visual.find('material')
  if old is not None:visual.remove(old)
  material(visual,.28 if 'wheel' in visual.get('name','') else .55)
 vehicle_path=bundle/'models/nomad_vehicle_test/model.sdf';save_xml(vehicle,vehicle_path)
 config=ET.Element('model');node(config,'name','nomad_vehicle_test');node(config,'version','1.0');node(config,'sdf','model.sdf',version='1.10');node(config,'description','Grey NOMAD test vehicle; payload changes chassis mass and inertia at its existing CG.');save_xml(config,bundle/'models/nomad_vehicle_test/model.config')
 spawn=cfg['spawn_pose'];world_vehicle=copy.deepcopy(model);node(world_vehicle,'pose',' '.join(str(x) for x in spawn));world.append(world_vehicle)
 assert len(spawn)==6 and abs(spawn[0])<ground[0]/2 and abs(spawn[1])<ground[1]/2
 for filename in ['forest.sdf','forest_bare.sdf']:save_xml(sdf,bundle/'worlds'/filename)
 total=sum(float(e.text) for e in model.findall('link/inertial/mass'))
 result={'lanes':lanes,'payload_kg':payload,'vehicle_total_mass_kg':total,'friction_mu':mu,'vehicle_model':'nomad_vehicle_test','world_name':'nomad_forest','drive_model':'existing velocity-command Ackermann; not a calibrated torque-limited motor','payload_assumption':'rectangular payload at original chassis inertial CG','spawn_pose':spawn}
 (bundle/'assets/test_layout.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
 print(f'Generated {bundle}: {len(lanes)} ramps, total vehicle mass {total:.3f} kg')

if __name__=='__main__':
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--package-dir',type=Path,default=Path(__file__).resolve().parents[1]);args=parser.parse_args();generate(args.package_dir.resolve())
