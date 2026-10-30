"""Generate reproducible scenario worlds from preserved HTML grid authoring."""
from pathlib import Path
import xml.etree.ElementTree as ET
import copy
import math
from .core import scenario, rectangles, elevation, workspace, save, resolved_config


def tag(parent,element,text=None,**attrs):
    node=ET.SubElement(parent,element,attrs)
    if text is not None:node.text=str(text)
    return node


def build(number,directory,slope_degrees=None):
    spec=scenario(number,slope_degrees);directory=Path(directory).resolve();directory.mkdir(parents=True,exist_ok=True)
    source=workspace()/'src/nomad_gazebo'
    tree=ET.parse(source/'worlds/forest_bare.sdf');world=tree.find('world')
    for item in list(world):
        if item.tag in ('model','include','actor'):world.remove(item)
        elif item.tag=='plugin' and ('Sensors' in item.get('name','') or 'Imu' in item.get('name','')):world.remove(item)
    # Keep original simulator physics, lighting and UI; replace map geometry only.
    camera=world.find('gui/plugin/camera_pose')
    if camera is not None:camera.text='-4 -10 26 0 .8 .6'
    floor=tag(world,'model',name='scenario_ground');tag(floor,'static','true');link=tag(floor,'link',name='ground')
    if number==3:
        # Triangulated continuous physical surface matching the declared extension.
        vertices=[];faces=[];r=spec['resolution'];w,h=spec['width'],spec['height']
        for y in range(h+1):
            for x in range(w+1):vertices.append((x*r,y*r,elevation(spec,x*r,y*r)))
        for y in range(h):
            for x in range(w):
                a=y*(w+1)+x+1;b=a+1;c=a+w+1;d=c+1
                faces.extend([(a,b,d),(a,d,c)])
        mesh=directory/'terrain.obj'
        normals=[]
        for a,b,c in faces:
            u=[vertices[b-1][i]-vertices[a-1][i] for i in range(3)]
            v=[vertices[c-1][i]-vertices[a-1][i] for i in range(3)]
            n=(u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0])
            length=math.sqrt(sum(x*x for x in n));normals.append(tuple(x/length for x in n))
        # DART's mesh collision loader requires normals for every face vertex.
        (directory/'terrain.mtl').write_text('newmtl terrain\nKd 0.55 0.62 0.43\n')
        mesh.write_text('\n'.join(['mtllib terrain.mtl','o ground','usemtl terrain']+
            ['v '+' '.join(map(str,v)) for v in vertices]+
            ['vn '+' '.join(map(str,n)) for n in normals]+
            ['f '+' '.join(f'{v}//{i+1}' for v in face) for i,face in enumerate(faces)])+'\n')
    for kind in ('collision','visual'):
        item=tag(link,kind,name='surface');geometry=tag(item,'geometry')
        if number==3:tag(tag(geometry,'mesh'),'uri',str(mesh))
        else:
            plane=tag(geometry,'plane');tag(plane,'normal','0 0 1');tag(plane,'size','80 80')
        if kind=='visual':tag(tag(item,'material'),'diffuse','.55 .62 .43 1')
        else:
            ode=tag(tag(tag(item,'surface'),'friction'),'ode');tag(ode,'mu',1.);tag(ode,'mu2',1.)
    for i,(x0,y0,x1,y1) in enumerate(rectangles(spec)):
        model=tag(world,'model',name=f'blocked_{i:03}');tag(model,'static','true');tag(model,'pose',f'{(x0+x1)/2} {(y0+y1)/2} .65 0 0 0')
        link=tag(model,'link',name='body')
        for kind in ('collision','visual'):
            item=tag(link,kind,name='body');tag(tag(tag(item,'geometry'),'box'),'size',f'{x1-x0} {y1-y0} 1.3')
            if kind=='visual':tag(tag(item,'material'),'diffuse','.38 .41 .37 1')
    import importlib.util, yaml
    loader=importlib.util.spec_from_file_location('lab_vehicle',source/'scripts/generate_vehicle.py')
    generator=importlib.util.module_from_spec(loader);loader.loader.exec_module(generator)
    config=resolved_config()['/**']['ros__parameters']
    metadata=generator.generate_vehicle(directory/'vehicle',source/'config/sensors.yaml',with_sensors=False,vehicle_config=config)
    vehicle=ET.parse(directory/'vehicle/models/nomad_vehicle/model.sdf').find('model')
    save(directory/'vehicle_profile.json',metadata)
    for parent in vehicle.iter():
        for child in list(parent):
            if child.tag=='sensor':parent.remove(child)
    x,y,yaw=spec['start'];tag(vehicle,'pose',f'{x} {y} {elevation(spec,x,y)+.03} 0 0 {yaw}')
    # Resolve any model-local meshes; vehicle mass/joints/controller remain original.
    for uri in vehicle.iter('uri'):
        if uri.text and '://' not in uri.text:uri.text=str((source/'models/nomad_vehicle'/uri.text).resolve())
    world.append(vehicle)
    path=directory/'world.sdf';tree.write(path,encoding='utf-8',xml_declaration=True)
    save(directory/'scenario.json',spec)
    save(directory/'provenance.json',dict(source='scenario_lab/authoring.json; original HTML XY grid',physical_extension=spec.get('terrain', 'legacy: scenario 3 smooth 0.30m mound; synthetic HTML cost independent of height'),obstacles='exact blocked cell union; 1.3m static walls',input_mode='GT radial LOS; not actual camera perception'))
    return path


def viewer_config(world,target):
    gui=ET.parse(world).find('world/gui')
    allowed={'MinimalScene','GzSceneManager','InteractiveViewControl','CameraTracking','WorldStats'}
    plugins=[ET.tostring(p,encoding='unicode') for p in gui.findall('plugin') if p.get('filename') in allowed]
    Path(target).write_text('<window><width>1100</width><height>760</height><dialog_on_exit>false</dialog_on_exit></window>\n'+'\n'.join(plugins))
