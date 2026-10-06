#!/usr/bin/env python3
"""Generate five reproducible NOMAD VIO terrain bundles, with shared route geometry."""
import argparse
import copy
import json
import importlib.util
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from PIL import Image


def node(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def save_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def height(x, y, kind):
    x,y=np.asarray(x),np.asarray(y)
    if kind=='reverse':
        bank=np.clip((np.abs(y)-2.2)/5,0,1)
        h=bank*bank*(3-2*bank)*(2.7+.8*np.sin(x/7)+.3*np.cos(y/3))
        # A real mesh road over the northern bank, joining the area behind the wall.
        from scipy.spatial import cKDTree
        bypass=reverse_bypass()
        d,idx=cKDTree(bypass).query(np.column_stack((x.ravel(),y.ravel())))
        s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(bypass,axis=0),axis=1))]
        road_z=3.2*np.sin(np.pi*s/s[-1])**2
        blend=np.clip((d-2.0)/2.0,0,1);blend=blend*blend*(3-2*blend)
        h=h*blend.reshape(x.shape)+road_z[idx].reshape(x.shape)*(1-blend.reshape(x.shape))
    else:
        dist=np.abs(np.sqrt((x/28)**2+(y/20)**2)-1)*20
        bank=np.clip((dist-2.4)/4,0,1);bank=bank*bank*(3-2*bank)
        if kind=='flat':h=bank*(.35+.24*np.sin(x/6)*np.cos(y/7))
        elif kind=='repeated':h=bank*(1.1+.5*np.sin(x/8)*np.cos(y/6))
        elif kind=='sparse':h=bank*(.5+1.6*np.exp(-((x+14)/12)**2-((y-6)/17)**2))
        else:
            h=1.8*np.exp(-((x-16)/9)**2-((y-12)/12)**2)
            h+=.9*np.exp(-((x+7)/11)**2-((y+19)/7)**2)+.22*np.sin(x/4)*np.sin(y/6)
            h+=bank*(.8+.4*np.cos(x/7)*np.sin(y/8)+5*np.exp(-((x-28)/10)**2-((y-23)/8)**2)+2.8*np.exp(-((x-28)/10)**2-((y+23)/8)**2))
    d = np.hypot(x+28, y)
    w = np.clip((d-5)/5, 0, 1)
    return h*w*w*(3-2*w)


def route(reverse):
    if reverse:
        return np.column_stack((np.linspace(-28, 23, 205), np.zeros(205)))
    t = np.linspace(math.pi, 3*math.pi, 401)
    return np.column_stack((28*np.cos(t), 20*np.sin(t)))


def distance_to_route(x, y, points):
    from scipy.spatial import cKDTree
    return cKDTree(points).query(np.column_stack((x.ravel(),y.ravel())),workers=2)[0].reshape(x.shape)


def branch_route():
    vertices=np.array([[-24,-20*math.sqrt(1-(24/28)**2)],[-18,-9],[-12,-13],[-8,-20*math.sqrt(1-(8/28)**2)]])
    return np.concatenate([a+(b-a)*np.linspace(0,1,40)[:,None] for a,b in zip(vertices[:-1],vertices[1:])])


def reverse_bypass():
    from scipy.interpolate import make_interp_spline
    vertices=np.array([[-14,0],[-8,7],[2,13],[16,16],[29,12],[34,5],[34,0]])
    s=np.r_[0,np.cumsum(np.linalg.norm(np.diff(vertices,axis=0),axis=1))]
    return make_interp_spline(s,vertices,k=3)(np.linspace(0,s[-1],301))


def terrain(bundle, cfg, points, package):
    geom = bundle/'assets/geometry'
    rough = cfg['kind']
    x, y = np.meshgrid(np.linspace(-40, 40, 161), np.linspace(-30, 30, 121))
    z = height(x, y, rough)
    # UV texture mapped once across the map, not a repeated road overlay.
    tx, ty = np.meshgrid(np.linspace(-40, 40, 2048), np.linspace(-30, 30, 1536))
    rng = np.random.default_rng(cfg['seed'])
    dist = distance_to_route(tx, ty, points)
    if cfg['kind']=='repeated':
        dist=np.minimum(dist,distance_to_route(tx,ty,branch_route()))
    if cfg['kind']=='reverse':
        dist=np.minimum(dist,distance_to_route(tx,ty,reverse_bypass()))
    mats=bundle/'assets/materials';mats.mkdir(exist_ok=True)
    for f in (package/'assets/materials').iterdir():
        if f.is_file():shutil.copy2(f,mats/f.name)
    def photo(name,tile_m,shift):
        a=np.asarray(Image.open(mats/name).convert('RGB').resize((round(tile_m*25.6),)*2,Image.Resampling.LANCZOS),dtype=np.float32)
        iy=(np.arange(tx.shape[0])+shift)%a.shape[0];ix=(np.arange(tx.shape[1])+shift*3)%a.shape[1]
        return a[iy[:,None],ix[None,:]]
    def smooth_noise(n):
        small=Image.fromarray(rng.integers(0,255,(n,n),dtype=np.uint8))
        return np.asarray(small.resize((2048,1536),Image.Resampling.BICUBIC),dtype=np.float32)/255-.5
    broad,fine=smooth_noise(22),smooth_noise(140)
    dirt=.74*photo('brown_mud_leaves_01_diff_1k.jpg',3.5,31)+.26*photo('brown_mud_leaves_01_diff_1k.jpg',5.1,113)
    litter=photo('leaves_forest_ground_diff_1k.jpg',4.7,79)
    weight=np.clip(.55+broad*.9+fine*.3,.15,.88)
    shoulder=np.clip((dist-cfg['road_width_m']/2)/1.2,0,1)
    weight*=.55+.45*shoulder
    base=litter*weight[...,None]+dirt*(1-weight[...,None])
    base*= (.95+broad*.23+fine*.1)[...,None]
    if cfg['kind']=='rough':base=base*.68+np.array([126,120,102])*.32
    if cfg['kind']=='repeated':base*=.86
    if cfg['kind'] == 'sparse':
        # One open sandy section, not half of the whole world erased.
        low=np.clip((tx-7)/7,0,1)*np.clip((27-np.abs(ty))/4,0,1)
        pale=np.array([136,123,94])+(broad*13+fine*2)[...,None]
        base=base*(1-low[...,None])+pale*low[...,None]
    rgb = np.uint8(np.clip(base, 0, 255))
    Image.fromarray(rgb[::-1]).save(geom/'terrain_texture.png')
    (geom/'terrain.mtl').write_text('newmtl dirt\nKa 0.6 0.6 0.6\nKd 1 1 1\nKs 0 0 0\nillum 1\nmap_Kd terrain_texture.png\n')
    verts = np.column_stack((x.ravel(), y.ravel(), z.ravel()))
    nx = x.shape[1]
    lines = ['mtllib terrain.mtl', 'o terrain', 'usemtl dirt']
    lines += [f'v {a:.7f} {b:.7f} {c:.7f}' for a,b,c in verts]
    lines += [f'vt {(a+40)/80:.7f} {(b+30)/60:.7f}' for a,b,c in verts]
    faces = []
    for j in range(x.shape[0]-1):
        for i in range(nx-1):
            a = j*nx+i
            faces.extend([(a,a+1,a+nx+1),(a,a+nx+1,a+nx)])
    for a,b,c in faces:
        n = np.cross(verts[b]-verts[a], verts[c]-verts[a]); n /= np.linalg.norm(n)
        lines.append('vn '+' '.join(f'{v:.8f}' for v in n))
    for k,f in enumerate(faces,1):
        lines.append('f '+' '.join(f'{v+1}/{v+1}/{k}' for v in f))
    (geom/'terrain.obj').write_text('\n'.join(lines)+'\n')
    gy,gx = np.gradient(z, .5, .5)
    return {'max_mesh_slope_deg': float(np.degrees(np.arctan(np.hypot(gx,gy))).max()), 'height_range_m': [float(z.min()),float(z.max())]}


def mesh_model(world, name, pos, asset, scale, collision=False):
    model=node(world,'model',name=name); node(model,'static','true')
    node(model,'pose',' '.join(str(v) for v in pos))
    link=node(model,'link',name='link')
    v=node(link,'visual',name='visual'); mesh=node(node(v,'geometry'),'mesh')
    node(mesh,'uri',asset); node(mesh,'scale',' '.join(map(str,scale)))
    if collision:
        col=node(link,'collision',name='trunk' if 'tree' in name else 'rock')
        if 'tree' in name:
            node(col,'pose',f'0 0 {1.6*scale[2]} 0 0 0')
            node(node(node(col,'geometry'),'cylinder'),'radius',.2*scale[0])
            node(col.find('geometry/cylinder'),'length',3.2*scale[2])
        else:
            mesh=node(node(col,'geometry'),'mesh'); node(mesh,'uri',asset)
            node(mesh,'scale',' '.join(map(str,scale)))
    return model


def broadleaf_asset(folder):
    """Small original clustered canopy asset; no downloaded third-party meshes."""
    verts=[];faces=[]
    def tri(a,b,c,mat):
        first=len(verts)+1;verts.extend([a,b,c]);faces.append((first,mat))
    for i in range(12):
        a=i*math.pi/6;b=(i+1)*math.pi/6
        lo=[.15*math.cos(a),.15*math.sin(a),0];ln=[.15*math.cos(b),.15*math.sin(b),0]
        hi=[.09*math.cos(a)+.1,.09*math.sin(a),3.1];hn=[.09*math.cos(b)+.1,.09*math.sin(b),3.1]
        tri(lo,ln,hn,'bark');tri(lo,hn,hi,'bark')
    rng=np.random.default_rng(420)
    for j in range(8):
        center=np.array([.85*math.cos(j*2.4),.85*math.sin(j*2.4),3.1+.6*math.sin(j)])
        radii=np.array([.9,.8,1.0])*rng.uniform(.8,1.1,3)
        def p(t,a):return center+radii*np.array([math.sin(t)*math.cos(a),math.sin(t)*math.sin(a),math.cos(t)])*(1+.08*math.sin(3*a+2*t))
        for k in range(6):
            t0=k*math.pi/6;t1=(k+1)*math.pi/6
            for n in range(10):
                a=n*math.pi/5;b=(n+1)*math.pi/5
                if k!=0:tri(p(t0,a),p(t1,a),p(t0,b),'leaf_'+str(j%3))
                if k!=5:tri(p(t0,b),p(t1,a),p(t1,b),'leaf_'+str(j%3))
    lines=['mtllib broadleaf_tree.mtl','o broadleaf_tree']
    lines += ['v '+' '.join(f'{v:.7f}' for v in p) for p in verts]
    for first,mat in faces:
        a,b,c=np.array(verts[first-1:first+2]);n=np.cross(b-a,c-a);n/=max(np.linalg.norm(n),1e-12)
        lines.append('vn '+' '.join(f'{v:.7f}' for v in n))
    previous=None
    for j,(first,mat) in enumerate(faces,1):
        if mat!=previous:lines.append('usemtl '+mat);previous=mat
        lines.append('f '+' '.join(f'{i}//{j}' for i in range(first,first+3)))
    (folder/'broadleaf_tree.obj').write_text('\n'.join(lines)+'\n')
    (folder/'broadleaf_tree.mtl').write_text('\n'.join('newmtl '+name+'\nKa .15 .15 .15\nKd '+' '.join(map(str,c))+'\nKs 0 0 0\nillum 1' for name,c in [('bark',(.28,.17,.09)),('leaf_0',(.19,.32,.09)),('leaf_1',(.28,.42,.12)),('leaf_2',(.35,.43,.16))])+'\n')


def make_map(package, bundle, cfg):
    if cfg['kind']=='reverse':
        cfg={**cfg,'description':'바위 막다른 길에서 후진 후 북측 고도 샛길로 우회하여 뒤편 목표점 (34, 0)에 도달하는 시험 환경.'}
    for f in ['assets/geometry','assets/models','assets/authoring','worlds','models/nomad_vehicle']:
        (bundle/f).mkdir(parents=True,exist_ok=True)
    points=route(cfg['kind']=='reverse')
    stats=terrain(bundle,cfg,points,package)
    for asset in ['pine_tree.obj','pine_tree.mtl','rock.obj','rock.mtl','grass_clump.obj','grass_clump.mtl']:
        shutil.copy2(package/'assets/models'/asset,bundle/'assets/models'/asset)
    broadleaf_asset(bundle/'assets/models')
    for asset in ['model.sdf','model.config']:
        shutil.copy2(package/'models/nomad_vehicle'/asset,bundle/'models/nomad_vehicle'/asset)
    template=ET.parse(package/'worlds/forest.sdf').getroot().find('world')
    sdf=ET.Element('sdf',version='1.9'); world=node(sdf,'world',name='nomad_forest')
    for tag in ['gravity','physics','plugin','scene','light','gui']:
        for e in template.findall(tag):world.append(copy.deepcopy(e))
    cam=world.find("gui/plugin[@filename='MinimalScene']/camera_pose")
    if cam is not None:cam.text='-50 -45 58 0 0.75 0.75'
    mesh_model(world,'terrain',[0,0,0,0,0,0],'../assets/geometry/terrain.obj',[1,1,1],True)
    rough=cfg['kind']; objects=[]; rng=np.random.default_rng(cfg['seed']);far=[];farleaf=[]
    spec=importlib.util.spec_from_file_location('nomad_authoring',package/'scripts/generate_world.py')
    authoring=importlib.util.module_from_spec(spec);spec.loader.exec_module(authoring)
    gx,gy=np.meshgrid(np.linspace(-40,40,161),np.linspace(-30,30,121),indexing='ij')
    heights=height(gx,gy,rough)
    data={'bounds':{'x_min':-40,'x_max':40,'y_min':-30,'y_max':30},'height_resolution_m':.5,'paths':{'route':points.tolist()}}
    if cfg['kind']=='repeated':data['paths']['branch']=branch_route().tolist()
    if cfg['kind']=='reverse':data['paths']['bypass']=reverse_bypass().tolist()
    def add_tree(x,y,scale,yaw=0):
        if cfg['kind']=='reverse' and np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<4.0:
            return
        if cfg['kind']=='repeated' and np.min(np.linalg.norm(branch_route()-[x,y],axis=1))<3:
            return
        z=float(height(x,y,rough)); idx=len(objects)
        leafy=cfg['kind'] in ['flat','sparse'] and rng.random()<.45
        asset='broadleaf_tree.obj' if leafy else 'pine_tree.obj'
        near=np.min(np.linalg.norm(points-[x,y],axis=1))<6
        if cfg['kind']=='reverse':near=near or np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<6
        if near:mesh_model(world,f'tree_{idx:03}',[x,y,z,0,0,yaw],'../assets/models/'+asset,[scale]*3,True)
        else:(farleaf if leafy else far).append({'center':[x,y],'scale':scale,'yaw_deg':math.degrees(yaw)})
        objects.append({'kind':'tree','x':x,'y':y,'z':z,'scale':scale,'species':'broadleaf' if leafy else 'pine'})
    if cfg['kind']=='reverse':
        # 3m clear corridor, foliage stays outside it; physical wall at far end.
        for _ in range(180):
            x=float(rng.uniform(-36,35));y=float(rng.uniform(-24,24))
            if abs(y)<5:continue
            add_tree(x,y,float(rng.uniform(.8,1.5)),float(rng.uniform(0,6.28)))
        # Organic end-face and rocky side banks, never a rectangular building wall.
        for x,y,s in [(26,-2,3.5),(27,0,4),(26,2,3.5)]:
            mesh_model(world,f'end_rock_{len(objects)}',[x,y,0,0,0,float(rng.uniform(0,6.28))],'../assets/models/rock.obj',[s,s,s],True)
            objects.append({'kind':'rock','x':x,'y':y})
        for _ in range(48):
            x=float(rng.uniform(-26,24));y=float(rng.choice([-1,1])*rng.uniform(3.4,5.4));s=float(rng.uniform(.9,2.0))
            if np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<4.0:continue
            mesh_model(world,f'side_rock_{len(objects)}',[x,y,float(height(x,y,rough)),0,0,float(rng.uniform(0,6.28))],'../assets/models/rock.obj',[s,s,s*1.3],True)
            objects.append({'kind':'rock','x':x,'y':y})
    elif cfg['kind']=='repeated':
        # Repeated copses, with the same irregular local pattern instead of rows.
        local=np.array([[-1.3,-.8],[.2,1.4],[1.8,-.2],[-.8,2.1],[2.4,1.7]])
        for t in np.linspace(0,2*math.pi,24,endpoint=False):
            for offset in [-6,6]:
                center=np.array([(28+offset)*math.cos(t),(20+offset)*math.sin(t)])
                for delta in local:
                    x,y=center+delta
                    if abs(x)<38 and abs(y)<28 and np.min(np.linalg.norm(points-[x,y],axis=1))>2.8:
                        add_tree(float(x),float(y),1.3,float(t))
    else:
        for _ in range(5000):
            x=float(rng.uniform(-37,37)); y=float(rng.uniform(-27,27))
            if cfg['kind']=='sparse' and x>8 and abs(y)<25:continue
            if cfg['kind']=='rough' and x>2 and rng.random()<.8:continue
            if np.min(np.linalg.norm(points-[x,y],axis=1))<2.9:continue
            if objects and min(math.hypot(x-o['x'],y-o['y']) for o in objects)<1.8:continue
            add_tree(x,y,float(rng.uniform(.85,1.85)),float(rng.uniform(0,6.28)))
            if len(objects)>= (240 if cfg['kind']=='rough' else 370):break
    # Different rock groups near the loop for reference, absent in repetitive map.
    if cfg['kind']!='repeated':
        for x,y,s in [(-33,0,1.3),(-5,-25,.9),(33,8,1.1),(4,25,1.6)]:
            if cfg['kind']=='sparse' and x>0:continue
            if cfg['kind']=='reverse' and np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<5.0:continue
            mesh_model(world,f'landmark_{len(objects)}',[x,y,float(height(x,y,rough)),0,0,.4],'../assets/models/rock.obj',[s,s,s],True)
            objects.append({'kind':'rock','x':x,'y':y})
    # Irregular clustered boulders on shoulders and exposed hillsides.
    for _ in range(80 if cfg['kind'] in ['rough','sparse'] else 28):
        x=float(rng.uniform(-35,35));y=float(rng.uniform(-26,26))
        if cfg['kind']=='sparse' and x>8 and abs(y)<24:continue
        if np.min(np.linalg.norm(points-[x,y],axis=1))<4.6:continue
        if cfg['kind']=='reverse' and np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<5.0:continue
        if cfg['kind']=='repeated' and np.min(np.linalg.norm(branch_route()-[x,y],axis=1))<4:continue
        scale=float(rng.uniform(1.6,4.3))
        mesh_model(world,f'boulder_{len(objects)}',[x,y,float(height(x,y,rough)),0,0,float(rng.uniform(0,6.28))],'../assets/models/rock.obj',[scale,scale*.8,scale*.7],True)
        objects.append({'kind':'rock','x':x,'y':y,'scale':scale})
    if far:
        authoring.batch_mesh(bundle/'assets/models/pine_tree.obj',bundle/'assets/geometry/forest_clusters.obj',far,heights,data)
        mesh_model(world,'forest_clusters',[0,0,0,0,0,0],'../assets/geometry/forest_clusters.obj',[1,1,1])
    if farleaf:
        authoring.batch_mesh(bundle/'assets/models/broadleaf_tree.obj',bundle/'assets/geometry/broadleaf_clusters.obj',farleaf,heights,data)
        mesh_model(world,'broadleaf_clusters',[0,0,0,0,0,0],'../assets/geometry/broadleaf_clusters.obj',[1,1,1])
    grass=[]
    for _ in range(6500):
        x=float(rng.uniform(-38,38));y=float(rng.uniform(-28,28))
        d=np.min(np.linalg.norm(points-[x,y],axis=1))
        if d<1.3:continue
        if cfg['kind']=='sparse' and x>7 and abs(y)<24:continue
        if cfg['kind']=='repeated' and np.min(np.linalg.norm(branch_route()-[x,y],axis=1))<1.3:continue
        if cfg['kind']=='reverse' and abs(y)<2.1:continue
        if cfg['kind']=='reverse' and np.min(np.linalg.norm(reverse_bypass()-[x,y],axis=1))<1.8:continue
        grass.append({'center':[x,y],'scale_xyz':[float(rng.uniform(.7,1.8)),float(rng.uniform(.7,1.8)),float(rng.uniform(.8,2.8))],'yaw_deg':float(rng.uniform(0,360))})
        if len(grass)>=1800:break
    authoring.batch_mesh(bundle/'assets/models/grass_clump.obj',bundle/'assets/geometry/undergrowth.obj',grass,heights,data)
    mesh_model(world,'undergrowth',[0,0,0,0,0,0],'../assets/geometry/undergrowth.obj',[1,1,1])
    detail_count=authoring.roadside_details(bundle/'assets/geometry',heights,data)
    mesh_model(world,'fallen_wood_and_stones',[0,0,0,0,0,0],'../assets/geometry/roadside_details.obj',[1,1,1])
    if cfg['kind']=='sparse':
        # Keep the intended open observation segment free of detail meshes too.
        # Exclude that arc when generating the shoulder detail batch.
        data['paths']['route']=points[points[:,0]<7].tolist()
        detail_count=authoring.roadside_details(bundle/'assets/geometry',heights,data)
    spawn=[-28,0,.03,0,0,0 if cfg['kind']=='reverse' else -math.pi/2]
    # Inline copy keeps sensor/drive/plugin names identical to the existing vehicle.
    model=copy.deepcopy(ET.parse(package/'models/nomad_vehicle/model.sdf').getroot().find('model'))
    pose=model.find('pose')
    if pose is not None:model.remove(pose)
    node(model,'pose',' '.join(map(str,spawn)));world.append(model)
    save_xml(sdf,bundle/'worlds/forest.sdf')
    bare=copy.deepcopy(sdf);bw=bare.find('world')
    for m in list(bw.findall('model')):
        if m.get('name','').startswith('tree_') or m.get('name') in ['forest_clusters','broadleaf_clusters','undergrowth']:bw.remove(m)
    save_xml(bare,bundle/'worlds/forest_bare.sdf')
    (bundle/'map.json').write_text(json.dumps({'name':cfg['name'],'world':'worlds/forest.sdf','world_bare':'worlds/forest_bare.sdf'},ensure_ascii=False,indent=2)+'\n')
    length=float(np.linalg.norm(np.diff(points,axis=0),axis=1).sum())
    data={**cfg,**stats,'spawn_pose':spawn,'route_length_m':length,'route_is_reference_not_gt':True,'route_xy':points.tolist(),'objects':objects,'return_mode':'reverse with unchanged heading' if cfg['kind']=='reverse' else 'same-direction loop, two laps recommended'}
    data.update(art_revision=2,tree_instances=sum(o['kind']=='tree' for o in objects),grass_instances=len(grass),shoulder_stones=detail_count,route_height_range_m=[float(height(points[:,0],points[:,1],rough).min()),float(height(points[:,0],points[:,1],rough).max())])
    if cfg['kind']=='repeated':data['optional_branch_xy']=branch_route().tolist()
    if cfg['kind']=='reverse':
        bypass=reverse_bypass();bz=height(bypass[:,0],bypass[:,1],rough)
        data.update(art_revision=3,optional_bypass_xy=bypass.tolist(),goal_xy=[34,0],recovery_branch_xy=[-14,0],bypass_height_range_m=[float(bz.min()),float(bz.max())])
        ds=np.linalg.norm(np.diff(bypass,axis=0),axis=1)
        data.update(bypass_length_m=float(ds.sum()),bypass_max_longitudinal_slope_deg=float(np.degrees(np.arctan(np.abs(np.diff(bz))/ds)).max()))
        np.savetxt(bundle/'bypass.csv',np.column_stack((bypass,bz)),delimiter=',',header='x_m,y_m,z_m',comments='')
    (bundle/'assets/authoring/scenario.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n')
    (bundle/'assets/layout.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    np.savetxt(bundle/'route.csv',np.column_stack((points,height(points[:,0],points[:,1],rough))),delimiter=',',header='x_m,y_m,z_m',comments='')
    (bundle/'README.md').write_text(f"# {cfg['name']}\n\n{cfg['description']}\n\n- 기준 경로: {length:.1f}m. `route.csv`는 계획 경로이며 실제 GT는 `/odom`.\n- {'23m 지점까지 전진 후 회전 없이 같은 길로 후진 복귀, 총 약 102m.' if cfg['kind']=='reverse' else '같은 방향으로 두 바퀴 주행. 1바퀴 약 152m.'}\n- 출발: (-28, 0). 도로 폭 {cfg['road_width_m']}m.\n- `map_preview.png`: 계획 경로와 물체 배치. Gazebo 화면 캡처 아님.\n- 기존 차량·센서·구동 설정 유지. 맵 생성·로딩 확인과 알고리즘 주행 성공은 별개.\n")
    if cfg['kind']=='reverse':
        with (bundle/'README.md').open('a') as f:
            f.write('\n- 우회 시험: 막힌 지점 (23, 0) → 후진하여 분기 (-14, 0) → 북측 샛길 → 목표 (34, 0).\n- `bypass.csv`: 높이 약 3.2m의 우회 계획 경로. 자동 경로 선택·후진 제어는 기존 자율주행 모듈에서 수행.\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(9,7))
    if cfg['kind']=='rough':
        x,y=np.meshgrid(np.linspace(-40,40,161),np.linspace(-30,30,121))
        im=ax.contourf(x,y,height(x,y,rough),levels=18,cmap='terrain');fig.colorbar(im,ax=ax,label='Height [m]')
    else:ax.imshow(Image.open(bundle/'assets/geometry/terrain_texture.png'),extent=(-40,40,-30,30),alpha=.9)
    for kind,color in [('tree','#315c32'),('rock','#666666')]:
        p=[o for o in objects if o['kind']==kind]
        if p:ax.scatter([o['x'] for o in p],[o['y'] for o in p],s=22 if kind=='tree' else 45,c=color,label=kind)
    ax.plot(*points.T,color='red',lw=2,label='Reference route')
    if cfg['kind']=='repeated':ax.plot(*branch_route().T,color='#a26e25',ls='--',lw=2,label='Similar branch')
    ax.scatter(-28,0,c='lime',edgecolors='black',s=75,label='Start')
    if cfg['kind']=='reverse':
        ax.plot([26,26],[-4,4],c='black',lw=5,label='Dead end')
        ax.annotate('Forward → / ← Reverse',xy=(0,0),xytext=(-9,5))
        ax.plot(*reverse_bypass().T,color='#f7b32b',lw=2.5,label='Elevated bypass')
        ax.scatter(34,0,c='#f7b32b',marker='*',s=160,edgecolors='black',label='Goal behind blockage')
    ax.set(xlim=(-40,40),ylim=(-30,30),xlabel='X [m]',ylabel='Y [m]',title=cfg['id']+' — planned route')
    ax.set_aspect('equal');ax.legend(loc='upper right');fig.tight_layout();fig.savefig(bundle/'map_preview.png',dpi=150);plt.close(fig)
    print(f"{cfg['id']}: {length:.1f}m, objects={len(objects)}, max terrain slope={stats['max_mesh_slope_deg']:.2f}deg")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--package-dir',type=Path,default=Path(__file__).resolve().parents[1])
    p.add_argument('--output-dir',type=Path)
    p.add_argument('--map',help='Regenerate only the specified map ID')
    args=p.parse_args();package=args.package_dir.resolve();output=args.output_dir or package/'maps'
    scenarios=[
        ('04_vio_flat_loop','혼합림 · 낙엽 숲 순환로','flat','활엽수·침엽수 혼합림, 풀·낙엽·바위 랜드마크. 평탄한 경로의 기본 위치 오차와 재방문 비교.'),
        ('05_vio_repeated_forest','침엽수 군락 · 반복 숲길','repeated','불규칙하지만 비슷한 형태의 침엽수 군락과 내부 분기에서 장소 혼동 비교.'),
        ('06_vio_sparse_clearing','건조 공터 · 숲 경계','sparse','식생·바위가 있는 숲에서 저텍스처 건조 공터로 진입 후 숲으로 복귀.'),
        ('07_vio_undulating_terrain','암석 능선 · 굴곡 야지','rough','노출된 바위·둔덕·횡경사·오르내림이 있는 지형에서 자세 변화 비교.'),
        ('08_vio_reverse_dead_end','바위 계곡 · 후진 복귀','reverse','지면 양쪽 계곡 둔덕과 바위 막다른 구간. 차량 방향을 유지하고 후진 복귀.'),
    ]
    for ident,name,kind,description in scenarios:
        if args.map and args.map!=ident:continue
        bundle=output/ident
        author=bundle/'assets/authoring/scenario.json'
        cfg=json.loads(author.read_text()) if author.exists() else {'id':ident,'name':name,'kind':kind,'description':description,'seed':740,'road_width_m':3 if kind=='reverse' else 4}
        cfg.update(name=name,description=description,art_revision=2)
        make_map(package,bundle,cfg)


if __name__=='__main__':main()
