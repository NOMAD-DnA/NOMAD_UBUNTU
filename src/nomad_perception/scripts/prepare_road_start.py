"""Runtime-only road-aligned staging; original map assets stay unchanged."""
from pathlib import Path
import json,math,argparse,sys
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import cKDTree
parser=argparse.ArgumentParser();parser.add_argument('--world',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args();world=args.world.resolve();root=world.parent.parent
layout=json.loads((root/'assets/authoring/layout.json').read_text());road=np.array(layout['paths']['entrance']);arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(road,axis=0),axis=1))]
i=int(np.argmin(abs(arc-4)));j=int(np.argmin(abs(arc-7)));xy=road[i];yaw=math.atan2(road[j,1]-xy[1],road[j,0]-xy[0])
vertices=[]
for line in (root/'assets/geometry/terrain.obj').open():
 if line.startswith('v '):vertices.append(list(map(float,line.split()[1:4])))
v=np.array(vertices);dist,idx=cKDTree(v[:,:2]).query(xy,k=8)
X=np.column_stack([v[idx,0]-xy[0],v[idx,1]-xy[1],np.ones(len(idx))]);z=float(np.linalg.lstsq(X,v[idx,2],rcond=None)[0][2])
tree=ET.parse(world);w=tree.getroot().find('world');inc=next(n for n in w.findall('include') if n.findtext('name')=='nomad_vehicle')
inc.find('pose').text=f'{xy[0]} {xy[1]} {z+.03} 0 0 {yaw}'
for uri in tree.findall('.//uri'):
 value=(uri.text or '').strip()
 if value and '://' not in value and not Path(value).is_absolute():uri.text=str((world.parent/value).resolve())
out=args.output;tree.write(out,encoding='unicode',xml_declaration=True)
report={'world':str(out),'spawn_xy':xy.tolist(),'ground_z':z,'spawn_yaw':yaw,'route_arc_m':float(arc[i]),'source_map_unchanged':True};out.with_suffix('.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),file=sys.stderr);print(out)
