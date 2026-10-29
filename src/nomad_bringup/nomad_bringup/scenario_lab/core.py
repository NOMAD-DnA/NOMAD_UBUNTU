"""ROS-free inputs, geometry, observation and result definitions."""
import json
import math
import os
from pathlib import Path
import numpy as np

PACKAGE = Path(__file__).resolve().parent
ALGORITHM = 'hybrid_astar+rpp'


def workspace():
    return Path(os.environ.get('NOMAD_WS_ROOT', PACKAGE.parents[3])).resolve()


def resolved_config():
    """Current shared mechanics, explicit scenario planning overrides."""
    import yaml
    data=yaml.safe_load((PACKAGE/'config.yaml').read_text())
    vehicle=yaml.safe_load((workspace()/'src/nomad_bringup/config/vehicle.yaml').read_text())['/**']['ros__parameters']
    data['/**']['ros__parameters'].update(vehicle)
    geometry_keys=('drive_layout','steering_axle','wheelbase','reference_offset','track','wheel_radius','joint_limit')
    for node,block in data.items():
        if node!='/**':
            for key in geometry_keys:block['ros__parameters'][key]=vehicle[key]
    return data


def catalog():
    return json.loads((PACKAGE/'authoring.json').read_text())


def scenario(number):
    data = catalog()
    selected = next(s for s in data['scenarios'] if s['id'] == int(number))
    return dict(selected, resolution=data['resolution'], width=data['width'], height=data['height'])


def save(path, value):
    path = Path(path); path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n')
    temp.replace(path)


def read(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


def rectangles(spec):
    """Exact union of blocked cells, merged in XY; no obstacle simplification."""
    a=np.array(spec['grid']).reshape(spec['height'],spec['width'])>=80
    active={}; result=[]; r=spec['resolution']
    for y,row in enumerate(a):
        spans=[]; start=None
        for x,value in enumerate(list(row)+[False]):
            if value and start is None:start=x
            if not value and start is not None:spans.append((start,x));start=None
        for span in list(active):
            if span not in spans:
                y0=active.pop(span);result.append((span[0]*r,y0*r,span[1]*r,y*r))
        for span in spans:active.setdefault(span,y)
    for span,y0 in active.items():result.append((span[0]*r,y0*r,span[1]*r,len(a)*r))
    return result


def elevation(spec,x,y):
    # HTML has only XY costs. Explicit, reproducible 3-D extension for scenario 3.
    # Smooth mound; topology/blocked-cell footprints remain identical to HTML.
    if spec['id']!=3 or not 7<x<17 or not 4<y<12:return 0.
    return .30*math.sin(math.pi*(x-7)/10)**2*math.sin(math.pi*(y-4)/8)**2


def observe(spec, known, pose, radius=7.5):
    """Accumulate radial line-of-sight GT observations; hidden cells stay -1."""
    w,h,r=spec['width'],spec['height'],spec['resolution'];truth=spec['grid']
    cx,cy=math.floor(pose[0]/r),math.floor(pose[1]/r); changed=0
    for y in range(max(0,cy-math.ceil(radius/r)),min(h,cy+math.ceil(radius/r)+1)):
        for x in range(max(0,cx-math.ceil(radius/r)),min(w,cx+math.ceil(radius/r)+1)):
            if math.hypot((x+.5)*r-pose[0],(y+.5)*r-pose[1])>radius:continue
            n=max(abs(x-cx),abs(y-cy))*2;visible=True
            for i in range(1,n):
                xx=math.floor(cx+(x-cx)*i/n+.5); yy=math.floor(cy+(y-cy)*i/n+.5)
                if (xx,yy)!=(x,y) and (not 0<=xx<w or not 0<=yy<h or truth[yy*w+xx]>=80):visible=False;break
            if visible and known[y*w+x]!=truth[y*w+x]:known[y*w+x]=truth[y*w+x];changed+=1
    return changed


def polygon_clearance(pose, boxes, half_length=.52, half_width=.42):
    """Exact 2-D rectangle-envelope to static obstacle rectangles distance.

    Returns zero on overlap, not contact-force evidence. Envelope encloses wheels.
    """
    x,y,yaw=pose;co,si=math.cos(yaw),math.sin(yaw)
    car=np.array([(x+u*co-v*si,y+u*si+v*co) for u,v in
                  [(-half_length,-half_width),(half_length,-half_width),(half_length,half_width),(-half_length,half_width)]])
    best=math.inf
    def point_segment(p,a,b):
        v=b-a;t=np.clip(np.dot(p-a,v)/max(np.dot(v,v),1e-20),0,1)
        return float(np.linalg.norm(p-a-t*v))
    for x0,y0,x1,y1 in boxes:
        lower=math.hypot(max(x0-x,x-x1,0),max(y0-y,y-y1,0))-math.hypot(half_length,half_width)
        if lower>best:continue
        box=np.array([(x0,y0),(x1,y0),(x1,y1),(x0,y1)])
        separate=False
        for axis in [(1,0),(0,1),(co,si),(-si,co)]:
            a=car@axis;b=box@axis
            if a.max()<b.min() or b.max()<a.min():separate=True;break
        if not separate:return 0.
        for a,b in [(car,box),(box,car)]:
            for point in a:
                for i in range(4):best=min(best,point_segment(point,b[i],b[(i+1)%4]))
    return best if math.isfinite(best) else None


def summarize(samples, result, spec, gpp_count, invalid_replans, cpu, started_at):
    values=[x['clearance_m'] for x in samples if x.get('clearance_m') is not None]
    completed=result=='arrived'; overlap=any(x<=0 for x in values)
    expected=(result == {5:'safe_stop',6:'forward_limit'}.get(spec['id'],'arrived')) and not overlap
    return dict(scenario=spec['id'],name=spec['name'],algorithm=ALGORITHM,result=result,
                completed=completed,expected_result_met=expected,started_at=started_at,
                elapsed_sim_s=(samples[-1]['time_s']-samples[0]['time_s'] if len(samples)>1 else None),
                clearance_p05_m=float(np.percentile(values,5)) if values else None,
                clearance_min_m=min(values) if values else None,
                clearance_coverage_pct=100*len(values)/len(samples) if samples else 0,
                envelope_overlap=overlap,gpp_path_publications=gpp_count,
                invalidity_replans=invalid_replans,cpu=cpu,
                metric_definition='p05: 10Hz sim-time samples; XY .52m/.42m rectangular envelope to GT obstacle rectangles. Zero means envelope overlap, not measured contact force.',
                input_mode='Gazebo physics + ground-truth pose and synthetic accumulated LOS map; no B0/VIO')


def strategy_timings(directory):
    """Per-call wall latency, distinct from whole-process CPU core-seconds."""
    import csv
    values={'gpp_strategy':[], 'lpp_strategy':[]}
    for path in (Path(directory)/'timing').glob('*.csv'):
        with path.open() as stream:
            for row in csv.DictReader(stream):
                if row.get('stage') not in values:continue
                try:value=float(row['elapsed_wall_ns'])/1e6
                except (KeyError,ValueError,TypeError):continue
                if math.isfinite(value) and value>=0:values[row['stage']].append(value)
    return {name:dict(samples=len(v),p95_ms=float(np.percentile(v,95)) if v else None)
            for name,v in values.items()}
