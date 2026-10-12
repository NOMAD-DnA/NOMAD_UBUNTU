"""Atomic semantic snapshots with cost/class CSV, PNG and numbered SVG exports."""
from pathlib import Path
from datetime import datetime,timezone
import csv,json,os,tempfile,hashlib,math
from xml.sax.saxutils import escape
import numpy as np
from .palette import NAMES,PALETTE,cost_rgb
METADATA_KEY='__metadata_json__'
EXPORT_FILES=('cost_map.png','class_map.png','numbered_cost_map.svg','numbered_class_map.svg','cells.csv','trajectory.csv')

def _json(value):
    return json.dumps(value,ensure_ascii=False,sort_keys=True,allow_nan=False,
        default=lambda v:v.item() if isinstance(v,np.generic) else str(v) if isinstance(v,Path) else (_ for _ in ()).throw(TypeError(type(v))))

def _atomic_write(path,writer):
    fd,tmp=tempfile.mkstemp(prefix='.'+path.name+'.',dir=path.parent)
    try:
        with os.fdopen(fd,'wb') as f:writer(f);f.flush();os.fsync(f.fileno())
        os.replace(tmp,path)
        fd=os.open(path.parent,os.O_RDONLY|os.O_DIRECTORY)
        try:os.fsync(fd)
        finally:os.close(fd)
    finally:
        Path(tmp).unlink(missing_ok=True)

def _validate(a,m):
    m=json.loads(_json(m));a={k:np.asarray(v) for k,v in a.items()}
    if any(v.dtype.hasobject or '/' in k or '\\' in k or '\x00' in k or k==METADATA_KEY for k,v in a.items()):raise ValueError('Invalid snapshot arrays')
    c=a['cost'];cls=a['representative_class']
    if c.dtype!=np.int8 or c.ndim!=2 or not all(c.shape) or np.any((c<-1)|(c>100)):raise ValueError('Invalid cost map')
    if cls.shape!=c.shape or cls.dtype!=np.int16 or np.any((cls<-1)|(cls>=64)):raise ValueError('Invalid original class map')
    for name in ['observed','semantic_raw','semantic_adjusted','q_s','semantic_stamp_s','representative_stamp_s','retained_points','semantic_support_points','representative_support_points','semantic_reused','representative_reused']:
        if a[name].shape!=c.shape:raise ValueError('Grid shapes differ')
    if not np.array_equal(a['observed'].astype(bool),c>=0):raise ValueError('Observed mask and costs differ')
    if np.any((cls>=0)&~a['observed'].astype(bool)):raise ValueError('Class outside observed map')
    for name in ['semantic_raw','semantic_adjusted']:
        v=a[name]
        if v.dtype!=np.float32 or np.isinf(v).any() or np.any((v[np.isfinite(v)]<5)|(v[np.isfinite(v)]>100)):raise ValueError('Invalid semantic floats')
    if not np.isfinite(a['q_s']).all() or np.any((a['q_s']<0)|(a['q_s']>1)):raise ValueError('Invalid support confidence')
    # Additive v2 diagnostics; older archives remain readable without them.
    if 'effective_q_s' in a:
        q=a['effective_q_s']
        if q.shape!=c.shape or not np.isfinite(q).all() or np.any((q<0)|(q>1)) or np.any(q<a['q_s']):raise ValueError('Invalid effective support confidence')
    for name in ['confidence_neighbor_corrected','confidence_neighbor_support_cells']:
        if name in a and a[name].shape!=c.shape:raise ValueError('Grid shapes differ')
    t=a['trajectory']
    if t.dtype!=np.float64 or t.ndim!=2 or t.shape[1]!=8 or not np.isfinite(t).all():raise ValueError('Invalid trajectory')
    if len(t) and np.any(np.linalg.norm(t[:,4:8],axis=1)<1e-12):raise ValueError('Invalid pose quaternion')
    for name in ['origin_x','origin_y','resolution_m','stamp_s']:
        if not math.isfinite(m[name]):raise ValueError('Invalid map coordinates')
    if m['resolution_m']<=0 or not isinstance(m['version'],int) or m['version']<0:raise ValueError('Invalid metadata')
    if m.get('schema')!='nomad-semantic-mapping-v1' or m.get('frame_id')!='odom':raise ValueError('Incompatible snapshot')
    m.update(width=c.shape[1],height=c.shape[0],unknown_cost=-1,missing_class=-1,class_vocabulary='GOOSE64_original')
    return a,m

def _exports(directory,a,m):
    res=m['resolution_m'];ox,oy=m['origin_x'],m['origin_y'];ys,xs=np.nonzero(a['observed'])
    with (directory/'cells.csv').open('w',newline='') as f:
        w=csv.writer(f);names=['cost','semantic_raw','semantic_adjusted','q_s','retained_points','semantic_support_points','representative_support_points','semantic_stamp_s','representative_stamp_s','semantic_reused','representative_reused']
        names += [k for k in ['effective_q_s','confidence_neighbor_corrected','confidence_neighbor_support_cells'] if k in a]
        w.writerow(['row','column','x_center_m','y_center_m','representative_class_id','representative_class_name',*names])
        for y,x in zip(ys,xs):
            cls=int(a['representative_class'][y,x]);w.writerow([y,x,ox+(x+.5)*res,oy+(y+.5)*res,cls,NAMES.get(cls,'N/A'),*[a[k][y,x].item() for k in names]])
    with (directory/'trajectory.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(['stamp_s','x_m','y_m','z_m','qx','qy','qz','qw']);w.writerows(a['trajectory'])
    # Each numeric SVG records stored values, not a recomputation from the representative class.
    h,w=a['cost'].shape;scale=10
    for stem,class_mode in [('numbered_cost_map',False),('numbered_class_map',True)]:
        with (directory/(stem+'.svg')).open('w') as f:
            f.write(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {w*scale} {h*scale}"><rect width="100%" height="100%" fill="#c3c8cf"/>')
            for y,x in zip(ys,xs):
                cls=int(a['representative_class'][y,x]);cost=int(a['cost'][y,x]);rgb=PALETTE[cls] if class_mode and cls>=0 else np.array([.6,.6,.6]) if class_mode else cost_rgb(cost)
                color='#'+''.join(f'{round(float(v)*255):02x}' for v in rgb);text=str(cls) if class_mode and cls>=0 else 'N/A' if class_mode else str(cost)
                title=NAMES.get(cls,'N/A') if class_mode else 'semantic cost'
                f.write(f'<g><title>{escape(title)}</title><rect x="{x*scale}" y="{(h-1-y)*scale}" width="10" height="10" fill="{color}"/><text x="{(x+.5)*scale}" y="{(h-y-.5)*scale}" text-anchor="middle" dominant-baseline="middle" font-size="3" fill="{ "white" if not class_mode and cost>=90 else "black" }">{text}</text></g>')
            f.write('</svg>')
    os.environ.setdefault('MPLCONFIGDIR',str(Path(tempfile.gettempdir())/'nomad_semantic_mpl'))
    import matplotlib;matplotlib.use('Agg',force=True)
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap,LinearSegmentedColormap,BoundaryNorm
    from matplotlib.patches import Patch
    costmap=LinearSegmentedColormap.from_list('semantic_cost',['white','#56c4b8','#ffe45e','#e84835','black']);costmap.set_bad('#c3c8cf')
    classmap=ListedColormap(PALETTE);classmap.set_bad('#c3c8cf')
    extent=[ox,ox+w*res,oy,oy+h*res]
    for filename,class_mode in [('cost_map.png',False),('class_map.png',True)]:
        fig,ax=plt.subplots(figsize=(10,8),constrained_layout=True)
        try:
            if class_mode:
                image=ax.imshow(np.ma.masked_less(a['representative_class'],0),origin='lower',extent=extent,cmap=classmap,norm=BoundaryNorm(np.arange(-.5,64.5),64),interpolation='nearest')
                present=np.unique(a['representative_class'][a['representative_class']>=0]);handles=[Patch(facecolor=PALETTE[i],label=f'{i}: {NAMES[i]}') for i in present]
                handles.append(Patch(facecolor='#c3c8cf',label='Unknown / insufficient class support'))
            else:
                image=ax.imshow(np.ma.masked_equal(a['cost'],-1),origin='lower',extent=extent,cmap=costmap,vmin=0,vmax=100,interpolation='nearest')
                fig.colorbar(image,ax=ax,shrink=.8).set_label('Support-adjusted semantic cost');handles=[]
            t=a['trajectory']
            if len(t):
                line,=ax.plot(t[:,1],t[:,2],color='#1265df',lw=1.2,label='Actual trajectory');handles.append(line)
            if handles:ax.legend(handles=handles,fontsize=8,loc='upper right')
            ax.set(title=('Representative original GOOSE class' if class_mode else 'Accumulated semantic cost')+'\n'+m.get('map_id',''),xlabel='odom x (m)',ylabel='odom y (m)');ax.set_aspect('equal')
            fig.savefig(directory/filename,dpi=150)
        finally:plt.close(fig)

def write_snapshot(session_dir,arrays,metadata,final=False):
    a,m=_validate(arrays,metadata);p=Path(session_dir).resolve();p.mkdir(parents=True,exist_ok=True)
    dest=p/('map.npz' if final else 'checkpoint.npz');m.update(status='closed' if final else 'running',snapshot_file=dest.name,written_at=datetime.now(timezone.utc).isoformat())
    if final:
        with tempfile.TemporaryDirectory(prefix='.export-',dir=p) as temp:
            q=Path(temp);_exports(q,a,m)
            for name in EXPORT_FILES:os.replace(q/name,p/name)
    _atomic_write(dest,lambda f:np.savez_compressed(f,**a,**{METADATA_KEY:np.array(_json(m))}))
    manifest={**m,'snapshot_sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'exports':list(EXPORT_FILES) if final else [],'known_cost_cells':int(np.count_nonzero(a['observed']))}
    _atomic_write(p/'manifest.json',lambda f:f.write((_json(manifest)+'\n').encode()))
    return dest

def load_snapshot(path):
    with np.load(path,allow_pickle=False) as f:
        m=json.loads(str(f[METADATA_KEY].item()));a={k:f[k].copy() for k in f.files if k!=METADATA_KEY}
    return _validate(a,m)

def resolve_snapshot(path):
    p=Path(path).expanduser().resolve()
    if not p.is_dir():load_snapshot(p);return p
    candidates=[]
    for name in ['map.npz','checkpoint.npz']:
        q=p/name
        try:
            _,m=load_snapshot(q);candidates.append((m['version'],q.stat().st_mtime_ns,q))
        except (OSError,ValueError,KeyError,EOFError):pass
    if not candidates:raise ValueError('No valid semantic snapshot')
    return max(candidates,key=lambda a:a[:2])[2]
