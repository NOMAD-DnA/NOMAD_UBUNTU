"""Experimental ZED label transfer to LiDAR/OAK measurements.

Based on nomad_zed_semantic_20261027 semantic_core/semantic_sync. No simulator
mount or GT labels. All stamps must share the VIO clock; output is diagnostic.
"""
from collections import Counter, deque
from pathlib import Path
import json, math, time
import cv2
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from .map_core import rigid
from .depth_filter import frame_voxels
from .capture_timing import trace as capture_trace, capture_range, stage
from .observation_support import observe_camera


def load_config(path):
    path = Path(path)
    c = json.loads(path.read_text())
    if c.get('confirmed_for_recording') is not True or c.get('image_alignment_confirmed') is not True or c.get('common_clock_confirmed') is not True:
        raise ValueError('Set recording, image alignment and common clock confirmations after real verification')
    for name in ('zed_optical_frame', 'calibration_provenance'):
        if not isinstance(c.get(name), str) or not c[name].strip(): raise ValueError('Missing '+name)
    if type(c.get('save_evidence')) is not bool: raise ValueError('save_evidence must be true or false')
    if c.get('depth_definition') != 'optical_z_m': raise ValueError('Depth must be optical Z in meters')
    c['T_zed_optical_from_lidar'] = rigid(c.get('T_zed_optical_from_lidar'))
    for key in ('expected_zed_camera_info','expected_oak_camera_info'):
        model=c.get(key)
        if model is None:continue
        if not isinstance(model,dict) or any(type(model.get(k)) is not int or model[k]<=0 for k in ('width','height')):
            raise ValueError('Invalid '+key+' dimensions')
        try:
            K=np.asarray(model['K'],dtype=float); D=np.asarray(model['D'],dtype=float)
            tol=float(model['pixel_tolerance'])
        except (KeyError,ValueError,TypeError) as e:raise ValueError('Invalid '+key+' parameters') from e
        if K.shape!=(9,) or D.ndim!=1 or not len(D) or not np.isfinite(K).all() or not np.isfinite(D).all() or K[0]<=0 or K[4]<=0 or not math.isfinite(tol) or tol<=0:
            raise ValueError('Invalid '+key+' K, D or pixel tolerance')
        if key=='expected_oak_camera_info':
            if type(model.get('socket')) is not int or not isinstance(model.get('distortion_tolerance'),(float,int)) or not math.isfinite(model['distortion_tolerance']) or model['distortion_tolerance']<=0:
                raise ValueError('Invalid expected_oak_camera_info socket or distortion tolerance')
        elif not isinstance(model.get('distortion_model'),str) or not model['distortion_model']:
            raise ValueError('Invalid expected_zed_camera_info distortion model')
    for key in ('labels','depth','camera_info'):
        if not isinstance(c.get('topics',{}).get(key),str) or not c['topics'][key].startswith('/'):
            raise ValueError('Missing absolute ROS topic '+key)
    for key in ('max_point_label_dt_ms','max_label_depth_dt_ms','max_pose_gap_ms','frame_wait_wall_s',
                'visibility_absolute_m','dynamic_retention_s','grid_resolution_m'):
        if not isinstance(c.get(key),(float,int)) or isinstance(c[key],bool) or not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError('Positive measured/configured value required: '+key)
    if not isinstance(c.get('visibility_relative'),(int,float)) or not math.isfinite(c['visibility_relative']) or c['visibility_relative']<0:
        raise ValueError('Invalid visibility_relative')
    if not isinstance(c.get('boundary_margin_px'),int) or not 0 <= c['boundary_margin_px'] <= 20: raise ValueError('Invalid boundary_margin_px')
    classes = {}
    for k,v in c.get('classes',{}).items():
        i=int(k)
        if not 0<=i<65535 or str(i)!=str(k) or not isinstance(v,dict) or not isinstance(v.get('name'),str) or v.get('kind') not in ('ground','obstacle','dynamic','other'):
            raise ValueError('Invalid class definition: '+str(k))
        cost=v.get('cost')
        if cost is not None and (type(cost) is not int or not 0<=cost<=100): raise ValueError('cost must be integer 0..100 or null')
        classes[i]=v
    grounds=c.get('ground_class_ids',[])
    if not grounds or any(type(i) is not int or i not in classes or classes[i]['kind']!='ground' for i in grounds):
        raise ValueError('Define nonempty ground_class_ids with matching ground classes')
    c['class_table']=classes
    c['config_path']=str(path.resolve())
    return c


def validate_oak_camera_metadata(metadata, expected):
    """Check the live CAM_B EEPROM geometry against the calibration image model."""
    factory = metadata.get('factory_calibration', {})
    entries = factory.get('cameraData', [])
    matches = [entry[1] for entry in entries if isinstance(entry, (list, tuple))
               and len(entry) == 2 and entry[0] == expected['socket']]
    if len(matches) != 1:
        raise ValueError('CAM_B EEPROM cameraData missing or ambiguous')
    camera = matches[0]
    if (camera.get('width'), camera.get('height')) != (expected['width'], expected['height']):
        raise ValueError('CAM_B EEPROM resolution differs from calibration')
    matrix = np.asarray(camera.get('intrinsicMatrix'), dtype=float)
    distortion = np.asarray(camera.get('distortionCoeff'), dtype=float)
    ref_matrix = np.asarray(expected['K'], dtype=float).reshape(3, 3)
    ref_distortion = np.asarray(expected['D'], dtype=float)
    if matrix.shape != (3, 3) or distortion.shape != ref_distortion.shape:
        raise ValueError('CAM_B EEPROM K/D shape differs from calibration')
    if not np.allclose(matrix, ref_matrix, rtol=0, atol=expected['pixel_tolerance']) or not np.allclose(distortion, ref_distortion, rtol=0, atol=expected['distortion_tolerance']):
        raise ValueError('CAM_B EEPROM K/D differs from selected LiDAR-OAK calibration')



def pose_at(rows, stamp, max_gap_ns):
    if not rows: raise ValueError('no_vio_pose')
    stamps=np.asarray([r[0] for r in rows], dtype=np.int64)
    i=int(np.searchsorted(stamps, stamp))
    if i<len(rows) and stamps[i]==stamp:
        r=rows[i];return Rotation.from_quat(r[4:8]).as_matrix(),np.asarray(r[1:4],float)
    if i==0 or i==len(rows): raise ValueError('pose_outside_history')
    left,right=rows[i-1],rows[i]; gap=int(stamps[i]-stamps[i-1])
    if gap<=0 or gap>max_gap_ns: raise ValueError('pose_gap')
    t=(int(stamp)-int(stamps[i-1]))/gap
    rot=Slerp([0.,1.],Rotation.from_quat([left[4:8],right[4:8]]))([t]).as_matrix()[0]
    pos=(1-t)*np.asarray(left[1:4])+t*np.asarray(right[1:4])
    return rot,pos


def camera_pose(rows,stamp,T_imu_zed,max_gap_ns):
    r,p=pose_at(rows,stamp,max_gap_ns)
    return r@T_imu_zed[:3,:3],p+r@T_imu_zed[:3,3]


def warp_depth(depth,K,depth_pose,label_pose):
    rd,td=depth_pose;rl,tl=label_pose
    if np.allclose(rd,rl,atol=1e-9) and np.allclose(td,tl,atol=1e-9): return depth.copy()
    y,x=np.nonzero(np.isfinite(depth)&(depth>0))
    out=np.full(depth.shape,np.inf,np.float32)
    if not len(x): return np.full(depth.shape,np.nan,np.float32)
    z=depth[y,x]; xyz=np.column_stack(((x-K[0,2])*z/K[0,0],(y-K[1,2])*z/K[1,1],z))
    world=xyz@rd.T+td; target=(world-tl)@rl
    valid=np.isfinite(target).all(axis=1)&(target[:,2]>0);target=target[valid]
    u=np.rint(K[0,0]*target[:,0]/target[:,2]+K[0,2]).astype(int)
    v=np.rint(K[1,1]*target[:,1]/target[:,2]+K[1,2]).astype(int)
    good=(u>=0)&(u<depth.shape[1])&(v>=0)&(v<depth.shape[0])
    np.minimum.at(out,(v[good],u[good]),target[good,2]);out[~np.isfinite(out)]=np.nan
    return out


def transfer(points, labels, depth, K, camera, c):
    """Return int32 class IDs; -1 is unknown, including unsupported IDs."""
    labels=np.asarray(labels);depth=np.asarray(depth)
    if labels.ndim!=2 or labels.shape!=depth.shape: raise ValueError('label_depth_shape_mismatch')
    margin=c['boundary_margin_px']
    if margin:
        kernel=np.ones((2*margin+1,2*margin+1),np.uint8)
        low=cv2.erode(labels,kernel);high=cv2.dilate(labels,kernel);safe=low==high
        safe[:margin]=False;safe[-margin:]=False;safe[:,:margin]=False;safe[:,-margin:]=False
    else:safe=np.ones(labels.shape,bool)
    R,p=camera; q=(np.asarray(points)-p)@R; z=q[:,2]
    valid=np.isfinite(q).all(axis=1)&(z>0)
    u=np.zeros(len(z),int);v=np.zeros(len(z),int)
    u[valid]=np.rint(K[0,0]*q[valid,0]/z[valid]+K[0,2]).astype(int)
    v[valid]=np.rint(K[1,1]*q[valid,1]/z[valid]+K[1,2]).astype(int)
    inside=valid&(u>=0)&(u<labels.shape[1])&(v>=0)&(v<labels.shape[0])
    idx=np.flatnonzero(inside); d=depth[v[idx],u[idx]]
    visible=np.isfinite(d)&(d>0)&(np.abs(d-z[idx])<=c['visibility_absolute_m']+c['visibility_relative']*z[idx])
    good=idx[visible];good=good[safe[v[good],u[good]]]
    result=np.full(len(z),-1,np.int32)
    ids=labels[v[good],u[good]].astype(np.int32)
    supported=np.isin(ids,list(c['class_table']))
    result[good[supported]]=ids[supported]
    return result,dict(total=len(points),outside_fov=int((~inside).sum()),depth_unmatched=int((~visible).sum()),
        boundary_or_unknown=int(len(idx)-np.count_nonzero(~visible)-np.count_nonzero(result>=0)),labelled=int(np.count_nonzero(result>=0)),
        ground=int(np.count_nonzero(np.isin(result,c['ground_class_ids']))))


def image_array(msg,kind):
    enc=msg.encoding
    if kind=='labels': formats={'mono8':('u1',1),'8UC1':('u1',1),'mono16':('u2',1),'16UC1':('u2',1)}
    else:formats={'16UC1':('u2',.001),'mono16':('u2',.001),'32FC1':('f4',1.)}
    if enc not in formats: raise ValueError('unsupported_'+kind+'_encoding:'+enc)
    dtype,scale=formats[enc];dt=np.dtype(dtype).newbyteorder('>' if msg.is_bigendian else '<')
    if msg.step<msg.width*dt.itemsize or len(msg.data)<msg.step*msg.height:raise ValueError('bad_image_stride')
    a=np.ndarray((msg.height,msg.width),dt,buffer=msg.data,strides=(msg.step,dt.itemsize)).copy()
    if kind=='depth':
        a=a.astype(np.float32)*scale;a[(a<=0)|~np.isfinite(a)]=np.nan
    return a


def stamp_ns(m):return int(m.header.stamp.sec)*1000000000+int(m.header.stamp.nanosec)


class SemanticGrid:
    """Sparse diagnostic semantic costs. Expired dynamic evidence becomes unknown."""
    def __init__(self,c):self.c=c;self.cells={};self.frame='world'
    def add(self,points,labels,stamp,source,*,point_stamps=None,camera_stamp_ns=None):
        if not len(points):return
        xy=np.floor(points[:,:2]/self.c['grid_resolution_m']).astype(np.int64)
        stamps=np.full(len(points),stamp,dtype=np.int64) if point_stamps is None else np.asarray(point_stamps)
        if stamps.shape!=(len(points),) or stamps.dtype.kind not in 'iu' or np.any(stamps<=0):
            raise ValueError('Positive per-point semantic observation stamps required')
        groups,inverse=np.unique(np.column_stack((xy,labels)),axis=0,return_inverse=True)
        last=np.zeros(len(groups),dtype=np.int64);np.maximum.at(last,inverse,stamps)
        for (x,y,k),observed in zip(groups,last):
            cell=self.cells.setdefault((int(x),int(y)),{'geometry_last_ns':0,'camera_observed_ns':0,'classes':{}})
            cell['geometry_last_ns']=max(cell['geometry_last_ns'],int(observed))
            if k<0:continue
            if camera_stamp_ns is not None:
                observe_camera(cell,camera_stamp_ns)
            evidence=cell['classes'].setdefault(int(k),{'last_ns':0,'votes':[0,0]})
            evidence['last_ns']=max(evidence['last_ns'],int(observed));evidence['votes'][source]+=1
    def snapshot(self,now_ns):
        rows=[]
        for (x,y),cell in sorted(self.cells.items()):
            active=[];expired_dynamic=False
            for k,ev in cell['classes'].items():
                spec=self.c['class_table'][k]
                if spec['kind']=='dynamic' and now_ns-ev['last_ns']>self.c['dynamic_retention_s']*1e9:
                    expired_dynamic=True;continue
                active.append((k,ev,spec))
            priority={'ground':0,'other':1,'obstacle':2,'dynamic':3}
            active.sort(key=lambda a:(priority[a[2]['kind']],a[1]['last_ns']),reverse=True)
            # Expiration alone never establishes free space. A later non-dynamic
            # observation can supersede it only after the last dynamic observation.
            last_dynamic=max((ev['last_ns'] for k,ev in cell['classes'].items() if self.c['class_table'][k]['kind']=='dynamic'),default=0)
            if expired_dynamic and not any(ev['last_ns']>last_dynamic for k,ev,spec in active): active=[]
            chosen=active[0] if active else None
            cost=chosen[2].get('cost') if chosen else None
            rows.append({'ix':x,'iy':y,'class_id':chosen[0] if chosen else None,'class_name':chosen[2]['name'] if chosen else None,
                         'cost':cost,'last_geometry_ns':cell['geometry_last_ns'], 'class_evidence':cell['classes'],
                         'last_semantic_ns':chosen[1]['last_ns'] if chosen else 0,
                         'camera_observed_ns':cell.get('camera_observed_ns',0),
                         'camera_capture_ns':list(cell.get('camera_capture_ns',[]))})
        return {'resolution_m':self.c['grid_resolution_m'],'frame':self.frame,'diagnostic_only':True,
                'cost_null_means_unknown':True,'snapshot_stamp_ns':now_ns,'cells':rows}


class SemanticBridge:
    def __init__(self,node,c,folder,camera_settings):
        from sensor_msgs.msg import Image,CameraInfo
        from rclpy.qos import qos_profile_sensor_data
        self.c=c;self.settings=camera_settings
        self.labels=deque(maxlen=96);self.depths=deque(maxlen=96);self.pending=[deque(),deque()]
        self.receipts={'label':{},'depth':{}};self.last_timing=None
        self.info=None;self.info_signature=None;self.grid=SemanticGrid(c);self.counts=Counter();self.last_source=[-1,-1];self.last_stamp=0
        self.subs=[node.create_subscription(Image,c['topics']['labels'],self.label,qos_profile_sensor_data),
                   node.create_subscription(Image,c['topics']['depth'],self.depth,qos_profile_sensor_data),
                   node.create_subscription(CameraInfo,c['topics']['camera_info'],self.camera_info,qos_profile_sensor_data)]
        self.evidence_pairs=set();self.evidence_index=0
    def label(self,m):self._receive(m,'label')
    def depth(self,m):self._receive(m,'depth')
    def _receive(self,m,kind):
        try:
            if m.header.frame_id!=self.c['zed_optical_frame']:raise ValueError('unexpected_frame')
            t=stamp_ns(m)
            if t<=0:raise ValueError('zero_timestamp')
            a=image_array(m,'labels' if kind=='label' else 'depth')
            (self.labels if kind=='label' else self.depths).append((t,a));self.counts[kind+'_frames']+=1
            self.receipts[kind][t]={'ros_ns':time.time_ns(),'monotonic_ns':time.monotonic_ns()}
            while len(self.receipts[kind])>96:self.receipts[kind].pop(next(iter(self.receipts[kind])))
        except (ValueError,TypeError) as e:self.counts[kind+'_invalid']+=1;self.counts['last_error:'+str(e)]+=1
    def camera_info(self,m):
        try:
            if m.header.frame_id!=self.c['zed_optical_frame']:raise ValueError('bad_camera_info_frame')
            if m.width<=0 or m.height<=0 or len(m.p)!=12:raise ValueError('invalid_camera_info')
            P=np.array(m.p).reshape(3,4);K=P[:3,:3]
            if not np.isfinite(K).all() or K[0,0]<=0 or K[1,1]<=0 or not np.allclose(P[:,3],0,atol=1e-9):raise ValueError('expected_left_rectified_camera_info')
            if not np.allclose(np.array(m.r).reshape(3,3),np.eye(3),atol=1e-5): raise ValueError('camera_info_rectification_rotation_not_identity')
            expected=self.c.get('expected_zed_camera_info')
            if expected is not None:
                expected_k=np.asarray(expected['K'],float).reshape(3,3)
                if (m.width,m.height)!=(expected['width'],expected['height']) or not np.allclose(K,expected_k,rtol=0,atol=expected['pixel_tolerance']):
                    raise ValueError('zed_camera_info_differs_from_calibration')
                if m.distortion_model!=expected['distortion_model'] or np.asarray(m.d).shape!=np.asarray(expected['D']).shape or not np.allclose(m.d,expected['D'],rtol=0,atol=1e-8):
                    raise ValueError('zed_rectified_distortion_differs_from_calibration')
            signature=(m.width,m.height,tuple(K.ravel()))
            if self.info_signature is not None and signature!=self.info_signature:raise ValueError('camera_info_changed')
            self.info_signature=signature;self.info=(m.width,m.height,K)
        except (ValueError,TypeError) as e:self.counts['camera_info_invalid']+=1;self.counts['last_error:'+str(e)]+=1;self.info=None
    def submit(self,points,stamp,source,camera_points,context,point_stamps=None,timing=None):
        if stamp<=self.last_source[source]:raise ValueError('reordered semantic input')
        self.last_source[source]=stamp
        if source==1:_,points,_,_=frame_voxels(camera_points,points,self.settings)
        q=self.pending[source]
        if len(q)>=30:
            old=q.popleft();self._unknown(old,'pending_overflow')
        if point_stamps is None:point_stamps=np.full(len(points),stamp,dtype=np.int64)
        else:
            point_stamps=np.asarray(point_stamps,dtype=np.int64)
            if point_stamps.shape!=(len(points),):raise ValueError('point timestamps do not match points')
        timing=capture_range(timing or capture_trace(stamp,receipt_scope='semantic_enqueue'),point_stamps)
        q.append((np.array(points,copy=True),int(stamp),source,time.monotonic(),context,point_stamps,timing))
    def _unknown(self,frame,reason):
        points,stamp,source,arrived,_,_,_=frame;self.grid.add(points,np.full(len(points),-1),stamp,source)
        self.counts[reason]+=1;self.last_stamp=max(self.last_stamp,stamp)
    def process(self,rows,T_imu_zed,output,flush=False,*,output_timed=None):
        maxgap=round(self.c['max_pose_gap_ms']*1e6)
        for source in (0,1):
            q=self.pending[source]
            while q:
                frame=q[0];points,stamp,src,arrived,context,point_stamps,timing=frame
                reason='missing_camera_info'; found=None
                if self.info is not None:
                    reference=int(np.median(point_stamps)) if len(point_stamps) else stamp
                    candidates=sorted((item for item in self.labels if np.any(np.abs(point_stamps-item[0])<=self.c['max_point_label_dt_ms']*1e6) or (not len(point_stamps) and abs(item[0]-stamp)<=self.c['max_point_label_dt_ms']*1e6)),key=lambda x:abs(x[0]-reference))
                    reason='no_label_in_window'
                    for lt,labels in candidates:
                        depths=sorted((item for item in self.depths if abs(item[0]-lt)<=self.c['max_label_depth_dt_ms']*1e6),key=lambda x:abs(x[0]-lt))
                        reason='no_depth_in_window'
                        for dt,depth in depths:
                            if labels.shape!=depth.shape or labels.shape!=(self.info[1],self.info[0]):reason='image_shape_mismatch';continue
                            try:
                                lp=camera_pose(rows,lt,T_imu_zed,maxgap);dp=camera_pose(rows,dt,T_imu_zed,maxgap)
                            except ValueError as e:reason=str(e);continue
                            found=(lt,labels,dt,depth,lp,dp);break
                        if found:break
                if not found:
                    if not flush and time.monotonic()-arrived<self.c['frame_wait_wall_s']:break
                    q.popleft();self._unknown(frame,reason);continue
                q.popleft();lt,labels,dt,depth,lp,dp=found
                warped=warp_depth(depth,self.info[2],dp,lp)
                ids,counts=transfer(points,labels,warped,self.info[2],lp,self.c)
                outside_time=np.abs(point_stamps-lt)>self.c['max_point_label_dt_ms']*1e6
                ids[outside_time]=-1
                timing=stage(timing,'semantic_ready',ros_ns=time.time_ns())
                timing.update(label_capture_ns=int(lt),depth_capture_ns=int(dt),
                              label_receipt=self.receipts['label'].get(lt),depth_receipt=self.receipts['depth'].get(dt),
                              label_depth_delta_ns=int(lt-dt))
                self.last_timing=timing
                counts['time_excluded']=int(outside_time.sum())
                counts['labelled']=int(np.count_nonzero(ids>=0))
                counts['ground']=int(np.count_nonzero(np.isin(ids,self.c['ground_class_ids'])))
                self.grid.add(points,ids,stamp,src,point_stamps=point_stamps,camera_stamp_ns=lt)
                self.last_stamp=max(self.last_stamp,stamp)
                ground=np.isin(ids,self.c['ground_class_ids'])
                if output_timed is None:output(points[ground],stamp,src,context)
                else:output_timed(points[ground],stamp,src,context,
                                  capture_range(timing,point_stamps[ground]),point_stamps[ground])
                self.counts['processed_frames']+=1;self.counts.update({k+'_'+str(src):v for k,v in counts.items()})
    def close(self,now_ns=None):
        return {'counts':dict(self.counts),'pending':[len(q) for q in self.pending]}
