"""Measured CAM_B <- LaserScan geometry, forward half scan and per-ray VIO poses."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT=Path(__file__).resolve().parents[1]

def uncalibrated_extrinsics():
    """REP-103 body -> optical axes only, coincident origins, no mounting fit.

    This is an explicit baseline assumption, not measured mounting geometry.
    https://github.com/ros-infrastructure/rep/blob/master/rep-0103.rst
    """
    return np.array([[0.,-1.,0.,0.],[0.,0.,-1.,0.],[1.,0.,0.,0.],[0.,0.,0.,1.]])

def rigid(matrix):
    T=np.asarray(matrix,float)
    if T.shape!=(4,4)or not np.isfinite(T).all()or not np.allclose(T[3],[0,0,0,1]):
        raise ValueError('Invalid homogeneous transform')
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6)or not np.isclose(np.linalg.det(T[:3,:3]),1,atol=1e-6):
        raise ValueError('Invalid rigid rotation')
    return T

def load_extrinsics(path):
    data=json.loads(Path(path).read_text())
    if data.get('source')!='real_sensor' or data.get('camera_frame')!='oak_cam_b_optical_frame' or data.get('lidar_frame')!='g2_laser':
        raise ValueError('Expected measured g2_laser -> raw CAM_B optical calibration')
    if data.get('convention')!='p_camera_optical = R_camera_lidar * p_lidar + t_camera_lidar_m':
        raise ValueError('Unrecognized calibration convention')
    T=np.eye(4);T[:3,:3]=data['R_camera_lidar'];T[:3,3]=data['t_camera_lidar_m']
    return rigid(T),data

def front_mask(points,T_camera_lidar):
    # Ray directions, not translated camera-origin point positions: an exact
    # 180 degree sector about the camera forward axis projected into scan XY.
    points=np.asarray(points,float);R=rigid(T_camera_lidar)[:3,:3]
    if np.linalg.norm(R[2,:2])<1e-6:raise ValueError('Camera forward axis is perpendicular to scan plane')
    direction_z=points@R[2]
    return np.isfinite(points).all(axis=1)&(direction_z>=-1e-10)

def scan_with_times(scan,cloud,T_camera_lidar):
    """Preserve calibration's LaserScan angle bins with SDK acquisition times.

    The driver uses ceil(raw_angle -> LaserScan bin) and the last sample wins.
    Using raw PointCloud XY directly would silently change the angular geometry
    used for the new static calibration. Timestamps remain original SDK offsets
    after the rear sector is discarded; they must never restart at zero.
    """
    if scan['stamp_ns']!=cloud['stamp_ns']:raise ValueError('Scan/cloud stamps differ')
    p=np.asarray(cloud['points'],float);times=np.asarray(cloud['offsets'],float)
    if p.ndim!=2 or p.shape[1]!=3 or times.shape!=(len(p),)or len(p)<2:raise ValueError('Bad point/time buffers')
    if not np.isfinite(p).all()or not np.isfinite(times).all()or times.min()<0 or times.max()>.3 or np.any(np.diff(times)<=0):
        raise ValueError('Invalid SDK acquisition time order')
    ranges=np.asarray([np.nan if r is None else r for r in scan['ranges']],float)
    amin=scan['angle_min'];inc=scan['angle_increment']
    if not inc>0:raise ValueError('Expected increasing scan bins')
    idx=np.ceil((np.arctan2(p[:,1],p[:,0])-amin)/inc).astype(int)
    last={}
    for i,b in enumerate(idx):
        if 0<=b<len(ranges):last[int(b)]=i
    raw_indices=np.array(sorted(last.values()),int)
    bins=idx[raw_indices];rr=ranges[bins];raw_r=np.linalg.norm(p[raw_indices,:2],axis=1)
    good=np.isfinite(rr)&(rr>=scan['range_min'])&(rr<=scan['range_max'])&(np.abs(rr-raw_r)<.003)
    raw_indices=raw_indices[good];bins=bins[good];rr=rr[good]
    angle=amin+bins*inc;local=np.column_stack([rr*np.cos(angle),rr*np.sin(angle),np.zeros(len(rr))])
    keep=front_mask(local,T_camera_lidar)
    intensities=np.asarray(cloud.get('intensities',[]),float)
    selected_intensity=intensities[raw_indices][keep]if intensities.shape==(len(p),)else np.zeros(int(keep.sum()))
    if keep.sum()<5:raise ValueError('Too few forward points with matched acquisition times')
    return {'stamp_ns':scan['stamp_ns'],'points_lidar_m':local[keep],
        'point_time_offsets_s':times[raw_indices][keep],'bin_indices':bins[keep],
        'intensities':selected_intensity,'input_points':len(p),'matched_bins':len(local),
        'rear_discarded':int((~keep).sum()),'unmatched_points':len(p)-len(local),
        'received_monotonic':cloud['received_monotonic']}

def deskew(scan,rows,T_imu_camera,T_camera_lidar,time_offset_s=0.,max_gap_ns=200_000_000):
    if len(rows)<2:raise ValueError('Not enough VIO poses')
    times=np.array([r[0]for r in rows],np.int64)
    if np.any(np.diff(times)<=0):raise ValueError('VIO timestamps are not increasing')
    offsets=np.asarray(scan['point_time_offsets_s'])
    query=np.int64(scan['stamp_ns'])+np.rint((offsets+time_offset_s)*1e9).astype(np.int64)
    if query.min()<times[0]or query.max()>times[-1]:raise ValueError('Ray time outside VIO history')
    right=np.minimum(np.searchsorted(times,query),len(times)-1);left=np.maximum(right-1,0)
    exact=times[right]==query
    if np.any((times[right]-times[left])[~exact]>max_gap_ns):raise ValueError('VIO bracket gap over 0.2 s')
    lo=max(0,int(left.min()));hi=int(right.max())+1;origin=int(times[lo])
    if hi-lo==1:rot=np.repeat(Rotation.from_quat([rows[lo][4:8]]).as_matrix(),len(query),axis=0)
    else:rot=Slerp((times[lo:hi]-origin)/1e9,Rotation.from_quat([r[4:8]for r in rows[lo:hi]]))((query-origin)/1e9).as_matrix()
    fraction=np.divide(query-times[left],times[right]-times[left],out=np.zeros(len(query)),where=times[right]!=times[left]);fraction[exact]=1
    xyz=np.array([r[1:4]for r in rows]);position=xyz[left]*(1-fraction[:,None])+xyz[right]*fraction[:,None]
    T=rigid(T_imu_camera)@rigid(T_camera_lidar)
    imu=np.asarray(scan['points_lidar_m'])@T[:3,:3].T+T[:3,3]
    return np.einsum('nij,nj->ni',rot,imu)+position

class Voxels:
    def __init__(self,size=.03):self.size=size;self.cells={}
    def add(self,points):
        points=np.asarray(points,float)
        if not np.isfinite(points).all():raise ValueError('Nonfinite mapped point')
        keys,inverse=np.unique(np.floor(points/self.size).astype(np.int64),axis=0,return_inverse=True)
        counts=np.bincount(inverse);sums=np.column_stack([np.bincount(inverse,weights=points[:,j])for j in range(3)])
        for key,total,n in zip(map(tuple,keys),sums,counts):
            if key in self.cells:self.cells[key][:3]+=total;self.cells[key][3]+=n
            else:self.cells[key]=np.r_[total,float(n)]
    def arrays(self):
        if not self.cells:return np.empty((0,3)),np.empty(0,np.uint32)
        a=np.array(list(self.cells.values()));return a[:,:3]/a[:,3,None],a[:,3].astype(np.uint32)
