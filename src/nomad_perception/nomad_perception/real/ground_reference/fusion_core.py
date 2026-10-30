"""Source-balanced measured XYZ fusion. No rendered mesh or semantic input.

0.2 s normalization is tied to the configured 5 Hz laser, not synchronization.
Every input is transformed with its own capture-time pose before arriving here.
"""
from collections import Counter
from dataclasses import asdict, replace
import math
import numpy as np
from scipy.spatial import cKDTree
from .core import VoxelMap, VoxelSnapshot
from .costmap_core import LidarSlopeMap
from ._geometry.traversability_core import CellEstimate, PlaneFit
from ._geometry.quality_mapping_core import spread_quality, QualityMappingCore


def representatives(points, size, dimensions=3):
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite fusion XYZ')
    if not len(points):
        return points, np.empty((0, dimensions), np.int64), np.empty(0, np.int64)
    keys, inverse, counts = np.unique(np.floor(points[:, :dimensions]/size).astype(np.int64),
                                     axis=0, return_inverse=True, return_counts=True)
    means = np.column_stack([np.bincount(inverse, weights=points[:, i])/counts for i in range(3)])
    return means, keys, counts


class BalancedVoxelMap:
    """One temporal representative/source/voxel, then equal source centroids.

    Each source is sampled at most once per 0.2s in the mapper, so a dense frame
    has exactly one vote per voxel. Camera never consumes LiDAR's capacity.
    """
    def __init__(self, voxel_size=.05, max_voxels=2_000_000):
        self.sources = [VoxelMap(voxel_size, max_voxels//2), VoxelMap(voxel_size, max_voxels//2)]
        self.voxel_size = voxel_size
    def add(self, points, source=0):
        reduced, _, _ = representatives(points, self.voxel_size)
        return self.sources[source].add(reduced)
    @property
    def dropped_points(self):
        return sum(m.dropped_points for m in self.sources)
    @property
    def voxel_count(self):
        return len(self.sources[0]._voxels.keys() | self.sources[1]._voxels.keys())
    def snapshot(self):
        source=[m.snapshot() for m in self.sources]
        keys,inverse=np.unique(np.concatenate([a.keys for a in source]),axis=0,return_inverse=True)
        values=np.concatenate([a.points for a in source]);counts=np.concatenate([a.counts for a in source])
        votes=np.bincount(inverse,minlength=len(keys))
        points=np.column_stack([np.bincount(inverse,weights=values[:,i],minlength=len(keys))/votes for i in range(3)])
        observations=np.zeros(len(keys),np.uint64);np.add.at(observations,inverse,counts)
        return VoxelSnapshot(points,keys,observations,self.voxel_size,
                             sum(m.accepted_points for m in self.sources),self.dropped_points)
    def source_arrays(self):
        keys=sorted(self.sources[0]._voxels.keys() | self.sources[1]._voxels.keys())
        return dict(source_mask=np.array([sum((1<<s) for s,m in enumerate(self.sources) if k in m._voxels) for k in keys],np.uint8),
                    source_observation_counts=np.array([[m._voxels[k][1] if k in m._voxels else 0 for m in self.sources] for k in keys],dtype=np.uint64).reshape(-1,2))


def weighted_tls(points, weights, config, quality):
    points=np.asarray(points,dtype=float);weights=np.asarray(weights,dtype=float)
    if len(points)<quality.min_xy_bins or not spread_quality(points,quality)[0]:return None
    weights=weights/weights.sum();center=np.sum(points*weights[:,None],axis=0);centered=points-center
    cov=(centered*weights[:,None]).T@centered
    xy=np.linalg.eigvalsh(cov[:2,:2]);values,vectors=np.linalg.eigh(cov)
    if (xy[1]<=0 or xy[0]<quality.min_minor_std_m**2 or xy[0]/xy[1]<quality.min_xy_variance_ratio
            or values[2]<=0 or values[1]-values[0]<=config.tls_gap_ratio*values[2]):return None
    normal=vectors[:,0]
    if normal[2]<0:normal=-normal
    residual=float(np.sqrt(np.sum(weights*(centered@normal)**2)))
    # No statistical downweighting: a geometrically incompatible surface is unknown.
    if residual>quality.max_refit_rms_m:return None
    slope=math.degrees(math.atan2(np.hypot(*normal[:2]),abs(normal[2])))
    qspace=float(np.clip(xy[0]/config.radius_m**2/config.space_reference_fraction,0,1))
    return PlaneFit(slope,tuple(normal),-float(normal@center),residual,qspace)


class FusedSlopeMap(LidarSlopeMap):
    """Recent source-tagged raw points -> 5cm XY reps -> joint weighted TLS.

    Height discontinuities inside an XY bin are rejected, not flattened into
    an artificial ground slope. Previous valid estimates persist only while
    fresh observations do not contradict them. Raw samples expire after 10s.
    """
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        self.samples={};self.last_source=[-math.inf,-math.inf];self.counts=Counter()
        self.observed=set();self.heights={};self.dirty=set();self.latest=-math.inf
        self.refitter=QualityMappingCore(self.config,self.quality)
    def ingest(self,points,stamp,source=0):
        points=np.asarray(points,dtype=float)
        if source not in (0,1) or points.ndim!=2 or points.shape[1]!=3 or not np.isfinite(points).all() or not math.isfinite(stamp):raise ValueError('Invalid fusion observation')
        if stamp<=self.last_source[source]:raise ValueError('Duplicate/reordered source observation')
        self.last_source[source]=stamp;self.latest=max(self.latest,stamp);self.last_stamp=self.latest
        bound=self.half_cells*self.config.resolution_m
        mask=np.all((points[:,:2]>=-bound)&(points[:,:2]<bound),axis=1)
        self.out_of_bounds+=int((~mask).sum());points=points[mask]
        self.frames+=1;self.points+=len(points);self.counts[f'source_{source}_frames']+=1;self.counts[f'source_{source}_points']+=len(points)
        # Cost retention is independent of the map voxel archive. Preserve
        # raw aligned XYZ, including separate height layers, until explicit caps.
        xyz=points
        cells=np.floor(xyz[:,:2]/self.config.resolution_m).astype(np.int64)
        order=np.lexsort((cells[:,1],cells[:,0]));cells=cells[order];xyz=xyz[order]
        boundaries=np.r_[0,np.flatnonzero(np.any(np.diff(cells,axis=0),axis=1))+1,len(cells)] if len(cells) else [0]
        for start,stop in zip(boundaries[:-1],boundaries[1:]):
            key=tuple(cells[start]);self.observed.add(key);self.dirty.add(key)
            group=self.samples.setdefault(key,{0:[],1:[]})[source]
            group.append((stamp,xyz[start:stop].astype(np.float32)))
            # Reserve equal independent retention limits for each source.
            excess=sum(len(p) for t,p in group)-self.config.per_cell_max_points//2
            while excess>0 and group:
                old_stamp,old_points=group[0]
                remove=min(excess,len(old_points));keep=len(old_points)-remove
                if keep:
                    indices=np.linspace(0,len(old_points)-1,keep,dtype=int)
                    group[0]=(old_stamp,old_points[indices])
                else:group.pop(0)
                excess-=remove;self.counts['capacity_evictions']+=remove
        cutoff=self.latest-self.config.raw_window_s
        for key,groups in list(self.samples.items()):
            for group in groups.values():
                while group and group[0][0]<cutoff:
                    group.pop(0);self.counts['expired_captures']+=1;self.dirty.add(key)
            if not any(groups.values()):del self.samples[key]
        # Global budget is split equally. Evict oldest source captures explicitly.
        for s in (0,1):
            groups=[(g[s][0][0],k,g[s]) for k,g in self.samples.items() if g[s]]
            total=sum(len(p) for _,_,g in groups for t,p in g)
            if total>self.config.global_max_points//2:
                for _,k,g in sorted(groups):
                    while g and total>self.config.global_max_points//2:
                        t,p=g.pop(0);total-=len(p);self.counts['global_cap_evictions']+=len(p);self.dirty.add(k)
    def _prepare(self):
        self.prepared=[]
        for source in (0,1):
            pieces=[p for groups in self.samples.values() for t,p in groups[source]]
            xyz=np.concatenate(pieces).astype(float) if pieces else np.empty((0,3))
            means,keys,counts=representatives(xyz,self.quality.bin_size_m,2)
            low=np.full(len(means),np.inf);high=np.full(len(means),-np.inf)
            if len(xyz):
                _,inv=np.unique(np.floor(xyz[:,:2]/self.quality.bin_size_m).astype(np.int64),axis=0,return_inverse=True)
                np.minimum.at(low,inv,xyz[:,2]);np.maximum.at(high,inv,xyz[:,2])
            self.prepared.append((means,counts,high-low,cKDTree(means[:,:2]) if len(means) else None))
    def _fit(self,key):
        center=(np.array(key)+.5)*self.config.resolution_m
        reps=[];sourceids=[];conflict=False;raw_count=0
        for source,(means,counts,spans,tree) in enumerate(self.prepared):
            indices=tree.query_ball_point(center,self.config.radius_m+1e-9) if tree is not None else []
            reps.extend(means[indices]);sourceids.extend([source]*len(indices));raw_count+=int(counts[indices].sum())
            conflict |= bool(np.any(spans[indices]>self.quality.max_connected_height_jump_m))
        reps=np.array(reps).reshape(-1,3);sourceids=np.array(sourceids)
        if len(reps):
            bins=np.floor(reps[:,:2]/self.quality.bin_size_m).astype(np.int64)
            _,inv=np.unique(bins,axis=0,return_inverse=True)
            low=np.full(inv.max()+1,np.inf);high=np.full(inv.max()+1,-np.inf)
            np.minimum.at(low,inv,reps[:,2]);np.maximum.at(high,inv,reps[:,2])
            conflict |= bool(np.any(high-low>self.quality.max_connected_height_jump_m))
        fitted=None
        if not conflict and raw_count>=self.quality.min_raw_points and len(reps):
            # Each observed XY region has one vote, split 5:5 if both sources
            # observe it. Non-overlap retains its measured spatial support.
            counts=np.bincount(inv);weights=1./counts[inv]
            if len(np.unique(inv))>=self.quality.min_xy_bins:
                fitted=weighted_tls(reps,weights,self.config,self.quality)
        if conflict:
            self.counts['multilayer_or_sensor_conflict']+=1;self.heights.pop(key,None)
        central=self.samples.get(key,{})
        centralpoints=[p for groups in central.values() for t,p in groups]
        if centralpoints and not conflict:
            medians=[float(np.median(np.concatenate([p for t,p in group])[:,2])) for group in central.values() if group]
            self.heights[key]=(float(np.mean(medians)),self.latest)
        if fitted is None and not conflict:
            fitted,_,_,_,_=self.refitter.refit_heights(key,self.heights)
        old=self.estimates.get(key)
        # Fresh evidence contradicting a retained fit invalidates it.
        if fitted is None and old is not None and old.normal is not None and not conflict:
            allpoints=np.concatenate(centralpoints) if centralpoints else np.empty((0,3))
            if not len(allpoints) or np.median(np.abs(allpoints@old.normal+old.plane_offset_m))<=self.quality.max_target_error_m:
                self.estimates[key]=replace(old,slope_reused=True);return
        if fitted is None:
            self.estimates[key]=CellEstimate(height=self.heights.get(key,(None,))[0],total_points=raw_count);return
        q=min(1.,len(np.unique(inv))/self.config.raw_reference_points if len(reps) else 0.,fitted.q_space)
        self.estimates[key]=CellEstimate(height=self.heights.get(key,(None,))[0],slope_valid=True,slope_deg=fitted.slope_deg,
            geometry_cost=100 if fitted.slope_deg>=self.config.slope_limit_deg else 0,q_g=q,normal=fitted.normal,
            plane_offset_m=fitted.offset_m,fit_residual_m=fitted.residual_m,total_points=raw_count)
    def snapshot(self):
        changed=set()
        reach=math.ceil(self.config.radius_m/self.config.resolution_m)
        for x,y in self.dirty:
            changed.update((x+dx,y+dy) for dx in range(-reach,reach+1) for dy in range(-reach,reach+1) if (x+dx,y+dy) in self.observed)
        if changed:self._prepare()
        for key in changed:self._fit(key)
        self.dirty.clear()
        # The base adapter renders estimates and applies the unchanged cost policy.
        stamp=self.last_stamp;self.last_stamp=-math.inf
        try:return super().snapshot()
        finally:self.last_stamp=stamp
    def metadata(self):
        return dict(schema='nomad-lidar-costmap-v1',input_provider='capture-time aligned actual camera depth and LiDAR XYZ',
                    sensor_weights={'lidar':.5,'camera':.5},weight_scope='per shared XY bin; 1 vote per spatial bin, split equally across observed sensors',
                    temporal_period_s=.2,raw_window_s=self.config.raw_window_s,
                    retained_payload='raw capture-time XYZ, independently bounded per source; map 3D voxels never used for slope; 5cm XY representatives formed only for fitting',
                    persistent_estimates=True,semantic_input=False,ground_segmentation=False,world_geometry_input=False,
                    candidate_settings=asdict(self.config),plane_quality=asdict(self.quality),cost_policy=self.policy.metadata(),
                    unknown_cost=-1,received_frames=self.frames,input_points=self.points,out_of_bounds_points=self.out_of_bounds,
                    core_counters=dict(self.counts),last_stamp_s=self.last_stamp if math.isfinite(self.last_stamp) else None)
