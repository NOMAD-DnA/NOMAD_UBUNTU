"""Persistent observation estimates and one bounded semantic cost output.

Retention eviction changes active-point diagnostics, never observation support.
No plane fit, geometric cost, fusion, collision-height test or LiDAR clearing.
"""
from dataclasses import dataclass
import math
import numpy as np
from nomad_perception.traversability_core import (
    CoreConfig, TraversabilityCore, CLASS_COSTS, ELIGIBLE_MASK)
from .cost_policy import SemanticCostPolicy, NEIGHBORS

@dataclass(frozen=True)
class SemanticEstimate:
    representative_class: int | None
    representative_stamp_s: float | None
    representative_support_points: int
    semantic_raw: float | None
    semantic_adjusted: float | None
    output_cost: int
    q_s: float
    total_points: int
    eligible_points: int
    semantic_support_points: int
    semantic_stamp_s: float | None
    observation_stamp_s: float
    representative_reused: bool
    semantic_reused: bool
    class_histogram: tuple
    effective_q_s: float
    confidence_neighbor_corrected: bool
    confidence_neighbor_support_cells: int
    semantic_support_histogram: tuple

class SemanticCore(TraversabilityCore):
    def __init__(self, config=None, cost_policy=None):
        config=config or CoreConfig()
        if config.geometry_only:
            raise ValueError('Semantic mapping must retain ground and nonground classes')
        super().__init__(config)
        self.cost_policy=cost_policy or SemanticCostPolicy()
        self._representative_support={}
        self._representative_versions={}
        self._semantic_versions={}
        self._semantic_histograms={}
        self._semantic_distributions={}
        self._neighbor_deadlines={}
        self.last_snapshot_changed_keys=frozenset()

    def reset(self):
        super().reset()
        self._representative_support.clear()
        self._representative_versions.clear()
        self._semantic_versions.clear()
        self._semantic_histograms.clear()
        self._semantic_distributions.clear()
        self._neighbor_deadlines.clear()
        self.last_snapshot_changed_keys=frozenset()

    def _mark_changed(self,key):
        self._summary_dirty.add(key)
        self._dirty.add(key)
        x,y=key
        for dx,dy in NEIGHBORS:
            other=(x+dx,y+dy)
            if other in self._cells:self._dirty.add(other)

    def ingest(self,xyz_odom,class_ids,stamp_s):
        changed=super().ingest(xyz_odom,class_ids,stamp_s)
        # Commit each capture before later captures/timers can expire its points.
        # Shared ingest already applied the per-cell/global retention bounds.
        for key in sorted(changed):
            self._update_summary(key,observation_stamp=float(stamp_s))
            self._summary_dirty.discard(key)
        return changed

    def _update_summary(self,key,observation_stamp=None):
        cell=self._cells[key];points=cell.points;ids=points['class_id']
        counts=np.bincount(ids.astype(np.intp),minlength=64)
        histogram={int(i):int(counts[i]) for i in np.flatnonzero(counts)}
        cell.semantic_hist=histogram
        # Passive expiry/cap eviction only updates the active-point histogram.
        if observation_stamp is None:return
        fresh=points['stamp']==observation_stamp
        if not fresh.any():return
        if len(points)>=self.config.semantic_min_points:
            cell.representative=self._representative(histogram,len(points))
            cell.representative_stamp=float(points['stamp'].max())
            self._representative_support[key]=len(points)
            self._representative_versions[key]=cell.version
        eligible=ELIGIBLE_MASK[ids]
        if (eligible&fresh).any():
            count=int(eligible.sum())
            raw=float(np.float32(CLASS_COSTS[ids[eligible]].sum(dtype=np.float64)/count))
            cell.last_semantic=(raw,min(1.,count/self.config.semantic_reference_points),count,
                                float(points['stamp'][eligible].max()))
            support_counts=counts.copy();support_counts[~ELIGIBLE_MASK]=0
            self._semantic_histograms[key]=tuple((int(i),int(support_counts[i]))
                for i in np.flatnonzero(support_counts))
            self._semantic_distributions[key]=support_counts/count
            self._semantic_versions[key]=cell.version

    def _effective_confidence(self,key,raw,q,now_s):
        policy=self.cost_policy
        self._neighbor_deadlines.pop(key,None)
        if (not policy.neighbor_confidence_enabled or raw is None or raw>=100 or
                q>=policy.confidence_low_threshold):return q,False
        own=self._semantic_distributions.get(key)
        source_q=[];deadlines=[];x,y=key
        for dx,dy in NEIGHBORS:
            other=(x+dx,y+dy);cell=self._cells.get(other)
            if cell is None or cell.last_semantic is None:return q,False
            _,neighbor_q,_,stamp=cell.last_semantic
            if (neighbor_q<policy.confidence_neighbor_min or
                    now_s-stamp>self.config.raw_window_s or
                    not policy.compatible(own,self._semantic_distributions.get(other))):return q,False
            source_q.append(neighbor_q);deadlines.append(stamp+self.config.raw_window_s)
        self._neighbor_deadlines[key]=min(deadlines)
        # Only original observation support is read, never another corrected q.
        return max(q,float(np.mean(source_q))),True

    def snapshot(self,now_s,targets=None):
        now_s=float(now_s);self._advance_time(now_s)
        self._dirty.update(key for key,deadline in self._neighbor_deadlines.items() if now_s>deadline)
        for key in sorted(self._summary_dirty):self._update_summary(key)
        self._summary_dirty.clear()
        self.last_snapshot_changed_keys=frozenset(self._dirty)
        for key in sorted(self._dirty):
            c=self._cells[key];raw=None;q=0.;support=0;stamp=None
            if c.last_semantic is not None:raw,q,support,stamp=c.last_semantic
            effective_q,corrected=self._effective_confidence(key,raw,q,now_s)
            adjusted=self.cost_policy.adjusted(raw,effective_q)
            score=100 if adjusted is None else int(math.floor(adjusted+.5))
            eligible=int(ELIGIBLE_MASK[c.points['class_id']].sum())
            c.estimate=SemanticEstimate(c.representative,c.representative_stamp,
                self._representative_support.get(key,0),raw,adjusted,score,q,len(c.points),eligible,support,stamp,
                float(c.last_observation),c.representative is not None and self._representative_versions.get(key)!=c.version,
                raw is not None and self._semantic_versions.get(key)!=c.version,tuple(sorted(c.semantic_hist.items())),
                effective_q,corrected,8 if corrected else 0,self._semantic_histograms.get(key,()))
        self._dirty.clear()
        requested=self._cells.keys() if targets is None else targets
        return {key:self._cells[key].estimate for key in requested if key in self._cells and self._cells[key].estimate is not None}

    def layers(self,grid,estimates):
        shape=(grid.height,grid.width)
        a={'cost':np.full(shape,-1,np.int8),'representative_class':np.full(shape,-1,np.int16),
           'observed':np.zeros(shape,np.uint8),'semantic_raw':np.full(shape,np.nan,np.float32),
           'semantic_adjusted':np.full(shape,np.nan,np.float32),'q_s':np.zeros(shape,np.float32),
           'effective_q_s':np.zeros(shape,np.float32),'confidence_neighbor_corrected':np.zeros(shape,np.uint8),
           'confidence_neighbor_support_cells':np.zeros(shape,np.uint8),
           'representative_stamp_s':np.full(shape,np.nan,np.float64),'semantic_stamp_s':np.full(shape,np.nan,np.float64),
           'retained_points':np.zeros(shape,np.uint32),'semantic_support_points':np.zeros(shape,np.uint32),
           'representative_support_points':np.zeros(shape,np.uint32),'semantic_reused':np.zeros(shape,np.uint8),
           'representative_reused':np.zeros(shape,np.uint8)}
        for (ix,iy),e in estimates.items():
            x,y=grid.cell((ix+.5)*grid.resolution,(iy+.5)*grid.resolution)
            if not(0<=x<grid.width and 0<=y<grid.height):continue
            a['observed'][y,x]=1;a['cost'][y,x]=e.output_cost
            for k,v in [('representative_class',e.representative_class),('semantic_raw',e.semantic_raw),
                        ('semantic_adjusted',e.semantic_adjusted),('q_s',e.q_s),
                        ('effective_q_s',e.effective_q_s),('confidence_neighbor_corrected',e.confidence_neighbor_corrected),
                        ('confidence_neighbor_support_cells',e.confidence_neighbor_support_cells),
                        ('representative_stamp_s',e.representative_stamp_s),('semantic_stamp_s',e.semantic_stamp_s),
                        ('retained_points',e.total_points),('semantic_support_points',e.semantic_support_points),
                        ('representative_support_points',e.representative_support_points),
                        ('semantic_reused',e.semantic_reused),('representative_reused',e.representative_reused)]:
                if v is not None:a[k][y,x]=v
        return a
