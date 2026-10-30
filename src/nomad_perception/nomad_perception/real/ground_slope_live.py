"""Live measured-ground slope adapter with the selected 12cm/3s profile."""
from dataclasses import fields, replace
from pathlib import Path
import hashlib
import json
import math
import multiprocessing as mp
import os
import queue
import signal
import time
import traceback

import numpy as np
import yaml
from .depth_filter import SORSettings, frame_voxels
from .ground_reference.costmap_core import CoreConfig, PlaneQualityConfig
from .ground_reference.slope_comparison_core import ForwardSlopeMap
from .worker_io import TerrainWorker, atomic_json
from .capture_timing import trace as capture_trace, capture_range, stage, summary as timing_summary, positive_ns

ROOT = Path(__file__).resolve().parents[1]
REFERENCE = Path(__file__).with_name('ground_reference')


def configuration():
    p = yaml.safe_load((REFERENCE / 'settings.yaml').read_text())['nomad_slope_compare']['ros__parameters']
    config = CoreConfig(**{f.name: p[f.name] for f in fields(CoreConfig)})
    quality = PlaneQualityConfig(**{f.name: p['plane_' + f.name] for f in fields(PlaneQualityConfig)})
    profile = os.environ.get('MAPPING_SLOPE_PROFILE', 'reference_7cm_10s')
    if profile == '12cm_3s_optimized':
        config = replace(config, resolution_m=.12, radius_m=.18, raw_window_s=3.)
        quality = replace(quality, refit_radii_m=(.18,))
        p = {**p, 'resolution_m': .12, 'radius_m': .18, 'raw_window_s': 3.,
             'plane_refit_radii_m': [.18], 'forward_update_radius_m': 3.,
             'calculation_optimization': True}
    elif profile != 'reference_7cm_10s':
        raise ValueError('Unknown MAPPING_SLOPE_PROFILE: ' + profile)
    p['live_profile'] = profile
    return config, quality, p


def sensor_mounts(imu_camera_b, camera_b_lidar, imu_camera_a):
    """Use the same physical LiDAR forward convention as the pinned replay."""
    lidar = np.asarray(imu_camera_b) @ np.asarray(camera_b_lidar)
    direction = np.r_[np.asarray(camera_b_lidar)[2, :2], 0.]
    length = np.linalg.norm(direction)
    if length < 1e-8:
        raise ValueError('LiDAR forward direction cannot be projected')
    direction /= length
    basis = np.eye(4)
    basis[:3, :3] = np.column_stack((direction, [-direction[1], direction[0], 0.], [0., 0., 1.]))
    return np.array([lidar @ basis, imu_camera_a])


class GroundSlope:
    def __init__(self, mounts, start_z, start_stamp_ns, camera_settings=None, semantic_prefiltered=False, frame='world'):
        if frame not in ('world','odom'):raise ValueError('Unsupported mapping frame')
        self.frame=frame
        config, quality, p = configuration()
        model_class = ForwardSlopeMap
        if p.get('calculation_optimization'):
            from .ground_reference.slope_optimized_core import OptimizedForwardSlopeMap
            model_class = OptimizedForwardSlopeMap
        self.model = model_class(config, quality, mounts, 'sensor_separate',
                                 p['forward_update_radius_m'], p['max_size_m'])
        self.live_profile = (p['live_profile'] if p.get('calculation_optimization')
                             else 'ground_7cm_no_height_cut')
        sources = [Path(__file__), REFERENCE/'slope_comparison_core.py']
        if p.get('calculation_optimization'):
            sources.append(REFERENCE/'slope_optimized_core.py')
        self.implementation_sha256 = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                                      for path in sources}
        self.mounts = np.asarray(mounts).copy()
        self.start_z = float(start_z)
        if not math.isfinite(self.start_z):
            raise ValueError('Finite first VIO Z required')
        self.start_stamp_ns = int(start_stamp_ns)
        self.settings = camera_settings or SORSettings(max_z_m=4.)
        self.semantic_prefiltered = bool(semantic_prefiltered)
        self.last_stamp = [-1, -1]
        self.last_bucket = [-1, -1]
        self.sampled = [0, 0]
        self.input_points = [0, 0]
        self.selected_points = [0, 0]
        self.height_excluded = [0, 0]
        self.camera_frames = 0
        self.latest_pose_stamp_ns = -1
        self.last_observed_ns = {}
        self.source_capture_ns = [0, 0]
        self.last_contribution_ns = None

    def ingest(self, points, stamp_ns, source, camera_points, pose, point_stamps=None):
        stamp_ns=positive_ns(stamp_ns)
        self.last_contribution_ns=None
        if source not in (0, 1) or stamp_ns <= self.last_stamp[source]:
            raise ValueError('Duplicate or reordered sensor timestamp')
        self.last_stamp[source] = stamp_ns
        # ROI follows the newest available VIO pose, never a delayed sensor pose.
        if pose[0] >= self.latest_pose_stamp_ns:
            self.model.roi.set_pose(pose[0] / 1e9, pose[1:4], pose[4:8])
            self.latest_pose_stamp_ns = int(pose[0])
        bucket = stamp_ns // 200_000_000
        if bucket == self.last_bucket[source]:
            self.sampled[source] += 1
            return None
        self.last_bucket[source] = bucket
        p = np.asarray(points, dtype=float).reshape(-1, 3)
        if not np.isfinite(p).all():
            raise ValueError('Nonfinite world points')
        self.input_points[source] += len(p)
        if source == 1 and not self.semantic_prefiltered:
            if camera_points is None:
                raise ValueError('OAK optical XYZ required for range filtering')
            _, p, _, _ = frame_voxels(camera_points, p, self.settings)
            self.camera_frames += 1
        elif source == 1:
            self.camera_frames += 1
        # This copy accepts points at every world height after the same preprocessing.
        # Keep the initial VIO Z as provenance only; it never selects slope input.
        self.selected_points[source] += len(p)
        stamps = np.full(len(p),stamp_ns,dtype=np.int64) if point_stamps is None else np.asarray(point_stamps)
        if stamps.shape!=(len(p),) or stamps.dtype.kind not in 'iu' or np.any(stamps<=0):
            raise ValueError('Positive integer capture time required for every selected point')
        bound=self.model.half_cells*self.model.config.resolution_m
        included=np.all((p[:,:2]>=-bound)&(p[:,:2]<bound),axis=1)
        if np.any(included):
            contributing=stamps[included]
            self.last_contribution_ns=(int(contributing.min()),int(contributing.max()))
            self.source_capture_ns[source]=max(self.source_capture_ns[source],self.last_contribution_ns[1])
            cells=np.floor(p[included,:2]/self.model.config.resolution_m).astype(np.int64)
            keys,inverse=np.unique(cells,axis=0,return_inverse=True)
            last=np.zeros(len(keys),dtype=np.int64);np.maximum.at(last,inverse,contributing)
            for key,ns in zip(map(tuple,keys),last):
                self.last_observed_ns[key]=max(self.last_observed_ns.get(key,0),int(ns))
        self.model.ingest(p, stamp_ns / 1e9, source)
        return p

    def snapshot(self):
        """Add direct-observation provenance without altering the frozen fitter."""
        snap=self.model.snapshot()
        last=np.zeros(snap['observed'].shape,dtype=np.int64)
        origin=np.rint(snap['origin_xy']/float(snap['resolution_m'])).astype(np.int64)
        rows,cols=np.nonzero(snap['observed'])
        for row,col in zip(rows,cols):
            last[row,col]=self.last_observed_ns.get((int(col+origin[0]),int(row+origin[1])),0)
        snap['last_observed_stamp_ns']=last
        snap['snapshot_capture_ns']=np.int64(last.max(initial=0))
        return snap

    def metadata(self):
        return {**self.model.metadata(), 'frame_id': self.frame, 'mode': 'fused',
            'live_profile': self.live_profile,
            'implementation_sha256': self.implementation_sha256,
            'ground_segmentation': self.semantic_prefiltered, 'accuracy_verified': False,
            'semantic_ground_policy': 'only points classified ground after visibility gate' if self.semantic_prefiltered else None,
            'height_selection': {'enabled': False, 'operator': None, 'start_z_m': self.start_z,
                'start_stamp_ns': self.start_stamp_ns, 'reference': 'first accepted Basalt VIO pose; recorded only, no height cutoff'},
            'camera_preprocessing': {'settings': vars(self.settings), 'semantic_prefiltered': self.semantic_prefiltered, 'repeated_frame_filter': False,
                'radius_support_filter': False, 'frames': self.camera_frames},
            'source_labels': {'0': 'LiDAR', '1': 'OAK'}, 'input_points': self.input_points,
            'selected_points': self.selected_points, 'height_excluded_points': self.height_excluded,
            'temporally_skipped_frames': self.sampled,
            'sensor_mounts': self.mounts.tolist(),
            'source_capture_ns':list(self.source_capture_ns),
            'timestamp_policy':'snapshot uses contributing capture times; per-cell last direct observation is preserved; calculated pose time is not observation time',
            'algorithm_provenance': json.loads((REFERENCE / 'provenance.json').read_text())}


def sparse_payload(snap):
    rows, cols = np.nonzero(snap['observed'])
    size = float(snap['resolution_m'])
    origin = np.rint(snap['origin_xy'] / size).astype(int)
    def number(value):
        return float(value) if np.isfinite(value) else None
    return {'resolution_m': size, 'radius_m': .105,
        'columns': ['ix', 'iy', 'slope_deg', 'height_m', 'confidence', 'slope_reused', 'last_calculated_stamp_s', 'last_observed_stamp_ns'],
        'cells': [[int(c + origin[0]), int(r + origin[1]), number(snap['slope_deg'][r, c]),
            number(snap['height_m'][r, c]), number(snap['confidence'][r, c]),
            bool(snap['slope_reused'][r, c]), number(snap['last_calculated_stamp_s'][r, c]),
            int(snap['last_observed_stamp_ns'][r,c]) if 'last_observed_stamp_ns' in snap else None]
            for r, c in zip(rows, cols)],
        'observed_cells': len(rows), 'valid_cells': int(snap['slope_valid'].sum())}




def _worker(directory, observations, synthetic, settings, semantic_prefiltered=False, frame='world'):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    directory = Path(directory)
    ground = None
    next_update = 0.
    finished = False
    changed = False
    updates = 0
    queue_delay_ms = 0.
    inspection_enabled = True
    last_snap = None
    last_live = None
    source_timing = {}
    try:
        while not finished:
            batch = []
            try:
                item = observations.get(timeout=.05)
                if item is None:
                    finished = True
                else:
                    batch.append(item)
            except queue.Empty:
                pass
            # Stop draining at the update deadline so a full queue cannot starve output.
            for _ in range(31):
                if finished or (batch and time.monotonic() >= next_update):
                    break
                try:
                    item = observations.get_nowait()
                except queue.Empty:
                    break
                if item is None:
                    finished = True
                    break
                batch.append(item)
            for item in batch:
                if isinstance(item[0], str):
                    inspection_enabled = bool(item[1])
                    if inspection_enabled and ground is not None:
                        # Resume with fresh support only, keeping the old output and initial pose record.
                        ground.model.samples.clear();ground.model.heights.clear()
                        ground.model.dirty.clear();ground.model.previous_active.clear()
                    changed = inspection_enabled
                    if last_live is not None:
                        last_live['inspection_enabled'] = inspection_enabled
                        atomic_json(directory / 'live.json', last_live)
                    current = json.loads((directory/'status.json').read_text()) if (directory/'status.json').exists() else {'enabled':True,'saved':False,'error':None,'modes':{}}
                    current['inspection_enabled'] = inspection_enabled
                    atomic_json(directory / 'status.json', current)
                    continue
                if not inspection_enabled:
                    continue
                stamp_ns, source, points, camera, pose, mounts, z0, start_ns, submitted, timing, point_stamps = item
                timing=stage(timing,'worker_started')
                queue_delay_ms = max(0., (time.monotonic() - submitted) * 1000)
                if ground is None:
                    ground = GroundSlope(mounts, z0, start_ns, settings, semantic_prefiltered=semantic_prefiltered,frame=frame)
                if z0 != ground.start_z or start_ns != ground.start_stamp_ns or not np.array_equal(mounts, ground.mounts):
                    raise ValueError('Fixed height reference or sensor mounts changed during collection')
                ground.ingest(points, stamp_ns, source, camera, pose,point_stamps=point_stamps)
                if ground.last_contribution_ns is not None:
                    timing['capture_first_ns'],timing['capture_last_ns']=ground.last_contribution_ns
                    source_timing[source]=timing
                changed = True
            now = time.monotonic()
            if finished or (inspection_enabled and changed and now >= next_update):
                started = time.monotonic()
                next_update = started + .5
                metadata = ground.metadata() if ground else {'height_selection': None, 'cost_computed': False}
                snap = (ground.snapshot() if inspection_enabled else last_snap) if ground else None
                if snap is not None:last_snap = snap
                if ground:
                    metadata = ground.metadata()
                data = sparse_payload(snap) if snap is not None else {'cells': [], 'valid_cells': 0, 'observed_cells': 0}
                config = ground.model.config if ground else configuration()[0]
                data['radius_m'] = config.radius_m
                updates += 1
                timing = {'snapshot_ms': ground.model.last_ms if ground else 0.,
                    'queue_wait_ms': queue_delay_ms, 'updates': updates,
                    'pose_input_gap_ms': max(0., (ground.latest_pose_stamp_ns / 1e9 - ground.model.latest) * 1000) if ground else None,
                    'scope': 'snapshot time excludes VIO, camera preprocessing, serialization, disk and display'}
                capture_timing=None
                if source_timing:
                    reference=max(source_timing.values(),key=lambda t:t['capture_last_ns'])
                    completed=stage(reference,'worker_completed',ros_ns=time.time_ns())
                    capture_timing=timing_summary(completed)
                    capture_timing.update(sequence=updates,snapshot_capture_ns=int(snap['snapshot_capture_ns']),
                        source_capture_ns=list(ground.source_capture_ns),inspection_enabled=inspection_enabled,
                        snapshot_reference_policy='last direct observation in emitted cells; slope reuse is separate',
                        navigation_ready=False)
                    atomic_json(directory/'timing.json',capture_timing)
                status = {'enabled': True, 'saved': finished, 'error': None,
                    'updated_monotonic': now, 'resolution_m': config.resolution_m, 'radius_m': config.radius_m,
                    'forward_update_radius_m': (ground.model.roi.radius_m if ground
                                                else configuration()[2]['forward_update_radius_m']),
                    'profile': ground.live_profile if ground else configuration()[2]['live_profile'], 'inspection_enabled': inspection_enabled, 'modes': {'fused': {'valid_cells': data['valid_cells'],
                        'observed_cells': data['observed_cells'], 'received_frames': ground.model.frames if ground else 0}},
                    'height_selection': metadata['height_selection'], 'timing': timing,
                    'capture_timing':capture_timing}
                live = {**status, 'synthetic': synthetic, 'modes': {'fused': data}, 'metadata': metadata}
                last_live = live
                atomic_json(directory / 'live.json', live)
                atomic_json(directory / 'status.json', status)
                changed = False
    except Exception as error:
        atomic_json(directory / 'status.json', {'enabled': True, 'saved': False, 'error': str(error)})
        (directory / 'error.log').write_text(traceback.format_exc())
        raise


class GroundSlopeWorker(TerrainWorker):
    def __init__(self, session, synthetic=False, camera_settings=None, semantic_prefiltered=False, frame='world'):
        self.directory = Path(session) / 'terrain'
        self.directory.mkdir(exist_ok=True)
        ctx = mp.get_context('spawn')
        self.queue = ctx.Queue(maxsize=200)
        self.process = ctx.Process(target=_worker, args=(str(self.directory), self.queue, synthetic, camera_settings, semantic_prefiltered,frame), name='ground-slope-7cm')
        self.process.start()
        self.closed = False

    def set_inspection(self, enabled):
        if not self.process.is_alive():raise RuntimeError('경사 계산 프로세스가 종료됐습니다.')
        self.queue.put(('inspection',bool(enabled)),timeout=2)

    def submit(self, points, stamp_ns, source, camera_points=None, *, pose, mounts, start_z, start_stamp_ns,
               timing=None,point_stamps=None):
        if not self.process.is_alive():
            raise RuntimeError('경사 계산 프로세스가 종료됐습니다. terrain/error.log를 확인하세요.')
        stamp_ns=positive_ns(stamp_ns)
        point_stamps=None if point_stamps is None else np.asarray(point_stamps,dtype=np.int64).copy()
        timing=stage(capture_range(timing or capture_trace(stamp_ns,receipt_scope='worker_enqueue'),point_stamps),'submitted')
        item = (stamp_ns, int(source), np.asarray(points, dtype=float).copy(),
            None if camera_points is None else np.asarray(camera_points, dtype=float).copy(),
            np.asarray(pose[:8], dtype=float).copy(), np.asarray(mounts, dtype=float).copy(),
            float(start_z), int(start_stamp_ns), time.monotonic(), timing,point_stamps)
        try:
            self.queue.put_nowait(item)
        except queue.Full as error:
            raise RuntimeError('경사 계산 대기열이 가득 찼습니다. 원시 기록을 보존하고 수집을 중단합니다.') from error
