"""Single-owner terrain state in a spawned process, with one in-flight capture."""
import multiprocessing as mp
import queue
import time
import traceback
import json
from pathlib import Path
import numpy as np
from .navigation_costmap import NavigationLayers, navigation_arrays
from .planning_layers import planning_layers


class TerrainEngine:
    def __init__(self, config):
        self.config = config
        self.layers = NavigationLayers(**config['layers'])
        self.count = 0
        self.pending_scans = []
        self.last_scan_stamp = -float("inf")

    def compute(self, job):
        started = time.perf_counter()
        layers = self.layers
        if job['bootstrap'] is not None:
            x, y, stamp, radius, yaw = job['bootstrap']
            layers.bootstrap_free(x, y, stamp, radius, yaw=yaw)
            layers.bootstrap_active = job['bootstrap_active']
        layers.body_z = job['body_z']
        cutoff=job['stamp_ns']*1e-9
        # Keep newer scans for the next capture instead of overwriting an older
        # observation and then rejecting it as future data in scan_free_memory.
        queued={args[-1]:args for args in self.pending_scans+job['scans'] if args[-1]>self.last_scan_stamp}
        ready=[queued[t] for t in sorted(queued) if t<=cutoff+1e-6]
        self.pending_scans=[queued[t] for t in sorted(queued) if t>cutoff+1e-6]
        if len(self.pending_scans)>256:raise ValueError('Scan/capture clock divergence: deferred scan queue overflow')
        for args in ready:
            layers.views.grid.integrate(*args)
            self.last_scan_stamp=args[-1]
        scan_ms = (time.perf_counter()-started)*1000
        stage = time.perf_counter()
        stamp = job['stamp_ns']*1e-9
        estimates = layers.ingest(job['xyz'], job['ids'], stamp)
        ingest_ms = (time.perf_counter()-stage)*1000
        stage = time.perf_counter()
        arrays = layers.arrays(estimates)
        arrays_ms = (time.perf_counter()-stage)*1000
        stage = time.perf_counter()
        maps = navigation_arrays(layers, estimates, arrays, stamp,
                                 self.config['slope_limit'], self.config['local_age'])
        costmap_ms = (time.perf_counter()-stage)*1000
        features = planning_layers(layers, arrays, self.config['slope_limit'])
        self.count += 1
        g = layers.views.grid
        result = dict(stamp_ns=job['stamp_ns'], arrays=arrays, maps=maps, features=features,
                    grid={k:getattr(g,k) for k in ('width','height','resolution','origin_x','origin_y')},
                    frames=self.count, plane_sources=layers.core.quality_metadata()['current_cell_sources'],
                    timings={'scan_complete':scan_ms, 'ingest_complete':ingest_ms,
                             'arrays_complete':arrays_ms, 'costmap_complete':costmap_ms},
                    processing_ms=(time.perf_counter()-started)*1000)
        from .observation_support import support_grid
        result['camera_support'] = support_grid(layers.block_evidence.history, g, stamp)
        result['obstacle_diagnostics'] = layers.obstacle_diagnostics
        result['scans_integrated'] = len(ready)
        result['scans_deferred'] = len(self.pending_scans)
        if job.get('save_directory'):
            report = make_report(result, job['report_metadata'])
            persist_result(job['save_directory'], result['arrays'], result['maps'], result['grid'],
                           report, job['bootstrap'], job['scan_stats'])
        return result



def make_report(result, metadata):
    arrays = result['arrays']
    return dict(metadata, frames=result['frames'], capture_stamp=result['stamp_ns']*1e-9,
                processing_ms=result['processing_ms'], height_cells=int(np.isfinite(arrays[0]).sum()),
                slope_cells=int(np.isfinite(arrays[1]).sum()), class_cells=int((arrays[2]>=0).sum()),
                plane_sources=result['plane_sources'],
                obstacle_diagnostics=result.get('obstacle_diagnostics', {}))


def persist_result(directory, arrays, maps, grid, report, bootstrap, scan_stats):
    root = Path(directory)
    root.mkdir(parents=True, exist_ok=True)
    origin = [grid['origin_x'],grid['origin_y']]
    if report.get('obstacle_diagnostics'):
        with (root/'obstacle_diagnostics.jsonl').open('a') as stream:
            stream.write(json.dumps(dict(capture_stamp=report['capture_stamp'],
                **report['obstacle_diagnostics']))+'\n')
    np.savez_compressed(root/'map.pending.npz', height=arrays[0], slope=arrays[1], classes=arrays[2],
                        origin=origin, resolution=grid['resolution'])
    (root/'map.pending.npz').replace(root/'map.npz')
    (root/'status.json').write_text(json.dumps(report, indent=2))
    np.savez_compressed(root/'costmaps.npz', global_costmap=maps[0], local_costmap=maps[1],
                        origin=origin, resolution=grid['resolution'], stamp=report['capture_stamp'],
                        bootstrap_origin=np.asarray(bootstrap[:4] if bootstrap else []),
                        scan_received=scan_stats['received'], scan_integrated=scan_stats['integrated'],
                        scan_expired=scan_stats['expired'])


def _run(config, requests, results):
    try:
        engine = TerrainEngine(config)
        while True:
            job = requests.get()
            if job is None:
                return
            result = engine.compute(job)
            results.put(result)
    except Exception:
        results.put({'error':traceback.format_exc()})


class TerrainWorker:
    def __init__(self, config):
        context = mp.get_context('spawn')
        self.requests = context.Queue(maxsize=1)
        self.results = context.Queue(maxsize=1)
        self.process = context.Process(target=_run, args=(config,self.requests,self.results),
                                       name='nomad-terrain-compute', daemon=True)
        self.busy = False
        self.submitted_at = None
        self.closed = False
        self.process.start()

    def submit(self, job):
        if self.closed or self.busy:
            return False
        self.requests.put_nowait(job)
        self.busy = True
        self.submitted_at = time.monotonic()
        return True

    def poll(self):
        try:
            result = self.results.get_nowait()
        except queue.Empty:
            if not self.closed and not self.process.is_alive():
                return {'error':f'Terrain worker exited ({self.process.exitcode})'}
            return None
        self.busy = False
        return result

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            self.requests.put_nowait(None)
        except queue.Full:
            pass
        self.process.join(timeout=2)
        if self.process.is_alive():
            self.process.terminate()  # Only the child owned by this instance.
            self.process.join(timeout=2)
        for channel in (self.requests,self.results):
            channel.cancel_join_thread()
            channel.close()
