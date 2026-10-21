import time
import numpy as np
from nomad_perception.terrain_worker import TerrainEngine, TerrainWorker

CONFIG = {'layers':dict(size=6.,max_size=6.),'slope_limit':20.,'local_age':30.}

def job(stamp=1_000_000_000):
    x,y=np.meshgrid(np.arange(-.5,.6,.1),np.arange(-.5,.6,.1))
    xyz=np.column_stack((x.ravel(),y.ravel(),np.zeros(x.size))).astype(np.float32)
    return dict(stamp_ns=stamp,xyz=xyz,ids=np.full(len(xyz),8,np.uint32),body_z=.3,
                bootstrap=(0.,0.,.5,1.5,0.),bootstrap_active=True,scans=[])

def wait(worker):
    deadline=time.monotonic()+15
    while time.monotonic()<deadline:
        result=worker.poll()
        if result is not None:return result
        time.sleep(.01)
    raise AssertionError('worker did not respond')

def test_spawned_worker_matches_serial_and_has_single_inflight():
    engine=TerrainEngine(CONFIG);worker=TerrainWorker(CONFIG)
    try:
        for stamp in (1_000_000_000,2_000_000_000):
            j=job(stamp);expected=engine.compute(j)
            assert worker.submit(j)
            assert not worker.submit(job(stamp+1))
            actual=wait(worker)
            assert 'error'not in actual and actual['stamp_ns']==stamp
            assert actual['grid']==expected['grid'] and actual['frames']==expected['frames']
            for a,b in zip(actual['arrays']+actual['maps'],expected['arrays']+expected['maps']):
                np.testing.assert_equal(a,b)
    finally:worker.close()
    assert not worker.process.is_alive()

def test_worker_error_and_exit_are_reported():
    worker=TerrainWorker(CONFIG)
    try:
        assert worker.submit(job())
        assert 'error'not in wait(worker)
        assert worker.submit(job()) # Duplicate capture cannot silently corrupt accumulated state.
        assert 'Non-monotonic' in wait(worker)['error']
    finally:worker.close()
    worker=TerrainWorker(CONFIG)
    try:
        worker.process.terminate();worker.process.join(timeout=3)
        assert 'exited' in worker.poll()['error']
    finally:worker.close()

def test_worker_recording_preserves_outputs(tmp_path):
    j=job();j.update(save_directory=str(tmp_path), report_metadata={'sync_dropped':0},
                     scan_stats={'received':0,'integrated':0,'expired':0})
    TerrainEngine(CONFIG).compute(j)
    assert (tmp_path/'map.npz').is_file() and (tmp_path/'costmaps.npz').is_file()
    assert (tmp_path/'status.json').is_file()


def test_newer_scan_deferred_then_integrated_without_future_free_cells():
 from nomad_perception.navigation_costmap import scan_free_memory
 import math
 cfg=dict(CONFIG,layers=dict(size=40.,max_size=40.))
 e=TerrainEngine(cfg)
 j=job(1_000_000_000);j['bootstrap']=None
 j['scans']=[((0.,0.,0.),[3.],-math.pi/2,1.,.1,12.,1.2)]
 a=e.compute(j);g=e.layers.views.grid;x,y=g.cell(0.,-1.)
 assert a['scans_integrated']==0 and a['scans_deferred']==1
 assert not scan_free_memory(e.layers,1.)[0][y,x]
 j=job(1_500_000_000);j['bootstrap']=None
 j['scans']=[((0.,0.,0.),[3.],-math.pi/2,1.,.1,12.,1.7)]
 b=e.compute(j)
 assert b['scans_integrated']==1 and b['scans_deferred']==1
 assert g.seen[y,x]==1.2
 assert scan_free_memory(e.layers,1.5)[0][y,x]

def test_remembered_support_never_overwrites_new_blocked_cells():
 e=TerrainEngine(CONFIG)
 for stamp in [1_000_000_000,1_800_000_000,2_600_000_000]:
  j=job(stamp);j['bootstrap']=None
  j['ids'][:]=1;j['xyz'][:,2]=.6  # Raised solid returns above body_z=.3.
  result=e.compute(j)
 blocked=result['maps'][0]==100
 assert blocked.any()
 assert np.any(result['camera_support'][blocked]>0)
 # A cell can have strong observation support AND remain strictly blocked.
 assert np.all(result['maps'][0][blocked]==100)
