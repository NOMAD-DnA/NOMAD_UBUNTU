import json
from types import SimpleNamespace as NS
from unittest.mock import Mock
from nomad_perception.map_diagnostics import MapDiagnostics
from nomad_perception.live_terrain import LiveTerrain
from nomad_perception.semantic_input import ExactCaptureSynchronizer


def test_disabled_and_writer_flush(tmp_path):
    off=MapDiagnostics(''); off.emit('test'); off.close()
    assert off.path is None
    log=MapDiagnostics(str(tmp_path)); log.emit('input_received',stamp_ns=123,sim_ns=456)
    log.close()
    data=[json.loads(x) for x in log.path.read_text().splitlines()]
    assert data[0]['stamp_ns']==123 and data[0]['mono_ns']>0
    assert data[-1]=={'event':'diagnostics_closed','dropped':0}
    assert log.error is None


def test_sync_expiry_reason_and_capture_id(tmp_path):
    log=MapDiagnostics(str(tmp_path))
    sync=ExactCaptureSynchronizer(streams=('depth','labels','confidence'),max_age_wall_sec=3,on_event=log.emit)
    msg=NS(header=NS(stamp=NS(sec=1,nanosec=23)))
    assert sync.add('depth',msg,now=0) is None
    sync.expire(now=4);log.close()
    row=json.loads(log.path.read_text().splitlines()[0])
    assert row['event']=='synchronization_excluded'
    assert row['stamp_ns']==1000000023 and row['reason']=='buffer_wall_age_expired'


def test_pending_overwrite_records_old_and_new_capture():
    depth=lambda sec:NS(header=NS(stamp=NS(sec=sec,nanosec=0)))
    old,new=depth(1),depth(2)
    node=NS(trace=Mock(),sync=NS(add=lambda k,m:{'depth':new}),pending=({'depth':old},0))
    LiveTerrain.receive(node,'depth',new)
    calls=node.trace.call_args_list
    assert calls[0].args==('input_received',)
    assert calls[1].args==('sync_complete',)
    assert calls[2].kwargs['stamp_ns']==1000000000
    assert calls[2].kwargs['replacement_stamp_ns']==2000000000
    assert node.pending[0]['depth'] is new


def test_pending_expiry_does_not_attempt_tf():
    depth=NS(header=NS(stamp=NS(sec=1,nanosec=0)))
    node=NS(trace=Mock(),pending=({'depth':depth},0))
    LiveTerrain.process(node)
    assert node.pending is None
    assert node.trace.call_args.args==('pending_expired',)
