import json
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock
import pytest
import numpy as np
from cv_bridge import CvBridge
from nomad_semantic.node import Semantic
from nomad_logging.diagnostics import Diagnostics


def node():
    bridge = CvBridge()
    msg = bridge.cv2_to_imgmsg(np.zeros((2, 3, 3), dtype=np.uint8), encoding='bgr8')
    msg.header.stamp.sec = 10
    msg.header.stamp.nanosec = 123
    return NS(latest=msg, received=time.monotonic(), trace=Mock(), bridge=bridge,
              get_parameter=lambda key:NS(value=.6),
              get_clock=lambda:NS(now=lambda:NS(nanoseconds=10_000_000_123)),
              engine=NS(predict=Mock(return_value=(np.zeros((2,3),np.uint8),
                        np.ones((2,3),np.float32), np.zeros((2,3,3),np.uint8),
                        np.zeros((2,3,3),np.uint8))), backend='fake',
                        meta={'class_names_verified':True,'classes':[]}),
              pubs={k:Mock() for k in ('labels','confidence','colors','overlay')},
              info=Mock(), count=0, last_log=time.monotonic(), show=False,
              get_logger=lambda:Mock())


def test_publish_capture_size_and_order():
    n=node(); Semantic.process(n)
    starts=[c for c in n.trace.call_args_list if c.args==('publish_start',)]
    completed=[c for c in n.trace.call_args_list if c.args==('publish_complete',)]
    assert [c.kwargs['stream'] for c in starts]==['labels','confidence','colors','overlay']
    assert len(completed)==4 and n.count==1 and n.latest is None
    assert starts[0].kwargs['size_bytes']==6
    assert starts[1].kwargs['size_bytes']==24
    assert all(c.kwargs['stamp_ns']==10_000_000_123 for c in starts+completed)
    for pub in n.pubs.values():assert pub.publish.call_count==1


def test_expired_input_does_not_infer():
    n=node();n.received-=2;Semantic.process(n)
    n.engine.predict.assert_not_called()
    assert n.trace.call_args.kwargs['reason']=='input_expired'


def test_latest_replacement_reports_original_stamp():
    n=node();old=n.latest;new=CvBridge().cv2_to_imgmsg(np.zeros((2,3,3),np.uint8),'bgr8')
    new.header.stamp.sec=11;Semantic.receive(n,new)
    excluded=n.trace.call_args
    assert excluded.kwargs['reason']=='latest_overwritten'
    assert excluded.kwargs['stamp_ns']==10_000_000_123
    assert n.latest is new


def test_publish_failure_is_not_reported_as_complete():
    n=node();n.pubs['confidence'].publish.side_effect=RuntimeError('test publish failure')
    with pytest.raises(RuntimeError):Semantic.process(n)
    completed=[c.kwargs['stream']for c in n.trace.call_args_list if c.args==('publish_complete',)]
    assert completed==['labels']
    assert n.trace.call_args.args==('process_error',)


def test_semantic_writer_flush(tmp_path):
    d=Diagnostics(str(tmp_path),prefix='semantic');d.emit('publish_complete',stamp_ns=42);d.close()
    assert d.path.name.startswith('semantic_')
    rows=[json.loads(l)for l in d.path.read_text().splitlines()]
    assert rows[0]['stamp_ns']==42 and rows[-1]['dropped']==0
