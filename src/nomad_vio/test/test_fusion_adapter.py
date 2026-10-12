import math
import numpy as np
import pytest
from nomad_vio.fusion_adapter import base_position,rotation


def test_parallel_imu_offset():
    np.testing.assert_allclose(base_position([2.46,3,.29],[0,0,0,1],[.46,0,.29]),[2,3,0],atol=1e-12)


def test_lever_arm_rotates_with_vehicle():
    q=[0,0,math.sin(math.pi/4),math.cos(math.pi/4)]
    np.testing.assert_allclose(base_position([2,3.46,.29],q,[.46,0,.29]),[2,3,0],atol=1e-12)


def test_slope_changes_antenna_reference():
    q=[0,math.sin(math.pi/12),0,math.cos(math.pi/12)]
    r=rotation(q);offset=np.array([.46,0,.29]);base=np.array([2.,3.,1.])
    np.testing.assert_allclose(base_position(base+r@offset,q,offset),base,atol=1e-12)


@pytest.mark.parametrize('q',[[0,0,0,0],[float('nan'),0,0,1]])
def test_invalid_attitude_is_rejected(q):
    with pytest.raises(ValueError):base_position([0,0,0],q,[.46,0,.29])


def test_gps_only_pose_waits_for_matching_invalid_diagnostic():
    from types import SimpleNamespace
    from nav_msgs.msg import Odometry
    from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
    from nomad_vio.fusion_adapter import FusionAdapter
    published=[]
    stub=SimpleNamespace(diagnostic_stamp=9.,pending_pose=None,
        valid_orientation=True,last_status_wall=0.,clock_fault=False,
        mode='FUSED',previous=None,last_wall=0.,
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),
        pub=SimpleNamespace(publish=published.append))
    stub.receive=lambda msg:FusionAdapter.receive(stub,msg)
    pose=Odometry();pose.header.stamp.sec=10
    stub.receive(pose)
    assert stub.pending_pose is not None and published==[]
    diag=DiagnosticArray();diag.header.stamp.sec=10
    status=DiagnosticStatus();status.name='fusion';status.message='GNSS_ONLY_ORIENTATION_HELD'
    value=KeyValue();value.key='orientation_valid';value.value='false';status.values=[value];diag.status=[status]
    FusionAdapter.diagnostics(stub,diag)
    assert stub.pending_pose is None and not stub.valid_orientation and published==[]
