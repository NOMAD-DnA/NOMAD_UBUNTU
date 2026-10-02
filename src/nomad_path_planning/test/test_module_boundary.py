import math
from nav_msgs.msg import OccupancyGrid
from nomad_interfaces.msg import ModuleStatus
from nomad_path_planning.contracts import valid_grid
from nomad_path_planning.input_bridge import status_ready
from nomad_path_planning.command_selector import PlanningCommandNode
from test_history_recovery import controller,motion,path


def test_explicit_steering_is_preserved_and_path_does_not_set_speed():
    node,commands=controller()
    msg=motion([0.,-.02,-.04])
    msg.target_speed=-.15
    msg.target_steering_angle=.25
    PlanningCommandNode.on_recovery_motion(node,msg)
    PlanningCommandNode.control(node)
    assert commands[-1].target_speed==-.15
    assert commands[-1].target_steering_angle==.25


def test_delayed_path_receipt_cannot_extend_trajectory_validity():
    node,commands=controller()
    node.recovery_active=True
    node.recovery_received_at=10.
    node.recovery_motion=motion([0.,-.02],sec=9)
    node.recovery_path=node.recovery_motion.path
    PlanningCommandNode.control(node)
    assert commands[-1].stop_requested


def test_recovery_hold_owns_control_despite_new_forward_rollout():
    node,commands=controller()
    node.global_path=path([0.,10.])
    PlanningCommandNode.on_recovery_motion(node,motion([0.]))
    PlanningCommandNode.on_local_motion(node,motion([0.,.03]))
    PlanningCommandNode.control(node)
    assert node.recovery_active and commands[-1].stop_requested


def test_external_map_rejects_frame_rotation_bad_values_and_shape():
    grid=OccupancyGrid()
    grid.header.frame_id='odom'
    grid.info.width=grid.info.height=2
    grid.info.resolution=.2
    grid.info.origin.orientation.w=1.
    grid.data=[0,35,80,-1]
    assert valid_grid(grid,'odom')
    grid.header.frame_id='map'
    assert not valid_grid(grid,'odom')
    grid.header.frame_id='odom'
    grid.info.origin.orientation.z=.1
    assert not valid_grid(grid,'odom')
    grid.info.origin.orientation.z=0.
    grid.data=[0,101,80,-1]
    assert not valid_grid(grid,'odom')
    grid.data=[0]
    assert not valid_grid(grid,'odom')


def test_heartbeat_does_not_outlive_lease():
    status=ModuleStatus(state=ModuleStatus.READY)
    status.header.stamp.sec=10
    status.valid_for.nanosec=600_000_000
    assert status_ready(status,10.1)
    assert not status_ready(status,10.7)
    status.state=ModuleStatus.WAITING
    assert not status_ready(status,10.1)
