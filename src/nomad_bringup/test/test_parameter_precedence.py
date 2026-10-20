"""Exercise actual ROS selector merging without simulation or spinning."""
from pathlib import Path
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from nomad_bringup.launching import merged_parameter_file


def test_named_override_wins_and_vehicle_defaults_remain(tmp_path):
    base=tmp_path/'planning.yaml'; vehicle=tmp_path/'vehicle.yaml'; override=tmp_path/'override.yaml'
    base.write_text('dstar_lite_gpp:\n  ros__parameters:\n    gpp_algorithm: dstar_lite\n')
    vehicle.write_text('/**:\n  ros__parameters:\n    max_steer: 0.4\n    wheelbase: 0.72\n')
    override.write_text('dstar_lite_gpp:\n  ros__parameters:\n    max_steer: 0.35\n')
    merged=merged_parameter_file([base,vehicle,override],dict(gpp_algorithm='hybrid_astar',lpp_algorithm='rpp'))
    for name,expected in [('dstar_lite_gpp',.35),('ackermann_rollout_lpp',.4)]:
        context=Context();rclpy.init(context=context,args=['--ros-args','--params-file',merged],domain_id=64)
        node=Node(name,context=context,enable_rosout=False,start_parameter_services=False)
        try:
            assert node.declare_parameter('max_steer',.1).value==expected
            assert node.declare_parameter('wheelbase',.1).value==.72
            assert node.declare_parameter('gpp_algorithm','dstar_lite').value=='hybrid_astar'
            assert node.declare_parameter('lpp_algorithm','rollout').value=='rpp'
        finally:
            node.destroy_node();context.shutdown()
    Path(merged).unlink()
