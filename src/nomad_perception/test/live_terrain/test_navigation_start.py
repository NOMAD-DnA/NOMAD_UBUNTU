from nomad_perception.navigation_start import ready_to_start
from nomad_interfaces.msg import ModuleStatus
from nav_msgs.msg import Odometry

def state():
 s=ModuleStatus();s.state=ModuleStatus.READY;p=Odometry();p.header.frame_id='odom';p.child_frame_id='base_link'
 return {'vio':(s,1.),'perception':(s,1.),'pose':(p,1.)}

def test_ready_requires_stationary_fresh_pose():
 latest=state();assert ready_to_start(latest,1.1)
 latest['pose'][0].twist.twist.linear.x=.2;assert not ready_to_start(latest,1.1)
 latest['pose'][0].twist.twist.linear.x=0;latest['pose'][0].twist.twist.angular.z=.1;assert not ready_to_start(latest,1.1)
 latest['pose'][0].twist.twist.angular.z=0;assert not ready_to_start(latest,2.)
 latest['pose'][0].twist.twist.linear.x=float('nan');assert not ready_to_start(latest,1.1)

def test_no_launch_on_missing_or_faulted_input():
 latest=state();latest.pop('pose');assert not ready_to_start(latest,1.1)
 latest=state();latest['perception'][0].state=ModuleStatus.WAITING;assert not ready_to_start(latest,1.1)
