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


def test_initial_teleop_handoff_is_not_needed_after_window_closed():
 from nomad_perception.navigation_start import finish_initialization_teleop
 class Node:
  def get_publishers_info_by_topic(self,_):return []
 assert finish_initialization_teleop(Node())


def test_initial_teleop_missing_service_blocks_handoff():
 from types import SimpleNamespace
 from nomad_perception.navigation_start import finish_initialization_teleop
 client=SimpleNamespace(wait_for_service=lambda timeout_sec:False)
 class Node:
  destroyed=False
  def get_publishers_info_by_topic(self,_):return [SimpleNamespace(node_name='nomad_gazebo_teleop')]
  def create_client(self,*_):return client
  def get_logger(self):return SimpleNamespace(warn=lambda _:None)
  def destroy_client(self,_):self.destroyed=True
 node=Node();assert not finish_initialization_teleop(node);assert node.destroyed


def test_initial_teleop_negative_ack_blocks_handoff(monkeypatch):
 from types import SimpleNamespace
 import nomad_perception.navigation_start as startup
 future=SimpleNamespace(done=lambda:True,exception=lambda:None,result=lambda:SimpleNamespace(success=False))
 client=SimpleNamespace(wait_for_service=lambda timeout_sec:True,call_async=lambda _:future)
 class Node:
  def get_publishers_info_by_topic(self,_):return [SimpleNamespace(node_name='nomad_gazebo_teleop')]
  def create_client(self,*_):return client
  def destroy_client(self,_):pass
 monkeypatch.setattr(startup.rclpy,'spin_until_future_complete',lambda *a,**k:None)
 assert not startup.finish_initialization_teleop(Node())


def test_initial_teleop_ack_requires_publisher_exit(monkeypatch):
 from types import SimpleNamespace
 import nomad_perception.navigation_start as startup
 future=SimpleNamespace(done=lambda:True,exception=lambda:None,result=lambda:SimpleNamespace(success=True))
 client=SimpleNamespace(wait_for_service=lambda timeout_sec:True,call_async=lambda _:future)
 class Node:
  calls=0
  def get_publishers_info_by_topic(self,_):
   self.calls+=1
   return [SimpleNamespace(node_name='nomad_gazebo_teleop')] if self.calls<=2 else []
  def create_client(self,*_):return client
  def destroy_client(self,_):pass
 monkeypatch.setattr(startup.rclpy,'spin_until_future_complete',lambda *a,**k:None)
 monkeypatch.setattr(startup.rclpy,'ok',lambda:True)
 spins=[]
 monkeypatch.setattr(startup.rclpy,'spin_once',lambda *a,**k:spins.append(True))
 node=Node();assert startup.finish_initialization_teleop(node);assert spins


def test_localization_can_release_teleop_while_moving_and_map_waits():
 from nomad_perception.navigation_start import localization_ready
 latest=state();latest['pose'][0].pose.pose.orientation.w=1.
 latest['pose'][0].twist.twist.linear.x=.4
 latest.pop('perception')
 assert localization_ready(latest,1.1)
 assert not ready_to_start(latest,1.1)
 latest['vio'][0].state=ModuleStatus.WAITING
 assert not localization_ready(latest,1.1)


def test_stationary_gate_uses_physical_feedback_not_global_correction_velocity():
 latest=state();physical=Odometry();latest['motion']=(physical,1.)
 latest['pose'][0].twist.twist.linear.x=.5
 assert ready_to_start(latest,1.1,motion_key='motion')
 physical.twist.twist.linear.x=.1
 assert not ready_to_start(latest,1.1,motion_key='motion')
 physical.twist.twist.linear.x=0
 latest['motion']=(physical,0.)
 assert not ready_to_start(latest,1.1,motion_key='motion')
