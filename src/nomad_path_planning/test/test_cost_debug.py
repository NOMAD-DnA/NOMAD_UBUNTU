import copy
from types import SimpleNamespace as NS
import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from nomad_path_planning.gpp.multicost import compose
from nomad_path_planning.cost_debug import CostDebugPublisher,display_values

def test_display_preserves_unknown_and_blocked_without_soft_100():
 assert display_values([0,80,160,999,np.inf,np.inf],[0,0,0,0,100,-1],160)==[0,50,99,99,100,-1]

def test_returning_components_does_not_change_planning_weights():
 g=dict(width=3,height=2,resolution=.2,data=[0,79,100,-1,0,0])
 args=([.2,np.nan,1,np.nan,.5,.1],[.1]*6)
 baseline=compose(g,*args,cluster_weight=32)
 result,terms=compose(g,*args,cluster_weight=32,return_components=True)
 assert result==baseline
 total=sum(terms.values())
 for i in [0,1,4,5]:assert total[i]==pytest.approx(result[i])

class FakeNode:
 def __init__(self):self.sent={}
 def declare_parameter(self,k,v):return NS(value=v)
 def create_publisher(self,cls,topic,qos):
  self.sent[topic]=[]
  assert qos.depth==1
  return NS(publish=lambda m:self.sent[topic].append(m))

def test_raw_and_display_share_geometry_and_do_not_modify_inputs():
 n=FakeNode();p=CostDebugPublisher(n);m=OccupancyGrid();m.header.frame_id='odom';m.header.stamp.sec=12
 m.info.width=3;m.info.height=1;m.info.resolution=.2;m.data=[0,-1,100]
 g=dict(data=[0,100,100],cell_weights=[12.345,np.inf,np.inf]);before=copy.deepcopy(g)
 p.publish(m,g,{'slope':np.array([2.,5.,1.])},(0,0,0))
 assert g==before and list(m.data)==[0,-1,100]
 raw=n.sent['/nomad/gpp_cost/total_raw'][0];view=n.sent['/nomad/gpp_cost/total'][0]
 assert raw.header==m.header and view.info==m.info
 assert np.frombuffer(raw.data,dtype='<f4')[0]==pytest.approx(12.345)
 assert list(view.data)==[8,-1,100]
 p.publish(m,g,{},(0,0,0));assert len(n.sent['/nomad/gpp_cost/total'])==1

def test_color_scale_and_cloud_geometry():
 from nomad_path_planning.cost_debug import cost_rgb,color_cloud
 colors=cost_rgb([0,5,10,10,np.inf],[0,0,0,100,-1],10)
 assert colors.tolist()==[[0,220,30],[255,220,30],[255,0,30],[0,0,0],[125,125,125]]
 m=OccupancyGrid();m.header.frame_id='odom';m.info.width=2;m.info.height=1;m.info.resolution=.2;m.info.origin.position.x=3.;m.info.origin.position.y=4.
 cloud=color_cloud(m,[0,10],[0,0],10)
 pts=np.frombuffer(cloud.data,dtype=[('x','<f4'),('y','<f4'),('z','<f4'),('rgb','<u4')])
 assert list(pts['x'])==pytest.approx([3.1,3.3]);assert list(pts['y'])==pytest.approx([4.1,4.1]);assert int(pts['rgb'][0])==(220<<8)|30
 assert cloud.header==m.header and cloud.point_step==16
 assert list(pts['z'])==pytest.approx([-.05,-.05])
