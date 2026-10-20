from types import SimpleNamespace
from nomad_perception.observation_support import support_grid

def test_distinct_capture_support_expires_and_requires_span():
 g=SimpleNamespace(width=3,height=3,resolution=1.,origin_x=0.,origin_y=0.)
 h={(1,1):[(1.,False),(1.3,False),(1.6,False)]}
 assert support_grid(h,g,1.6)[1,1]==100
 assert support_grid(h,g,4.)[1,1]==0
 assert support_grid({(1,1):[(1.6,True)]},g,1.6)[1,1]==0
 assert support_grid(h,g,1.6)[0,0]==0
