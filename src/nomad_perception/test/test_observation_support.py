from types import SimpleNamespace
from nomad_perception.observation_support import support_grid

def test_distinct_capture_support_expires_and_requires_span():
 g=SimpleNamespace(width=3,height=3,resolution=1.,origin_x=0.,origin_y=0.)
 h={(1,1):[(1.,False),(1.3,False),(1.6,False)]}
 assert support_grid(h,g,1.6)[1,1]==100
 assert support_grid(h,g,4.)[1,1]==100
 assert support_grid(h,g,11.6)[1,1]==100
 assert support_grid(h,g,21.6)[1,1]==50
 assert support_grid(h,g,31.6)[1,1]==0
 assert support_grid({(1,1):[(1.6,True)]},g,1.6)[1,1]==0
 assert support_grid(h,g,1.6)[0,0]==0


def test_support_quality_does_not_grow_without_new_observations():
 g=SimpleNamespace(width=3,height=3,resolution=1.,origin_x=0.,origin_y=0.)
 h={(1,1):[(1.,False),(1.6,False)]}
 assert support_grid(h,g,1.6)[1,1]==67
 assert support_grid(h,g,11.6)[1,1]==67
 assert support_grid(h,g,21.6)[1,1]==33
 assert support_grid(h,g,31.6)[1,1]==0
 # Repeated timestamps and future captures cannot fabricate support.
 assert support_grid({(1,1):[(1.,False)]*3},g,2.)[1,1]==0
 assert support_grid({(1,1):[(9.,False),(10.,False),(11.,False)]},g,1.)[1,1]==0


def test_memory_duration_validation():
 import pytest
 g=SimpleNamespace(width=1,height=1,resolution=1.,origin_x=0.,origin_y=0.)
 for hold,expire in [(30.,10.),(-1.,30.),(10.,10.)]:
  with pytest.raises(ValueError):support_grid({},g,1.,hold,expire)
