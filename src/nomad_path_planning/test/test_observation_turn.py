import numpy as np
from nomad_path_planning.observation_turn import turn_weights, near_path_support

def grid():
 return dict(width=30,height=30,resolution=.2,origin_x=-1.,origin_y=-3.,data=[0]*900,cell_weights=[1.]*900)

def test_turn_requires_missing_support_to_increase_cost():
 g=grid();s=np.zeros(900);a=turn_weights(g,s,(0,0,0),80);b=turn_weights(g,np.full(900,100),(0,0,0),80)
 forward=15*30+15;side=23*30+8
 assert a['cell_weights'][side]>a['cell_weights'][forward]
 assert b['cell_weights']==g['cell_weights']
 assert a['data']==g['data'] and g['cell_weights']==[1.]*900

def test_guard_rejects_missing_direct_support_and_ignores_rear():
 g=grid();path=[(x,0.,0.) for x in np.arange(0,2,.1)];s=[0]*900
 assert not near_path_support(path,g,s)[0]
 for i in range(30):s[15*30+i]=100
 assert near_path_support(path,g,s)[0]

def test_invalid_support_fails():
 import pytest
 with pytest.raises(ValueError):turn_weights(grid(),[-1]*900,(0,0,0),80)
