import math
from common import WORKSPACE
from batch_missions import ALL_MAPS, mission


def test_existing_single_goals_and_default_lane():
    root=WORKSPACE/'src/nomad_gazebo/maps'
    for mid in ALL_MAPS[:2]:
        m=mission(root, mid)
        assert m['goal']==[23.,16.]
        assert 'waypoints' not in m
    assert mission(root,ALL_MAPS[2])['goal']==[20.,-25.]


def test_all_loop_missions_close_twice():
    root=WORKSPACE/'src/nomad_gazebo/maps'
    for mid in ALL_MAPS[3:7]:
        m=mission(root,mid);n=m['waypoints_per_lap']
        assert m['waypoints'][:n]==m['waypoints'][n:]
        assert len(m['waypoints'])==2*n
        assert math.dist(m['waypoints'][-1],[-28,0])<1e-6
        assert m['waypoints'][0][1]<0


def test_dead_end_has_distinct_return_and_bypass_stages():
    m=mission(WORKSPACE/'src/nomad_gazebo/maps',ALL_MAPS[-1]);w=m['waypoints']
    ingress, recovery, bypass=m['stages']
    assert w[ingress['end']-1]==[23.,0.]
    assert w[recovery['end']-1]==[-14.,0.]
    assert w[-1]==[34.,0.]
    assert all(b[0]<a[0] for a,b in zip(w[recovery['begin']-1:recovery['end']-1], w[recovery['begin']:recovery['end']]))
    assert any(y>5 for x,y in w[bypass['begin']:])
