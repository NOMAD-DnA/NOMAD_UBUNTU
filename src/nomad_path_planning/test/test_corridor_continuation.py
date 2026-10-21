import numpy as np
from nomad_path_planning.corridor_selector import CorridorSelector, Candidate


def grid(a):
    return dict(width=a.shape[1],height=a.shape[0],resolution=.2,
                origin_x=0.,origin_y=0.,data=a.ravel().tolist())


def pocket():
    a=np.full((70,70),100)
    a[20:31,5:27]=0
    return a


def test_new_wall_invalidates_reachable_target_and_cached_path():
    s=CorridorSelector(continuation_distance=3.)
    start=(3.1,5.1,0.);target=(4.1,5.1);goal=(12.,5.)
    path=[start,(3.5,5.1,0.),(*target,0.)]
    s.accept(Candidate(target,0.,1.,1.,0.),path)
    assert s.invalidate_unreachable(grid(np.zeros((70,70),int)),start,1.,goal=goal) is None
    assert s.invalidate_unreachable(grid(pocket()),start,2.,goal=goal)=='target_continuation_blocked'
    assert s.active is None and s.active_path is None
    assert s.rejected


def test_unknown_and_map_boundary_are_not_confirmed_dead_ends():
    s=CorridorSelector(continuation_distance=3.)
    a=pocket();a[25,27]=-1
    assert not s.continuation_blocked(grid(a),(3.1,5.1,0.),(4.1,5.1),(12.,5.))
    assert not s.continuation_blocked(grid(np.zeros((30,30),int)),(3.,3.,0.),(5.1,3.),(12.,3.))


def test_side_exit_is_allowed_without_straight_line_shortcut():
    s=CorridorSelector(continuation_distance=3.)
    a=pocket();a[30:35,20:40]=0;a[20:35,35:40]=0
    assert not s.continuation_blocked(grid(a),(3.1,5.1,0.),(4.1,5.1),(12.,5.))


def test_final_goal_and_disabled_mode_keep_existing_behavior():
    a=pocket();start=(3.1,5.1,0.);target=(4.1,5.1)
    assert not CorridorSelector().continuation_blocked(grid(a),start,target,(12.,5.))
    assert not CorridorSelector(continuation_distance=3.).continuation_blocked(grid(a),start,target,target)


def test_retained_path_does_not_bypass_continuation_check():
    s=CorridorSelector(continuation_distance=3.)
    start=(3.1,5.1,0.);target=(4.1,5.1)
    s.accept(Candidate(target,0.,1.,1.,0.),[start,(*target,0.)])
    assert s.retained_path(grid(pocket()),start,(12.,5.)) is None
