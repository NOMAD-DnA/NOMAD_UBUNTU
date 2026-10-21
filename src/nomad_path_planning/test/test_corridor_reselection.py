import numpy as np
from nomad_path_planning.corridor_selector import CorridorSelector, Candidate


def grid(a, ox=0.):
    return dict(width=a.shape[1],height=a.shape[0],resolution=.2,origin_x=ox,origin_y=0.,data=a.ravel().tolist())

START=(3.1,5.1,0.)
TARGET=(7.1,5.1)
GOAL=(15.,5.1)
PATH=[START,(*TARGET,0.)]


def selector():
    s=CorridorSelector(reselect_new_blocked=True,incremental=True,seconds=2.)
    s.accept(Candidate(TARGET,0.,4.,2.,0.),PATH)
    return s


def test_new_off_path_block_releases_all_retention_without_cooldown():
    a=np.zeros((70,100),int);s=selector()
    assert s.invalidate_unreachable(grid(a),START,0.,goal=GOAL) is None
    a[29,40]=100  # Beyond target and off its safe path, but within direction band.
    assert s.invalidate_unreachable(grid(a),START,1.,goal=GOAL)=='new_blocked_in_target_direction_reselect'
    assert s.active is None and s.active_path is None and not s.rejected
    assert s.retained_path(grid(a),START,GOAL) is None
    fresh=CorridorSelector(reselect_new_blocked=True,incremental=True,seconds=2.)
    actual=s.candidates(grid(a),START,GOAL,1.)
    expected=fresh.candidates(grid(a),START,GOAL,1.)
    assert actual==expected and actual
    assert all(np.isfinite(c.score) for c in actual)
    s.accept(Candidate(TARGET,0.,4.,2.,0.),PATH)
    assert s.invalidate_unreachable(grid(a),START,2.,goal=GOAL) is None


def test_behind_side_unknown_and_soft_changes_do_not_release():
    a=np.zeros((70,100),int);s=selector();s.invalidate_unreachable(grid(a),START,0.,goal=GOAL)
    a[25,10]=100;a[40,30]=100;a[28,40]=-1;a[27,40]=79
    assert s.invalidate_unreachable(grid(a),START,1.,goal=GOAL) is None
    a[28,40]=100
    assert s.invalidate_unreachable(grid(a),START,2.,goal=GOAL)=='new_blocked_in_target_direction_reselect'


def test_grid_origin_shift_is_not_a_new_obstacle():
    a=np.zeros((70,100),int);a[29,40]=100;s=selector()
    s.invalidate_unreachable(grid(a),START,0.,goal=GOAL)
    b=np.pad(a,((0,0),(5,0)))
    assert s.invalidate_unreachable(grid(b,-1.),START,1.,goal=GOAL) is None


def test_actual_path_block_still_uses_safety_invalidation():
    a=np.zeros((70,100),int);s=selector();s.invalidate_unreachable(grid(a),START,0.,goal=GOAL)
    a[25,25]=100
    assert s.invalidate_unreachable(grid(a),START,1.,goal=GOAL)=='remaining_path_blocked_or_unknown'
    assert s.rejected


def test_final_target_keeps_lock_and_reset_clears_history():
    a=np.zeros((70,100),int);s=selector();s.final_approach_goal=TARGET
    s.invalidate_unreachable(grid(a),START,0.,goal=TARGET)
    a[29,40]=100
    assert s.invalidate_unreachable(grid(a),START,1.,goal=TARGET) is None
    s.reset();assert s.previous_blocked is None
