import numpy as np
from nomad_path_planning.corridor_selector import CorridorSelector,Candidate


def grid(a):
 return dict(width=a.shape[1],height=a.shape[0],resolution=.2,origin_x=0.,origin_y=0.,data=a.ravel().tolist())


def test_connected_goal_outside_radius_is_proposal_not_lock():
 s=CorridorSelector(observed_goal_lock=True,final_approach_radius=3.)
 a=np.zeros((35,55),int);start=(1.1,3.1,0.);goal=(7.1,3.1)
 s.accept(Candidate((4.1,3.1),0.,3.,1.,0.),[start,(4.1,3.1,0.)])
 c=s.observed_goal_candidate(grid(a),start,goal,1.)
 assert c and c.final and s.final_approach_goal is None
 assert s.observed_goal_candidate(grid(a),start,goal,2.) is None
 # Only validated acceptance commits the mission target.
 s.accept(c,[start,(*goal,0.)]);assert s.final_approach_goal==goal
 a[15,35]=100
 assert s.candidates(grid(a),start,goal,4.)==[]
 assert s.final_approach_goal==goal
 s.reset();assert s.final_approach_goal is None


def test_unknown_goal_or_gap_and_wall_do_not_propose_lock():
 for value in (-1,100):
  s=CorridorSelector(observed_goal_lock=True)
  a=np.zeros((35,55),int);a[:,20]=value
  assert s.observed_goal_candidate(grid(a),(1.1,3.1,0.),(7.1,3.1),1.) is None
  assert s.final_approach_goal is None
 s=CorridorSelector(observed_goal_lock=True);a=np.zeros((35,55),int);a[15,35]=-1
 assert s.observed_goal_candidate(grid(a),(1.1,3.1,0.),(7.1,3.1),1.) is None


def test_final_lock_survives_view_loss_and_failure():
 s=CorridorSelector(observed_goal_lock=True);goal=(7.1,3.1);start=(1.1,3.1,0.)
 c=Candidate(goal,0.,6.,0.,0.,True);s.accept(c,[start,(*goal,0.)])
 s.reject(goal,2.);assert not s.rejected
 a=np.full((35,55),-1);a[15,5]=0
 s.invalidate_unreachable(grid(a),start,3.,goal=goal)
 assert s.final_approach_goal==goal and s.active_path is None
 assert s.candidates(grid(a),start,goal,3.)==[]


def test_disabled_policy_preserves_legacy():
 s=CorridorSelector();a=np.zeros((35,55),int)
 assert s.observed_goal_candidate(grid(a),(1.,3.,0.),(7.,3.),0.) is None
