"""GPP feedback must not keep rebuilding an unchanged graph."""
import math
from types import MethodType, SimpleNamespace
from unittest.mock import Mock

from nomad_path_planning.dstar_lite import DStarLite
from nomad_path_planning.recovery_gpp import RecoveryGPP


def test_new_goal_bypasses_previous_retry_throttle():
    pose = lambda x: SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=0.)))
    node = SimpleNamespace(goal_pose=pose(1.), last_attempt=100., failed_since=99.,
                           feedback_replanned=True, try_plan=Mock())
    goal = pose(3.)
    RecoveryGPP.goal_callback(node, goal)
    assert node.goal_pose is goal and node.last_attempt == -math.inf
    assert node.failed_since is None and not node.feedback_replanned
    node.try_plan.assert_called_once()


def test_lpp_failure_rebuilds_once_then_uses_incremental_updates(monkeypatch):
    width = height = 10
    data = [-1] * (width * height)
    start, goal = (1, 1), (8, 8)
    planner = DStarLite(width, height, data, start, goal, unknown_penalty=1.1)
    assert planner.compute_shortest_path()

    position = lambda x, y: SimpleNamespace(x=x, y=y)
    pose = lambda x, y: SimpleNamespace(pose=SimpleNamespace(position=position(x, y)))
    info = SimpleNamespace(width=width, height=height, resolution=1.0,
                           origin=SimpleNamespace(position=position(0, 0)))
    published = []
    node = SimpleNamespace(
        costmap=SimpleNamespace(info=info, data=data),
        current_pose=pose(1.5, 1.5), goal_pose=pose(8.5, 8.5),
        planner=planner, last_map_signature=(width, height, 1.0, 0, 0),
        retry_pending=False, failed_since=0.0, feedback_replanned=False,
        last_attempt=-math.inf, unknown_penalty=1.1,
        map_signature=lambda: (width, height, 1.0, 0, 0),
        publish_path=lambda path: published.append(path),
        report=lambda status: None,
        stop_and_retry=lambda reason: None,
    )
    node.feedback_due = MethodType(RecoveryGPP.feedback_due, node)
    stops = []

    def stop_and_retry(reason):
        node.retry_pending = True
        stops.append(reason)

    node.stop_and_retry = stop_and_retry
    clock = [2.0]
    monkeypatch.setattr('nomad_path_planning.recovery_gpp.time.monotonic', lambda: clock[0])

    RecoveryGPP.try_plan(node)
    first_rebuild = node.planner
    assert first_rebuild is not planner
    assert node.feedback_replanned and len(published) == 1

    clock[0] = 2.3
    RecoveryGPP.try_plan(node)
    assert node.planner is first_rebuild
    assert len(published) == 1

    # A new lidar observation should update that planner without rebuilding it.
    node.costmap.data = data.copy()
    node.costmap.data[5 * width + 5] = 100
    clock[0] = 2.6
    RecoveryGPP.try_plan(node)
    assert node.planner is first_rebuild
    assert len(published) == 2

    # A recovered LPP followed by a fresh failure starts a new episode.
    clock[0] = 3.0
    RecoveryGPP.on_lpp(node, SimpleNamespace(data=False))
    assert node.failed_since is None and not node.feedback_replanned
    clock[0] = 3.1
    RecoveryGPP.on_lpp(node, SimpleNamespace(data=True))
    clock[0] = 4.2
    RecoveryGPP.try_plan(node)
    assert node.planner is not first_rebuild
    assert node.feedback_replanned

    # No global route remains a retry case even after feedback was consumed.
    wall = node.costmap.data.copy()
    for y in range(height):
        wall[y * width + 5] = 100
    node.costmap.data = wall
    clock[0] = 5.5
    RecoveryGPP.try_plan(node)
    assert node.retry_pending and stops[-1].startswith('GPP_WAIT: NO_PATH')
    blocked_planner = node.planner
    clock[0] = 6.7
    RecoveryGPP.try_plan(node)
    assert node.planner is not blocked_planner
    assert len(stops) == 2
