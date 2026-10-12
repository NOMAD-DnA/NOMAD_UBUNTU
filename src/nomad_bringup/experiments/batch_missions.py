"""Map-specific mission inputs; planners still own every executable path."""
import csv
import json
import math
from route_mission import loop_waypoints

ALL_MAPS = ('01_original', '02_fork2_gentle', '03_vehicle_incline_test',
            '04_vio_flat_loop', '05_vio_repeated_forest', '06_vio_sparse_clearing',
            '07_vio_undulating_terrain', '08_vio_reverse_dead_end')


def sample_open(points, spacing=4.):
    goals = []
    distance = 0.
    for a, b in zip(points, points[1:]):
        distance += math.dist(a, b)
        if distance >= spacing:
            goals.append(list(b))
            distance = 0.
    if goals and math.dist(goals[-1], points[-1]) < spacing / 2:
        goals.pop()
    goals.append(list(points[-1]))
    return goals


def csv_points(path):
    with path.open() as stream:
        return [(float(r['x_m']), float(r['y_m'])) for r in csv.DictReader(stream)]


def mission(map_root, mid):
    folder = map_root / mid
    base = dict(pilot=False, timeout_sim_s=1200., mission='single_goal')
    if mid[:2] in ('01', '02'):
        layout = json.loads((folder/'assets/authoring/layout.json').read_text())
        return dict(base, goal=layout['waypoints']['goal'], timeout_sim_s=600.,
                    route_source='Existing layout.json goal; no intermediate waypoints')
    if mid.startswith('03'):
        config = json.loads((folder/'assets/authoring/vehicle_test.json').read_text())
        end_x = config['ramp_start_x_m'] + 2*config['ascent_horizontal_m'] + 4.
        return dict(base, goal=[end_x, config['spawn_pose'][1]], timeout_sim_s=600.,
                    route_source='Default spawn lane (5 degrees); goal 4m beyond ramp; newly defined experiment goal')
    if mid[:2] in ('04', '05', '06', '07'):
        goals, count = loop_waypoints(folder/'route.csv')
        return dict(base, goal=goals[0], waypoints=goals, waypoints_per_lap=count,
                    waypoint_radius_m=1., mission='ordered_route', laps=2,
                    route_source='route.csv, two ordered laps; design guidance, not shortest-path truth')
    route = csv_points(folder/'route.csv')
    bypass = csv_points(folder/'bypass.csv')
    branch = min(range(len(route)), key=lambda i: math.dist(route[i], bypass[0]))
    ingress = sample_open(route)
    recovery = sample_open(list(reversed(route[branch:])))
    detour = sample_open(bypass)
    goals = ingress + recovery + detour
    return dict(base, goal=goals[0], waypoints=goals, waypoint_radius_m=1.,
                mission='ordered_route', route_source='route.csv ingress, return to bypass start, bypass.csv to goal',
                stages=[dict(name='ingress', begin=0, end=len(ingress)),
                        dict(name='return_to_branch', begin=len(ingress), end=len(ingress)+len(recovery)),
                        dict(name='bypass', begin=len(ingress)+len(recovery), end=len(goals))],
                reverse_policy='Existing planner/recovery decides motion direction; reverse travel is measured, not forced by this mission')
