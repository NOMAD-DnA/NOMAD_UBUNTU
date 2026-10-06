#!/usr/bin/env bash
# Source-tree entry point: environment -> forest -> planning -> RViz.
set -eo pipefail
planning_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
planning_workspace="$(cd -- "$planning_package_dir/../.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo 'Usage: run_all.sh [--check] [headless:=true] [rviz:=false] [vegetation:=false] [world_file:=PATH] [gpp:=NAME] [lpp:=NAME]'
    echo 'Starts NOMAD environment, Gazebo, planning and RViz in the foreground. Ctrl+C stops this run.'
    echo 'Select a map by number, or set NOMAD_MAP to a map folder name. Non-interactive default: 02_fork2_gentle.'
    echo 'An explicit world_file bypasses map selection; vegetation:=false selects the bare world.'
    echo 'GPP: dstar_lite (default), astar, weighted_astar, hybrid_astar, state_lattice, field_dstar'
    echo 'LPP: rollout (default), rpp'
    echo 'Interactive: select a map, then a planner pair by number (1-6; Enter selects 3).'
    echo 'NOMAD_PLANNER_PAIR=1..6 selects a pair without a prompt.'
    echo 'Explicit gpp/lpp/params_file arguments bypass the planner menu and NOMAD_PLANNER_PAIR.'
    echo '--check validates setup and checks for running instances without starting anything.'
    exit 0
fi

# Exactly the same setup as the existing interactive `nomad` function.
source "$planning_workspace/.nomad/env.sh"
if ! ros2 pkg prefix nomad_gazebo >/dev/null 2>&1 ||
   ! ros2 pkg prefix nomad_bringup >/dev/null 2>&1 ||
   [[ ! -f "$planning_workspace/install/nomad_bringup/share/nomad_bringup/launch/forest.launch.py" ]]; then
    echo 'Build first from nomad_ws: source .nomad/env.sh && colcon build --base-paths src --packages-up-to nomad_bringup nomad_gazebo --symlink-install' >&2
    exit 1
fi

# Serialize this wrapper, then reject existing same-domain/partition instances.
exec 9>"/tmp/nomad_all_${UID}_${ROS_DOMAIN_ID}.lock"
if ! flock -n 9; then
    echo 'NOMAD all-in-one is already running in this ROS domain.' >&2
    exit 1
fi
python3 - <<'PY'
import os
from pathlib import Path

domain = os.environ['ROS_DOMAIN_ID']
partition = os.environ['GZ_PARTITION']
for entry in Path('/proc').iterdir():
    if not entry.name.isdigit():
        continue
    try:
        args = (entry/'cmdline').read_bytes().split(b'\0')
        env = dict(item.split(b'=', 1) for item in (entry/'environ').read_bytes().split(b'\0') if b'=' in item)
    except (OSError, ValueError):
        continue
    # Match command arguments, not arbitrary shell text containing these words.
    planner = any(any(('/lib/'+p+'/').encode() in arg for p in ('nomad_path_planning','nomad_perception','nomad_vio','nomad_control')) for arg in args)
    launch = (b'launch' in args and any(p in args for p in (b'nomad_gazebo', b'nomad_path_planning', b'nomad_bringup')))
    gazebo = bool(args) and (args[0].startswith(b'gz sim') or
                             (b'sim' in args and any(arg.endswith(b'/gz') for arg in args)))
    same_domain = env.get(b'ROS_DOMAIN_ID', b'0').decode() == domain
    same_partition = env.get(b'GZ_PARTITION', b'').decode() == partition
    if ((planner or launch) and same_domain) or (gazebo and same_partition):
        raise SystemExit(f'Already running: PID {entry.name}. Stop the existing NOMAD launch with Ctrl+C first; no process was terminated.')
PY

if [[ "${1:-}" == --check ]]; then
    echo "Ready: $planning_workspace (ROS_DOMAIN_ID=$ROS_DOMAIN_ID, GZ_PARTITION=$GZ_PARTITION)"
    exit 0
fi
# Reuse the forest launcher map menu after the setup-only check.
planning_custom_world=false
planning_bare_map=false
for planning_argument in "$@"; do
    case "$planning_argument" in
        world_file:=*) planning_custom_world=true ;;
        vegetation:=*)
            case "${planning_argument#vegetation:=}" in
                False|false|0) planning_bare_map=true ;;
                *) planning_bare_map=false ;;
            esac ;;
    esac
done
if ! $planning_custom_world; then
    planning_selector_args=()
    if $planning_bare_map; then planning_selector_args+=(--bare); fi
    planning_selected_world="$(python3 "$planning_workspace/src/nomad_gazebo/scripts/select_map.py" "${planning_selector_args[@]}")"
    set -- "world_file:=$planning_selected_world" "$@"
fi
# Explicit algorithm/config arguments retain their existing precedence.
planning_explicit_algorithms=false
for planning_argument in "$@"; do
    case "$planning_argument" in
        gpp:=*|lpp:=*|params_file:=*) planning_explicit_algorithms=true ;;
    esac
done
if ! $planning_explicit_algorithms; then
    planning_selected_algorithms="$(python3 "$planning_package_dir/scripts/select_algorithms.py")"
    if [[ -n "$planning_selected_algorithms" ]]; then
        mapfile -t planning_algorithm_args <<< "$planning_selected_algorithms"
        set -- "$@" "${planning_algorithm_args[@]}"
    fi
fi
mkdir -p -- "$planning_workspace/log/gazebo-runs"
planning_run_dir="$(mktemp -d "$planning_workspace/log/gazebo-runs/planning-XXXXXXXX")"
cd -- "$planning_run_dir"
echo "Starting Gazebo + planning + RViz; Ctrl+C to stop. Logs: $planning_run_dir"
# Replace this shell in the terminal's foreground process group so ROS launch
# receives Ctrl+C directly and owns shutdown of its child processes.
# Keep fd 9 open across exec to hold the duplicate-run lock for this launch.
exec ros2 launch nomad_bringup forest.launch.py \
    topics_file:="$planning_workspace/src/nomad_bringup/config/topics.yaml" \
    modules_file:="$planning_workspace/src/nomad_bringup/config/modules.yaml" \
    vehicle_file:="$planning_workspace/src/nomad_bringup/config/vehicle.yaml" "$@"
