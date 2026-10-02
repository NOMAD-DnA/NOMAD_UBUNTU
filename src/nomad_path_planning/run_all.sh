#!/usr/bin/env bash
# Source-tree entry point: environment -> forest -> planning -> RViz.
set -eo pipefail
planning_package_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
planning_workspace="$(cd -- "$planning_package_dir/../.." && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
    echo 'Usage: run_all.sh [--check] [headless:=true] [rviz:=false] [vegetation:=false]'
    echo 'Starts NOMAD environment, Gazebo, planning and RViz in the foreground. Ctrl+C stops this run.'
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
