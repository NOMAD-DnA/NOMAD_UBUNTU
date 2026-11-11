#!/usr/bin/env bash
# Starts a passive ROS subscriber. This script never opens VESC USB or sends commands.
set -euo pipefail
recorder_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
if [[ -f "$recorder_dir/workspace_paths.sh" ]]; then source "$recorder_dir/workspace_paths.sh"; fi
workspace=${NOMAD_WORKSPACE:-$HOME/nomad_cpp_ws}
output_root=${NOMAD_RECORDING_ROOT:-$workspace/log/diagnostics}
mkdir -p "$output_root"
# The singleton lock belongs to this recorder and ROS domain, not the VESC device.
domain=${ROS_DOMAIN_ID:-187}
localhost_only=${ROS_LOCALHOST_ONLY:-1}
[[ $domain =~ ^[0-9]+$ ]] || { echo 'Invalid ROS_DOMAIN_ID for recording' >&2; exit 1; }
exec 7>"$output_root/recorder-domain${domain}.lock"
if ! flock -n 7; then
    echo "Recorder already running: $output_root"
    exit 0
fi
set +u
if [[ -f "$recorder_dir/ros_env.sh" ]]; then
    source "$recorder_dir/ros_env.sh"
else
    if ! command -v ros2 >/dev/null; then source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"; fi
    source "${NOMAD_VESC_WORKSPACE:-$HOME/vesc_ws}/install/setup.bash"
    source "$workspace/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID=$domain
export ROS_LOCALHOST_ONLY=$localhost_only
# Child keeps FD7 for singleton locking, but cannot inherit the GUI/device locks.
nohup python3 -u "$recorder_dir/record_control_trials.py" --workspace "$workspace" --output-root "$output_root" "$@" \
    >> "$output_root/recorder-domain${domain}.log" 2>&1 < /dev/null 8>&- 9>&- &
pid=$!
echo "$pid" > "$output_root/recorder-domain${domain}.pid"
echo "Passive recorder started ($pid): $output_root"
