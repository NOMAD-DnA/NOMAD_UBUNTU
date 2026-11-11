#!/usr/bin/env bash
# Source this file to load the existing ROS environment and available overlays.
_nomad_env_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$_nomad_env_dir/workspace_paths.sh" || return 1

_nomad_source_environment() {
    local setup candidate
    if ! command -v ros2 >/dev/null 2>&1; then
        setup=
        for candidate in "${ROS_DISTRO:-}" humble jazzy; do
            if [[ -n $candidate && -f /opt/ros/$candidate/setup.bash ]]; then
                setup="/opt/ros/$candidate/setup.bash"
                break
            fi
        done
        [[ -n $setup ]] || { echo 'Source an installed ROS environment first.' >&2; return 1; }
        source "$setup" || return 1
    fi
    for setup in "$NOMAD_VESC_WORKSPACE/install/setup.bash" "$NOMAD_WORKSPACE/install/setup.bash"; do
        [[ ! -f $setup ]] || source "$setup" || return 1
    done
}

# ROS setup scripts may reference unset variables; restore the caller's nounset.
_nomad_env_nounset=false
if [[ $- == *u* ]]; then _nomad_env_nounset=true; set +u; fi
_nomad_env_result=0
_nomad_source_environment || _nomad_env_result=$?
if $_nomad_env_nounset; then set -u; fi
return "$_nomad_env_result" 2>/dev/null || exit "$_nomad_env_result"
