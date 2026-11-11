#!/usr/bin/env bash
# Resolve deployment paths without starting ROS nodes or changing vehicle state.
_nomad_paths_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
_nomad_package_dir=$(dirname -- "$_nomad_paths_dir")
if [[ -z ${NOMAD_WORKSPACE:-} ]]; then
    if [[ $(basename -- "$(dirname -- "$_nomad_package_dir")") == src ]]; then
        NOMAD_WORKSPACE=$(dirname -- "$(dirname -- "$_nomad_package_dir")")
    else
        NOMAD_WORKSPACE="$HOME/nomad_cpp_ws"
    fi
fi
NOMAD_VESC_WORKSPACE=${NOMAD_VESC_WORKSPACE:-$HOME/vesc_ws}
export NOMAD_WORKSPACE NOMAD_VESC_WORKSPACE
