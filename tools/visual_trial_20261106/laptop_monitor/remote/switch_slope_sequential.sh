#!/usr/bin/env bash
set -eo pipefail
[[ "$(id -un)" = nvidia ]] || exit 2
exec python3 /home/nvidia/nomad_ws/tools/visual_trial_20261106/slope_control/slope_control.py sequential
