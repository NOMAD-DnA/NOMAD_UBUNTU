#!/usr/bin/env bash
set -eo pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/.nomad/env.sh"
exec ros2 run nomad_gazebo teleop.py "$@"
