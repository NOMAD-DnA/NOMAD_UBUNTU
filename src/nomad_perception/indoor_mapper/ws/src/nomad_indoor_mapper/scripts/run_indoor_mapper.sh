#!/usr/bin/env bash
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
config="${INDOOR_CONFIG:-$script_dir/../config/indoor.json}"
exec ros2 run nomad_indoor_mapper nomad_indoor_mapper --indoor-config "$config" "$@"
