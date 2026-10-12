#!/usr/bin/env bash
set -eo pipefail
terrain_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$terrain_root/.nomad/env.sh"
for arg in "$@"; do
 case "$arg" in
  --show-args|-s|--help|-h) exec ros2 launch nomad_perception live_terrain.launch.py "$arg" ;;
 esac
done
terrain_custom=false
for arg in "$@";do [[ "$arg" == world_file:=* ]] && terrain_custom=true;done
if ! $terrain_custom;then
 terrain_world=$(python3 "$terrain_root/src/nomad_gazebo/scripts/select_map.py")
 set -- "world_file:=$terrain_world" "$@"
fi
exec ros2 launch nomad_perception live_terrain.launch.py "$@"
