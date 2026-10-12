#!/usr/bin/env bash
set -eo pipefail
terrain_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$terrain_root/.nomad/env.sh"
terrain_custom=false
for arg in "$@";do [[ "$arg" == world_file:=* ]] && terrain_custom=true;done
if ! $terrain_custom;then
 terrain_world=$(python3 "$terrain_root/src/nomad_gazebo/scripts/select_map.py")
 if [[ "$terrain_world" == */02_fork2_gentle/worlds/forest.sdf ]];then
  terrain_world=$(/usr/bin/python3 "$terrain_root/src/nomad_perception/scripts/prepare_road_start.py" --world "$terrain_world" --output "/tmp/nomad-road-start-${ROS_DOMAIN_ID}.sdf")
 fi
 set -- "world_file:=$terrain_world" "$@"
fi
terrain_explicit_algorithms=false
for arg in "$@";do
 case "$arg" in gpp:=*|lpp:=*|params_file:=*) terrain_explicit_algorithms=true ;; esac
done
if ! $terrain_explicit_algorithms;then
 terrain_selected_algorithms=$(python3 "$terrain_root/src/nomad_path_planning/scripts/select_algorithms.py")
 if [[ -n "$terrain_selected_algorithms" ]];then
  mapfile -t terrain_algorithm_args <<< "$terrain_selected_algorithms"
  set -- "$@" "${terrain_algorithm_args[@]}"
 fi
fi
exec ros2 launch nomad_perception terrain_navigation.launch.py "$@"
