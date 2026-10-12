#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$root/.nomad/env.sh"
module=${1:-help}
if (( $# )); then shift; fi
share="$root/src/nomad_perception"
case "$module" in
  sim) exec "$root/run_forest.sh" publish_ground_truth_tf:=false "$@" ;;
  localization) exec ros2 launch nomad_vio fusion.launch.py manage_vio:=true auto_restart:=false output_warmup_s:=2.0 "$@" ;;
  semantic) exec "$root/run_semantic.sh" "$@" ;;
  map) exec ros2 run nomad_perception navigation_terrain --ros-args --params-file "$share/config/navigation_terrain.yaml" "$@" ;;
  planning)
    selected=$(python3 "$root/src/nomad_path_planning/scripts/select_algorithms.py")
    algorithm_args=()
    if [[ -n "$selected" ]]; then mapfile -t algorithm_args <<< "$selected"; fi
    exec ros2 launch nomad_bringup planning.launch.py modules_file:="$share/config/modules_terrain_navigation.yaml" params_file:="$share/config/terrain_planning.yaml" "${algorithm_args[@]}" "$@" ;;
  control) exec ros2 launch nomad_bringup control.launch.py modules_file:="$share/config/modules_terrain_navigation.yaml" "$@" ;;
  rviz) exec rviz2 -d "$share/rviz/terrain_navigation.rviz" --ros-args -p use_sim_time:=true "$@" ;;
  teleop) exec "$root/run_teleop.sh" "$@" ;;
  help|--help|-h) echo 'Usage: ./run_module.sh {sim|localization|semantic|map|planning|control|rviz|teleop} [ROS arguments]' ;;
  *) echo "Unknown module: $module" >&2; exit 2 ;;
esac
