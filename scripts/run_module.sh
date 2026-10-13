#!/usr/bin/env bash
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
source "$root/.nomad/env.sh"
module=${1:-help}
if (( $# )); then shift; fi
share="$root/src/nomad_perception"
# GUI display flags are stripped before forwarding the original module arguments.
nomad_gui_display=false
nomad_gui_rviz=false
nomad_gui_map_view=false
nomad_gui_rviz_config=""
nomad_module_args=()
for nomad_arg in "$@"; do
  case "$nomad_arg" in
    gui_rviz:=*) nomad_gui_display=true; nomad_gui_rviz="${nomad_arg#*:=}" ;;
    gui_rviz_config:=*) nomad_gui_display=true; nomad_gui_rviz_config="${nomad_arg#*:=}" ;;
    gui_map_view:=*) nomad_gui_display=true; nomad_gui_map_view="${nomad_arg#*:=}" ;;
    *) nomad_module_args+=("$nomad_arg") ;;
  esac
done
if $nomad_gui_display; then
  nomad_module_json=$(python3 -c 'import json,sys; print(json.dumps(sys.argv[1:]))' "${nomad_module_args[@]}")
  exec ros2 launch nomad_bringup module_gui.launch.py module:="$module" module_args:="$nomad_module_json" rviz:="$nomad_gui_rviz" rviz_config:="$nomad_gui_rviz_config" map_view:="$nomad_gui_map_view"
fi
case "$module" in
  sim) exec "$root/scripts/run_forest.sh" publish_ground_truth_tf:=false "$@" ;;
  localization) exec ros2 launch nomad_vio fusion.launch.py manage_vio:=true auto_restart:=false output_warmup_s:=2.0 "$@" ;;
  semantic) exec "$root/scripts/run_semantic.sh" "$@" ;;
  map) exec ros2 run nomad_perception navigation_terrain --ros-args --params-file "$share/config/navigation_terrain.yaml" "$@" ;;
  planning)
    selected=$(python3 "$root/src/nomad_path_planning/scripts/select_algorithms.py")
    algorithm_args=()
    if [[ -n "$selected" ]]; then mapfile -t algorithm_args <<< "$selected"; fi
    exec ros2 launch nomad_bringup planning.launch.py modules_file:="$share/config/modules_terrain_navigation.yaml" params_file:="$share/config/terrain_planning.yaml" "${algorithm_args[@]}" "$@" ;;
  control) exec ros2 launch nomad_bringup control.launch.py modules_file:="$share/config/modules_terrain_navigation.yaml" "$@" ;;
  rviz) exec rviz2 -d "$share/rviz/terrain_navigation.rviz" --ros-args -p use_sim_time:=true "$@" ;;
  teleop) exec "$root/scripts/run_teleop.sh" "$@" ;;
  help|--help|-h) echo 'Usage: ./scripts/run_module.sh {sim|localization|semantic|map|planning|control|rviz|teleop} [ROS arguments]' ;;
  *) echo "Unknown module: $module" >&2; exit 2 ;;
esac
