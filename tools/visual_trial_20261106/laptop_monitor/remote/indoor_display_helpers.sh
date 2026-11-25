#!/usr/bin/env bash
# Display-only camera/inference chain. No mapper inputs or transforms are changed.
nomad_indoor_display_check() {
  local ws="${NOMAD_DISPLAY_WS:-$HOME/nomad_ws}"
  local root="$ws/tools/visual_trial_20261106"
  local model=/mnt/ssd/nomad/segformer_indoor5_20261124
  for f in "$ws/semantic.sh" "$ws/tools/runtime/module_env.bash" \
    "$root/launch/zed_camera.preview.launch.py" "$root/config/indoor_display_20261125.yaml" \
    "$model/segformer_ade20k_indoor5_640x360_fp16.engine"; do
    [[ -r "$f" ]] || { echo "INDOOR_DISPLAY_MISSING=$f"; return 1; }
  done
  [[ -x "$model/runtime/build/nomad_segformer_indoor5" ]] || return 1
  grep -aFq oak_color_jpeg "$root/build_monitor_probe/nomad_monitor_probe" || { echo INDOOR_DISPLAY_COLLECTOR_UPDATE_REQUIRED; return 1; }
  python3 "$model/runtime/indoor_postprocess_settings.py" ros-args >/dev/null || return 1
  python3 - "$root/config/indoor_display_20261125.yaml" <<'DISPLAY_YAML'
import sys,yaml
v=yaml.safe_load(open(sys.argv[1]))['/**']['ros__parameters']
assert v['depth']['depth_mode']=='NONE'
assert v['pos_tracking']['pos_tracking_enabled'] is False
assert v['pos_tracking']['publish_tf'] is False and v['pos_tracking']['publish_map_tf'] is False
assert abs(float(v['general']['pub_frame_rate'])-3.3333333333)<1e-6
DISPLAY_YAML
}
nomad_indoor_display_start() {
  local run="$1" ws="${NOMAD_DISPLAY_WS:-$HOME/nomad_ws}"
  local root="$ws/tools/visual_trial_20261106"
  cp -- "$root/config/indoor_display_20261125.yaml" "$run/zed_preview.yaml"
  setsid env ROS_LOG_DIR="$run/ros" bash -c '
    source "$1/tools/runtime/module_env.bash"
    nomad_run zed ros2 launch "$1/tools/visual_trial_20261106/launch/zed_camera.preview.launch.py" \
      camera_model:=zed2 camera_name:=zed node_name:=zed_node \
      publish_tf:=false publish_map_tf:=false ros_params_override_path:="$2"
  ' indoor-zed-display "$ws" "$run/zed_preview.yaml" 9>&- </dev/null >"$run/zed.log" 2>&1 &
  printf '%s\n' "$!" > "$run/zed.pid"
  setsid env ROS_LOG_DIR="$run/ros" bash "$ws/semantic.sh" --indoor -p rate_hz:=1.0 \
    9>&- </dev/null >"$run/semantic.log" 2>&1 &
  printf '%s\n' "$!" > "$run/semantic.pid"
  printf '{"model":"indoor","display_only":true,"started_with_trial":true}\n' > "$run/semantic_model.json"
  printf '{"oak_rgb":true,"zed_indoor5":true,"display_only":true}\n' > "$run/indoor_display.json"
}
