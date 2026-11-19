#!/usr/bin/env bash
# Start the improved legacy mapper (single process). Same /nomad output topics as
# the legacy manager and mapper2, so it refuses to run next to either of them.
# usage: run_terrain_mapper.sh RUN_CONFIG_DIR [LEGACY_CONFIG_DIR]
#   RUN_CONFIG_DIR/terrain.json  (GUI settings; package default when missing)
#   LEGACY_CONFIG_DIR            semantic.real.json, slope.json, oak_g2b_selected_20261030.json (unchanged legacy runtime files)
set -eo pipefail
run_cfg=${1:?run config dir}
legacy_cfg=${2:-$HOME/nomad_ws/tools/visual_trial_20261106/runtime-map-preview/share/nomad_costmap_cpp/config}
ws=$HOME/nomad_terrain_ws
set +u
source /opt/ros/humble/install/setup.bash
source "$HOME/nomad_ws/install/setup.bash"
source "$ws/install/setup.bash"
set -u
for other in nomad_mapper nomad_mapper2; do
  if pgrep -x "$other" >/dev/null; then echo "$other is running: refusing to publish the same /nomad map topics" >&2; exit 3; fi
done
mkdir -p "$run_cfg"
[[ -f "$run_cfg/terrain.json" ]] || cp "$ws/install/nomad_terrain_mapper/share/nomad_terrain_mapper/config/terrain.json" "$run_cfg/terrain.json"
for f in semantic.real.json slope.json oak_g2b_selected_20261030.json; do
  [[ -r "$legacy_cfg/$f" ]] || { echo "missing legacy config $legacy_cfg/$f" >&2; exit 2; }
done
exec "$ws/install/nomad_terrain_mapper/lib/nomad_terrain_mapper/nomad_terrain_mapper" \
  --semantic-config "$legacy_cfg/semantic.real.json" --slope-config "$legacy_cfg/slope.json" \
  --lidar-calibration "$legacy_cfg/oak_g2b_selected_20261030.json" --terrain-config "$run_cfg/terrain.json"
