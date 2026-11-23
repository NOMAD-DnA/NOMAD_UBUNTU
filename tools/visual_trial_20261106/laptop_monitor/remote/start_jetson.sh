#!/usr/bin/env bash
# Existing final preview startup. Run on the verified Jetson after LAN setup.
# Reuses existing binaries/private calibration. No GUI, control, or Planning.
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
# 2026-11-12: OAK cloud source chosen before Start (GUI trial settings).
cloud_source=${NOMAD_CLOUD_SOURCE:-opencv}
case "$cloud_source" in opencv|device|opencv3way) ;; *) echo "Invalid NOMAD_CLOUD_SOURCE: $cloud_source"; exit 2 ;; esac
# 2026-11-15: "opencv3way" = OpenCV source with SGBM 3WAY, 128 disparities and left padding (patched
# external_stereo_cloud parameters sgbm_mode/pad_left). The runtime OpenCV<->OAK switch keeps working ("opencv").
cloud_initial=$cloud_source
cloud_extra=()
if [[ "$cloud_source" == opencv3way ]]; then
  cloud_initial=opencv
  cloud_extra=(-p sgbm_mode:=3way -p pad_left:=true -p num_disparities:=128)
fi
# 2026-11-13: OAK depth cloud rate from the GUI trial settings (was fixed 1 Hz). Older GUIs send nothing -> 1.
oak_cloud_hz=${NOMAD_OAK_CLOUD_HZ:-1}
case "$oak_cloud_hz" in 1|2|3|5) ;; *) echo "Invalid NOMAD_OAK_CLOUD_HZ: $oak_cloud_hz"; exit 2 ;; esac
# 2026-11-13: start the chosen SegFormer directly (b2 or indoor) instead of b2 + a later switch.
# FCN still starts as b2 and is switched by the GUI (separate binary/adapter chain).
semantic_model=${NOMAD_SEMANTIC_MODEL:-b2}
case "$semantic_model" in b2|indoor|fcn) ;; *) echo "Invalid NOMAD_SEMANTIC_MODEL: $semantic_model"; exit 2 ;; esac
start_token=${NOMAD_START_TOKEN:-}
# Embedded helper: SSH supplies this script on stdin, with no installed sibling.
nomad_write_planning_output_profile() (
  set -e
  output_file=$1
  wired_local=$2
  wired_peer=$3
  wlan_local=$(ip -4 -o addr show dev wlan0 scope global | awk 'NR == 1 {split($4,a,"/"); print a[1]}')
  planning_peer=${NOMAD_PLANNING_PEER:-}
  if [[ -z "$planning_peer" ]]; then
    if [[ -r /etc/nomad/network-peers.json ]]; then
      # Laptop hotspot networks carry their own fixed Pi4 address; otherwise
      # the classroom (top-level) address. Chosen from this board's wlan0.
      planning_peer=$(python3 -c 'import ipaddress,json,sys
v=json.load(open("/etc/nomad/network-peers.json"))
peer=v.get("pi4")
for n in v.get("networks",[]):
    try:
        if sys.argv[1] and ipaddress.IPv4Address(sys.argv[1]) in ipaddress.ip_network(n["cidr"]):
            peer=n["pi4"]
    except (ValueError,KeyError,TypeError):
        pass
print(ipaddress.IPv4Address(peer))' "$wlan_local") || return 2
    else
      planning_peer=$(getent ahostsv4 lpp.local | awk 'NR==1 {print $1}' || true)
      planning_peer=${planning_peer:-192.168.3.142}
    fi
  fi
  for address in "$wired_local" "$wired_peer" "$planning_peer" "$wlan_local"; do
    if ! awk -v address="$address" 'BEGIN {
      if (split(address,a,".") != 4) exit 1;
      for (i=1; i<=4; i++) if (a[i] !~ /^[0-9]+$/ || a[i]+0 > 255) exit 1;
    }'; then
      echo 'PLANNING_DDS_ERROR: wired, planning peer and wlan0 addresses must be IPv4' >&2
      exit 2
    fi
  done
  umask 077
  cat > "$output_file" <<XML
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
 <transport_descriptors><transport_descriptor>
  <transport_id>planning_output_udp</transport_id><type>UDPv4</type>
  <maxInitialPeersRange>32</maxInitialPeersRange>
  <interfaceWhiteList><address>127.0.0.1</address><address>$wired_local</address><address>$wlan_local</address></interfaceWhiteList>
 </transport_descriptor></transport_descriptors>
 <participant profile_name="planning_output_default" is_default_profile="true"><rtps>
  <builtin>
   <metatrafficUnicastLocatorList><locator/></metatrafficUnicastLocatorList>
   <initialPeersList>
    <locator><udpv4><address>127.0.0.1</address></udpv4></locator>
    <locator><udpv4><address>$wired_peer</address></udpv4></locator>
    <locator><udpv4><address>$planning_peer</address></udpv4></locator>
   </initialPeersList>
  </builtin>
  <useBuiltinTransports>false</useBuiltinTransports>
  <userTransports><transport_id>planning_output_udp</transport_id></userTransports>
 </rtps></participant>
</profiles>
XML
)

[[ "$(id -un)" = nvidia ]] || { echo "WRONG_BOARD: Jetson user must be nvidia" >&2; exit 1; }
ws=/home/nvidia/nomad_ws
root="$ws/tools/visual_trial_20261106"
prefix="$root/runtime-map-preview"
launch="$root/launch/zed_camera.preview.launch.py"
run=''

fail() { printf 'RESUME_FAILED: %s\n' "$*" >&2; exit 1; }
trap 'rc=$?; printf "RESUME_FAILED exit=%s run=%s; existing/new processes were not killed\n" "$rc" "${run:-not-created}" >&2' ERR

# GUI class-boundary px / initial safe area, plus terrain moving-region controls (null = keep the
# board file value). Applied to per-run copies only; legacy manager files are never changed.
#   terrain  laptop '기존 매퍼 개선 설정' (terrain_settings.py): OVERRIDES = a terrain.json, section
#            "gui_overrides" (GUI-only; the terrain binary rejects unknown keys, so strip removes it)
#   mapper3  laptop '새 매퍼3 v3 설정' (mapper_settings.py): OVERRIDES =
#            ~/nomad_settings/mapper_overrides/current.json, section "mapper3"
#   check KIND OVERRIDES          exit 0 = has values, 1 = nothing to apply, 2 = invalid file
#   apply KIND OVERRIDES TARGET   rewrite TARGET (other keys kept) only when there are values
#   show  KIND TARGET             print the effective values only
#   strip terrain FILE            remove "gui_overrides" from FILE (a per-run copy) when present
# Each prints "MAPPER_SETTINGS kind=... <key>=<value> ... overridden=<keys|none>" (apply/show).
# Terrain also prints MOVING_FREE_SETTINGS. Optional check target validates merged geometry without writing.
mapper_overrides="$HOME/nomad_settings/mapper_overrides/current.json"
# BEGIN_MAPPER_OVERRIDE_PY
nomad_mapper_override_py=$(cat <<'NOMAD_MAPPER_PY'
import json, math, os, sys
SPEC = {"terrain": (("boundary_margin_px", ("boundary_margin_px",), "px"),
                    ("startup_radius_m", ("startup_free_region", "radius_m"), "m"),
                    ("moving_front_enabled", ("moving_front_free_region", "enabled"), "choice"),
                    ("moving_front_radius_m", ("moving_front_free_region", "radius_m"), "radius"),
                    ("moving_rear_length_m", ("moving_front_free_region", "rear_length_m"), "m"),
                    ("moving_rear_half_width_m", ("moving_front_free_region", "rear_half_width_m"), "m")),
        "mapper3": (("erode_px", ("sem_fit", "erode_px"), "px"),
                    ("start_free_radius_m", ("lidar", "start_free_radius_m"), "m"))}
SECTION = {"terrain": "gui_overrides", "mapper3": "mapper3"}
def die(message):
    print("MAPPER_OVERRIDE_ERROR " + message)
    sys.exit(2)
def indent_of(text):
    for line in text.splitlines()[1:]:
        if line.strip():
            return max(1, len(line) - len(line.lstrip(" ")))
    return 2
mode, kind = sys.argv[1], sys.argv[2]
if kind not in SPEC or mode not in ("check", "apply", "show", "strip") or (mode == "strip" and kind != "terrain"):
    die("usage")
if mode == "strip":
    with open(sys.argv[3], encoding="utf-8") as handle:
        text = handle.read()
    root = json.loads(text)
    if isinstance(root, dict) and SECTION[kind] in root:
        root.pop(SECTION[kind])
        temporary = sys.argv[3] + ".tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            handle.write(json.dumps(root, ensure_ascii=False, indent=indent_of(text)) + "\n")
        os.replace(temporary, sys.argv[3])
    sys.exit(0)
values = {}
if mode != "show":
    try:
        with open(sys.argv[3], encoding="utf-8") as handle:
            data = json.load(handle)
    except FileNotFoundError:
        data = {}
    except (OSError, ValueError) as exc:
        die("cannot read %s: %s" % (sys.argv[3], exc))
    section = data.get(SECTION[kind]) if isinstance(data, dict) else None
    if section is None:
        section = {}
    if not isinstance(section, dict):
        die(kind + " section is not an object")
    for key, _path, unit in SPEC[kind]:
        value = section.get(key)
        if value is None:
            continue
        if unit == "choice":
            if value == "보드값":
                continue
            if value not in ("켬", "끔"):
                die(key + " must be 보드값, 켬 or 끔")
            values[key] = value == "켬"
            continue
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            die(key + " is not a number")
        if unit == "px":
            if value != int(value) or not 0 <= value <= 20:
                die(key + " must be an integer 0..20")
            value = int(value)
        else:
            if not (0.001 if unit == "radius" else 0) <= value <= 5:
                die(key + (" must be 0.001..5 m" if unit == "radius" else " must be 0..5 m"))
            value = float(value)
        values[key] = value
    # 2026-11-16 '경사 점수 계수' k (slope_k.py): mapper3 only; null or 4 = board file (nothing to apply).
    slope_k = section.get("slope_k") if kind == "mapper3" else None
    if slope_k is not None:
        if isinstance(slope_k, bool) or not isinstance(slope_k, (int, float)) or not math.isfinite(slope_k) \
                or not 1.78 <= slope_k <= 16:
            die("slope_k must be 1.78..16")
        if abs(slope_k - 4.0) > 1e-9:
            values["slope_k"] = float(slope_k)
# Validate the merged dimensions against the same limits as MovingFrontFreeRegion.
# A check with a target validates before any sensors start, without writing the file.
moving_values = {k: v for k, v in values.items() if k.startswith("moving_")}
def validate_moving(root):
    if not moving_values:
        return
    options = dict(root.get("moving_front_free_region", {}))
    for key, path, _unit in SPEC["terrain"]:
        if key in moving_values:
            options[path[-1]] = values[key]
    radius = options.get("radius_m", 0.5)
    rear = options.get("rear_length_m", 0.0)
    width = options.get("rear_half_width_m", 0.0)
    front = options.get("driven_front_m", 0.025)
    def number(v):
        return not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v)
    if not number(radius) or not 0 < radius <= 5:
        die("moving front radius must be in (0,5] m")
    if any(not number(v) or not 0 <= v <= radius for v in (rear, width)):
        die("moving rear length and half width must be within [0,radius]")
    if options.get("driven_path_memory", False) and (
            rear <= 0 or width <= 0 or not number(front) or not 0 <= front <= radius):
        die("driven path memory requires positive rear dimensions and driven_front_m <= radius")
if mode == "check":
    if len(sys.argv) > 4 and moving_values:
        with open(sys.argv[4], encoding="utf-8") as handle:
            validate_moving(json.load(handle))
    sys.exit(0 if values else 1)
target = sys.argv[4] if mode == "apply" else sys.argv[3]
with open(target, encoding="utf-8") as handle:
    text = handle.read()
root = json.loads(text)
indent = indent_of(text)
validate_moving(root)
shown = []
moving_shown = []
for key, path, _unit in SPEC[kind]:
    node = root
    for part in path[:-1]:
        if kind == "terrain":
            if isinstance(node, dict):
                node = node.get(part, {}) if key.startswith("moving_") and key not in values else node.get(part)
            else:
                node = None
        else:
            node = node.setdefault(part, {}) if key in values else node.get(part, {})
        if not isinstance(node, dict):
            die("%s: %s is missing" % (target, ".".join(path)))
    if key in values:
        if kind == "terrain" and not key.startswith("moving_") and path[-1] not in node:
            die("%s: %s is missing" % (target, ".".join(path)))
        node[path[-1]] = values[key]
    (moving_shown if key.startswith("moving_") else shown).append("%s=%s" % (key, node.get(path[-1], "-")))
slope_line = None
if "slope_k" in values:
    # mapper3 piecewise slope cost: break angles x 4/k (code defaults 8/20/25 deg when absent)
    cost = root.setdefault("cost", {})
    if not isinstance(cost, dict):
        die("%s: cost is not an object" % target)
    scaled = []
    for key, default in (("slope_free_deg", 8.0), ("slope_mid_deg", 20.0), ("slope_high_deg", 25.0)):
        base = cost.get(key, default)
        if isinstance(base, bool) or not isinstance(base, (int, float)) or not math.isfinite(base):
            die("%s: cost.%s is not a number" % (target, key))
        cost[key] = round(float(base) * 4.0 / values["slope_k"], 4)
        scaled.append("%s=%g" % (key, cost[key]))
    slope_line = "MAPPER_SLOPE kind=mapper3 slope_k=%g %s" % (values["slope_k"], " ".join(scaled))
if values:
    temporary = target + ".tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        handle.write(json.dumps(root, ensure_ascii=False, indent=indent) + "\n")
    os.replace(temporary, target)
print("MAPPER_SETTINGS kind=%s %s overridden=%s" % (kind, " ".join(shown), ",".join(values) or "none"))
if moving_shown:
    print("MOVING_FREE_SETTINGS " + " ".join(moving_shown))
if slope_line:
    print(slope_line)
NOMAD_MAPPER_PY
)
# END_MAPPER_OVERRIDE_PY
nomad_mapper_override() { python3 -I -c "$nomad_mapper_override_py" "$@"; }
# 2026-11-16: terrain '경사 점수 계수' (laptop terrain_settings.py writes slope_score_per_deg/slope_block_deg
# to terrain.json only when k != 4). Terrain binaries built before the terrain_slope_k patch reject unknown
# terrain.json keys, so: show FILE prints the values; strip FILE removes both keys from that (per-run) file.
# BEGIN_TERRAIN_SLOPE_PY
nomad_terrain_slope_py=$(cat <<'NOMAD_TERRAIN_SLOPE_PY'
import json, os, sys
KEYS = ("slope_score_per_deg", "slope_block_deg")
mode, target = sys.argv[1], sys.argv[2]
with open(target, encoding="utf-8") as handle:
    text = handle.read()
root = json.loads(text)
if not isinstance(root, dict) or not any(key in root for key in KEYS):
    sys.exit(0)
if mode == "show":
    k = root.get(KEYS[0], 4.0)
    print("MAPPER_SLOPE kind=terrain slope_k=%g slope_block_deg=%g" % (k, root.get(KEYS[1], 80.0 / k)))
    sys.exit(0)
if mode != "strip":
    sys.exit(2)
indent = 2
for line in text.splitlines()[1:]:
    if line.strip():
        indent = max(1, len(line) - len(line.lstrip(" ")))
        break
for key in KEYS:
    root.pop(key, None)
temporary = target + ".tmp"
with open(temporary, "w", encoding="utf-8") as handle:
    handle.write(json.dumps(root, ensure_ascii=False, indent=indent) + "\n")
os.replace(temporary, target)
print("MAPPER_SLOPE kind=terrain status=unsupported")
NOMAD_TERRAIN_SLOPE_PY
)
# END_TERRAIN_SLOPE_PY
nomad_terrain_slope() { python3 -I -c "$nomad_terrain_slope_py" "$@"; }

[[ -d "$root" && -d "$prefix" ]] || fail 'Existing external tools/private runtime missing'
# Board-owned time readiness (2026-11-11): the installed checker decides.
if [[ -x /usr/local/lib/nomad-network/time_agent.py ]]; then
  nomad_time=$(python3 /usr/local/lib/nomad-network/time_agent.py --check || true)
  [[ "$nomad_time" == *'"ok": true'* ]] || { printf 'TIME_NOT_READY: %s\n' "${nomad_time#NOMAD_TIME_RESULT=}" >&2; exit 1; }
fi
exec 9>/tmp/nomad_resume_jetson_final_preview.lock
flock -n 9 || fail 'Another final-preview startup is active'

# Fail before starting anything if a required module already exists. Match
# executable/module tokens, not grep command lines. Never kill a prior module.
duplicate=0
for cmdline in /proc/[0-9]*/cmdline; do
  [[ -r "$cmdline" ]] || continue
  args=()
  mapfile -d '' -t args < "$cmdline" 2>/dev/null || continue
  [[ ${#args[@]} -gt 0 ]] || continue
  for arg in "${args[@]}"; do
    token=${arg##*/}
    case "$token" in
      oak_publisher|run_oak_cpp.sh|run_zed_sensors.sh|zed_camera.launch.py|zed_camera.preview.launch.py|ydlidar_ros2_driver_node|nomad_external_frame_alias|nomad_semantic_id_adapter|nomad_fcn_preview|external_stereo_cloud|nomad_runtime_manager|nomad_mapper|nomad_mapper2|nomad_mapper3|nomad_terrain_mapper|nomad_monitor|semantic.sh|nomad_perception.modules.semantic)
        printf 'DUPLICATE_MODULE pid=%s token=%s\n' "${cmdline#/proc/}" "$token" >&2
        duplicate=1
        break
        ;;
      component_container|component_container_isolated)
        if [[ " ${args[*]} " == *zed_container* ]]; then
          printf 'DUPLICATE_ZED_CONTAINER pid=%s\n' "${cmdline#/proc/}" >&2
          duplicate=1
          break
        fi
        ;;
    esac
  done
done
[[ "$duplicate" -eq 0 ]] || fail 'Required module already running; parent must inspect/stop it first'

for file in \
  "$launch" \
  "$root/slope_control/slope_control.py" \
  "$ws/scripts/run_oak_cpp.sh" \
  "$ws/tools/runtime/module_env.bash" \
  "$ws/semantic.sh" \
  "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh" \
  "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  "$ws/config/oak_fastdds_wired.xml" \
  "$root/config/ade20k.preview.json" \
  "$root/config/factory_eeprom.json" \
  "$root/config/oak_basalt_calib.json" \
  "$prefix/runtime_manifest.json" \
  "$prefix/share/nomad_costmap_cpp/config/g2b.yaml" \
  "$prefix/share/nomad_costmap_cpp/config/semantic.real.json" \
  "$prefix/share/nomad_costmap_cpp/config/oak_g2b_selected_20261030.json"; do
  [[ -f "$file" ]] || fail "Required existing file missing: $file"
done
for binary in \
  "$root/build_external_frame_alias/nomad_external_frame_alias" \
  "$root/build_external_cloud/external_stereo_cloud" \
  "$root/build_external_cloud/cloud_source_control" \
  "$prefix/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" \
  "$prefix/lib/nomad_costmap_cpp/nomad_runtime_manager"; do
  [[ -x "$binary" ]] || fail "Existing executable missing: $binary"
done
# 2026-11-15: an unpatched cloud would silently ignore sgbm_mode and run 5-direction SGBM; refuse instead.
if [[ "$cloud_source" == opencv3way ]] && ! grep -aq 'sgbm_mode' "$root/build_external_cloud/external_stereo_cloud"; then
  fail 'external_stereo_cloud has no SGBM 3WAY option (rebuild with the 2026-11-15 sgbm_mode patch)'
fi
for binary in "$ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher" "$root/build_external_cloud/external_stereo_cloud"; do
  if [[ "$binary" = "$ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher" ]]; then
    # Depth capabilities remain stable when an IMU update changes the build ID.
    for feature in 'enable_device_depth' '/oak/device_depth/image_rect' '/oak/device_depth/camera_info'; do
      grep -aFq "$feature" "$binary" || { echo "OAK_CLOUD_FEATURE_MISSING: $binary" >&2; exit 1; }
    done
  else
    grep -aFq 'oak-cloud-select-20261110-1' "$binary" || { echo "OAK_CLOUD_FEATURE_MISSING: $binary" >&2; exit 1; }
  fi
done
# The external copy must already contain the approved single CameraInfo route.
grep -Fq "('/zed/zed_node/left/camera_info', '/nomad_preview/zed/raw_camera_info')" "$launch" \
  || fail 'Approved CameraInfo route is absent from external launch copy'

# Preserve the previous 2Hz/3.333333Hz YAML; only the new run's RGB publish rate changes.
zed_config=${NOMAD_ZED_PREVIEW_YAML:-}
if [[ -z "$zed_config" && -r /tmp/nomad_integrated_jetson_run_dir ]]; then
  previous_run=$(cat /tmp/nomad_integrated_jetson_run_dir)
  [[ ! -f "$previous_run/zed_preview.yaml" ]] || zed_config="$previous_run/zed_preview.yaml"
fi
if [[ -z "$zed_config" ]]; then
  shopt -s nullglob
  # 2026-11-13: after a Jetson reboot (/tmp pointer gone) use the newest saved trial copy;
  # the 2026-11-06 folders no longer exist. Names sort by start time; rate check below still applies.
  previous_configs=("$ws"/log/visual_trial_*/zed_preview.yaml)
  shopt -u nullglob
  [[ ${#previous_configs[@]} -eq 0 ]] || zed_config=${previous_configs[${#previous_configs[@]}-1]}
fi
[[ -n "$zed_config" && -f "$zed_config" ]] || fail 'Previous copied ZED YAML missing; set NOMAD_ZED_PREVIEW_YAML explicitly'
awk '/^[[:space:]]*pub_frame_rate[[:space:]]*:/ {rate=$2+0; delta=rate-3.3333333333; if (rate == 2 || (delta > -0.000001 && delta < 0.000001)) ok=1; else bad=1} END {exit (!ok || bad)}' "$zed_config" \
  || fail 'Existing copied ZED YAML must specify RGB pub_frame_rate=2 or approximately 3.333333'

port=${NOMAD_LIDAR_PORT:-/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0}
[[ -c "$port" && -r "$port" && -w "$port" ]] || fail "LiDAR port absent/inaccessible: $port"
if command -v fuser >/dev/null 2>&1 && fuser "$port" >/dev/null 2>&1; then
  fail 'LiDAR serial port is already in use'
fi

source /opt/ros/humble/install/setup.bash
source "$ws/install/setup.bash"
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml" RMW_FASTRTPS_USE_QOS_FROM_XML=0
export LD_LIBRARY_PATH="$prefix/lib:/opt/ros/humble/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1

# 2026-11-16: check the GUI class-boundary / initial-area values before any sensor starts (apply happens
# at the mapper step): terrain = active_config/terrain.json "gui_overrides", mapper3 = mapper_overrides file.
case "${NOMAD_MAPPER:-legacy}" in
  terrain)
    override_rc=0
    nomad_mapper_override check terrain "$HOME/nomad_terrain_ws/active_config/terrain.json" "$prefix/share/nomad_costmap_cpp/config/semantic.real.json" || override_rc=$?
    [[ "$override_rc" -le 1 ]] || fail 'Invalid GUI values (gui_overrides) in ~/nomad_terrain_ws/active_config/terrain.json; sensors were not started'
    ;;
  mapper3)
    override_rc=0
    nomad_mapper_override check mapper3 "$mapper_overrides" >/dev/null || override_rc=$?
    [[ "$override_rc" -le 1 ]] || fail 'Invalid GUI mapper settings ~/nomad_settings/mapper_overrides/current.json; sensors were not started'
    ;;
esac
run="$ws/log/visual_trial_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
mkdir "$run"
mkdir -p "$run/ros" "$run/oak/ros"
planning_output_profile="$run/planning_output_fastdds.xml"
nomad_write_planning_output_profile "$planning_output_profile" 10.77.0.1 10.77.0.2
cp -- "$zed_config" "$run/zed_preview.yaml"
sed -i -E '/^[[:space:]]*pub_frame_rate[[:space:]]*:/ s/:[[:space:]]*[^[:space:]#]+/: 3.3333333333/' "$run/zed_preview.yaml"
printf '%s\n' "$run" > /tmp/nomad_integrated_jetson_run_dir
# Lets the GUI tell this run's OAK readiness from a previous run's (start_token is random per Start).
printf '%s\n' "$start_token" > "$run/start_token"
printf '%s\n' "$run/oak" > /tmp/nomad_oak_cpp_run_dir
printf '%s\n' "$zed_config" > "$run/zed_preview_source"
printf '%s\n' "$prefix" > "$run/private_runtime_path"
printf '%s\n' "$port" > "$run/lidar_port"

start_module() {
  name=$1
  shift
  setsid env ROS_LOG_DIR="$run/ros" "$@" 9>&- </dev/null > "$run/$name.log" 2>&1 &
  printf '%s\n' "$!" > "$run/$name.pid"
}
check_module() {
  name=$1
  pid=$(cat "$run/$name.pid")
  if ! kill -0 "$pid" 2>/dev/null; then
    tail -n 6 "$run/$name.log" >&2 || true
    fail "$name exited; run logs/PIDs preserved for parent cleanup"
  fi
  printf '%s RUNNING pid=%s\n' "$name" "$pid"
}

# Sensor stage: OAK 10Hz/IMU100 (oak_cpp.yaml), copied ZED RGB every 0.3s, native G2B5Hz.
# Front color video (2026-11-11): the OAK JPEG-encodes CAM_A 640x360 at 10 Hz on
# the camera itself -> /oak/color/image/compressed -> Pi5 front_recorder saves it.
# Mono/IMU/depth graph unchanged. Remove the three color -p lines to switch it off.
# 2026-11-16: GUI 모듈 세부 설정 (~/nomad_settings/<module>/overrides.yaml, changed keys only).
# Copied into the run folder and added as a later --params-file (the -p values here still win).
# Without saved changes nothing is copied and every command line is unchanged.
nomad_module_override() {
  local src="$HOME/nomad_settings/$1/overrides.yaml"
  [[ -f "$src" && ! -L "$src" ]] && grep -q 'ros__parameters:' "$src" || return 1
  if ! grep -qxF "$2:" "$src"; then echo "MODULE_SETTINGS invalid=$1"; return 1; fi
  cp -- "$src" "$3" && echo "MODULE_SETTINGS applied=$1"
}
oak_args=()
if nomad_module_override oak /oak "$run/oak_overrides.yaml"; then oak_args=(--params-file "$run/oak_overrides.yaml"); fi
cloud_args=()
if nomad_module_override cloud nomad_external_stereo_cloud "$run/cloud_overrides.yaml"; then cloud_args=(--params-file "$run/cloud_overrides.yaml"); fi
# Perception rate: -p rate_hz:=1.0 as before; with GUI settings a generated file (rate_hz 1.0) + the override.
semantic_args=(-p rate_hz:=1.0)
if nomad_module_override perception '/**' "$run/perception_overrides.yaml"; then
  printf '/**:\n  ros__parameters:\n    rate_hz: 1.0\n' > "$run/perception_params.yaml"
  semantic_args=(--params-file "$run/perception_params.yaml" --params-file "$run/perception_overrides.yaml")
fi
setsid env ROS_LOG_DIR="$run/oak/ros" bash "$ws/scripts/run_oak_cpp.sh" \
  -p enable_device_depth:=true -p device_depth_output_hz:="$oak_cloud_hz.0" \
  -p enable_color_jpeg:=true -p color_fps:=10.0 -p color_jpeg_quality:=80 \
  -p rectified_calibration_json:="$root/config/oak_basalt_calib.json" \
  -p factory_calibration_json:="$root/config/factory_eeprom.json" "${oak_args[@]}" 9>&- </dev/null > "$run/oak/oak.log" 2>&1 &
printf '%s\n' "$!" > "$run/oak/oak.pid"
printf '%s\n' "$!" > "$run/oak.pid"
ln -s oak/oak.log "$run/oak.log"
start_module zed bash -c '
  source /home/nvidia/nomad_ws/tools/runtime/module_env.bash
  nomad_run zed ros2 launch /home/nvidia/nomad_ws/tools/visual_trial_20261106/launch/zed_camera.preview.launch.py \
    camera_model:=zed2 camera_name:=zed node_name:=zed_node \
    publish_tf:=false publish_map_tf:=false ros_params_override_path:="$1"
' zed-preview "$run/zed_preview.yaml"
start_module lidar env AMENT_PREFIX_PATH="$prefix:/opt/ros/humble/install" \
  "$prefix/lib/ydlidar_ros2_driver/ydlidar_ros2_driver_node" \
  --ros-args --params-file "$prefix/share/nomad_costmap_cpp/config/g2b.yaml" -p "port:=$port"
# 2026-11-13: wait until OAK streams and the LiDAR has started instead of a fixed 8 s
# (at least 3 s so an early ZED/OAK/LiDAR crash is still caught; at most 15 s).
alive() { local pid; pid=$(cat "$run/$1.pid"); kill -0 "$pid" 2>/dev/null; }
for tick in $(seq 1 30); do
  sleep 0.5
  for name in oak zed lidar; do alive "$name" || check_module "$name"; done
  if (( tick >= 6 )) && grep -q 'OAK: rectified' "$run/oak/oak.log" 2>/dev/null &&
     grep -q 'Lidar has started' "$run/lidar.log" 2>/dev/null; then break; fi
done
for name in oak zed lidar; do check_module "$name"; done

# Existing independent inference and external adapters. No SDK/ROS rebuilds.
if [[ "$semantic_model" == indoor ]]; then
  start_module semantic bash "$ws/semantic.sh" --indoor "${semantic_args[@]}"
else
  start_module semantic bash "$ws/semantic.sh" "${semantic_args[@]}"
fi
start_module frame_alias "$root/build_external_frame_alias/nomad_external_frame_alias" --ros-args \
  -p confirmed_camera_serial:=23472639 -p same_left_rectified_optical_axis_confirmed:=true \
  -p input_camera_info_topic:=/nomad_preview/zed/raw_camera_info \
  -p output_camera_info_topic:=/zed/zed_node/left/camera_info
if [[ "$semantic_model" == indoor ]]; then
  # Same adapter launch as switch_model.sh uses for the indoor model.
  start_module semantic_adapter bash "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh"
  printf '{"model":"indoor","status":"running","started_with_trial":true}\n' > "$run/semantic_model.json"
else
  start_module semantic_adapter env NOMAD_SEMANTIC_MAPPING="$root/config/ade20k.preview.json" \
    bash "$ws/src/nomad_perception/traversability/run_semantic_adapter.sh"
fi
start_module external_cloud "$root/build_external_cloud/external_stereo_cloud" --ros-args \
  -p factory_json:="$root/config/factory_eeprom.json" \
  -p vio_calib_json:="$root/config/oak_basalt_calib.json" -p output_hz:="$oak_cloud_hz.0" -p initial_source:="$cloud_initial" \
  "${cloud_args[@]}" "${cloud_extra[@]}"
# 2026-11-13: until the adapters and the cloud report their first line (was a fixed 3 s; 1..6 s).
for tick in $(seq 1 12); do
  sleep 0.5
  for name in semantic frame_alias semantic_adapter external_cloud; do alive "$name" || check_module "$name"; done
  if (( tick >= 2 )) && grep -q 'Selectable OAK cloud' "$run/external_cloud.log" 2>/dev/null &&
     grep -q 'Verified serial' "$run/frame_alias.log" 2>/dev/null &&
     grep -q 'Verified serial' "$run/semantic_adapter.log" 2>/dev/null; then break; fi
done
for name in semantic frame_alias semantic_adapter external_cloud; do check_module "$name"; done

# 2026-11-13: GUI choice "새 매퍼" (NOMAD_MAPPER=mapper2) runs the new height-map mapper
# (~/nomad_mapper2_ws, A+B+C in one process) instead of the legacy manager. Same output topics,
# same DDS profile; settings uploaded by the GUI to ~/nomad_mapper2_ws/active_config.
if [[ "${NOMAD_MAPPER:-legacy}" == mapper2 ]]; then
  m2="$HOME/nomad_mapper2_ws"
  m2_script="$m2/install/nomad_mapper2_bringup/share/nomad_mapper2_bringup/scripts/run_mapper2.sh"
  [[ -x "$m2/install/nomad_mapper2_bringup/lib/nomad_mapper2_bringup/nomad_mapper2" && -r "$m2_script" ]] || fail 'mapper2 is not built on this Jetson'
  mkdir -p "$run/mapper2_cfg"
  cp -p "$m2/active_config/"*.json "$run/mapper2_cfg/" 2>/dev/null || true
  printf 'mapper2\n' > "$run/mapper_kind"
  start_module mapper2 env FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
    bash "$m2_script" "$run/mapper2_cfg" /nomad "$prefix/share/nomad_costmap_cpp/config"
  # 2026-11-13: the start marker can take ~7 s while the other modules start; wait up to 12 s
  # (1 s steps), and fail at once if the process exits.
  for _ in $(seq 1 12); do
    sleep 1
    check_module mapper2 > /dev/null
    grep -q mapper2_started "$run/mapper2.log" && break
  done
  check_module mapper2
  grep -q mapper2_started "$run/mapper2.log" || { tail -n 8 "$run/mapper2.log" >&2 || true; fail 'mapper2 did not start within 12 s'; }
  printf 'RUN=%s\nLIDAR_PORT=%s\nMAPPER=mapper2\n' "$run" "$port"
  for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud mapper2; do
    printf '%s LOG\n' "$name"
    tail -n 2 "$run/$name.log" || true
  done
  printf 'STARTED_ONLY: readiness/physical accuracy must be checked separately; no GUI/motor/Planning started\n'
  exit 0
fi

# 2026-11-15: GUI choice "새 매퍼3 v3" (NOMAD_MAPPER=mapper3) runs nomad_mapper3 from the Jetson SSD
# (/mnt/ssd/nomad/nomad_mapper3_ws, Humble build, recommended v3 config configs/mapper3.json) instead of the
# legacy manager. Same output topics (/nomad/navigation_costmap ...), same DDS profile. Inputs: /oak/depth/points,
# /scan, /nomad/localization/odometry (Pi5), /tf_static, /oak/calibration/factory, semantic class ids.
if [[ "${NOMAD_MAPPER:-legacy}" == mapper3 ]]; then
  m3=/mnt/ssd/nomad/nomad_mapper3_ws
  m3_bin="$m3/install/nomad_mapper3/lib/nomad_mapper3/nomad_mapper3"
  findmnt -n /mnt/ssd >/dev/null || fail 'SSD /mnt/ssd is not mounted (mapper3 v3 lives on the SSD)'
  [[ -x "$m3_bin" && -r "$m3/configs/mapper3.json" && -r "$HOME/nomad_mapper2_ws/install/setup.bash" ]] || fail 'mapper3 v3 is not built on this Jetson SSD'
  mkdir -p "$run/mapper3_cfg"
  cp -p "$m3/configs/mapper3.json" "$run/mapper3_cfg/mapper3.json"
  # 2026-11-16: GUI '새 매퍼3 v3 설정' (sem_fit.erode_px, lidar.start_free_radius_m, slope k) on this run's copy only;
  # with no saved values the copy is not rewritten.
  nomad_mapper_override apply mapper3 "$mapper_overrides" "$run/mapper3_cfg/mapper3.json" \
    || fail 'GUI mapper settings (~/nomad_settings/mapper_overrides/current.json) could not be applied to mapper3'
  printf 'mapper3\n' > "$run/mapper_kind"
  start_module mapper3 env FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
    bash -c 'set +u; source /opt/ros/humble/install/setup.bash; source "$HOME/nomad_ws/install/setup.bash"
      source "$HOME/nomad_mapper2_ws/install/setup.bash"; source "$1/install/setup.bash"
      exec "$2" --config "$3" --legacy-config-dir "$4" --summary "$5"' \
    mapper3 "$m3" "$m3_bin" "$run/mapper3_cfg/mapper3.json" "$prefix/share/nomad_costmap_cpp/config" "$run/mapper3_summary.json"
  for _ in $(seq 1 12); do
    sleep 1
    check_module mapper3 > /dev/null
    grep -q 'nomad_mapper3 started' "$run/mapper3.log" && break
  done
  check_module mapper3
  grep -q 'nomad_mapper3 started' "$run/mapper3.log" || { tail -n 8 "$run/mapper3.log" >&2 || true; fail 'mapper3 v3 did not start within 12 s'; }
  printf 'RUN=%s\nLIDAR_PORT=%s\nMAPPER=mapper3\n' "$run" "$port"
  for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud mapper3; do
    printf '%s LOG\n' "$name"
    tail -n 2 "$run/$name.log" || true
  done
  printf 'STARTED_ONLY: readiness/physical accuracy must be checked separately; no GUI/motor/Planning started\n'
  exit 0
fi

# 2026-11-13: GUI choice "기존 매퍼 개선" (NOMAD_MAPPER=terrain) runs the legacy map logic in one
# process (~/nomad_terrain_ws: no JSON IPC between processes, 20 m map window, optional gravity slope
# angle / slope refresh / label accumulation from active_config/terrain.json). It reads the same legacy
# runtime config files as the manager below and publishes the same topics.
if [[ "${NOMAD_MAPPER:-legacy}" == terrain ]]; then
  tw="$HOME/nomad_terrain_ws"
  t_script="$tw/install/nomad_terrain_mapper/share/nomad_terrain_mapper/scripts/run_terrain_mapper.sh"
  [[ -x "$tw/install/nomad_terrain_mapper/lib/nomad_terrain_mapper/nomad_terrain_mapper" && -r "$t_script" ]] || fail 'terrain mapper is not built on this Jetson'
  mkdir -p "$run/terrain_cfg"
  cp -p "$tw/active_config/terrain.json" "$run/terrain_cfg/" 2>/dev/null || true
  # 2026-11-16: slope k keys (only present when k != 4) need a terrain build that knows them; otherwise they
  # are removed from this run's copy (the saved active_config file is kept) and the old 4/20 deg behaviour runs.
  if [[ -f "$run/terrain_cfg/terrain.json" ]] && grep -qE '"slope_(score_per_deg|block_deg)"' "$run/terrain_cfg/terrain.json"; then
    if LC_ALL=C grep -aqF slope_score_per_deg "$tw/install/nomad_terrain_mapper/lib/nomad_terrain_mapper/nomad_terrain_mapper"; then
      nomad_terrain_slope show "$run/terrain_cfg/terrain.json" || true
    else
      nomad_terrain_slope strip "$run/terrain_cfg/terrain.json" || fail 'terrain slope k keys could not be removed from the run copy'
      printf 'WARNING: 경사 계수 미적용: 개선 매퍼 재빌드 필요 (installed terrain mapper does not know slope_score_per_deg)\n'
    fi
  fi
  # 2026-11-16: GUI '기존 매퍼 개선 설정' class-boundary px / initial safe area (terrain.json "gui_overrides",
  # GUI-only). The object is always removed from this run's copy before the mapper starts (the terrain binary
  # rejects unknown keys; the saved active_config file is kept). Only when it has values, a per-run copy of the
  # legacy config folder is patched (boundary_margin_px, startup_free_region.radius_m) and passed instead of
  # the shared folder. A "terrain" section left in ~/nomad_settings/mapper_overrides/current.json by the earlier
  # separate window is not used any more (one note line when it still has values).
  t_legacy="$prefix/share/nomad_costmap_cpp/config"
  override_rc=1
  if [[ -f "$run/terrain_cfg/terrain.json" ]]; then
    override_rc=0
    nomad_mapper_override check terrain "$run/terrain_cfg/terrain.json" || override_rc=$?
  fi
  if [[ "$override_rc" -eq 0 ]]; then
    mkdir "$run/terrain_legacy_cfg"
    cp -pR -- "$t_legacy"/. "$run/terrain_legacy_cfg/"
    nomad_mapper_override apply terrain "$run/terrain_cfg/terrain.json" "$run/terrain_legacy_cfg/semantic.real.json" \
      || fail 'GUI mapper settings could not be applied to the terrain mapper copy'
    t_legacy="$run/terrain_legacy_cfg"
  elif [[ "$override_rc" -eq 1 ]]; then
    nomad_mapper_override show terrain "$t_legacy/semantic.real.json" || true
  else
    fail 'Invalid GUI values (gui_overrides) in ~/nomad_terrain_ws/active_config/terrain.json'
  fi
  if [[ -f "$run/terrain_cfg/terrain.json" ]]; then
    nomad_mapper_override strip terrain "$run/terrain_cfg/terrain.json" \
      || fail 'GUI-only gui_overrides could not be removed from the terrain run copy'
  fi
  if [[ -f "$mapper_overrides" ]] && python3 -I -c 'import json,sys; t=json.load(open(sys.argv[1])).get("terrain"); sys.exit(0 if isinstance(t, dict) and any(v is not None for v in t.values()) else 1)' "$mapper_overrides" 2>/dev/null; then
    printf 'NOTE: old terrain values in ~/nomad_settings/mapper_overrides/current.json are ignored; set them in 기존 매퍼 개선 설정\n'
  fi
  printf 'terrain\n' > "$run/mapper_kind"
  start_module terrain env FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
    bash "$t_script" "$run/terrain_cfg" "$t_legacy"
  for _ in $(seq 1 12); do
    sleep 1
    check_module terrain > /dev/null
    grep -q terrain_mapper_started "$run/terrain.log" && break
  done
  check_module terrain
  grep -q terrain_mapper_started "$run/terrain.log" || { tail -n 8 "$run/terrain.log" >&2 || true; fail 'terrain mapper did not start within 12 s'; }
  printf 'RUN=%s\nLIDAR_PORT=%s\nMAPPER=terrain\n' "$run" "$port"
  for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud terrain; do
    printf '%s LOG\n' "$name"
    tail -n 2 "$run/$name.log" || true
  done
  printf 'STARTED_ONLY: readiness/physical accuracy must be checked separately; no GUI/motor/Planning started\n'
  exit 0
fi

# Existing private native map: preserve mount guess, 2px tolerance and all K.
# LiDAR is already owned above; manager must not start another serial driver.
id="manual_map_$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"
NOMAD_RUNTIME_DIR="$prefix" ROS_LOG_DIR="$run/ros" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
  bash "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  --action start --run-id "$id" --state-root "$run/costmap_runs" \
  --external-lidar --startup-timeout 900 > "$run/costmap_start.json" 2> "$run/costmap_start.err"
printf '%s\n' "$id" > "$run/costmap_run_id"
sed -n 's/.*"supervisor_pid":[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$run/costmap_start.json" > "$run/costmap.pid"
sleep 1
NOMAD_RUNTIME_DIR="$prefix" FASTRTPS_DEFAULT_PROFILES_FILE="$planning_output_profile" \
  bash "$ws/src/nomad_perception/traversability/run_costmap.sh" \
  --action status --run-id "$id" --state-root "$run/costmap_runs" > "$run/costmap_status.json"

printf 'RUN=%s\nLIDAR_PORT=%s\nCOSTMAP_RUN=%s\n' "$run" "$port" "$id"
cat "$run/costmap_status.json"
for name in oak zed lidar semantic frame_alias semantic_adapter external_cloud; do
  printf '%s LOG\n' "$name"
  tail -n 2 "$run/$name.log" || true
done
printf 'STARTED_ONLY: readiness/physical accuracy must be checked separately; no GUI/motor/Planning started\n'
