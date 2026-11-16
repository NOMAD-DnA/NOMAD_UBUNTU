#!/usr/bin/env bash
# Explicit service request only. No sensor startup, process restart or motion.
set -o pipefail
exec 2>/dev/null

finish() {
    printf 'NOMAD_HEADING_RESET_RESULT={"ok":%s,"state":"%s","reason":"%s"}\n' "$1" "$2" "$3"
    exit 0
}
fail() { finish false HEADING_RESET_FAILED "$1"; }

[[ "$(id -un)" = user ]] || fail wrong_board
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = Raspberry\ Pi\ 5* ]] || fail wrong_board
ws=/home/user/nomad_ws
[[ -d "$ws/src" && -f /home/user/nomad_ros.sh && -f "$ws/install/setup.bash" ]] || fail workspace_unavailable
source /home/user/nomad_ros.sh >/dev/null 2>&1 || fail environment_unavailable
source "$ws/install/setup.bash" >/dev/null 2>&1 || fail environment_unavailable
command -v ros2 >/dev/null 2>&1 || fail environment_unavailable

profile="$ws/config/oak_fastdds_wired.xml"
if [[ -f /tmp/nomad_integrated_pi_run_dir ]]; then
    IFS= read -r run < /tmp/nomad_integrated_pi_run_dir
    run=$(readlink -f -- "$run")
    case "$run" in
        "$ws"/log/*)
            [[ ! -f "$run/planning_output_fastdds.xml" ]] || profile="$run/planning_output_fastdds.xml"
            ;;
    esac
fi
[[ -f "$profile" ]] || fail profile_unavailable
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$profile" RMW_FASTRTPS_USE_QOS_FROM_XML=0
export ROS2CLI_NO_DAEMON=1

response=$(timeout --signal=TERM --kill-after=1s 8s ros2 service call \
    /fusion/reset_heading_from_magnetometer std_srvs/srv/Trigger '{}' 2>&1)
result=$?
if [[ "$result" = 124 || "$result" = 137 ]]; then
    if [[ "$response" = *"waiting for service"* && "$response" != *"making request"* ]]; then
        fail service_unavailable
    fi
    fail service_timeout
fi
[[ "$result" = 0 ]] || fail service_call_failed
if [[ "$response" =~ (^|[\(,[:space:]])success=True([\),[:space:]]|$) ]]; then
    finish true HEADING_RESET ok
fi
[[ "$response" =~ (^|[\(,[:space:]])success=False([\),[:space:]]|$) ]] || fail invalid_service_response
reason=reset_rejected
if [[ "$response" =~ message=[\'\"]([A-Za-z0-9_]+)[\'\"] ]]; then
    candidate=${BASH_REMATCH[1]}
    case "$candidate" in
        VIO_INPUT_NOT_READY|VIO_ONLY_REQUIRED|OUTPUT_FRAME_INVALID|WAITING_FOR_CONTINUOUS_VIO_ONLY|HOLD_VEHICLE_STILL|MAG_IMU_NOT_FRESH|MAGNETOMETER_NOT_FRESH|MAGNETIC_HEADING_UNAVAILABLE|MAG_CALIBRATION_OR_EXTRINSICS_UNCONFIRMED|mag_calibration_or_extrinsics_unconfirmed|invalid_mag_or_timestamp|mag_field_norm|mag_field_norm_jump|mag_tilt_orientation_unavailable|mag_horizontal_field_too_small|mag_heading_innovation|reset_rejected)
            reason=$candidate ;;
    esac
fi
fail "$reason"
