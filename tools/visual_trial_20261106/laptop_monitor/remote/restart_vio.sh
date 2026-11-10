#!/usr/bin/env bash
# Restart only a tracked private Basalt group. Fusion/cameras/GNSS stay intact.
set -uo pipefail
exec 2>/dev/null
declare -A starts=() groups=()
current_uid=$(id -u)

finish() {
    printf 'NOMAD_VIO_RESTART_RESULT={"ok":%s,"state":"%s","reason":"%s","fusion_preserved":%s}\n' "$1" "$2" "$3" "$4"
    exit 0
}
fail() { finish false VIO_RESTART_FAILED "$1" false; }

identity() {
    local pid=$1 line rest
    [[ "$pid" =~ ^[1-9][0-9]*$ && -r "/proc/$pid/stat" ]] || return 1
    IFS= read -r line < "/proc/$pid/stat" || return 1
    rest=${line##*) }
    read -r -a fields <<< "$rest"
    [[ ${#fields[@]} -ge 20 ]] || return 1
    process_state=${fields[0]}; process_group=${fields[2]}; process_session=${fields[3]}; process_start=${fields[19]}
    process_uid=$(stat -c %u "/proc/$pid") || return 1
    [[ "$process_state" != Z && "$process_state" != X && "$process_uid" == "$current_uid" ]]
}
command_is() {
    local pid=$1 expected=$2 token
    local -a argv=()
    mapfile -d '' -t argv < "/proc/$pid/cmdline" || return 1
    [[ ${#argv[@]} -gt 0 ]] || return 1
    [[ "${argv[0]}" == "$expected" ]] && return 0
    [[ "${argv[0]##*/}" == bash && ${#argv[@]} -gt 1 && "${argv[1]}" == "$expected" ]]
}
vio_command() {
    command_is "$1" "$ws/scripts/run_basalt_cpp.sh" ||
    command_is "$1" "$ws/install/basalt_ros/lib/basalt_ros/vio_frontend_node" ||
    command_is "$1" "$ws/install/basalt_ros/lib/basalt_ros/vio_backend_node"
}
same() {
    local pid=$1
    identity "$pid" && [[ "$process_start" == "${starts[$pid]}" && "$process_group" == "${groups[$pid]}" ]]
}
signal_vio() {
    local pid
    for pid in "${!starts[@]}"; do
        same "$pid" && kill -s "$1" -- "$pid" || true
    done
}
remaining_vio() {
    local pid group uid
    while read -r pid group uid; do
        [[ "$group" == "$vio_group" && "$uid" == "$current_uid" ]] || continue
        identity "$pid" && return 0
    done < <(ps -eo pid=,pgid=,uid=)
    return 1
}

[[ "$(id -un)" = user ]] || fail wrong_board
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = Raspberry\ Pi\ 5* ]] || fail wrong_board
ws=/home/user/nomad_ws
[[ -d "$ws/src" && -f /home/user/nomad_ros.sh && -f "$ws/install/setup.bash" && -f "$ws/scripts/run_basalt_cpp.sh" ]] || fail workspace_unavailable
pointer=/tmp/nomad_integrated_pi_run_dir
[[ -e "$pointer" ]] || fail NO_TRIAL_RUN
[[ -f "$pointer" && ! -L "$pointer" && $(stat -c %u "$pointer") == "$current_uid" ]] || fail current_run_unknown
run=$(cat "$pointer")
[[ "$run" == "$ws"/log/visual_trial_* && -d "$run" && $(readlink -f "$run") == "$run" && $(stat -c %u "$run") == "$current_uid" ]] || fail current_run_invalid
exec 8>"/tmp/nomad_visual_trial_stop_${current_uid}.lock"
flock -n 8 || fail trial_stop_busy
exec 9>"/tmp/nomad_vio_restart_${current_uid}.lock"
flock -n 9 || fail restart_busy

# The older standalone C++ session owns its own group/control channel.
# Never take over that process from this integrated-trial restart button.
if [[ -f /tmp/nomad_vio_cpp.pid && ! -L /tmp/nomad_vio_cpp.pid ]]; then
    standalone=$(cat /tmp/nomad_vio_cpp.pid)
    if identity "$standalone" && command_is "$standalone" "$ws/install/nomad_vio_session/lib/nomad_vio_session/vio_session"; then
        fail standalone_vio_session_active
    fi
fi

fusion_pid=; fusion_start=; fusion_group=
if [[ -f "$run/fusion.pid" ]]; then
    [[ ! -L "$run/fusion.pid" && $(stat -c %u "$run/fusion.pid") == "$current_uid" ]] || fail fusion_identity_mismatch
    fusion_pid=$(cat "$run/fusion.pid")
    if identity "$fusion_pid"; then
        command_is "$fusion_pid" "$ws/install/nomad_localization/lib/nomad_localization/fusion_node" || fail fusion_identity_mismatch
        fusion_start=$process_start; fusion_group=$process_group
    else
        fusion_pid=
    fi
fi

vio_pid=; vio_group=
if [[ -f "$run/vio.pid" ]]; then
    [[ ! -L "$run/vio.pid" && $(stat -c %u "$run/vio.pid") == "$current_uid" ]] || fail vio_identity_mismatch
    vio_pid=$(cat "$run/vio.pid")
    if identity "$vio_pid"; then
        [[ "$process_group" == "$vio_pid" && "$process_session" == "$vio_pid" ]] || fail vio_identity_mismatch
        command_is "$vio_pid" "$ws/scripts/run_basalt_cpp.sh" || fail vio_identity_mismatch
        vio_group=$process_group
    fi
fi
# Inspect exact known executable tokens; never use process-name-wide killing.
while read -r pid uid; do
    [[ "$uid" == "$current_uid" ]] || continue
    if vio_command "$pid" && identity "$pid"; then
        [[ -n "$vio_group" && "$process_group" == "$vio_group" && "$process_session" == "$vio_group" ]] || fail untracked_vio_running
        starts[$pid]=$process_start; groups[$pid]=$process_group
    fi
done < <(ps -eo pid=,uid=)
[[ -z "$fusion_pid" || "$fusion_group" != "$vio_group" ]] || fail vio_identity_mismatch

# Resolve the native ROS environment before changing the tracked VIO process.
set +u
source /home/user/nomad_ros.sh >/dev/null 2>&1 || fail environment_unavailable
source "$ws/install/setup.bash" >/dev/null 2>&1 || fail environment_unavailable
set -u
command -v ros2 >/dev/null 2>&1 || fail environment_unavailable
profile="$ws/config/oak_fastdds_wired.xml"
[[ ! -f "$run/planning_output_fastdds.xml" ]] || profile="$run/planning_output_fastdds.xml"
[[ -f "$profile" ]] || fail profile_unavailable
[[ -f "$ws/config/oak_fastdds_wired.xml" ]] || fail profile_unavailable
export ROS_DOMAIN_ID=59 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export RMW_FASTRTPS_USE_QOS_FROM_XML=0 ROS2CLI_NO_DAEMON=1
unset ROS_DISCOVERY_SERVER

if [[ -n "$vio_group" ]]; then
    signal_vio INT
    for attempt in {1..20}; do remaining_vio || break; sleep .1; done
    if remaining_vio; then
        signal_vio TERM
        for attempt in {1..20}; do remaining_vio || break; sleep .1; done
    fi
    if remaining_vio; then
        signal_vio KILL
        sleep .2
    fi
    remaining_vio && fail vio_stop_incomplete
fi

preserved=false
if [[ -n "$fusion_pid" ]]; then
    identity "$fusion_pid" && [[ "$process_start" == "$fusion_start" && "$process_group" == "$fusion_group" ]] || fail fusion_exited_during_restart
    epoch=$(date +%s)
    if [[ -f "$run/vio_restart_epoch" && ! -L "$run/vio_restart_epoch" ]]; then
        previous_epoch=$(cat "$run/vio_restart_epoch")
        if [[ "$previous_epoch" =~ ^[0-9]{1,10}$ ]] && (( previous_epoch >= epoch && previous_epoch < 4294967295 )); then
            epoch=$((previous_epoch + 1))
        fi
    fi
    (( epoch > 0 && epoch <= 4294967295 )) || fail epoch_publish_failed
    export FASTRTPS_DEFAULT_PROFILES_FILE="$profile"
    timeout --signal=TERM --kill-after=1s 6s ros2 topic pub --once --qos-durability transient_local \
        --qos-reliability reliable --wait-matching-subscriptions 1 --keep-alive .3 \
        /fusion/vio_epoch std_msgs/msg/UInt32 "{data: $epoch}" >/dev/null 2>&1 || fail epoch_publish_failed
    printf '%s\n' "$epoch" > "$run/vio_restart_epoch"
    preserved=true
fi

# Use the existing sensor-input transport for Basalt, separate from planning output.
export FASTRTPS_DEFAULT_PROFILES_FILE="$ws/config/oak_fastdds_wired.xml"
[[ -f "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || fail profile_unavailable
[[ ! -f "$run/vio.log" ]] || mv -- "$run/vio.log" "$run/vio_before_restart_$(date +%Y%m%d_%H%M%S)_$$.log" || fail vio_start_failed
setsid env ROS_LOG_DIR="$run/ros" bash "$ws/scripts/run_basalt_cpp.sh" 8>&- 9>&- </dev/null > "$run/vio.log" 2>&1 &
new_pid=$!
printf '%s\n' "$new_pid" > "$run/vio.pid.next" && mv -- "$run/vio.pid.next" "$run/vio.pid" || fail vio_start_failed
sleep 1
identity "$new_pid" && command_is "$new_pid" "$ws/scripts/run_basalt_cpp.sh" || fail vio_start_failed
if [[ "$preserved" = true ]]; then
    identity "$fusion_pid" && [[ "$process_start" == "$fusion_start" && "$process_group" == "$fusion_group" ]] || fail fusion_exited_during_restart
    finish true VIO_RESTARTED fusion_preserved true
fi
finish true VIO_RESTARTED new_vio_origin false
