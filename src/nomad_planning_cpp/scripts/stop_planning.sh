#!/usr/bin/env bash
set -euo pipefail
lock_root=/tmp
pidfile="$lock_root/nomad-planning-${UID}.pid"
[[ -f "$pidfile" ]] || { printf '%s\n' 'Planner is not running.'; exit 0; }
read -r task_pid < "$pidfile"
[[ "$task_pid" =~ ^[0-9]+$ ]] || { printf '%s\n' 'Invalid planner PID file.' >&2; exit 1; }
[[ -r "/proc/$task_pid/cmdline" ]] || { rm -f "$pidfile"; exit 0; }
actual_exe="$(readlink "/proc/$task_pid/exe")"
expected_exe="${NOMAD_PLANNING_WS:-/home/user/nomad_cpp_ws}/install/nomad_planning_cpp/lib/nomad_planning_cpp/planner_node"
[[ "$(readlink -f "$actual_exe")" == "$(readlink -f "$expected_exe")" ]] || { printf '%s\n' 'PID belongs to a different process; left untouched.' >&2; exit 1; }
kill -INT "$task_pid"
printf '%s\n' 'Planner stop requested; its owned output session will close. Sensors are untouched.'
