#!/bin/bash
# Stop only the OAK publisher started by run_oak_cpp.sh.
set -eo pipefail
binary=/home/nvidia/nomad_ws/install/nomad_oak_cpp/lib/nomad_oak_cpp/oak_publisher
pid_file=/tmp/nomad_oak_cpp.pid

publisher_running() {
  local proc executable process_state
  for proc in /proc/[0-9]*; do
    [ -r "$proc/cmdline" ] || continue
    executable=
    IFS= read -r -d '' executable < "$proc/cmdline" || true
    process_state=$(awk '{print $3}' "$proc/stat" 2>/dev/null) || continue
    if [ "$executable" = "$binary" ] && [ -n "$process_state" ] && [ "$process_state" != Z ]; then
      return 0
    fi
  done
  return 1
}

if ! publisher_running; then
  printf 'OAK_STOPPED already_stopped\n'
  exit 0
fi
if [ ! -r "$pid_file" ]; then
  printf 'OAK stop failed: publisher is running but its PID file is missing.\n' >&2
  exit 1
fi
pid=$(cat "$pid_file")
case "$pid" in ''|*[!0-9]*) printf 'OAK stop failed: invalid PID file.\n' >&2; exit 1 ;; esac
executable=
if [ -r "/proc/$pid/cmdline" ]; then
  IFS= read -r -d '' executable < "/proc/$pid/cmdline" || true
fi
if [ "$executable" != "$binary" ]; then
  printf 'OAK stop failed: PID belongs to another process; no signal sent.\n' >&2
  exit 1
fi
started=$(awk '{print $22}' "/proc/$pid/stat")
same_process_alive() {
  [ -r "/proc/$pid/stat" ] &&
    [ "$(awk '{print $22}' "/proc/$pid/stat" 2>/dev/null)" = "$started" ] &&
    [ "$(awk '{print $3}' "/proc/$pid/stat" 2>/dev/null)" != Z ]
}
wait_stopped() {
  local i
  for ((i=0; i<$1; ++i)); do
    same_process_alive || return 0
    sleep 0.1
  done
  ! same_process_alive
}
kill -INT "$pid" 2>/dev/null || true
if ! wait_stopped 50; then
  same_process_alive && kill -TERM "$pid" 2>/dev/null || true
  if ! wait_stopped 30; then
    same_process_alive && kill -KILL "$pid" 2>/dev/null || true
    wait_stopped 10 || { printf 'OAK stop failed: publisher did not exit.\n' >&2; exit 1; }
  fi
fi
if publisher_running; then
  printf 'OAK stop failed: an OAK publisher is still running.\n' >&2
  exit 1
fi
printf 'OAK_STOPPED\n'
