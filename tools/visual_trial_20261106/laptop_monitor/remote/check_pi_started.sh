#!/usr/bin/env bash
set -eo pipefail
# Return controlled script diagnostics without exposing SSH/sudo stderr.
exec 2>&1
ws=/home/user/nomad_ws
run=$(cat /tmp/nomad_integrated_pi_run_dir)
[[ "$run" = "$ws/log/visual_trial_"* && -d "$run" ]] || { echo 'INVALID_PI_RUN' >&2; exit 1; }
for name in gnss vio fusion; do
  pid=$(cat "$run/$name.pid")
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || { echo "EXITED=$name" >&2; exit 1; }
  cmd=$(tr '\0' ' ' < "/proc/$pid/cmdline")
  [[ "$cmd" = *"$ws/"* ]] || { echo "STALE_PID=$name" >&2; exit 1; }
  printf 'RUNNING=%s pid=%s\n' "$name" "$pid"
done
printf 'STARTED_ONLY=pi5\nRUN=%s\n' "$run"
