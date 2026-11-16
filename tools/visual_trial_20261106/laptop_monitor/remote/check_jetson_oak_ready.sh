#!/usr/bin/env bash
# 2026-11-13: read-only. Is THIS Start's OAK streaming? (start_token separates it from a previous run.)
# The GUI prepends: export NOMAD_START_TOKEN=<random>
set -u
pointer=/tmp/nomad_integrated_jetson_run_dir
[[ -r "$pointer" ]] || { echo OAK_NOT_READY; exit 0; }
run=$(cat "$pointer")
[[ -r "$run/start_token" && "$(cat "$run/start_token")" == "${NOMAD_START_TOKEN:-none}" ]] || { echo OAK_NOT_READY; exit 0; }
if [[ -r "$run/oak.pid" ]] && kill -0 "$(cat "$run/oak.pid")" 2>/dev/null && grep -q 'OAK: rectified' "$run/oak/oak.log" 2>/dev/null; then
  echo OAK_READY
else
  echo OAK_NOT_READY
fi
