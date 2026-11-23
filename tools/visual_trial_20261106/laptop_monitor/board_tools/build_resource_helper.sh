#!/usr/bin/env bash
# Build only the read-only SSH log/resource helper; never launch vehicle nodes.
set -euo pipefail
src=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
out=${1:?Usage: build_resource_helper.sh /absolute/path/to/module_log_tail}
[[ "$out" = /* ]] || { echo 'Output must be an absolute path' >&2; exit 2; }
mkdir -p -- "$(dirname -- "$out")"
tmp=$(mktemp "${out}.resource-build.XXXXXX")
trap 'rm -f -- "$tmp"' EXIT
g++ -std=c++17 -O2 -Wall -Wextra -Werror "$src/module_log_tail.cpp" -o "$tmp"
chmod 755 "$tmp"
if [[ -e "$out" ]]; then
  backup="${out}.before-resources-$(date +%Y%m%dT%H%M%S)-$$"
  cp -p -- "$out" "$backup"
  printf 'Backup: %s\n' "$backup"
fi
mv -f -- "$tmp" "$out"
printf 'Built: %s\n' "$out"
