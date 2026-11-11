#!/usr/bin/env bash
set -e
script_dir=$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec bash "$script_dir/ensure_gui.sh"
