#!/usr/bin/env bash
# Read-only check of the Pi5 front video recorder (2026-11-11).
# Reports the current trial's recorder status and how many earlier trials have
# front video. Starts, stops and deletes nothing.
set -uo pipefail
exec 2>/dev/null
current_uid=$(id -u)
marker=NOMAD_FRONT_RESULT=

runs_json='[]'
notes_json='{}'
finish() {  # ok state reason current_json alive previous_runs previous_bytes
    printf '%s{"ok":%s,"state":"%s","reason":"%s","alive":%s,"previous_runs":%s,"previous_bytes":%s,"status":%s,"runs":%s,"notes":%s}\n' \
        "$marker" "$1" "$2" "$3" "${5:-false}" "${6:-0}" "${7:-0}" "${4:-null}" "$runs_json" "$notes_json"
    exit 0
}
# Shared titles/comments: one file keyed by trial name (front_manage.sh writes it).
notes_file=/home/user/nomad_ws/log/front_video_notes.json
if [[ -f "$notes_file" && ! -L "$notes_file" && $(stat -c %s "$notes_file") -le 262144 ]]; then
    notes_json=$(/usr/bin/python3 -I -c 'import json,sys; d=json.load(open(sys.argv[1])); r=d.get("runs") if isinstance(d,dict) else None; print(json.dumps(r if isinstance(r,dict) else {}, ensure_ascii=False, separators=(",",":")))' "$notes_file" 2>/dev/null || echo '{}')
    [[ -n "$notes_json" ]] || notes_json='{}'
fi
flat_number() {  # key file -> number from the recorder's flat status line, or 0
    local value
    value=$(sed -nE 's/.*"'"$1"'":(-?[0-9]+(\.[0-9]+)?).*/\1/p' "$2" 2>/dev/null | head -n 1)
    printf '%s' "${value:-0}"
}

[[ "$(id -un)" = user ]] || finish false UNCONFIRMED wrong_board
model=$(tr -d '\0' < /proc/device-tree/model)
[[ "$model" = Raspberry\ Pi\ 5* ]] || finish false UNCONFIRMED wrong_board
ws=/home/user/nomad_ws
installed=false
[[ -x "$ws/install/nomad_front_recorder/lib/nomad_front_recorder/front_recorder" ]] && installed=true

# Earlier trials that left front video (current run excluded below).
pointer=/tmp/nomad_integrated_pi_run_dir
run=
if [[ -f "$pointer" && ! -L "$pointer" && $(stat -c %u "$pointer") == "$current_uid" ]]; then
    run=$(cat "$pointer")
    [[ "$run" == "$ws"/log/visual_trial_* && -d "$run" && $(readlink -f "$run") == "$run" ]] || run=
fi
previous_runs=0; previous_bytes=0
for dir in "$ws"/log/visual_trial_*/front_video; do
    [[ -d "$dir" && ! -L "$dir" ]] || continue
    [[ -n "$run" && "$dir" == "$run/front_video" ]] && continue
    compgen -G "$dir/seg_*.avi" >/dev/null || compgen -G "$dir/seg_*.mjpeg" >/dev/null || continue
    previous_runs=$((previous_runs + 1))
    previous_bytes=$((previous_bytes + $(du -sb "$dir" | cut -f1)))
done

# One line per recorded trial, newest first (at most 20).
entries=()
for dir in "$ws"/log/visual_trial_*/front_video; do
    [[ -d "$dir" && ! -L "$dir" ]] || continue
    compgen -G "$dir/seg_*.avi" >/dev/null || compgen -G "$dir/seg_*.mjpeg" >/dev/null || continue
    name=${dir%/front_video}; name=${name##*/}
    [[ "$name" =~ ^visual_trial_[A-Za-z0-9_-]+$ ]] || continue
    st="$dir/status.json"; state=UNKNOWN; duration=0; frames=0; dropped=0
    if [[ -f "$st" && ! -L "$st" ]]; then
        state=$(sed -nE 's/.*"state":"([A-Z_]+)".*/\1/p' "$st" | head -n 1); state=${state:-UNKNOWN}
        duration=$(flat_number duration_s "$st"); frames=$(flat_number frames "$st"); dropped=$(flat_number dropped "$st")
    fi
    segments=$(compgen -G "$dir/seg_*.[am][vj]*" | grep -cE "\.(avi|mjpeg)$")
    bytes=$(du -sb "$dir" | cut -f1)
    current=false; [[ -n "$run" && "$dir" == "$run/front_video" ]] && current=true
    entries+=("{\"name\":\"$name\",\"state\":\"$state\",\"duration_s\":$duration,\"frames\":$frames,\"dropped\":$dropped,\"segments\":$segments,\"bytes\":$bytes,\"current\":$current}")
done
if (( ${#entries[@]} )); then
    sorted=$(printf '%s\n' "${entries[@]}" | sort -r | head -n 20 | paste -sd, -)
    runs_json="[$sorted]"
fi

[[ "$installed" == true ]] || finish true NOT_INSTALLED recorder_not_installed null false "$previous_runs" "$previous_bytes"
[[ -n "$run" ]] || finish true NO_TRIAL_RUN no_current_run null false "$previous_runs" "$previous_bytes"

alive=false
if [[ -f "$run/front_recorder.pid" && ! -L "$run/front_recorder.pid" ]]; then
    pid=$(cat "$run/front_recorder.pid")
    if [[ "$pid" =~ ^[1-9][0-9]*$ && -r "/proc/$pid/cmdline" ]]; then
        argv0=$(tr '\0' '\n' < "/proc/$pid/cmdline" | head -n 1)
        [[ "$argv0" == "$ws/install/nomad_front_recorder/lib/nomad_front_recorder/front_recorder" ]] && alive=true
    fi
fi
status="$run/front_video/status.json"
[[ -f "$status" && ! -L "$status" ]] || finish true NOT_STARTED no_status_file null "$alive" "$previous_runs" "$previous_bytes"
# The recorder writes one flat JSON line; pass it through only if it looks like one.
line=$(head -c 2048 "$status" | head -n 1)
[[ "$line" =~ ^\{[^{}]*\}$ ]] || finish false UNCONFIRMED status_unreadable null "$alive" "$previous_runs" "$previous_bytes"
finish true CHECKED status_read "$line" "$alive" "$previous_runs" "$previous_bytes"
