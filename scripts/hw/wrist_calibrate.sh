#!/usr/bin/env bash
# Capture both wrist views of the glove tags, then fit and validate each wrist-to-camera transform.
set -euo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cell="$repo/experiments/hardware/cell"

usage() {
    cat <<'EOF'
scripts/hw/wrist_calibrate.sh capture <both|silver|blue> <session_dir> <fit_01|check_01|...> [--paper TAG20_MM TAG21_MM]
    Keep the target still until capture ends. The arms stay at their operator-selected pose.
scripts/hw/wrist_calibrate.sh fit <silver|blue> <session_dir> [--floor-reference BACKGROUND] [--fit-floor BACKGROUND]
    Fit with fit_* captures and validate on independent check_* captures.
EOF
}

if [[ "${1:-}" == --help || "${1:-}" == -h || $# -eq 0 ]]; then usage; exit 0; fi
action="$1"
robot="${2:?specify both, silver or blue}"
session="${3:?specify a session directory}"
case "$action:$robot" in
    fit:silver|fit:blue)
        shift 3
        exec python3 "$here/wrist_calibrate.py" "$robot" "$session" "$@"
        ;;
    capture:both|capture:silver|capture:blue)
        [ "$#" -ge 4 ] || { usage >&2; exit 2; }
        label="$4"
        [[ "$label" =~ ^(fit|check)_[A-Za-z0-9_-]+$ ]] || { usage >&2; exit 2; }
        shift 4
        mode=()
        if [ "$#" -ne 0 ]; then
            [[ "$#" -eq 3 && "$1" == --paper && "$2" =~ ^[0-9]+([.][0-9]+)?$ &&
               "$3" =~ ^[0-9]+([.][0-9]+)?$ ]] || { usage >&2; exit 2; }
            mode=(--paper-tag-width-mm "$2" "$3")
        fi
        ;;
    *) usage >&2; exit 2 ;;
esac

mkdir -p -- "$session"
session="$(realpath -- "$session")"
if [ -e "$session/calibration.yaml" ]; then
    cmp -s "$cell/calibration.yaml" "$session/calibration.yaml" || {
        echo "Cell calibration changed. Start a new session directory." >&2
        exit 1
    }
else
    cp "$cell/calibration.yaml" "$session/"
fi
pose="$session/$label"
mkdir -- "$pose"
# shellcheck source=lab_ssh.sh
source "$here/lab_ssh.sh"
lab_ssh_setup

cams=(nuc:overhead)
if [[ "$robot" == silver || "$robot" == both ]]; then cams+=(silver:silver_wrist); fi
if [[ "$robot" == blue || "$robot" == both ]]; then cams+=(blue:blue_wrist); fi
start_at="$(($(date +%s) + 5))"
echo "Keep the tags still through the shared 8-second capture."
for pair in "${cams[@]}"; do
    host="${pair%%:*}"
    cam="${pair##*:}"
    (
        if ! "$here/lab.sh" session "$host" exec python3 - --camera "$cam" \
                --cell-dir fabric_ws/experiments/hardware/cell --calibration --start-at "$start_at" \
                --seconds 8 --rate 5 --save-every 0 "${mode[@]}" < "$here/glove_check.py" > "$pose/$cam.log" 2>&1; then
            echo "$cam capture failed. See $pose/$cam.log" >&2
            exit 1
        fi
        sed -n 's/^CALIBRATION_JSON://p' "$pose/$cam.log" > "$pose/$cam.jsonl"
        printf '%s: %s frames with tag centres and TF\n' "$cam" "$(wc -l < "$pose/$cam.jsonl")"
    ) &
done
wait
echo "Capture saved in $pose"
if [ ! -s "$pose/overhead.jsonl" ]; then
    echo "Overhead needs a visible tag with registered depth. See $pose/overhead.log." >&2
    exit 1
fi
usable=0
for pair in "${cams[@]:1}"; do
    if python3 "$here/wrist_calibrate.py" "${pair%%:*}" "$session" --inspect "$label"; then usable=1; fi
done
if [ "$usable" -ne 1 ]; then
    echo "Check the wrist logs and try another hold." >&2
    exit 1
fi
