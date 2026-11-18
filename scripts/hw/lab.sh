#!/usr/bin/env bash
# Drive fabric hardware sessions on the lab machines from the compute laptop.
#   lab.sh snapshot <host>              record the stock system's state and compare it with the baseline
#   lab.sh setup <host> [git ref]       create or update ~/fabric_ws there and build it (no sudo)
#   lab.sh stock <host> stop|start      pause or resume the stock service (asks for the lab sudo password)
#   lab.sh session <host> <command> ... run scripts/hw/fabric_session.sh there (start, start-cell, stop, status, logs)
# Hosts: blue, silver, tablet, nuc. Set LAB_SSH to replace the ssh command (it gets "<user@host> <command>").
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
ssh_cmd="${LAB_SSH:-ssh}"

usage() { sed -n '2,7p' "$0"; exit 2; }
[ $# -ge 2 ] || usage
cmd="$1"; name="$2"; shift 2
case "$name" in
    blue)   target=oarbot_blue@192.168.1.102;   role=helper; unit=oarbot_blue_helper_laptop_launch.service ;;
    silver) target=oarbot_silver@192.168.1.104; role=helper; unit=oarbot_silver_helper_laptop_launch.service ;;
    tablet) target=tablet@192.168.1.99;         role=tablet ;;
    nuc)    target=nuc@192.168.1.90;            role=camera; unit=nuc_launch.service ;;
    *) echo "Unknown host $name (blue, silver, tablet or nuc)" >&2; exit 2 ;;
esac

case "$cmd" in
    snapshot)
        dir="$repo/experiments/hardware/stock_snapshots/$name"
        mkdir -p "$dir"
        out="$dir/$(date +%Y%m%d_%H%M%S).txt"
        $ssh_cmd "$target" 'bash -s' < "$here/stock_snapshot.sh" > "$out"
        baseline="$(find "$dir" -name '*.txt' | sort | head -1)"
        if [ "$baseline" = "$out" ]; then
            echo "Baseline saved: $out"
        elif diff -u "$baseline" "$out"; then
            echo "$name: stock system unchanged since $(basename "$baseline" .txt)"
        else
            echo "$name: stock system differs from the baseline (diff above)" >&2
            exit 1
        fi ;;
    setup)
        $ssh_cmd "$target" "FABRIC_OWNER='$(git -C "$repo" config user.name)' bash -s -- $role ${1:-}" < "$here/setup_fabric_ws.sh" ;;
    stock)
        [ -n "$unit" ] || { echo "The tablet has no stock service; close its GUI by hand" >&2; exit 2; }
        action="${1:?stop or start}"
        case "$action" in stop|start) ;; *) echo "stop or start" >&2; exit 2 ;; esac
        ssh -t "$target" "sudo systemctl $action $unit; echo \"$unit: \$(systemctl is-active $unit)\"" ;;
    session)
        $ssh_cmd "$target" "bash ~/fabric_ws/scripts/hw/fabric_session.sh $*" ;;
    *) usage ;;
esac
