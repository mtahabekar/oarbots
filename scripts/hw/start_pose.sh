#!/usr/bin/env bash
# Both arms to the experiment's start pose (experiments/hardware/cell/grasp.yaml), only when they are off it.
#   scripts/hw/start_pose.sh                    # check; if an arm is off by more than the tolerance, bring both back
#   scripts/hw/start_pose.sh --check            # only report (exit 1 if off)
#   scripts/hw/start_pose.sh --tolerance 1.5    # degrees, largest joint (default 1.5; a move ends within ~1.1)
#   scripts/hw/start_pose.sh --pose <yaml>      # use an alternate pair of joint targets, e.g. warmup_grasp.yaml
# `fabric_run.sh go` runs this first; `fabric_run.sh recentre` runs it without the tolerance check.
# The move: scripts/hw/grasp_joints.py on each helper, ONE ARM AT A TIME (both at once hit each other on
# 2026-11-21), a straight line in joint space, MoveIt-checked against the other arm's actual pose and kept 15 cm
# from it. An arm moves only while the tablet deadman is held with Fabric Follow (CBF) on: release, press, hold
# for each arm; releasing stops it, pressing again resumes. Each arm holds still at the start pose until the
# deadman is released. The laptop speaks the steps (spd-say).
# The CBF bridges share the arms' joint input: they are stopped for the move and started again (disarmed).
# First it restarts both arms' Kinova velocity API (the driver's silent latch, see below).
# The whole script is one block, so bash reads all of it before running any: editing this file while a run
# from it is still going (2026-11-19 16:13: a go then executed part of deploy-helpers) cannot change that run.
{
set -uo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cd "$repo"
tol=1.5; check=0; force=0; pose="$repo/experiments/hardware/cell/grasp.yaml"
while [ $# -gt 0 ]; do case "$1" in
    --check) check=1 ;; --tolerance) tol="$2"; shift ;; --force) force=1 ;;
    --pose) pose="$(realpath -- "$2")"; shift ;;
    *) sed -n '2,13p' "$0"; exit 2 ;; esac; shift; done
# shellcheck source=lab_ssh.sh
source "$here/lab_ssh.sh"; lab_ssh_setup
set +u
# shellcheck disable=SC1091
source "$here/laptop_env.sh" >/dev/null 2>&1
set -u
state="${XDG_RUNTIME_DIR:-/tmp}/fabric_lab"; mkdir -p "$state"
say() { echo "$*"; spd-say -r -10 "$*" >/dev/null 2>&1 || true; }
echo "Start pose: $pose"
target_rows="$(python3 - "$pose" <<'PY'
import math, sys, yaml
pose = yaml.safe_load(open(sys.argv[1]))
for robot in ('blue', 'silver'):
    joints = list(map(float, pose[robot]))
    if len(joints) != 6 or not all(map(math.isfinite, joints)):
        raise SystemExit(f'{robot}: expected six finite joint angles')
    print(robot, *joints)
PY
)" || exit 1
declare -A target
while read -r robot joints; do target[$robot]="$joints"; done <<< "$target_rows"

# The patched Kinova driver latches "Velocity stream timed out: API stopped; explicit start service required" after
# any gap over 0.2 s in the guard's stream, and from then on ignores commands while bridge and guard look normal
# (Silver at 14:08 on 2026-11-19). Start both arms' API every time: it answers "Arm started" either way.
if [ "$check" = 0 ]; then
    for r in blue silver; do
        echo "  $r driver: $(timeout 40 scripts/hw/lab.sh session "$r" exec ros2 service call "/oarbot_$r/kinova/j2n6s300_driver/in/start" kinova_msgs/srv/Start 2>&1 | sed -n "s/.*start_result='\([^']*\)'.*/\1/p")"
    done
fi
offsets="$(python3 "$here/cell_check.py" --arm-offsets --pose "$pose")" || exit 1
echo "$offsets" | sed 's/^/  off the start pose (largest joint, deg): /'
if echo "$offsets" | grep -q none; then echo "No joint states from an arm: is its session running? (cell_up.sh)" >&2; exit 1; fi
if [ $force = 0 ] && echo "$offsets" | awk -v t="$tol" '$2 > t {bad=1} END {exit bad}'; then
    echo "Both arms at the start pose (within $tol deg)."; exit 0
fi
[ $check = 1 ] && { echo "Off the start pose: scripts/hw/start_pose.sh moves them (deadman)."; exit 1; }

# The bridges publish on the arms' joint input; the preflight refuses a second publisher: stop them first.
for r in blue silver; do scripts/hw/lab.sh session "$r" exec systemctl --user stop fabric-bridge >/dev/null 2>&1; done
# One arm at a time (2026-11-21 10:02: both arms moving at once from Home hit each other at 7 % of the path. Each
# preflight had checked its own path against the other arm standing still, 0.205 / 0.185 m, while both moved toward
# each other.) Each arm is preflighted against the other arm's actual pose just before it moves: the arm with more
# clearance goes first, the second is checked again after the first has arrived. An arm already within the
# tolerance stays (with --force both move).
restart_bridges() { for b in blue silver; do scripts/hw/lab.sh session "$b" start-cell-bridge "/home/oarbot_$b/fabric_ws/experiments/hardware/cell" >/dev/null 2>&1; done; }
preflight() {  # preflight <robot>: prints the clearance to the other arm (m); fails if MoveIt or the 15 cm check fails
    local out
    out="$(scripts/hw/lab.sh session "$1" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$1" --to ${target[$1]} --check-only 2>&1 | grep -E 'Preflight|ABORT')"
    echo "  $1 preflight: $(sed -E 's/.*"joint_change_deg": (\[[^]]*\]).*"closest_to_other_arm_m": ([0-9.]*).*/joint change \1 deg, other arm \2 m away/' <<< "$out")" >&2
    if ! grep -q 'Preflight passed' <<< "$out" || grep -q ABORT <<< "$out"; then say "Start pose: $1 failed its check" >&2; return 1; fi
    sed -E 's/.*"closest_to_other_arm_m": ([0-9.]*).*/\1/' <<< "$out"
}
move_arm() {  # move_arm <robot>: deadman-supervised move of one arm; fails on an abort
    local r="$1" name="${1^}" pid told=0
    say "Moving $name only. Release, then press and hold the deadman."
    scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --to ${target[$r]} --wait 600 > "$state/start_pose_$r.log" 2>&1 &
    pid=$!
    while kill -0 "$pid" 2>/dev/null; do
        if [ $told = 0 ] && grep -q Arrived "$state/start_pose_$r.log" 2>/dev/null; then say "$name at the start pose. Release the deadman."; told=1; fi
        sleep 0.5
    done
    wait "$pid"
    echo "  $r: $(grep -E 'Done|ABORT' "$state/start_pose_$r.log" | tail -1)"
    if grep -q ABORT "$state/start_pose_$r.log"; then say "Start pose aborted on $name"; return 1; fi
}
mapfile -t movers < <(echo "$offsets" | awk -v t="$tol" -v f="$force" '$2 > t || f == 1 {print $1}')
if [ "${#movers[@]}" = 2 ]; then
    cb="$(preflight blue)" || { restart_bridges; exit 1; }
    cs="$(preflight silver)" || { restart_bridges; exit 1; }
    if awk -v a="$cb" -v b="$cs" 'BEGIN {exit !(a >= b)}'; then movers=(blue silver); else movers=(silver blue); fi
    echo "  order: ${movers[0]} first (more clearance), then ${movers[1]} (checked again once ${movers[0]} has arrived)"
fi
for i in "${!movers[@]}"; do
    r="${movers[$i]}"
    if [ "$i" -gt 0 ] || [ "${#movers[@]}" = 1 ]; then preflight "$r" >/dev/null || { restart_bridges; exit 1; }; fi
    move_arm "$r" || { restart_bridges; exit 1; }
done
for r in blue silver; do
    scripts/hw/lab.sh session "$r" start-cell-bridge "/home/oarbot_$r/fabric_ws/experiments/hardware/cell" 2>&1 | tail -1 | sed "s/^/  $r: /"
done
offsets="$(python3 "$here/cell_check.py" --arm-offsets --pose "$pose")" || {
    say "Could not verify the start pose"; exit 1; }
echo "$offsets" | sed 's/^/  now off the start pose (deg): /'
if echo "$offsets" | awk -v t="$tol" '$2 == "none" || $2 > t {bad=1} END {exit bad}'; then
    say "Start pose reached"; exit 0
fi
say "An arm is still off the start pose"; exit 1
exit
}
