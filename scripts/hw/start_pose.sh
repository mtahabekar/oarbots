#!/usr/bin/env bash
# Both arms to the experiment's start pose (experiments/hardware/cell/grasp.yaml), only when they are off it.
#   scripts/hw/start_pose.sh                    # check; if an arm is off by more than the tolerance, bring both back
#   scripts/hw/start_pose.sh --check            # only report (exit 1 if off)
#   scripts/hw/start_pose.sh --tolerance 1.5    # degrees, largest joint (default 1.5; a move ends within ~1.1)
# `fabric_run.sh go` runs this first; `fabric_run.sh recentre` runs it without the tolerance check.
# The move: scripts/hw/grasp_joints.py on each helper, both arms at once, a straight line in joint space,
# MoveIt-checked and kept 15 cm from the other arm. The arms move only while the tablet deadman is held with
# Fabric Follow (CBF) on for both: release, press, hold; releasing stops them, pressing again resumes. Each arm
# holds still at the start pose until the deadman is released. The laptop speaks the steps (spd-say).
# The CBF bridges share the arms' joint input: they are stopped for the move and started again (disarmed).
# First it restarts both arms' Kinova velocity API (the driver's silent latch, see below).
# The whole script is one block, so bash reads all of it before running any: editing this file while a run
# from it is still going (2026-11-19 16:13: a go then executed part of deploy-helpers) cannot change that run.
{
set -uo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cd "$repo"
tol=1.5; check=0; force=0
while [ $# -gt 0 ]; do case "$1" in
    --check) check=1 ;; --tolerance) tol="$2"; shift ;; --force) force=1 ;;
    *) sed -n '2,12p' "$0"; exit 2 ;; esac; shift; done
# shellcheck source=lab_ssh.sh
source "$here/lab_ssh.sh"; lab_ssh_setup
set +u
# shellcheck disable=SC1091
source "$here/laptop_env.sh" >/dev/null 2>&1
set -u
state="${XDG_RUNTIME_DIR:-/tmp}/fabric_lab"; mkdir -p "$state"
say() { echo "$*"; spd-say -r -10 "$*" >/dev/null 2>&1 || true; }

# The patched Kinova driver latches "Velocity stream timed out: API stopped; explicit start service required" after
# any gap over 0.2 s in the guard's stream, and from then on ignores commands while bridge and guard look normal
# (Silver at 14:08 on 2026-11-19). Start both arms' API every time: it answers "Arm started" either way.
for r in blue silver; do
    echo "  $r driver: $(timeout 40 scripts/hw/lab.sh session "$r" exec ros2 service call "/oarbot_$r/kinova/j2n6s300_driver/in/start" kinova_msgs/srv/Start 2>&1 | sed -n "s/.*start_result='\([^']*\)'.*/\1/p")"
done
offsets="$(python3 "$here/cell_check.py" --arm-offsets)"
echo "$offsets" | sed 's/^/  off the start pose (largest joint, deg): /'
if echo "$offsets" | grep -q none; then echo "No joint states from an arm: is its session running? (cell_up.sh)" >&2; exit 1; fi
if [ $force = 0 ] && echo "$offsets" | awk -v t="$tol" '$2 > t {bad=1} END {exit bad}'; then
    echo "Both arms at the start pose (within $tol deg)."; exit 0
fi
[ $check = 1 ] && { echo "Off the start pose: scripts/hw/start_pose.sh moves them (deadman)."; exit 1; }

# The bridges publish on the arms' joint input; the preflight refuses a second publisher: stop them first.
for r in blue silver; do scripts/hw/lab.sh session "$r" exec systemctl --user stop fabric-bridge >/dev/null 2>&1; done
for r in blue silver; do
    out="$(scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --check-only 2>&1 | grep -E 'Preflight|ABORT')"
    echo "  $r preflight: $(sed -E 's/.*"joint_change_deg": (\[[^]]*\]).*"closest_to_other_arm_m": ([0-9.]*).*/joint change \1 deg, other arm \2 m away/' <<< "$out")"
    if grep -q ABORT <<< "$out"; then
        say "Start pose: $r failed its check"
        for b in blue silver; do scripts/hw/lab.sh session "$b" start-cell-bridge "/home/oarbot_$b/fabric_ws/experiments/hardware/cell" >/dev/null 2>&1; done
        exit 1
    fi
done
say "Arms are off the start pose. Release, then press and hold the deadman to bring them back."
for r in blue silver; do
    scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --wait 600 > "$state/start_pose_$r.log" 2>&1 &
done
told=0   # 1: arrival announced, 2: abort announced
while [ -n "$(jobs -rp)" ]; do
    if [ $told = 0 ] && grep -q Arrived "$state/start_pose_blue.log" 2>/dev/null && grep -q Arrived "$state/start_pose_silver.log" 2>/dev/null; then
        say "Arms at the start pose. Release the deadman."; told=1
    fi
    if [ $told != 2 ] && grep -q ABORT "$state"/start_pose_*.log 2>/dev/null; then say "Start pose aborted"; told=2; fi
    sleep 0.5
done
wait
for r in blue silver; do echo "  $r: $(grep -E 'Done|ABORT' "$state/start_pose_$r.log" | tail -1)"; done
for r in blue silver; do
    scripts/hw/lab.sh session "$r" start-cell-bridge "/home/oarbot_$r/fabric_ws/experiments/hardware/cell" 2>&1 | tail -1 | sed "s/^/  $r: /"
done
offsets="$(python3 "$here/cell_check.py" --arm-offsets)"
echo "$offsets" | sed 's/^/  now off the start pose (deg): /'
if echo "$offsets" | awk -v t="$tol" '$2 == "none" || $2 > t {bad=1} END {exit bad}'; then
    say "Start pose reached"; exit 0
fi
say "An arm is still off the start pose"; exit 1
exit
}
