#!/usr/bin/env bash
# The fabric twin and its CBF driving the simulated robots, unattended: today's settings against the baseline.
#   run_twin_check.sh [baseline|today|both|limits|obstacle|all] [REPEATS=1] [OUT_ROOT]
# Scenario (the live demo's): the robots grasp at ~9-10 s simulated; the leader corners (the hands, oracle input)
# hold to ~11 s, descend DESCENT_M (default 0.5 m) at 0.1 m/s toward the table and hold to the end (25 s). From
# their 1.05 m, 0.5 m leaves the sheet ~10 cm above the 0.32 m table (the CBF barely acts); DESCENT_M=0.6 makes
# the hands pull the sheet toward it (the CBF must hold the robot grips up). The twin (sensors
# only: cameras, oracle hands, joint states) commands both robot grips (--external-followers); the plant only
# clamps them to --follower-max-speed and records the true sheet.
#   baseline  the simulated cell's twin.yaml (10 cm/s, CBF margin 4 cm, stretch slack 3 cm; capsule_cbf's capsule
#             around the table box, cap scale 2); plant follower limit 0.10 m/s
#   today     the 2026-11-19 hardware limits (config/twin_limits_2026-11-19.yaml: 15 cm/s, margin 4.5 cm, slack
#             3.5 cm); the table as a detected obstacle: prism + capsule fitted to it (table_obstacle.py), so the
#             twin's sheet collides with the prism and the CBF keeps clear of the fitted capsule; plant limit 0.15 m/s
#   limits    today's limits and plant speed, baseline obstacle;  obstacle  baseline limits, today's obstacle (each
#             changes one thing, to tell which one a difference comes from); all = the four modes
# All use today's twin code (a virtual collision no longer zeroes the commands). Needs the ExtermeLinux disk
# (Isaac Sim, oarbot_simulation) and no other sim-cell or twin process. Each run ~3 min; Isaac headless.
# Output: OUT_ROOT/<mode>_<n>/ (run_sim_experiment.sh's files + twin.npz, twin_eval.json) and OUT_ROOT/report.md.
{
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
REPO=$(realpath "$HERE/../../..")
SIM=${OARBOT_SIM:-/media/taha/ExtermeLinux/oarbot_simulation}
[ -d "$SIM" ] || { echo "Simulation not found at $SIM: connect the ExtermeLinux disk (mounted at /media/taha/ExtermeLinux) or set OARBOT_SIM" >&2; exit 2; }
WS=${COLCON_WS:-/home/taha/ros2_colcon_ws}
MODE=${1:-both}; REPEATS=${2:-1}
OUT_ROOT=$(realpath -m "${3:-$SIM/logs/sim_cell/twin_check_$(date +%Y%m%d_%H%M%S)_descent${DESCENT_M:-0.5}}")
case "$MODE" in baseline|today|limits|obstacle) MODES=("$MODE") ;; both) MODES=(baseline today) ;;
  all) MODES=(baseline today limits obstacle) ;; *) echo "mode: baseline, today, both, limits, obstacle or all" >&2; exit 2 ;; esac
head -1 "$WS/install/fabric_twin/lib/fabric_twin/twin" | grep -q venvs/fabric_ros || {
    echo "The twin's launcher does not use ~/venvs/fabric_ros: run scripts/hw/fabric_run.sh build-laptop" >&2; exit 2; }
mkdir -p "$OUT_ROOT"
git -C "$REPO" rev-parse HEAD > "$OUT_ROOT/commit.txt"; git -C "$REPO" status --short >> "$OUT_ROOT/commit.txt"
python3 "$HERE/table_obstacle.py" "$OUT_ROOT/table_obstacle.yaml" | tee "$OUT_ROOT/table_obstacle.txt"

PLANT="--attach --route-cbf --external-followers --steps 1500 --render-mode due --render-sync --wrist-resolution 640 360 \
--overhead-scale 0.5 --physics-hz 60 --robot-collisions none --physics-fabric --cbf-period 4 --iterations 8 --linear-iterations 8 \
--state-publish-every 1000000 --demo-leader 0 0 -0.1 --demo-after 120 --demo-duration $(python3 -c "print(round(${DESCENT_M:-0.5}/0.1*60))")"
LAUNCH="hands:=oracle profile:=fast twin:=true twin_commands:=true"
for mode in "${MODES[@]}"; do
  for n in $(seq "$REPEATS"); do
    out="$OUT_ROOT/${mode}_$n"
    limits_cfg="twin_config:=$WS/install/fabric_sim_cell/share/fabric_sim_cell/config/twin_limits_2026-11-19.yaml"
    obstacle_arg="twin_obstacle_file:=$OUT_ROOT/table_obstacle.yaml"
    case "$mode" in
      today)    plant="$PLANT --follower-max-speed 0.15"; launch="$LAUNCH $limits_cfg $obstacle_arg" ;;
      limits)   plant="$PLANT --follower-max-speed 0.15"; launch="$LAUNCH $limits_cfg" ;;
      obstacle) plant="$PLANT --follower-max-speed 0.10"; launch="$LAUNCH $obstacle_arg" ;;
      *)        plant="$PLANT --follower-max-speed 0.10"; launch="$LAUNCH" ;;
    esac
    launch="$launch twin_record:=$out/twin.npz"
    echo "== $mode $n -> $out ($(date +%H:%M:%S))"
    TWIN_START=12 "$HERE/run_sim_experiment.sh" "$out" "$plant" "$launch" attached 10.0 25.0 descent 11.0 16.0 after 16.0 25.0 \
      > "$out.console.log" 2>&1
    echo "   exit $? ($(date +%H:%M:%S)); $(grep -m1 -o 'SIM_EXPERIMENT_ABORTED.*' "$out.console.log" || echo 'completed')"
  done
done
python3 "$HERE/twin_check_report.py" "$OUT_ROOT" --start 10
echo "Report: $OUT_ROOT/report.md"
exit
}
