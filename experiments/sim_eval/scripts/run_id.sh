#!/usr/bin/env bash
# One identification dataset (study S1): the Isaac plant with a hidden material, the full
# perception stack, the twin driving the robots (warm-up excitation, then formation following;
# CBF unfiltered, stretch guard on), and recording of every input the offline tests replay.
#
#   run_id.sh STRUCTURE MATERIAL REPEAT
#     STRUCTURE  matched: plant mesh and solver identical to the twin (28 x 28 cells, 6/3 iterations)
#                fine:    the default rehearsal plant (56 x 56 cells, 8/8 iterations, 3 x 3 = 3 cm grip patches)
#     MATERIAL   A (bend x1, shear x1), B (x3, x0.4), C (x1/3, x2.5) applied to the plant only
#     REPEAT     integer; repeats differ only by run-to-run timing of the asynchronous stack
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
ROOT=$(realpath "$HERE/..")
REPO=$(realpath "$ROOT/../..")
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
STRUCTURE=$1; MATERIAL=$2; REPEAT=$3
case $MATERIAL in
  A) BEND=1.0; SHEAR=1.0 ;;
  B) BEND=3.0; SHEAR=0.4 ;;
  C) BEND=0.3333333; SHEAR=2.5 ;;
  *) echo "unknown material $MATERIAL" >&2; exit 2 ;;
esac
case $STRUCTURE in
  # --gripper-size 2: a 3 cm pinned patch as in the twin (the plant's default 3 x 3 particles is 6 cm at
  # this spacing; with it, a twin with the true coefficients sagged 14 mm away from the plant in 2 s).
  matched) MESH="--grid-x 28 --grid-y 28 --cell-x 0.03 --cell-y 0.03 --gripper-size 2 --iterations 6 --linear-iterations 3" ;;
  fine) MESH="" ;;
  *) echo "unknown structure $STRUCTURE" >&2; exit 2 ;;
esac
# Plant stiffness = the twin's nominal Style3D values (TwinSpec) times the hidden scales.
MATERIAL_ARGS=$(python3 -c "b,s=$BEND,$SHEAR; print(f'--edge-ke-weft {0.002*b:.6g} --edge-ke-warp {0.001*b:.6g} --edge-ke-shear {0.0005*b:.6g} --tri-ke-shear {6000*s:.6g}')")
NAME=id_${STRUCTURE}_${MATERIAL}_r${REPEAT}
OUT=$DATA/id/$NAME
if [[ -f "$OUT/manifest.json" ]]; then echo "$NAME exists; skipping"; exit 0; fi
rm -rf "$OUT"; mkdir -p "$OUT"
CONFIG=$ROOT/config/twin_id.yaml
cp "$CONFIG" "$OUT/twin.yaml"
LAUNCH="twin:=true twin_commands:=true twin_config:=$CONFIG twin_record:=$OUT/twin.npz twin_record_depth:=true"
PLANT="--external-followers --overhead-hz 15 $MESH $MATERIAL_ARGS"
START=$(date -u +%FT%TZ)
DURATION=147 "$REPO/src/fabric_sim_cell/scripts/run_teleop_replay.sh" "$DATA/id_script.npz" "$OUT" gloves "$LAUNCH" "$PLANT" > "$OUT/run.log" 2>&1
STATUS=$?
python3 - "$OUT" <<EOF
import json, subprocess, sys
repo = '$REPO'
commit = subprocess.run(['git', '-C', repo, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
dirty = bool(subprocess.run(['git', '-C', repo, 'status', '--porcelain', 'src'], capture_output=True, text=True).stdout.strip())
json.dump(dict(study='S1-identification-datasets', name='$NAME', structure='$STRUCTURE', material='$MATERIAL',
               true_scales=dict(bend=$BEND, shear=$SHEAR), repeat=$REPEAT, exit_status=$STATUS, started='$START',
               hand_script='$DATA/id_script.npz', twin_config='$CONFIG', plant_args='$PLANT', launch_args='$LAUNCH',
               warmup=dict(start_s=15.0, end_s=93.333), validation_window_s=[97.5, 140.4],
               oarbots_commit=commit, oarbots_src_dirty=dirty), open(sys.argv[1]+'/manifest.json', 'w'), indent=1)
EOF
echo "$NAME exit $STATUS"
if [[ $STATUS -ne 0 ]]; then  # keep failed attempts for diagnosis, out of the way of a retry
  mkdir -p "$DATA/id/failed"; mv "$OUT" "$DATA/id/failed/${NAME}_$(date +%H%M%S)"
fi
exit $STATUS
