#!/usr/bin/env bash
# One closed-loop carry (study S4): the twin's commands drive the robots past the obstacle.
#
#   run_closed.sh CONDITION MATERIAL REPEAT [BEND SHEAR]
#     CONDITION  B0  no clearance CBF (formation following + stretch guard)
#                B1  CBF on the grip-driven twin (correction off), nominal coefficients
#                B2  CBF on the grip-driven twin, warm-up-fitted coefficients BEND SHEAR
#                P0  CBF on the corrected twin, nominal coefficients
#                P   CBF on the corrected twin, warm-up-fitted coefficients BEND SHEAR
#                G0  P without the stretch guard
#     MATERIAL   hidden plant material (A, B, C as in run_id.sh); the plant is the fine 56 x 56 mesh
# Fitted coefficients come from study S2 (offline fit of the identification datasets of the same
# plant), so a closed-loop result does not depend on one live calibration.
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
ROOT=$(realpath "$HERE/..")
REPO=$(realpath "$ROOT/../..")
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
CONDITION=$1; MATERIAL=$2; REPEAT=$3; FIT_BEND=${4:-1.0}; FIT_SHEAR=${5:-1.0}
case $MATERIAL in
  A) BEND=1.0; SHEAR=1.0 ;;
  B) BEND=3.0; SHEAR=0.4 ;;
  C) BEND=0.3333333; SHEAR=2.5 ;;
  *) echo "unknown material $MATERIAL" >&2; exit 2 ;;
esac
case $CONDITION in B0|B1|P0) FIT_BEND=1.0; FIT_SHEAR=1.0 ;; B2|P|G0) ;; *) echo "unknown condition $CONDITION" >&2; exit 2 ;; esac
MATERIAL_ARGS=$(python3 -c "b,s=$BEND,$SHEAR; print(f'--edge-ke-weft {0.002*b:.6g} --edge-ke-warp {0.001*b:.6g} --edge-ke-shear {0.0005*b:.6g} --tri-ke-shear {6000*s:.6g}')")
NAME=cl_${CONDITION}_${MATERIAL}_r${REPEAT}
OUT=$DATA/closed/$NAME
if [[ -f "$OUT/manifest.json" ]]; then echo "$NAME exists; skipping"; exit 0; fi
rm -rf "$OUT"; mkdir -p "$OUT"
python3 - "$REPO/src/fabric_sim_cell/config/twin.yaml" "$OUT/twin.yaml" "$CONDITION" "$FIT_BEND" "$FIT_SHEAR" <<'EOF'
import sys, yaml
source, target, condition, bend, shear = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), float(sys.argv[5])
c = yaml.safe_load(open(source))
c['runtime']['delay_sec'] = 0.15            # as in the carry3 rehearsals (overhead camera at 15 Hz)
c['runtime']['warmup_at_sec'] = -1.0        # no warm-up: coefficients are given
c['sheet']['bend'] = [float(x)*bend for x in c['sheet']['bend']]
c['sheet']['stretch'][2] = float(c['sheet']['stretch'][2])*shear
c.setdefault('feedback', {})['enabled'] = condition not in ('B1', 'B2')
c['cbf']['controller'] = 'none' if condition == 'B0' else 'sensitivity'
c['stretch_guard']['enabled'] = condition != 'G0'
open(target, 'w').write(f'# sim_eval closed-loop condition {condition}; fitted scales bend x{bend}, shear x{shear}\n'
                        + yaml.safe_dump(c, sort_keys=False))
EOF
LAUNCH="twin:=true twin_commands:=true twin_config:=$OUT/twin.yaml twin_record:=$OUT/twin.npz"
PLANT="--external-followers --overhead-hz 15 $MATERIAL_ARGS"
START=$(date -u +%FT%TZ)
DURATION=55 "$REPO/src/fabric_sim_cell/scripts/run_teleop_replay.sh" "$DATA/carry_script.npz" "$OUT" gloves "$LAUNCH" "$PLANT" > "$OUT/run.log" 2>&1
STATUS=$?
python3 - "$OUT" <<EOF
import json, subprocess, sys
repo = '$REPO'
commit = subprocess.run(['git', '-C', repo, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
dirty = bool(subprocess.run(['git', '-C', repo, 'status', '--porcelain', 'src'], capture_output=True, text=True).stdout.strip())
json.dump(dict(study='S4-closed-loop', name='$NAME', condition='$CONDITION', structure='fine', material='$MATERIAL',
               true_scales=dict(bend=$BEND, shear=$SHEAR), twin_scales=dict(bend=$FIT_BEND, shear=$FIT_SHEAR),
               repeat=$REPEAT, exit_status=$STATUS, started='$START', hand_script='$DATA/carry_script.npz',
               plant_args='$PLANT', launch_args='$LAUNCH', carry_window_s=[12.0, 54.8],
               oarbots_commit=commit, oarbots_src_dirty=dirty), open(sys.argv[1]+'/manifest.json', 'w'), indent=1)
EOF
echo "$NAME exit $STATUS"
if [[ $STATUS -ne 0 ]]; then
  mkdir -p "$DATA/closed/failed"; mv "$OUT" "$DATA/closed/failed/${NAME}_$(date +%H%M%S)"
fi
exit $STATUS
