#!/usr/bin/env bash
# Full live loop (study S5): hidden plant material, live warm-up, live parameter-bank calibration
# released to the twin and its CBF model, online adaptation every 10 s, then the obstacle carry
# under the CBF on the corrected twin. Fine plant.
#   run_full.sh MATERIAL REPEAT
# The hands stay still until 480 s (simulation time), long enough for the live calibration of the
# 13-hold warm-up to finish beside Isaac Sim; the carry then repeats the S4 hand motion.
set -o pipefail
HERE=$(dirname "$(realpath "$0")"); ROOT=$(realpath "$HERE/.."); REPO=$(realpath "$ROOT/../..")
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
MATERIAL=$1; REPEAT=$2
case $MATERIAL in
  A) BEND=1.0; SHEAR=1.0 ;; B) BEND=3.0; SHEAR=0.4 ;; C) BEND=0.3333333; SHEAR=2.5 ;;
  *) echo "unknown material $MATERIAL" >&2; exit 2 ;;
esac
MATERIAL_ARGS=$(python3 -c "b,s=$BEND,$SHEAR; print(f'--edge-ke-weft {0.002*b:.6g} --edge-ke-warp {0.001*b:.6g} --edge-ke-shear {0.0005*b:.6g} --tri-ke-shear {6000*s:.6g}')")
NAME=full_${MATERIAL}_r${REPEAT}
OUT=$DATA/full/$NAME
if [[ -f "$OUT/manifest.json" ]]; then echo "$NAME exists; skipping"; exit 0; fi
rm -rf "$OUT"; mkdir -p "$OUT"
python3 - "$REPO/src/fabric_sim_cell/config/twin_identification.yaml" "$OUT/twin.yaml" <<'PY'
import sys, yaml
c = yaml.safe_load(open(sys.argv[1]))
c['runtime']['delay_sec'] = 0.15
c['cbf']['controller'] = 'sensitivity'
open(sys.argv[2], 'w').write('# sim_eval S5 full live loop: extended warm-up at 15 s, CBF and correction on\n'
                             + yaml.safe_dump(c, sort_keys=False))
PY
LAUNCH="twin:=true twin_commands:=true twin_bank:=true twin_adapt_period_sec:=10.0 twin_config:=$OUT/twin.yaml twin_record:=$OUT/twin.npz twin_record_depth:=true"
PLANT="--external-followers --overhead-hz 15 $MATERIAL_ARGS"
START=$(date -u +%FT%TZ)
DURATION=525 "$REPO/src/fabric_sim_cell/scripts/run_teleop_replay.sh" "$DATA/full_script.npz" "$OUT" gloves "$LAUNCH" "$PLANT" > "$OUT/run.log" 2>&1
STATUS=$?
python3 - "$OUT" <<PY
import json, subprocess, sys
repo = '$REPO'
commit = subprocess.run(['git', '-C', repo, 'rev-parse', 'HEAD'], capture_output=True, text=True).stdout.strip()
json.dump(dict(study='S5-full-live-loop', name='$NAME', structure='fine', material='$MATERIAL', condition='full',
               true_scales=dict(bend=$BEND, shear=$SHEAR), repeat=$REPEAT, exit_status=$STATUS, started='$START',
               hand_script='$DATA/full_script.npz', plant_args='$PLANT', launch_args='$LAUNCH',
               warmup=dict(start_s=15.0, end_s=93.333), carry_window_s=[478.0, 520.8], oarbots_commit=commit),
          open(sys.argv[1]+'/manifest.json', 'w'), indent=1)
PY
echo "$NAME exit $STATUS"
if [[ $STATUS -ne 0 ]]; then mkdir -p "$DATA/full/failed"; mv "$OUT" "$DATA/full/failed/${NAME}_$(date +%H%M%S)"; fi
exit $STATUS
