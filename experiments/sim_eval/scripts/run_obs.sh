#!/usr/bin/env bash
# Observer isolation (study S1b): replay one identification dataset's recorded camera
# observations, glove detections, TF and joint states through the estimator stack (no Isaac Sim).
#   run_obs.sh DATASET CONDITION     CONDITION: vision | novision | oracle_vision
#     vision         glove hand filter, camera factors on (the live configuration)
#     novision       glove hand filter, no camera factors: the ensemble driven by grips alone
#     oracle_vision  plant mocap hands (labelled oracle), camera factors on
set -o pipefail
HERE=$(dirname "$(realpath "$0")"); ROOT=$(realpath "$HERE/.."); REPO=$(realpath "$ROOT/../..")
DATA=${SIM_EVAL_DATA:-/media/taha/ExtermeLinux/oarbot_simulation/logs/sim_eval}
DATASET=$1; CONDITION=$2
RUN=$DATA/id/$DATASET; OUT=$DATA/obs/${DATASET}.${CONDITION}
[[ -f "$OUT/estimate.npz" ]] && { echo "$DATASET $CONDITION exists; skipping"; exit 0; }
INSTALLED=/home/taha/ros2_colcon_ws/install/fabric_sim_cell/share/fabric_sim_cell/config/estimator_fast.yaml
case $CONDITION in
  vision) ARGS="hands:=gloves profile:=fast estimator_config:=$INSTALLED" ;;
  novision) ARGS="hands:=gloves profile:=fast estimator_config:=$ROOT/config/estimator_novision.yaml" ;;
  oracle_vision) ARGS="hands:=oracle profile:=fast estimator_config:=$INSTALLED" ;;
  *) echo "unknown condition $CONDITION" >&2; exit 2 ;;
esac
mkdir -p "$OUT"
"$REPO/src/fabric_sim_cell/scripts/replay_bag.sh" "$RUN" "$OUT" "$ARGS" attached 10.0 147 > "$OUT/run.log" 2>&1
STATUS=$?
echo "{\"dataset\": \"$DATASET\", \"condition\": \"$CONDITION\", \"launch\": \"$ARGS\", \"exit_status\": $STATUS}" > "$OUT/manifest.json"
echo "$DATASET $CONDITION exit $STATUS"
exit $STATUS
