#!/usr/bin/env bash
# The simulated-cell experiment set, run one at a time (one Isaac process on the GPU).
#   run_experiment_set.sh OUT_ROOT [NAME ...]     names: baseline gloves forces_on forces_off calib_errors material
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
WS=${COLCON_WS:-/home/taha/ros2_colcon_ws}
SHARE="$WS/install/fabric_sim_cell/share/fabric_sim_cell/config"
OUT=$(realpath -m "$1"); shift
mkdir -p "$OUT"
# Robots attach at ~9.0 s and report closed fingers at ~10.0 s, when the grasp gate starts estimation.
# The leaders then hold for 2 s (11.0 s), descend 0.5 m over 5 s (to 16.0 s) and hold to the end (18.5 s).
DESCEND="--attach --route-cbf --demo-leader 0 0 -0.1 --demo-after 120 --demo-duration 300 --steps 1110"
WINDOWS=(attached 10.0 18.5 hold 10.0 11.0 descent 11.0 16.0 after 16.0 18.5)
declare -A PLANT LAUNCH
PLANT[baseline]="$DESCEND";                 LAUNCH[baseline]="hands:=oracle"
PLANT[gloves]="$DESCEND";                   LAUNCH[gloves]="hands:=gloves"
PLANT[forces_on]="$DESCEND --couple-forces";  LAUNCH[forces_on]="hands:=oracle force_torque:=true estimator_config:=$SHARE/estimator_forces.yaml"
PLANT[forces_off]="$DESCEND --couple-forces"; LAUNCH[forces_off]="hands:=oracle force_torque:=true"
PLANT[calib_errors]="$DESCEND";             LAUNCH[calib_errors]="hands:=oracle errors_file:=$SHARE/errors_example.yaml"
PLANT[material]="$DESCEND";                 LAUNCH[material]="hands:=oracle estimator_config:=$SHARE/estimator_perturbed.yaml"
NAMES=("$@"); [[ ${#NAMES[@]} -eq 0 ]] && NAMES=(baseline gloves forces_on forces_off calib_errors material)
for name in "${NAMES[@]}"; do
  echo "== $name"
  "$HERE/run_sim_experiment.sh" "$OUT/$name" "${PLANT[$name]}" "${LAUNCH[$name]}" "${WINDOWS[@]}" > "$OUT/$name.run.log" 2>&1
  grep -E "^analysis_attached|^analysis_descent|SIM_CELL_EVALUATED|Error|Traceback" "$OUT/$name.run.log" | head -4
done
echo EXPERIMENT_SET_DONE
