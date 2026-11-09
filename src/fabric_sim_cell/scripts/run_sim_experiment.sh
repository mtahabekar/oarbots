#!/usr/bin/env bash
# One experiment against the simulated cell: estimator stack first, then the plant, then scoring.
#   run_sim_experiment.sh OUT_DIR "PLANT_ARGS" "LAUNCH_ARGS" [WINDOWS...]
# OARBOT_SIM: path to oarbot_simulation (default /media/taha/ExtermeLinux/oarbot_simulation).
set -o pipefail  # not -u: ROS setup scripts read unset variables
OUT=$(realpath -m "$1"); PLANT_ARGS=$2; LAUNCH_ARGS=$3; shift 3
SIM=${OARBOT_SIM:-/media/taha/ExtermeLinux/oarbot_simulation}
WS=${COLCON_WS:-/home/taha/ros2_colcon_ws}

# A consumer stack left from an earlier run shares every topic and TF frame with this one (its
# calibration publisher, for instance, competes with a deliberately perturbed one). Refuse to start.
STALE=$(pgrep -af "ros2 launch fabric_sim_cell|/install/(fabric_sim_cell|fabric_perception|human_hand_tracking|fabric_state_estimator)/lib/|run_oarbot_fabric.py" || true)
if [[ -n "$STALE" ]]; then
  echo "SIM_EXPERIMENT_ABORTED: sim-cell processes are already running; stop them first:" >&2
  echo "$STALE" >&2
  exit 3
fi
mkdir -p "$OUT"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$SIM/config/fastdds_large_messages.xml"

# The stack gets its own process group, and SIGINT is restored to its default first: a command
# started in the background of a non-interactive shell inherits SIGINT as ignored, and such a
# launch never shuts down (its nodes then linger across later runs).
setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("bash", ["bash", "-c", sys.argv[1]])' \
  "source /opt/ros/jazzy/setup.bash; source '$WS/install/setup.bash'; exec ros2 launch fabric_sim_cell sim_cell.launch.py urdf_dir:='$SIM/assets/oarbots_jazzy' record:='$OUT/estimate.npz' $LAUNCH_ARGS" \
  > "$OUT/stack.log" 2>&1 &
STACK=$!
stop_stack() {
  [[ -z "$STACK" ]] && return
  kill -INT "$STACK" 2>/dev/null
  for _ in $(seq 60); do kill -0 "$STACK" 2>/dev/null || break; sleep 0.5; done
  if kill -0 "$STACK" 2>/dev/null; then
    echo "[run_sim_experiment] stack still running 30 s after SIGINT; terminating its process group" >> "$OUT/stack.log"
    kill -TERM -- "-$STACK" 2>/dev/null; sleep 10
  fi
  kill -KILL -- "-$STACK" 2>/dev/null  # anything that outlived the launch
  wait "$STACK" 2>/dev/null
  STACK=
}
trap stop_stack EXIT
# The estimator compiles its Newton kernels before accepting captures.
for _ in $(seq 120); do grep -q "Preparing physics backend" "$OUT/stack.log" 2>/dev/null && break; sleep 1; done
sleep 20
nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader > "$OUT/gpu_before_plant.txt" 2>/dev/null
# PLANT_GUI=1 opens the Isaac Sim window (live demo); default headless.
HEADLESS="--headless"; [[ "${PLANT_GUI:-0}" == 1 ]] && HEADLESS=""
( cd "$SIM" && timeout 5400 ./run_oarbot_fabric.sh $HEADLESS --fast --validate --sensors --gloves --mocap-hands \
    --truth-output "$OUT/truth.npz" --report "$OUT/plant.json" $PLANT_ARGS ) > "$OUT/plant.log" 2>&1 &
PLANT=$!
PEAK=0
while kill -0 $PLANT 2>/dev/null; do
  used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
  [[ -n "$used" && "$used" -gt "$PEAK" ]] && PEAK=$used
  sleep 5
done
PLANT_RESULT=0
wait "$PLANT" || PLANT_RESULT=$?
echo "plant exit $PLANT_RESULT" >> "$OUT/plant.log"
echo "$PEAK" > "$OUT/gpu_peak_mib.txt"
sleep 3
stop_stack
if [[ "$PLANT_RESULT" -ne 0 ]] || ! grep -q COMBINED_VALIDATED "$OUT/plant.log"; then
  echo "SIM_EXPERIMENT_ABORTED: plant validation failed (exit $PLANT_RESULT); see $OUT/plant.log" >&2
  exit 4
fi
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
WINDOWS=()
while [[ $# -ge 3 ]]; do WINDOWS+=(--window "$1" "$2" "$3"); shift 3; done
CONFIG=$(python3 -c "import sys; a=sys.argv[1].split(); print(next((x.split(':=')[1] for x in a if x.startswith('estimator_config:=')), ''))" "$LAUNCH_ARGS")
if [[ -z "$CONFIG" ]]; then  # the launch default, which profile:=fast changes
  SUFFIX=""; [[ " $LAUNCH_ARGS " == *" profile:=fast "* ]] && SUFFIX="_fast"
  [[ " $LAUNCH_ARGS " == *" profile:=monitor "* ]] && SUFFIX="_monitor"
  CONFIG="$WS/install/fabric_sim_cell/share/fabric_sim_cell/config/estimator$SUFFIX.yaml"
fi
ros2 run fabric_sim_cell evaluate "$OUT/truth.npz" "$OUT/estimate.npz" "$CONFIG" "$OUT/eval" "${WINDOWS[@]}" 2>&1 | tee "$OUT/eval.log"
