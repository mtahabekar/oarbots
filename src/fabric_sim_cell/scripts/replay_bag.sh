#!/usr/bin/env bash
# Replay a recorded simulated-cell run through the estimator stack, without Isaac Sim.
#
#   replay_bag.sh RUN_DIR OUT_DIR [LAUNCH_ARGS] [WINDOWS...]
#
# RUN_DIR is the output of run_teleop_replay.sh (bag/ and truth.npz). The bag's compact camera
# observations, glove measurements, TF and joint states are played at real-time rate on a bag
# clock; hand fusion, the grasp gate, the estimator and the recorder consume them exactly as in
# a live run. The result is scored against RUN_DIR/truth.npz. RATE=0.5 slows the playback.
set -o pipefail
RUN=$(realpath "$1"); OUT=$(realpath -m "$2"); LAUNCH_ARGS=${3:-"hands:=gloves profile:=fast"}; shift 3 2>/dev/null || shift $#
WS=${COLCON_WS:-/home/taha/ros2_colcon_ws}
[[ -d "$RUN/bag" && -f "$RUN/truth.npz" ]] || { echo "RUN_DIR needs bag/ and truth.npz" >&2; exit 2; }
STALE=$(pgrep -af "ros2 launch fabric_sim_cell|/install/(fabric_sim_cell|fabric_perception|human_hand_tracking|fabric_state_estimator|fabric_twin)/lib/|run_oarbot_fabric.py|ros2 bag play" || true)
if [[ -n "$STALE" ]]; then
  echo "REPLAY_ABORTED: sim-cell processes are already running:" >&2; echo "$STALE" >&2; exit 3
fi
mkdir -p "$OUT"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-43} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"
setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp("bash", ["bash", "-c", sys.argv[1]])' \
  "exec ros2 launch fabric_sim_cell sim_cell.launch.py replay:=true record:='$OUT/estimate.npz' $LAUNCH_ARGS" > "$OUT/stack.log" 2>&1 &
STACK=$!
stop_stack() {
  [[ -z "$STACK" ]] && return
  kill -INT "$STACK" 2>/dev/null
  for _ in $(seq 40); do kill -0 "$STACK" 2>/dev/null || break; sleep 0.5; done
  kill -KILL -- "-$STACK" 2>/dev/null
  wait "$STACK" 2>/dev/null
  STACK=
}
trap stop_stack EXIT
for _ in $(seq 120); do grep -q "Preparing physics backend" "$OUT/stack.log" 2>/dev/null && break; sleep 1; done
sleep ${WARMUP:-15}
INPUTS="/tf /tf_static /sim/mocap/glove_left /sim/mocap/glove_right"
for camera in /nuc /oarbot_blue/azure_kinect /oarbot_silver/azure_kinect; do INPUTS+=" $camera/fabric/observations $camera/fabric/gloves"; done
for robot in oarbot_blue oarbot_silver; do INPUTS+=" /$robot/kinova/j2n6s300_driver/out/joint_state"; done
ros2 bag play "$RUN/bag" --clock 200 --rate "${RATE:-1.0}" --topics $INPUTS > "$OUT/play.log" 2>&1
sleep 2
stop_stack
CONFIG=$(python3 -c "import sys; a=sys.argv[1].split(); print(next((x.split(':=')[1] for x in a if x.startswith('estimator_config:=')), ''))" "$LAUNCH_ARGS")
if [[ -z "$CONFIG" ]]; then
  SUFFIX=""; [[ " $LAUNCH_ARGS " == *" profile:=fast "* ]] && SUFFIX="_fast"
  [[ " $LAUNCH_ARGS " == *" profile:=monitor "* ]] && SUFFIX="_monitor"
  CONFIG="$WS/install/fabric_sim_cell/share/fabric_sim_cell/config/estimator$SUFFIX.yaml"
fi
WINDOWS=()
while [[ $# -ge 3 ]]; do WINDOWS+=(--window "$1" "$2" "$3"); shift 3; done
[[ ${#WINDOWS[@]} -eq 0 ]] && WINDOWS=(--window attached 10.0 60)
ros2 run fabric_sim_cell evaluate "$RUN/truth.npz" "$OUT/estimate.npz" "$CONFIG" "$OUT/eval" "${WINDOWS[@]}" 2>&1 | tee "$OUT/eval.log"
if [[ -f "$OUT/twin.npz" ]]; then
  python3 -m fabric_twin.evaluate "$RUN/truth.npz" "$OUT/twin.npz" "$OUT/twin_eval.json" --start "${TWIN_START:-12}" 2>&1 | tee -a "$OUT/eval.log"
fi
