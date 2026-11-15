#!/usr/bin/env bash
# Repeat an earlier teleoperated run without a keyboard, and record the sensor-side topics.
#
#   run_teleop_replay.sh TRUTH.npz OUT_DIR [oracle|gloves] [EXTRA_LAUNCH_ARGS] [EXTRA_PLANT_ARGS]
#
# The leader (hand) corner motion stored in TRUTH.npz drives the plant through
# /fabric/leader_{left,right}/cmd_vel. The run is scored like run_demo.sh teleop, and the compact
# observations, glove measurements, TF and joint states go to OUT_DIR/bag (MCAP, stamped in
# simulation time) so the estimator can later be replayed with replay_bag.sh without Isaac Sim.
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
SIM=${OARBOT_SIM:-/media/taha/ExtermeLinux/oarbot_simulation}
REPO=$(realpath "$HERE/../../..")
TRUTH=$(realpath "$1"); OUT=$(realpath -m "$2"); HANDS=${3:-gloves}; LAUNCH_EXTRA=${4:-}; PLANT_EXTRA=${5:-}
[[ -f "$TRUTH" ]] || { echo "no truth file $TRUTH" >&2; exit 2; }
DURATION=${DURATION:-60}
PLANT="--attach --route-cbf --steps $((DURATION * 60)) --render-mode due --render-sync --wrist-resolution 640 360 \
--overhead-scale 0.5 --physics-hz 60 --robot-collisions none --physics-fabric --cbf-period 4 --iterations 8 \
--linear-iterations 8 --state-publish-every 1000000 $PLANT_EXTRA"
TOPICS="/tf /tf_static /fabric/hand_states /fabric/hand_states_ungated /fabric/estimated_state /fabric/model \
/fabric/leader_left/cmd_vel /fabric/leader_right/cmd_vel /sim/mocap/glove_left /sim/mocap/glove_right"
for camera in /nuc /oarbot_blue/azure_kinect /oarbot_silver/azure_kinect; do
  TOPICS+=" $camera/fabric/observations $camera/fabric/gloves"
done
for robot in oarbot_blue oarbot_silver; do
  TOPICS+=" /$robot/kinova/j2n6s300_driver/out/joint_state /$robot/force_torque_raw"
done
export SIM_EXTRA="ros2 run fabric_sim_cell leader_replay --ros-args -p truth:='$TRUTH' & \
ros2 bag record --use-sim-time --storage mcap --output \"\$OUT/bag\" \
--qos-profile-overrides-path '$REPO/config/recording_qos.yaml' $TOPICS & wait"
"$HERE/run_sim_experiment.sh" "$OUT" "$PLANT" "hands:=$HANDS profile:=fast $LAUNCH_EXTRA" attached 10.0 "$DURATION"
