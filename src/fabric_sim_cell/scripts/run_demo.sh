#!/usr/bin/env bash
# Demo of the fabric estimator against the simulated OARBot cell.
#
#   run_demo.sh live [oracle|gloves] [OUT]   one run you can watch: Isaac Sim window (the true cloth)
#                                            and RViz (the estimate from sensors only, drawn over the
#                                            overhead and wrist camera images); scored at the end
#   run_demo.sh teleop [oracle|gloves] [SECONDS] [OUT]
#                                            the same, but you move the leader corners (the hands) with
#                                            oarbot_simulation/tools/fabric_leader_teleop.py in a second
#                                            terminal; runs SECONDS of simulated time (default 60)
#   run_demo.sh evidence [OUT] [REPEATS]     the six-experiment set REPEATS times (default 3, ~18 min
#                                            each) and the scorecard against the project targets
#
# Scenario (live): robots grasp at ~9-10 s (estimation starts when both grippers report closed), the
# leader corners (the hands) hold to 11 s, descend 0.5 m at 0.1 m/s to 16 s and hold to 18.5 s.
# In teleop the leaders hold still until the grasp and then follow only your keys.
#
# live and teleop use the real-time profile (PROFILE=fast, default): half-resolution cameras rendered
# only when due, 60 Hz robot physics, CBF at 15 Hz, 8x8 PD/PCG cloth iterations, and the stack's fast
# configs (8x8 estimator mesh, no KLT tracks). PROFILE=full runs the original, validated settings, which
# `evidence` always uses. See oarbot_simulation FABRIC_INTEGRATION.md, "Real time".
set -o pipefail
HERE=$(dirname "$(realpath "$0")")
SIM=${OARBOT_SIM:-/media/taha/ExtermeLinux/oarbot_simulation}
WS=${COLCON_WS:-/home/taha/ros2_colcon_ws}
STAMP=$(date +%Y%m%d_%H%M%S)
MODE=${1:-live}
PROFILE=${PROFILE:-fast}
case "$PROFILE" in
  fast) PLANT_PROFILE="--render-mode due --wrist-resolution 640 360 --overhead-scale 0.5 --physics-hz 60 \
--robot-collisions none --physics-fabric --cbf-period 4 --iterations 8 --linear-iterations 8 --state-publish-every 1000000"
        LAUNCH_PROFILE="profile:=fast" ;;
  full) PLANT_PROFILE=""; LAUNCH_PROFILE="" ;;
  *) echo "PROFILE must be fast or full" >&2; exit 2 ;;
esac
score() {
  ( source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; ros2 run fabric_sim_cell scorecard "$1" )
}
case "$MODE" in
  live|teleop)
    HANDS=${2:-gloves}
    [[ "$HANDS" == oracle || "$HANDS" == gloves ]] || { echo "hands must be oracle or gloves" >&2; exit 2; }
    if [[ "$MODE" == live ]]; then
      OUT=$(realpath -m "${3:-$SIM/logs/sim_cell/demo_$STAMP}")
      LEADERS="--demo-leader 0 0 -0.1 --demo-after 120 --demo-duration 300 --steps 1110"
      WINDOWS=(attached 10.0 18.5 hold 10.0 11.0 descent 11.0 16.0 after 16.0 18.5)
      echo "Live demo ($HANDS hands, $PROFILE profile) -> $OUT/$HANDS. Isaac Sim and RViz open in ~30 s; the run takes ~2-3 min."
    else
      # Without --demo-leader the plant takes the leaders from /fabric/leader_{left,right}/cmd_vel.
      DURATION=${3:-60}
      [[ "$DURATION" =~ ^[0-9]+$ && "$DURATION" -ge 15 ]] || { echo "SECONDS must be a whole number, at least 15" >&2; exit 2; }
      OUT=$(realpath -m "${4:-$SIM/logs/sim_cell/teleop_$STAMP}")
      LEADERS="--steps $((DURATION * 60))"  # 60 cloth frames per simulated second
      WINDOWS=(attached 10.0 "$DURATION")
      echo "Teleop demo ($HANDS hands, ${DURATION} s simulated, $PROFILE profile) -> $OUT/$HANDS. Isaac Sim and RViz open in ~30 s."
      echo "Drive the hands from a second terminal (it waits for the simulator; keys act once the robots"
      echo "have grasped, ~9 s simulated; the plant runs slower than real time, so motion looks slowed down):"
      echo "  source /opt/ros/jazzy/setup.bash"
      echo "  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$SIM/config/fastdds_large_messages.xml"
      echo "  python3 $SIM/tools/fabric_leader_teleop.py --wait 600"
    fi
    PLANT_GUI=${PLANT_GUI:-1} QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb} "$HERE/run_sim_experiment.sh" "$OUT/$HANDS" \
      "--attach --route-cbf $LEADERS $PLANT_PROFILE" "hands:=$HANDS rviz:=${RVIZ:-true} $LAUNCH_PROFILE" "${WINDOWS[@]}" || exit $?
    score "$OUT"
    ;;
  evidence)
    OUT=$(realpath -m "${2:-$SIM/logs/sim_cell/evidence_$STAMP}"); REPEATS=${3:-3}
    for i in $(seq "$REPEATS"); do
      echo "== repeat $i of $REPEATS"
      "$HERE/run_experiment_set.sh" "$OUT/set_$i" || exit $?
    done
    score "$OUT"
    ;;
  *)
    sed -n '/^set /q; 2,$p' "$0"; exit 2 ;;
esac
