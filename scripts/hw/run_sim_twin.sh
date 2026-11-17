#!/usr/bin/env bash
# The simulated cell with the fabric twin commanding the robots while you jog the gloves. The twin's
# safe follower velocities (/oarbot_{blue,silver}/fabric_cbf/cmd_world) drive the simulated robots
# (--external-followers) and, through cmd_relay.py and each robot's bridge, guard and the tablet
# deadman, the real robots too.
#   scripts/hw/run_sim_twin.sh [SIM_SECONDS=600] [HANDS=oracle|gloves]
# Before: the fabric session running, with `lab.sh session <robot> start-bridge` on both robots.
# Then: glove keys in a second terminal (printed below); on the tablet, Fabric Follow on and the deadman held;
# arm the bridges with `ros2 topic pub --once /oarbot_<robot>/fabric_cbf/arm std_msgs/Bool "{data: true}"`
# after `source scripts/hw/laptop_env.sh`.
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
SIM=${OARBOT_SIM:-/media/taha/ExtermeLinux/oarbot_simulation}
DURATION=${1:-600}
HANDS=${2:-oracle}
[[ "$DURATION" =~ ^[0-9]+$ && "$DURATION" -ge 30 ]] || { echo "SIM_SECONDS must be a whole number, at least 30" >&2; exit 2; }
[[ "$HANDS" == oracle || "$HANDS" == gloves ]] || { echo "HANDS must be oracle or gloves" >&2; exit 2; }
OUT=$SIM/logs/sim_cell/twin_teleop_$(date +%Y%m%d_%H%M%S)

# The fast real-time profile of run_demo.sh, without its in-plant filter driving the robots.
PLANT="--attach --route-cbf --external-followers --steps $((DURATION * 60)) --render-mode due --render-sync \
--wrist-resolution 640 360 --overhead-scale 0.5 --physics-hz 60 --robot-collisions none --physics-fabric \
--cbf-period 4 --iterations 8 --linear-iterations 8 --state-publish-every 1000000"
[[ "$HANDS" == gloves ]] && PLANT+=" --overhead-hz 15"   # glove tags need the overhead at 15 Hz, as on the cell
LAUNCH="hands:=$HANDS rviz:=true profile:=fast twin:=true twin_commands:=true twin_bank:=true twin_record:=$OUT/twin.npz"

sim_env() {
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$SIM/config/fastdds_large_messages.xml
    unset CYCLONEDDS_URI
}
mkdir -p "$OUT"
( sim_env; exec python3 "$here/cmd_relay.py" sim ) > "$OUT/relay_sim.log" 2>&1 &
RELAY_SIM=$!
( source "$here/laptop_env.sh" > /dev/null; exec python3 "$here/cmd_relay.py" real ) > "$OUT/relay_real.log" 2>&1 &
RELAY_REAL=$!
trap 'kill $RELAY_SIM $RELAY_REAL 2>/dev/null; wait $RELAY_SIM $RELAY_REAL 2>/dev/null' EXIT

cat <<EOF
Simulated cell with the twin commanding the robots ($HANDS hands, $DURATION s simulated) -> $OUT
Isaac Sim and RViz open in ~30 s. The robots grasp at ~9 s simulated; the glove keys act after that.
Relay logs: $OUT/relay_sim.log, $OUT/relay_real.log

Glove keys, in a second terminal:
  source /opt/ros/jazzy/setup.bash
  export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp FASTRTPS_DEFAULT_PROFILES_FILE=$SIM/config/fastdds_large_messages.xml
  python3 $SIM/tools/fabric_leader_teleop.py --wait 600 --speed 0.1
EOF
( sim_env; PLANT_GUI=${PLANT_GUI:-1} QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb} \
    "$repo/src/fabric_sim_cell/scripts/run_sim_experiment.sh" "$OUT" "$PLANT" "$LAUNCH" attached 10.0 "$DURATION" )
