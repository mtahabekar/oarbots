#!/usr/bin/env bash
# Start, stop and inspect the fabric test stack on a lab machine, from the deployed ~/fabric_ws.
#   fabric_session.sh start [launch args...]   helper: drivers, guard, bridges; tablet: GUI
#   fabric_session.sh start-cell <cell_dir>    helper only: wrist perception and CBF bridge
#   fabric_session.sh stop | status | logs [cell]
# From the laptop: scripts/hw/lab.sh session <blue|silver|tablet> <command> [args...]
#
# The stack runs as transient user services (fabric-stack, fabric-cell): nothing is installed,
# nothing starts at boot, and a reboot always returns the machine to the stock system.
set -eo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$here/lib.sh"
ws="$(cd -- "$here/../.." && pwd)"
run="$ws/run"
mkdir -p "$run"

account="$(whoami)"
case "$account" in
    oarbot_blue|oarbot_silver) role=helper; robot="${account#oarbot_}"; stock_unit="${account}_helper_laptop_launch.service" ;;
    tablet) role=tablet ;;
    *) echo "No fabric session role for account $account" >&2; exit 2 ;;
esac

record() { echo "$(date -Is) $* commit=$(git -C "$ws" rev-parse --short HEAD)" >> "$run/sessions.log"; }
unit_active() { systemctl --user is-active --quiet "$1"; }

stock_running() {
    if [ "$role" = helper ]; then
        systemctl is-active --quiet "$stock_unit"
    else
        pgrep -f "$HOME/ros2_ws/install/tablet_gui" > /dev/null
    fi
}

launch_unit() {
    local unit="$1"; shift
    local picked iface address profile="$run/cyclonedds.xml"
    picked="$(fabric_pick_address)"
    [ -n "$picked" ] || { echo "No ${FABRIC_SUBNET_PREFIX}x address on this machine" >&2; exit 1; }
    read -r iface address <<< "$picked"
    fabric_write_cyclone_profile "$profile" "$address"
    local env=(--setenv=RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --setenv=ROS_DOMAIN_ID="$FABRIC_DOMAIN"
               --setenv=CYCLONEDDS_URI="file://$profile" --setenv=ROS_LOG_DIR="$run/ros_log")
    if [ "$role" = helper ]; then
        # Same display and GPU settings as the stock helper launch script (Azure Kinect needs them).
        env+=(--setenv=DISPLAY=:2 --setenv=__GLX_VENDOR_LIBRARY_NAME=nvidia
              --setenv=__NV_PRIME_RENDER_OFFLOAD=1 --setenv=__VK_LAYER_NV_optimus=NVIDIA_only)
    fi
    systemd-run --user --quiet --collect --unit="$unit" \
        -p KillMode=control-group -p TimeoutStopSec=15 \
        -p StandardOutput="append:$run/$unit.log" -p StandardError="append:$run/$unit.log" \
        "${env[@]}" bash -c 'source /opt/ros/jazzy/setup.bash && source "$1/install/setup.bash" && shift && exec "$@"' \
        fabric "$ws" "$@"
    echo "$unit started on $iface $address, domain $FABRIC_DOMAIN. Log: $run/$unit.log"
}

cmd="${1:-status}"; shift || true
case "$cmd" in
    start)
        if stock_running; then
            if [ "$role" = helper ]; then
                echo "The stock stack is running. Stop it first (from the laptop):" >&2
                echo "  scripts/hw/lab.sh stock $robot stop" >&2
            else
                echo "The stock tablet GUI is running. Close it first." >&2
            fi
            exit 1
        fi
        unit_active fabric-stack && { echo "fabric-stack is already running" >&2; exit 1; }
        if [ "$role" = helper ]; then
            defaults=()
            [ -f "$here/hosts/$robot.args" ] && mapfile -t defaults < <(grep -vE '^\s*(#|$)' "$here/hosts/$robot.args")
            # Later arguments win, so command-line arguments override the host defaults.
            launch_unit fabric-stack ros2 launch oarbot_launch "oarbot_${robot}_helper_laptop.launch.py" "${defaults[@]}" "$@"
        else
            launch_unit fabric-stack ros2 launch "$ws/src/oarbot_launch/launch/tablet_gui.launch.py" "gui_python:=$ws/.venv/bin/python3" "$@"
        fi
        record start "$@" ;;
    start-cell)
        [ "$role" = helper ] || { echo "start-cell runs on the helper laptops" >&2; exit 2; }
        cell_dir="${1:?usage: start-cell <absolute cell_dir>}"
        unit_active fabric-stack || { echo "Start the stack first" >&2; exit 1; }
        unit_active fabric-cell && { echo "fabric-cell is already running" >&2; exit 1; }
        launch_unit fabric-cell ros2 launch fabric_cell robot_host.launch.py "robot:=$robot" "cell_dir:=$cell_dir"
        record start-cell "$cell_dir" ;;
    stop)
        for unit in fabric-cell fabric-stack; do
            unit_active "$unit" && systemctl --user stop "$unit" && echo "$unit stopped"
        done
        record stop
        [ "$role" = helper ] && echo "To give the robot back: scripts/hw/lab.sh stock $robot start (or reboot it)"
        true ;;
    status)
        echo "deployed: $(tr '\n' ' ' < "$ws/DEPLOYED" 2>/dev/null || echo 'not built')"
        echo "checkout: $(git -C "$ws" describe --tags --always --dirty)"
        for unit in fabric-stack fabric-cell; do echo "$unit: $(systemctl --user is-active "$unit" 2>/dev/null || true)"; done
        if [ "$role" = helper ]; then echo "stock $stock_unit: $(systemctl is-active "$stock_unit" 2>/dev/null || true)"; fi
        echo "lab address: $(fabric_pick_address || true)"
        echo "rmem_max: $(cat /proc/sys/net/core/rmem_max)" ;;
    logs)
        tail -n 200 "$run/fabric-${1:-stack}.log" ;;
    *)
        sed -n '2,9p' "$0"; exit 2 ;;
esac
