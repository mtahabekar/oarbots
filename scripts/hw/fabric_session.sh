#!/usr/bin/env bash
# Start, stop and inspect the fabric test stack on a lab machine, from the deployed ~/fabric_ws.
#   fabric_session.sh start [launch args...]   helper: drivers, guard, bridges; tablet: GUI; nuc: overhead camera
#   fabric_session.sh start-cell <cell_dir>    helper: wrist perception and CBF bridge; nuc: overhead perception
#   fabric_session.sh start-bridge [--box X Y Z] helper only: CBF bridge executing the simulated cell's twin
#   fabric_session.sh start-cell-bridge <cell_dir> helper only: CBF bridge for the real cell (its bridge.yaml), alone
#   fabric_session.sh stop | status | logs [cell]
#   fabric_session.sh exec <command...>        run a command in the foreground with the session's ROS settings
# From the laptop: scripts/hw/lab.sh session <blue|silver|tablet|nuc> <command> [args...]
#
# The stack runs as transient user services (fabric-stack, fabric-cell): nothing is installed,
# nothing starts at boot, and a reboot always returns the machine to the stock system.
#
# hosted_ft:=<robot> in hosts/<this robot>.args (or after `start`) is read by this script, not passed to
# the launch: it starts the other robot's wrist sensor, plugged into this laptop, as fabric-ft-<robot>.
# hosted_ft:=none cancels it.
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
    nuc) role=camera; stock_unit=nuc_launch.service ;;
    *) echo "No fabric session role for account $account" >&2; exit 2 ;;
esac

record() { echo "$(date -Is) $* commit=$(git -C "$ws" rev-parse --short HEAD)" >> "$run/sessions.log"; }
unit_active() { systemctl --user is-active --quiet "$1"; }

stock_running() {
    if [ -n "$stock_unit" ]; then
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
    elif [ "$role" = camera ]; then
        # The Azure Kinect depth engine needs the desktop's OpenGL context, as in the stock NUC unit.
        env+=(--setenv=DISPLAY=:0 --setenv=XAUTHORITY="/run/user/$(id -u)/gdm/Xauthority")
    fi
    systemd-run --user --quiet --collect --unit="$unit" \
        -p KillMode=control-group -p TimeoutStopSec=15 \
        -p StandardOutput="append:$run/$unit.log" -p StandardError="append:$run/$unit.log" \
        "${env[@]}" bash -c 'source /opt/ros/jazzy/setup.bash && source "$1/install/setup.bash" && shift && exec "$@"' \
        fabric "$ws" "$@"
    echo "$unit started on $iface $address, domain $FABRIC_DOMAIN. Log: $run/$unit.log"
}

# Another robot's Rokubi plugged into this laptop (a ground loop makes Blue's arm jitter when its
# sensor is on Blue's own laptop; lab log 2026-11-18). The serial port comes from that robot's args.
start_hosted_ft() {
    local other="$1" port
    case "$other" in blue|silver) ;; *) echo "hosted_ft: unknown robot '$other'" >&2; return 1 ;; esac
    [ "$other" != "$robot" ] || { echo "hosted_ft names this robot; its own sensor runs with ft:=true" >&2; return 1; }
    port="$(sed -n 's/^ft_serial_port:=//p' "$here/hosts/$other.args" 2>/dev/null | tail -1)"
    [ -n "$port" ] || { echo "hosted_ft: no ft_serial_port in hosts/$other.args" >&2; return 1; }
    if [ ! -e "$port" ]; then
        echo "hosted_ft: $other's sensor ($port) is not plugged into this laptop; fabric-ft-$other not started" >&2
        return 0
    fi
    launch_unit "fabric-ft-$other" ros2 launch "$here/hosted_ft.launch.py" "robot:=$other" "serial_port:=$port"
}
units=(fabric-bridge fabric-cell fabric-ft-blue fabric-ft-silver fabric-stack)

cmd="${1:-status}"; shift || true
case "$cmd" in
    start)
        if stock_running; then
            if [ -n "$stock_unit" ]; then
                echo "The stock stack is running. Stop it first (from the laptop):" >&2
                echo "  scripts/hw/lab.sh stock ${robot:-nuc} stop" >&2
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
            launch_args=(); hosted=""
            for arg in "${defaults[@]}" "$@"; do
                case "$arg" in
                    hosted_ft:=*) hosted="${arg#hosted_ft:=}"; [ "$hosted" = none ] && hosted="" ;;
                    *) launch_args+=("$arg") ;;
                esac
            done
            launch_unit fabric-stack ros2 launch oarbot_launch "oarbot_${robot}_helper_laptop.launch.py" "${launch_args[@]}"
            [ -z "$hosted" ] || start_hosted_ft "$hosted" || echo "fabric-stack runs without $hosted's sensor" >&2
        elif [ "$role" = camera ]; then
            launch_unit fabric-stack ros2 launch "$here/nuc_camera.launch.py" "$@"
        else
            # Fabric sessions use the toggle deadman (user, 2026-11-21): press once to start, press again to stop.
            # The guards still need its 50 Hz stream fresh; e-stop, serial faults and a stale Arduino turn it off.
            launch_unit fabric-stack ros2 launch "$ws/src/oarbot_launch/launch/tablet_gui.launch.py" "gui_python:=$ws/.venv/bin/python3" "deadman_mode:=toggle" "$@"
        fi
        record start "$@" ;;
    start-cell)
        [ "$role" = helper ] || [ "$role" = camera ] || { echo "start-cell runs on the helpers and the NUC" >&2; exit 2; }
        cell_dir="${1:?usage: start-cell <absolute cell_dir>}"
        unit_active fabric-stack || { echo "Start the stack first" >&2; exit 1; }
        unit_active fabric-cell && { echo "fabric-cell is already running" >&2; exit 1; }
        if [ "$role" = camera ]; then
            # fabric_cell is not built on the NUC (it depends on the estimator and twin); its camera
            # launch needs only fabric_perception and the cell directory, so it runs by path.
            launch_unit fabric-cell ros2 launch "$ws/src/fabric_cell/launch/camera_host.launch.py" camera:=overhead "cell_dir:=$cell_dir"
        else
            launch_unit fabric-cell ros2 launch fabric_cell robot_host.launch.py "robot:=$robot" "cell_dir:=$cell_dir"
        fi
        record start-cell "$cell_dir" ;;
    start-cell-bridge)
        [ "$role" = helper ] || { echo "start-cell-bridge runs on the helper laptops" >&2; exit 2; }
        cell_dir="${1:?usage: start-cell-bridge <absolute cell_dir>}"
        unit_active fabric-stack || { echo "Start the stack first" >&2; exit 1; }
        unit_active fabric-bridge && { echo "fabric-bridge is already running" >&2; exit 1; }
        launch_unit fabric-bridge ros2 launch "$here/cell_bridge.launch.py" "robot:=$robot" "cell_dir:=$cell_dir"
        record start-cell-bridge "$cell_dir" ;;
    start-bridge)
        [ "$role" = helper ] || { echo "start-bridge runs on the helper laptops" >&2; exit 2; }
        unit_active fabric-stack || { echo "Start the stack first" >&2; exit 1; }
        for unit in fabric-bridge fabric-cell; do
            unit_active "$unit" && { echo "$unit is already running" >&2; exit 1; }
        done
        params="$run/sim_bridge.yaml"
        ( source /opt/ros/jazzy/setup.bash && source "$ws/install/setup.bash" \
          && fabric_export_ros_env "$run/cyclonedds.xml" && export ROS_LOG_DIR="$run/ros_log" \
          && python3 "$here/sim_bridge.py" --robot "$robot" --out "$params" "$@" ) || exit 1
        launch_unit fabric-bridge ros2 launch "$here/sim_bridge.launch.py" "robot:=$robot" "params_file:=$params"
        record start-bridge "$@" ;;
    stop)
        for unit in "${units[@]}"; do
            unit_active "$unit" && systemctl --user stop "$unit" && echo "$unit stopped"
        done
        record stop
        [ -n "$stock_unit" ] && echo "To give the machine back: scripts/hw/lab.sh stock ${robot:-nuc} start (or reboot it)"
        true ;;
    status)
        echo "deployed: $(tr '\n' ' ' < "$ws/DEPLOYED" 2>/dev/null || echo 'not built')"
        echo "checkout: $(git -C "$ws" describe --tags --always --dirty)"
        for unit in "${units[@]}"; do echo "$unit: $(systemctl --user is-active "$unit" 2>/dev/null || true)"; done
        if [ -n "$stock_unit" ]; then echo "stock $stock_unit: $(systemctl is-active "$stock_unit" 2>/dev/null || true)"; fi
        echo "lab address: $(fabric_pick_address || true)"
        echo "rmem_max: $(cat /proc/sys/net/core/rmem_max)" ;;
    logs)
        tail -n 200 "$run/fabric-${1:-stack}.log" ;;   # stack, cell, bridge, ft-blue, ft-silver
    exec)
        [ $# -ge 1 ] || { echo "usage: exec <command...>" >&2; exit 2; }
        unit_active fabric-stack || { echo "Start the stack first" >&2; exit 1; }
        record exec "$@"
        source /opt/ros/jazzy/setup.bash
        source "$ws/install/setup.bash"
        fabric_export_ros_env "$run/cyclonedds.xml"
        export ROS_LOG_DIR="$run/ros_log"
        exec "$@" ;;
    *)
        sed -n '2,9p' "$0"; exit 2 ;;
esac
