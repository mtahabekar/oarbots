#!/usr/bin/env bash
# One fabric run on the real cell, from the compute laptop (repo root), after scripts/hw/cell_up.sh:
#   fabric_run.sh recentre              both arms to their grasp joints (grasp.yaml), joint space, deadman held;
#                                       stops the bridges first (they share the arm's joint input)
#   fabric_run.sh perception [seconds]  perception on all three cameras (workspace profile), default 3600 s
#   fabric_run.sh perception-stop
#   fabric_run.sh bridges start|stop    the joint-mode CBF bridges with the cell's bridge.yaml (start disarmed)
#   fabric_run.sh twin [run_dir]        estimator + twin + CBF + RViz, commands OFF, recording into run_dir
#                                       (default: a new experiments/hardware/sessions/<today>_fabric_run/live_<time>)
#   fabric_run.sh go [run_dir]          for an operator away from the screen: perception and bridges must run;
#                                       starts the twin (commands off) in the background, then switches commands
#                                       on and arms the bridges at the FIRST DEADMAN PRESS once the twin is
#                                       ready; speaks what happens (scripts/hw/auto_start.py). Ctrl+C: commands
#                                       off, bridges disarmed (the twin keeps running: stop-twin, then save)
#   fabric_run.sh status [seconds]      go/no-go readout (scripts/hw/twin_monitor.py)
#   fabric_run.sh watch [seconds]       readout every 5 s during a commanded run
#   fabric_run.sh enable                twin commands on (formation captured now) + both bridges armed
#   fabric_run.sh disable               twin commands off + both bridges disarmed
#   fabric_run.sh stop-twin             end the compute launch and every node it started (writes the recordings)
#   fabric_run.sh save <run_dir>        large recordings to /home/taha/fabric_hw_data, camera logs from the hosts
#   fabric_run.sh check                 read-only readiness: arms vs grasp and box, fingers, guards (state, joint
#                                       cap), bridges (state, force/box stop), perception output, camera frame age
#   fabric_run.sh review <run_dir>      after a run: per arm, commanded vs delivered speed, box faces, where it
#                                       stopped; the twin's faults while enabled (scripts/hw/run_review.py)
#   fabric_run.sh build-laptop          rebuild the laptop's fabric packages with ~/venvs/fabric_ros (Newton,
#                                       Warp); plain colcon gives the twin /usr/bin/python3 and it dies at start
#   fabric_run.sh deploy-helpers        both helpers to the fork's branch head: sessions stopped (arms hold
#                                       their pose), rebuilt, started again with scripts/hw/cell_up.sh;
#                                       then perception and bridges must be started again
# The arms move only while the tablet deadman is held with Fabric Follow (CBF) on for that robot; the person
# holding the sheet cannot hold it, so a second person does. Procedure and evidence:
# experiments/hardware/sessions/2026-11-18_s5_closed_loop/README.md.
set -uo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cd "$repo"
# shellcheck source=lab_ssh.sh
source "$here/lab_ssh.sh"; lab_ssh_setup
declare -A HOST=([nuc]=nuc@192.168.1.90 [blue]=oarbot_blue@192.168.1.71 [silver]=oarbot_silver@192.168.1.110)
declare -A CAMERA=([nuc]=overhead [blue]=blue_wrist [silver]=silver_wrist)
state="${XDG_RUNTIME_DIR:-/tmp}/fabric_lab"; mkdir -p "$state"
ros_env() {
    set +u  # the ROS setup scripts read unset variables
    # shellcheck disable=SC1091
    source "$here/laptop_env.sh" >/dev/null 2>&1
    export QT_QPA_PLATFORM=xcb WARP_CACHE_PATH=/tmp/oarbots-fabric-audit-gpu DISPLAY="${DISPLAY:-:0}"
    export PYTHONPATH="$repo/src/fabric_state_estimator:$repo/src/human_hand_tracking:$repo/src/fabric_sim_cell:${PYTHONPATH:-}"
}
pub_bool() { timeout 15 ros2 topic pub --once "$1" std_msgs/msg/Bool "{data: $2}" >/dev/null 2>&1; }
twin_pids() { pgrep -f "ros2 launch .*compute.launch.py"; }

cmd="${1:-}"; shift || true
case "$cmd" in
    recentre)
        for r in blue silver; do scripts/hw/lab.sh session "$r" exec systemctl --user stop fabric-bridge >/dev/null 2>&1; done
        for r in blue silver; do
            scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --check-only 2>&1 |
                grep -E 'Preflight|ABORT' | sed -E 's/.*"joint_change_deg": (\[[^]]*\]).*"closest_to_other_arm_m": ([0-9.]*).*/joint change \1 deg, other arm \2 m away/' | sed "s/^/$r: /"
        done
        echo "Both arms move together once the deadman is held (Fabric Follow on for both): release, press, hold ~5 s."
        for r in blue silver; do
            scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --wait 600 > "$state/recentre_$r.log" 2>&1 &
        done
        wait
        for r in blue silver; do grep -E 'Done|ABORT' "$state/recentre_$r.log" | sed "s/^/$r: /"; done
        echo "Bridges are stopped: fabric_run.sh bridges start" ;;
    perception)
        seconds="${1:-3600}"; tag="$(date +%Y%m%d_%H%M%S)"
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'pkill -INT -f "measure_live[.]py"; true'
            nohup scripts/hw/lab.sh session "$m" exec bash fabric_ws/scripts/hw/live_perception.sh "${CAMERA[$m]}" "$seconds" \
                "/dev/shm/run_${CAMERA[$m]}_$tag" > "$state/perception_$m.log" 2>&1 &
        done
        sleep 12; head -qn1 "$state"/perception_*.log
        echo "Logs on each host in /dev/shm/run_<camera>_$tag (fabric_run.sh save copies them)" ;;
    perception-stop)
        for m in nuc blue silver; do "$LAB_SSH" "${HOST[$m]}" 'pkill -INT -f "measure_live[.]py"; true'; done; echo stopped ;;
    bridges)
        case "${1:-}" in
            start) for r in blue silver; do scripts/hw/lab.sh session "$r" start-cell-bridge "/home/oarbot_$r/fabric_ws/experiments/hardware/cell" 2>&1 | tail -1; done ;;
            stop) for r in blue silver; do scripts/hw/lab.sh session "$r" exec systemctl --user stop fabric-bridge && echo "$r bridge stopped"; done ;;
            *) echo "bridges start|stop"; exit 2 ;;
        esac ;;
    twin|go)
        if [ -n "$(twin_pids)" ]; then echo "A compute launch is already running (pid $(twin_pids | tr '\n' ' ')): fabric_run.sh stop-twin first" >&2; exit 1; fi
        launcher="$repo/../../install/fabric_twin/lib/fabric_twin/twin"
        if ! head -1 "$launcher" | grep -q venvs/fabric_ros; then
            echo "The twin's launcher does not use ~/venvs/fabric_ros (it would die: no warp). Run: fabric_run.sh build-laptop" >&2; exit 1; fi
        run="${1:-$repo/experiments/hardware/sessions/$(date +%F)_fabric_run/live_$(date +%H%M%S)}"
        mkdir -p "$run"; echo "$run" > "$state/last_run"
        git rev-parse HEAD > "$run/commit.txt"; cp experiments/hardware/cell/*.yaml "$run/" 2>/dev/null; mkdir -p "$run/cell"; mv "$run"/*.yaml "$run/cell/"
        echo "Run folder: $run"
        echo "The twin starts once both hands and both robot grips are measured (the estimator needs ~2 min first)."
        ros_env
        launch=(ros2 launch "$repo/src/fabric_cell/launch/compute.launch.py" cell_dir:="$repo/experiments/hardware/cell"
                nominal_grips:=true twin:=true bank:=false commands:=false rviz:=true record:="$run")
        if [ "$cmd" = twin ]; then "${launch[@]}" 2>&1 | tee "$run/compute.log"; exit; fi
        setsid nohup "${launch[@]}" > "$run/compute.log" 2>&1 < /dev/null &
        sleep 20
        if grep -q "process has died" "$run/compute.log"; then
            grep -B3 "process has died" "$run/compute.log" | tail -8; spd-say "twin failed to start" 2>/dev/null
            echo "A node died at start (log above, $run/compute.log). fabric_run.sh stop-twin, fix, retry." >&2; exit 1
        fi
        echo "Twin running in the background (log $run/compute.log, RViz open). Go to the robots: the laptop speaks."
        python3 "$here/auto_start.py" --run-dir "$run"
        echo "Commands off. Then: fabric_run.sh stop-twin; fabric_run.sh save $run; fabric_run.sh review $run" ;;
    status) ros_env; python3 "$here/twin_monitor.py" "${1:-15}" ;;
    watch)  ros_env; python3 "$here/twin_monitor.py" "${1:-60}" --every 5 ;;
    enable)
        ros_env
        pub_bool /fabric/twin/enable_commands true
        for r in blue silver; do pub_bool "/oarbot_$r/fabric_cbf/arm" true; done
        sleep 2; python3 "$here/twin_monitor.py" 6 | tail -1
        echo "Armed. Deadman: release, press, hold. Release stops both arms. fabric_run.sh disable to end." ;;
    disable)
        ros_env
        pub_bool /fabric/twin/enable_commands false &
        for r in blue silver; do pub_bool "/oarbot_$r/fabric_cbf/arm" false & done
        wait; sleep 1; python3 "$here/twin_monitor.py" 5 | tail -1 ;;
    stop-twin)
        launch="$(twin_pids | head -1)"
        [ -z "$launch" ] && { echo "no compute launch running"; exit 0; }
        kids="$(pgrep -P "$launch" | tr '\n' ' ')"
        kill -INT "$launch"; sleep 10
        for p in $kids; do kill -0 "$p" 2>/dev/null && kill -INT "$p"; done; sleep 8
        for p in $launch $kids; do kill -0 "$p" 2>/dev/null && kill -TERM "$p"; done; sleep 2
        [ -n "$(twin_pids)" ] && echo "still running: $(twin_pids | tr '\n' ' ')" || echo "compute launch stopped; recordings in $(cat "$state/last_run" 2>/dev/null)" ;;
    save)
        run="${1:?save <run_dir>}"; name="$(basename "$(dirname "$run")")_$(basename "$run")"
        data="/home/taha/fabric_hw_data/$name"; mkdir -p "$data" "$run/cameras"
        for f in "$run"/*.npz; do [ -e "$f" ] && mv "$f" "$data/"; done
        [ -e "$run/twin.cbf.jsonl" ] && gzip -9 -f "$run/twin.cbf.jsonl"
        (cd "$data" && sha256sum ./* > MANIFEST.sha256 2>/dev/null)
        cp "$data/MANIFEST.sha256" "$run/large_files.sha256" 2>/dev/null
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'cd /dev/shm && tar -czf - run_* 2>/dev/null' > "$run/cameras/${m}_perception_logs.tar.gz"
        done
        echo "Large files: $data (checksums also in $run/large_files.sha256); camera logs: $run/cameras" ;;
    check)
        ros_env; python3 "$here/cell_check.py"
        echo "== perception (latest run on each camera host)"
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'd=$(ls -dt /dev/shm/run_*_* 2>/dev/null | head -1); [ -z "$d" ] && { echo "  no perception run"; exit; }
                now=$(date +%s); o=$(wc -l < $d/observations.jsonl 2>/dev/null); g=$(wc -l < $d/gloves.jsonl 2>/dev/null)
                age=$(( now - $(stat -c %Y $d/observations.jsonl 2>/dev/null || echo 0) ))
                hands=$(tail -n 200 $d/gloves.jsonl 2>/dev/null | grep -vc "hands.: \\[\\]")
                echo "  $(basename $d): $o analyses, $g glove frames, last written ${age} s ago, frames with hands (last 200): $hands"' 2>/dev/null | sed "s/^/$m/"
        done
        echo "== camera frame age on each host (want under 0.1 s)"
        for cam in nuc:/nuc blue:/oarbot_blue/azure_kinect silver:/oarbot_silver/azure_kinect; do
            echo "  ${cam%%:*}: $(scripts/hw/lab.sh session "${cam%%:*}" exec python3 fabric_ws/scripts/hw/frame_age.py "${cam#*:}" 3 2>/dev/null | grep '^image' || echo 'NO images')"
        done ;;
    build-laptop)
        # Test guide section 5. The venv's python must run colcon: the installed launchers get its shebang.
        (   set +u; cd "$repo/../.." && source /opt/ros/jazzy/setup.bash
            ~/venvs/fabric_ros/bin/python -m colcon build --packages-select fabric_state_msgs | tail -1
            source install/setup.bash
            ~/venvs/fabric_ros/bin/python -m colcon build --packages-select fabric_perception human_hand_tracking \
                fabric_state_estimator fabric_reference_reconstruction fabric_sim_cell fabric_twin fabric_cell | tail -1
            echo "twin launcher: $(head -1 install/fabric_twin/lib/fabric_twin/twin)" ) ;;
    review) ros_env; python3 "$here/run_review.py" "${1:?review <run_dir>}" ;;
    deploy-helpers)
        git fetch -q fork; echo "deploying fork/jazzy-platform-hardening $(git rev-parse --short fork/jazzy-platform-hardening) to both helpers"
        for r in blue silver; do
            ( scripts/hw/lab.sh session "$r" stop >/dev/null 2>&1; scripts/hw/lab.sh setup "$r" > "$state/deploy_$r.log" 2>&1
              echo "$r: $(grep -E '^Deployed|Failed|failed' "$state/deploy_$r.log" | tail -1)" ) &
        done
        wait
        "$here/cell_up.sh"
        echo "Now: fabric_run.sh perception 3600; fabric_run.sh bridges start; fabric_run.sh check" ;;
    *) sed -n '2,/^set -uo/p' "$0" | sed '$d'; exit 2 ;;
esac
