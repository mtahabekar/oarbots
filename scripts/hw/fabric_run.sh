#!/usr/bin/env bash
# One fabric run on the real cell, from the compute laptop (repo root), after scripts/hw/cell_up.sh:
#   fabric_run.sh start-pose [--check]  both arms to the experiment's start pose (grasp.yaml) if either is more
#                                       than 1.5 deg off; deadman held; bridges restarted (scripts/hw/start_pose.sh)
#   fabric_run.sh recentre              the same move whatever the offset
#   fabric_run.sh stop-go               end a running go from another terminal (as Ctrl+C there)
#   fabric_run.sh arm-start             restart both arms' Kinova velocity API (the driver's silent latch); start-pose does it
#   fabric_run.sh perception [seconds]  perception on all three cameras (workspace profile), default 3600 s
#   fabric_run.sh perception-stop
#   fabric_run.sh bridges start|stop    the joint-mode CBF bridges with the cell's bridge.yaml (start disarmed)
#   fabric_run.sh twin [run_dir]        estimator + twin + CBF + RViz, commands OFF, recording into run_dir
#                                       (default: a new experiments/hardware/sessions/<today>_fabric_run/live_<time>)
#   fabric_run.sh go [run_dir]          for an operator away from the screen: perception and bridges must run;
#                                       starts the twin (commands off) in the background, then switches commands
#                                       on and arms the bridges at the FIRST DEADMAN PRESS once the twin is
#                                       ready; speaks what happens (scripts/hw/auto_start.py). Ctrl+C: commands
#                                       off, bridges disarmed, twin stopped, run saved and reviewed
#   fabric_run.sh obstacle background   refresh the reusable empty-work-area reference from all three cameras
#                                       at the grasp pose (sheet off or held away, nobody in view)
#   fabric_run.sh obstacle detect [name] [detector options]   reuse the saved background with the object placed,
#                                       with the arms at the grasp pose: finds the object and
#                                       writes obstacle.yaml/.urdf/.obj/.png (scripts/hw/obstacle_detect.py)
#   fabric_run.sh obstacle fit <capture_dir> [--background dir] [detector options]
#                                       rebuild a model from saved depth without connecting to the cell
#   fabric_run.sh obstacle use [dir]    make a detected obstacle the twin's (cell/obstacle.yaml; the next
#                                       twin/go uses it); default: the last successful detect or fit
#   fabric_run.sh obstacle clear | show back to twin.yaml's virtual box | what the twin will use
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
# The whole script is one block, so bash reads all of it before running any: editing this file while a run
# from it is still going (2026-11-19 16:13: a go then executed part of deploy-helpers) cannot change that run.
{
set -uo pipefail
here="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "$here/../.." && pwd)"
cd "$repo"
if [ "${1:-}" != obstacle ]; then
    # shellcheck source=lab_ssh.sh
    source "$here/lab_ssh.sh"; lab_ssh_setup
fi
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

stop_twin() {
        launch="$(twin_pids | head -1)"
        [ -z "$launch" ] && { echo "no compute launch running"; return 0; }
        kids="$(pgrep -P "$launch" | tr '\n' ' ')"
        kill -INT "$launch"; sleep 10
        for p in $kids; do kill -0 "$p" 2>/dev/null && kill -INT "$p"; done; sleep 8
        for p in $launch $kids; do kill -0 "$p" 2>/dev/null && kill -TERM "$p"; done; sleep 2
        [ -n "$(twin_pids)" ] && echo "still running: $(twin_pids | tr '\n' ' ')" || echo "compute launch stopped; recordings in $(cat "$state/last_run" 2>/dev/null)"
}
save_run() {
        local run="${1:?save <run_dir>}" name data; name="$(basename "$(dirname "$run")")_$(basename "$run")"
        data="/home/taha/fabric_hw_data/$name"; mkdir -p "$data" "$run/cameras"
        for f in "$run"/*.npz; do [ -e "$f" ] && mv "$f" "$data/"; done
        [ -e "$run/twin.cbf.jsonl" ] && gzip -9 -f "$run/twin.cbf.jsonl"
        (cd "$data" && sha256sum ./* > MANIFEST.sha256 2>/dev/null)
        cp "$data/MANIFEST.sha256" "$run/large_files.sha256" 2>/dev/null
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'cd /dev/shm && tar -czf - run_* 2>/dev/null' > "$run/cameras/${m}_perception_logs.tar.gz"
        done
        echo "Large files: $data (checksums also in $run/large_files.sha256); camera logs: $run/cameras"
}

cmd="${1:-}"; shift || true
case "$cmd" in
    recentre) "$here/start_pose.sh" --force ;;   # both arms to grasp.yaml now, whatever their offset
    start-pose) "$here/start_pose.sh" "$@" ;;
    stop-go)     # end a running `go` from another terminal: same as Ctrl+C there (commands off, bridges disarmed,
                 # then that go stops the twin, saves and reviews the run)
        pkill -INT -f "scripts/hw/auto_start.py" && echo "go is ending: commands off; the twin stops and the run is saved" || echo "no go running" ;;
    arm-start)   # clear the Kinova driver's silent "Velocity stream timed out: API stopped" latch on both arms
        for r in blue silver; do echo "$r: $(timeout 40 scripts/hw/lab.sh session "$r" exec ros2 service call "/oarbot_$r/kinova/j2n6s300_driver/in/start" kinova_msgs/srv/Start 2>&1 | grep -o "start_result='[^']*'")"; done ;;
    perception)
        seconds="${1:-3600}"; tag="$(date +%Y%m%d_%H%M%S)"
        trial="$repo/experiments/hardware/sessions/2026-11-18_stationary_camera_trial"
        # Keep all hosts on the compute laptop's cell profile even when their installed
        # packages were deployed at different commits. Stage everything before stopping
        # any existing perception process.
        for m in nuc blue silver; do
            out="/dev/shm/run_${CAMERA[$m]}_$tag"
            if ! tar -cf - -C "$repo/experiments/hardware/cell" calibration.yaml perception_workspace.yaml \
                    "perception_${CAMERA[$m]}.yaml" -C "$trial" measure_live.py \
                    -C "$here" live_perception.sh glove_record.py |
                    "$LAB_SSH" "${HOST[$m]}" "mkdir -p '$out/config' && tar -xf - -C '$out/config'"; then
                echo "$m: could not stage perception config" >&2
                exit 1
            fi
        done
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'pkill -INT -f "measure_live[.]py"; true' || exit 1
            out="/dev/shm/run_${CAMERA[$m]}_$tag"
            nohup scripts/hw/lab.sh session "$m" exec bash "$out/config/live_perception.sh" \
                "${CAMERA[$m]}" "$seconds" "$out" "$out/config" > "$state/perception_$m.log" 2>&1 &
        done
        for m in nuc blue silver; do
            out="/dev/shm/run_${CAMERA[$m]}_$tag"
            if "$LAB_SSH" "${HOST[$m]}" "for i in \$(seq 1 15); do
                    if test -s '$out/observations.jsonl' && test -s '$out/gloves.jsonl'; then exit 0; fi
                    sleep 2
                done; exit 1"; then
                echo "$m perception publishing"
            else
                tail -12 "$state/perception_$m.log" >&2
                echo "$m perception has no output in $out" >&2
                exit 1
            fi
        done
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
        if [ "$cmd" = go ] && [ -n "$(twin_pids)" ]; then
            echo "Stopping the compute launch still running from before (its recordings are written first)"; stop_twin; fi
        if [ -n "$(twin_pids)" ]; then echo "A compute launch is already running (pid $(twin_pids | tr '\n' ' ')): fabric_run.sh stop-twin first" >&2; exit 1; fi
        launcher="$repo/../../install/fabric_twin/lib/fabric_twin/twin"
        if ! head -1 "$launcher" | grep -q venvs/fabric_ros; then
            echo "The twin's launcher does not use ~/venvs/fabric_ros (it would die: no warp). Run: fabric_run.sh build-laptop" >&2; exit 1; fi
        if [ "$cmd" = go ] && [ "${NO_START_POSE:-0}" != 1 ]; then
            "$here/start_pose.sh" || { echo "Not at the start pose: fix it or rerun (NO_START_POSE=1 skips this)" >&2; exit 1; }
            for r in blue silver; do scripts/hw/lab.sh session "$r" start-cell-bridge "/home/oarbot_$r/fabric_ws/experiments/hardware/cell" >/dev/null 2>&1; done
        fi
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
        trap 'true' INT TERM HUP   # a handler, not '' (ignored would be inherited: Ctrl+C would not reach auto_start); bash runs the wrap-up after it
        python3 "$here/auto_start.py" --run-dir "$run"
        echo "Stopping the twin (writes the recordings), saving, reviewing ..."
        stop_twin; save_run "$run"
        python3 "$here/run_review.py" "$run" > "$run/review.txt" 2>&1; grep -E '^(blue|silver|twin)' "$run/review.txt"
        echo "Done. Full review: $run/review.txt. fabric_run.sh go to run again." ;;
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
    stop-twin) stop_twin ;;
    save) save_run "${1:?save <run_dir>}" ;;
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
    obstacle)
        sub="${1:-show}"; shift || true
        obs_root="$repo/experiments/hardware/sessions/$(date +%F)_obstacle"
        cell="$repo/experiments/hardware/cell"
        background_ref="$cell/obstacle_background.txt"
        active="$cell/obstacle.yaml"
        detector="$here/obstacle_detect.py"
        obstacle_help() {
            cat <<'EOF'
fabric_run.sh obstacle background   Capture the reusable empty reference at the start pose.
fabric_run.sh obstacle detect [name] [detector options]
                                    Capture depth at the start pose and fit an obstacle candidate.
fabric_run.sh obstacle fit <capture_dir> [--background dir] [detector options]
                                    Fit a saved depth capture offline, using its original background by default.
fabric_run.sh obstacle use [dir]    Select a successful candidate for the next twin launch.
fabric_run.sh obstacle show         Show the selected model, background and latest candidate.
fabric_run.sh obstacle clear        Switch back to the virtual box in twin.yaml.
fabric_run.sh obstacle detect --help
                                    List fit options such as --roi, --expected-size-m and --measured-box-m.
EOF
        }
        replace_file() {
            local tmp
            tmp="$(mktemp "$1.XXXXXX")" || return 1
            if cat > "$tmp" && { [ ! -f "$1" ] || chmod --reference="$1" "$tmp"; } && mv -f "$tmp" "$1"; then
                return 0
            fi
            rm -f -- "$tmp"
            return 1
        }
        saved_background() {
            if [ -f "$background_ref" ]; then
                cat "$background_ref"
            else
                cat "$state/obstacle_background" 2>/dev/null
            fi
        }
        snapshot() {   # all three cameras into $1, in parallel, each on its own host
            local c m prefix root name
            for c in nuc:/nuc:nuc/camera_base:overhead blue:/oarbot_blue/azure_kinect:oarbot_blue/base_link:blue_wrist \
                     silver:/oarbot_silver/azure_kinect:oarbot_silver/base_link:silver_wrist; do
                IFS=: read -r m prefix root name <<< "$c"
                ( if { scripts/hw/lab.sh session "$m" exec python3 - "$prefix" "$root" 2 < "$here/depth_snapshot.py" |
                       sed -n 's/^DEPTHNPZ://p' | base64 -d > "$1/$name.npz"; } 2>"$1/$name.err" &&
                      [ -s "$1/$name.npz" ]; then
                      echo "  $name ok"
                      rm -f "$1/$name.err"
                  else
                      echo "  $name FAILED (see $1/$name.err)" >&2
                      rm -f "$1/$name.npz"
                  fi ) &
            done
            wait
        }
        capture_folder() {
            mkdir -p "$obs_root" && mkdir "$1" && cp "$cell/calibration.yaml" "$1/calibration.yaml"
        }
        summarize_model() {
            sed -n '/^obstacle:/,/^source:/{ /^source:/d; /^    footprint:/d; /^    - /d; p; }' "$active"
        }
        fit_candidate() {
            local capture="$1" bg="$2" output="$3"; shift 3
            if ! python3 -u "$detector" "$bg" "$capture" "$@" --output-dir "$output" 2>&1 | tee "$output/fit.log"; then
                echo "Obstacle fit failed. See $output/fit.log" >&2
                return 1
            fi
            if [ -f "$output/box_registration.json" ]; then
                echo "Wrist registration check passed. Report: $output/box_registration.json"
                return 0
            fi
            printf '%s\n' "$output" | replace_file "$state/obstacle_last" || return 1
            echo "Review $output/obstacle.png and fit.log, then select it with:"
            printf '  scripts/hw/fabric_run.sh obstacle use %q\n' "$output"
        }
        case "$sub" in
            help|-h|--help) obstacle_help ;;
            background)
                [ "$#" -eq 0 ] || { obstacle_help >&2; exit 2; }
                "$here/start_pose.sh" --check || exit 1
                dir="$obs_root/background_$(date +%H%M%S)"; echo "Empty work area -> $dir"
                capture_folder "$dir" || exit 1
                snapshot "$dir" || exit 1
                "$here/start_pose.sh" --check || exit 1
                python3 -u "$detector" "$dir" --check-background 2>&1 | tee "$dir/validation.log" || exit 1
                printf '%s\n' "${dir#"$repo"/}" | replace_file "$background_ref" || exit 1
                echo "Reusable obstacle background: $dir" ;;
            detect)
                name=object
                if [[ "${1:-}" && "$1" != -* ]]; then name="$1"; shift; fi
                if [[ " $* " == *' --help '* || " $* " == *' -h '* ]]; then
                    python3 "$detector" --help
                    exit
                fi
                [[ "$name" =~ ^[a-zA-Z0-9_-]+$ ]] || { echo "Use letters, digits, dashes or underscores in the capture name." >&2; exit 2; }
                bg="$(saved_background)"
                [ -d "$bg" ] || { echo "No background yet: fabric_run.sh obstacle background (work area empty)" >&2; exit 1; }
                "$here/start_pose.sh" --check || exit 1
                dir="$obs_root/${name}_$(date +%H%M%S)"; echo "Object -> $dir (background $bg)"
                capture_folder "$dir" || exit 1
                printf '%s\n' "${bg#"$repo"/}" > "$dir/background.txt" || exit 1
                snapshot "$dir" || exit 1
                "$here/start_pose.sh" --check || exit 1
                fit_candidate "$dir" "$bg" "$dir" "$@" ;;
            fit)
                if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
                    obstacle_help
                    python3 "$detector" --help
                    exit
                fi
                capture="${1:?obstacle fit needs an object capture directory}"; shift
                capture="$(realpath -e -- "$capture")" && [ -d "$capture" ] || exit 2
                if [ "${1:-}" = --background ]; then
                    bg="${2:?--background needs a directory}"; shift 2
                elif [ -s "$capture/background.txt" ]; then
                    bg="$(cat "$capture/background.txt")"
                elif [ -f "$capture/obstacle.yaml" ]; then
                    bg="$(python3 -c 'import sys, yaml; print(yaml.safe_load(sys.stdin).get("source", {}).get("background", ""))' \
                        < "$capture/obstacle.yaml")" || exit 1
                else
                    bg=
                fi
                [ -d "$bg" ] || { echo "Original background unavailable. Pass --background <matching capture directory>." >&2; exit 1; }
                mkdir -p "$obs_root" || exit 1
                output="$(mktemp -d "$obs_root/refit_${capture##*/}_$(date +%H%M%S)_XXXXXX")" || exit 1
                fit_candidate "$capture" "$bg" "$output" "$@" ;;
            use)
                dir="${1:-$(cat "$state/obstacle_last" 2>/dev/null)}"
                [ -n "$dir" ] && [ -f "$dir/obstacle.yaml" ] || { echo "No candidate in '${dir:-latest}'. Pass the output directory from detect or fit." >&2; exit 1; }
                replace_file "$active" < "$dir/obstacle.yaml" || exit 1
                echo "Twin obstacle: $dir (experiments/hardware/cell/obstacle.yaml). The next fabric_run.sh go/twin uses it."
                summarize_model ;;
            clear) rm -f "$active" && echo "Twin obstacle: twin.yaml's virtual box" ;;
            show)
                bg="$(saved_background)"
                latest="$(cat "$state/obstacle_last" 2>/dev/null)"
                echo "Reusable background: ${bg:-none}"
                echo "Latest candidate: ${latest:-none in this runtime}"
                if [ -f "$active" ]; then
                    echo "Twin obstacle: $active"
                    summarize_model
                    sed -n '/^  object: /s/^  object: /Capture: /p' "$active"
                else
                    echo "Twin obstacle: twin.yaml's virtual box"
                    sed -n '/^obstacle:/,/^feedback:/p' "$cell/twin.yaml" | sed '$d'
                fi ;;
            *) obstacle_help >&2; exit 2 ;;
        esac ;;
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
exit
}
