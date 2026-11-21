#!/usr/bin/env bash
# One fabric run on the real cell, from the compute laptop (repo root), after scripts/hw/cell_up.sh:
#   fabric_run.sh start-pose [--check]  both arms to the experiment's start pose (grasp.yaml) if either is more
#                                       than 1.5 deg off; deadman held; bridges restarted (scripts/hw/start_pose.sh)
#   fabric_run.sh recentre              the same move whatever the offset
#   fabric_run.sh warmup-start [--check]  separate, narrower warm-up pose (warmup_grasp.yaml), through the deadman
#   fabric_run.sh stop-go               end a running go from another terminal (as Ctrl+C there)
#   fabric_run.sh arm-start             restart both arms' Kinova velocity API (the driver's silent latch); start-pose does it
#   fabric_run.sh arm-to <robot> <home|grasp|six joints> [--retreat]   ONE arm in joint space (grasp_joints.py), MoveIt-
#                                       checked, 15 cm from the other arm, deadman held; --retreat when it is already
#                                       closer: the path may only move away from the other arm (2026-11-21 collision)
#   fabric_run.sh perception [seconds]  perception on all three cameras (workspace profile), default 3600 s
#   fabric_run.sh perception-stop
#   fabric_run.sh bridges start|stop    the joint-mode CBF bridges with the cell's bridge.yaml (start disarmed)
#   fabric_run.sh twin [run_dir]        estimator + twin + CBF + RViz, commands OFF, recording into run_dir;
#                                       BANK=true also starts the parameter bank (needed for `warmup`);
#                                       MATERIAL="10 0.1" starts twin and bank at bend x10, shear x0.1 (warm-up test)
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
#   NO_OBSTACLE=1 fabric_run.sh go      baseline with no obstacle at all (no model, no virtual box; CBF and stretch guard on)
#   fabric_run.sh status [seconds]      go/no-go readout (scripts/hw/twin_monitor.py)
#   fabric_run.sh watch [seconds]       readout every 5 s during a commanded run
#   fabric_run.sh enable                twin commands on (formation captured now) + both bridges armed; checks that
#                                       the twin accepted it (it refuses while a hand is unseen) and retries twice
#   MATERIAL="10 0.1" fabric_run.sh warmup-twin [run_dir]  the twin for a warm-up (own terminal): bank on,
#                                       twin and bank start at the given wrong bend/shear scales, empty work
#                                       area (collider 50 m below; the area under the sheet must be clear),
#                                       bank search widened to x19.9 (span 8, 4 rounds) to undo one decade
#   fabric_run.sh warmup-auto ["10 0.1" "10 10"]   the whole warm-up test, spoken (scripts/hw/warmup_auto.py):
#                                       BANK_WORKERS=2 uses parallel fitting (start MPS first; default 1)
#                                       run it and walk to the sheet; each run: twin restarted distorted, 15 s
#                                       to reach the sheet, hands checked, commands on, deadman prompt, warm-up,
#                                       fit, saved and reviewed. Ctrl+C: commands off, twin stopped and saved
#   fabric_run.sh warmup [--shadow]     warm-up calibration (scripts/hw/warmup.py): refuses unless commands are on,
#                                       both bridges armed and the deadman held; robots step 2 and 4 cm toward
#                                       the hands and back (21 s); waits for search convergence, then prints the fit
#   fabric_run.sh disable               twin commands off + both bridges disarmed
#   fabric_run.sh stop-twin             end the compute launch and every node it started (writes the recordings)
#   fabric_run.sh save <run_dir>        large recordings to /home/taha/fabric_hw_data, camera logs from the hosts
#   fabric_run.sh check                 read-only readiness: arms vs grasp and box, fingers, guards (state, joint
#                                       cap), bridges (state, force/box stop), perception output, camera frame age
#   fabric_run.sh clocks [--fix]        each host's clock against the laptop (target 5 ms, tablet 10); --fix restarts
#                                       systemd-timesyncd on a host over the target so it syncs to the laptop again
#                                       (a host that booted without the lab link falls back to campus NTP, 2026-11-21)
#   fabric_run.sh deadman-watch [seconds]   read-only: print every change of /tablet_gui/deadman and e-stop and their
#                                       rate (check the toggle deadman: press once -> ON stays, press again -> OFF)
#   fabric_run.sh bases [seconds]       read-only: each Dingo base from its roof marker (overhead camera, on the NUC)
#                                       against calibration.yaml; OK/WARN/FAIL by the grip shift it causes
#                                       (scripts/hw/base_check.py; also cell_up.sh step 7)
#   fabric_run.sh review <run_dir>      after a run: per arm, commanded vs delivered speed, box faces, where it
#                                       stopped; the twin's faults while enabled (scripts/hw/run_review.py)
#   fabric_run.sh audit <run_dir>       offline: twin and estimator against raw depth, and the real sheet's clearance
#                                       to the obstacle from depth vs the twin's (scripts/hw/twin_audit.py -> audit.json)
#   fabric_run.sh identify <run_dir> [options]   offline, GPU: which twin parameters a recording informs
#                                       (--mode sensitivity) and their fit by ES-MDA over parallel worlds (--mode fit),
#                                       or a residual check (--mode evaluate); see scripts/hw/identify_replay.py --help
#   fabric_run.sh grip-search <run_dir> --out <dir> [options]   offline, GPU: each corner's grip point (pinch
#                                       offset from TF/glove and inset from the sheet corner) fitted on its own to raw
#                                       depth with the strip-measured material (scripts/hw/grip_point_search.py)
#   fabric_run.sh grip-fit <run_dir> [--validate <run>:still ...] [grip-search options]   offline, GPU, minutes: static
#                                       fit of the four grip points (down/in/side) on the person's still hold before
#                                       the warm-up motion, 40 x 40 mesh, keep-moving search; result in <run>/grip_fit
#                                       (apply with grip-apply). Refuses while a twin runs (memory; 2026-11-21 OOM)
#   fabric_run.sh static-fit <run_dir> [<run_dir> ...] [options]   offline, GPU, ~minutes: grip points (corner-weighted)
#                                       then stretch on each run's still hold, Levenberg-Marquardt with a parallel
#                                       Jacobian (scripts/hw/static_fit.py); stretch should agree across runs
#   fabric_run.sh fit-apply <run_dir>   after a warm-up: static-fit on its still hold at the twin's own mesh and solver, then
#                                       grip offsets into the cell (grip-apply --write, added to the recorded ones) and the
#                                       stretch scale into twin.yaml (refused outside x0.3-x3); cell files backed up first
#   fabric_run.sh grip-apply <result.json> --bag <bag_dir> [--write]   write those grip points into the cell config:
#                                       fabric_grip transforms, hand offsets, twin grip insets (scripts/hw/grip_point_apply.py)
#   fabric_run.sh warmup-calibrate <warmup_run> --grips <result.json> --out <dir> [--validate run:window ...]
#                                       offline, GPU: from a warm-up, detect the required mesh, bending and shear, then
#                                       test resolution, grip points, bending and shear alone and in every combination
#                                       on held-out runs (scripts/hw/warmup_calibrate.py)
#   fabric_run.sh warmup-design --out <dir> [--candidates name ...] [--anchors live:<run>:<a>:<b>]
#                                       offline, GPU: simulate candidate warm-up motions and report which can tell
#                                       shear, bending and stretch apart from grip-point errors (scripts/hw/warmup_design.py)
#   fabric_run.sh twin-timing [mesh:substeps:iterations ...]   GPU ms per twin frame at candidate numerics
#                                       (run alone on the GPU; scripts/hw/twin_timing.py)
#   fabric_run.sh mps start|stop|restart|status   CUDA MPS daemon so parallel GPU workers (online bank, grip-search,
#                                       warmup-calibrate, twin-compare) share the GPU; restart it after killing a run
#                                       (its server is left unusable: "Failed to acquire primary context")
#   fabric_run.sh twin-compare --windows run:a:b ... --configs name:mesh:k=v,... --out x.json   named twin
#                                       configurations scored on recorded windows (scripts/hw/twin_compare.py)
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
if [ "${1:-}" != obstacle ] && [ "${1:-}" != audit ] && [ "${1:-}" != identify ] && [ "${1:-}" != grip-search ] && [ "${1:-}" != grip-apply ] && [ "${1:-}" != warmup-calibrate ] && [ "${1:-}" != warmup-design ] && [ "${1:-}" != twin-timing ] && [ "${1:-}" != twin-compare ] && [ "${1:-}" != bank-replay ] && [ "${1:-}" != mps ]; then
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
export CUDA_MPS_PIPE_DIRECTORY="${CUDA_MPS_PIPE_DIRECTORY:-$HOME/.cache/mps/pipe}" CUDA_MPS_LOG_DIRECTORY="${CUDA_MPS_LOG_DIRECTORY:-$HOME/.cache/mps/log}"
offline_env() {  # offline analysis: no lab network or session domain needed; the fabric venv has Newton and Warp
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash; source "$repo/../../install/setup.bash" 2>/dev/null
    export WARP_CACHE_PATH=/tmp/oarbots-fabric-audit-gpu
    export PYTHONPATH="$repo/src/fabric_twin:$repo/src/fabric_state_estimator:$repo/src/human_hand_tracking:${PYTHONPATH:-}"
}
fenced() {  # offline GPU jobs in a memory-capped scope: an overrun kills only the job, never the desktop/VS Code/twin
    local avail_kb; avail_kb="$(awk '/MemAvailable/ {print $2}' /proc/meminfo)"
    local cap_gb=$(( avail_kb / 1048576 - 4 )); [ "$cap_gb" -lt 4 ] && cap_gb=4
    echo "memory fence: this job may use at most ${cap_gb} GB (available minus 4 GB)" >&2
    systemd-run --user --scope -q -p MemoryMax="${cap_gb}G" -p MemorySwapMax=0 "$@"
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
        data="/home/taha/fabric_hw_data/$name"; mkdir -p "$data/cameras"
        for f in "$run"/*.npz; do [ -e "$f" ] && mv "$f" "$data/"; done
        if [ -d "$run/bag" ]; then if [ -e "$data/bag" ]; then echo "bag already saved in $data/bag" >&2; else mv "$run/bag" "$data/bag"; fi; fi
        # Camera perception logs (3-9 MB each) go with the large files, not into git (2026-11-21, .gitignore).
        for m in nuc blue silver; do
            "$LAB_SSH" "${HOST[$m]}" 'cd /dev/shm && tar -czf - run_* 2>/dev/null' > "$data/cameras/${m}_perception_logs.tar.gz"
        done
        (cd "$data" && find . -type f ! -name MANIFEST.sha256 -exec sha256sum {} + > MANIFEST.sha256 2>/dev/null)
        [ -e "$run/twin.cbf.jsonl" ] && gzip -9 -f "$run/twin.cbf.jsonl"
        cp "$data/MANIFEST.sha256" "$run/large_files.sha256" 2>/dev/null
        echo "Large files and camera logs: $data (checksums also in $run/large_files.sha256)"
}

cmd="${1:-}"; shift || true
case "$cmd" in
    recentre) "$here/start_pose.sh" --force ;;   # both arms to grasp.yaml now, whatever their offset
    start-pose) "$here/start_pose.sh" "$@" ;;
    warmup-start) "$here/start_pose.sh" --pose "$repo/experiments/hardware/cell/warmup_grasp.yaml" "$@" ;;
    stop-go)     # end a running `go` from another terminal: same as Ctrl+C there (commands off, bridges disarmed,
                 # then that go stops the twin, saves and reviews the run)
        pkill -INT -f "scripts/hw/auto_start.py" && echo "go is ending: commands off; the twin stops and the run is saved" || echo "no go running" ;;
    arm-to)      # one arm, deadman-supervised; bridges stopped for the move (they share the joint input), then restarted
        r="${1:?arm-to <blue|silver> <home|grasp|six joints> [--retreat]}"; shift
        [ "$r" = blue ] || [ "$r" = silver ] || { echo "arm-to: robot is blue or silver" >&2; exit 2; }
        retreat=(); target=(); for a in "$@"; do [ "$a" = --retreat ] && retreat=(--retreat) || target+=("$a"); done
        [ ${#target[@]} -gt 0 ] || { echo "arm-to: give home, grasp or six joint angles" >&2; exit 2; }
        for b in blue silver; do scripts/hw/lab.sh session "$b" exec systemctl --user stop fabric-bridge >/dev/null 2>&1; done
        echo "  $r driver: $(timeout 40 scripts/hw/lab.sh session "$r" exec ros2 service call "/oarbot_$r/kinova/j2n6s300_driver/in/start" kinova_msgs/srv/Start 2>&1 | sed -n "s/.*start_result='\([^']*\)'.*/\1/p")"
        pre="$(scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --to "${target[@]}" "${retreat[@]}" --check-only 2>&1 | grep -E 'Preflight|ABORT')"
        echo "  $r preflight: $pre"
        rc=1
        if grep -q 'Preflight passed' <<< "$pre" && ! grep -q ABORT <<< "$pre"; then
            spd-say -r -10 "Moving ${r^} only. Press the deadman once to start. Press again to stop." >/dev/null 2>&1
            scripts/hw/lab.sh session "$r" exec python3 fabric_ws/scripts/hw/grasp_joints.py --robot "$r" --to "${target[@]}" "${retreat[@]}" --wait 600 |
                while IFS= read -r line; do echo "  $line"; [[ "$line" == Arrived* ]] && spd-say -r -10 "${r^} arrived. Press the deadman once to end." >/dev/null 2>&1; done
            rc=${PIPESTATUS[0]}
        else spd-say -r -10 "${r^} failed its check" >/dev/null 2>&1; fi
        for b in blue silver; do scripts/hw/lab.sh session "$b" start-cell-bridge "/home/oarbot_$b/fabric_ws/experiments/hardware/cell" 2>&1 | tail -1 | sed "s/^/  $b: /"; done
        exit $rc ;;
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
    twin|go|warmup-twin)
        if [ "$cmd" = warmup-twin ]; then
            # The warm-up session: parameter bank on, deliberately wrong starting material, and an empty work area
            # (no real obstacle model; the virtual collider 50 m below the cell, as in the 2026-11-20 07:53 session)
            # so the CBF does not reshape the warm-up motion. Needs the area under the sheet clear.
            if [ -z "${MATERIAL:-}" ] || { [ "$MATERIAL" = "1 1" ] && [ "${NOMINAL:-0}" != 1 ]; }; then
                echo "warmup-twin needs a distorted start: MATERIAL=\"10 0.1\" (then \"10 10\"); NOMINAL=1 MATERIAL=\"1 1\" on purpose" >&2
                exit 1; fi
            BANK=true
        fi
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
        if [ "$cmd" = warmup-twin ]; then
            "$here/start_pose.sh" --pose "$repo/experiments/hardware/cell/warmup_grasp.yaml" --check || {
                echo "Warm-up requires its narrower start pose: fabric_run.sh warmup-start" >&2; exit 1; }
        fi
        prefix=live; [ "$cmd" = warmup-twin ] && prefix="warmup_$(echo "${MATERIAL:-1 1}" | tr ' ' '_')"
        run="${1:-$repo/experiments/hardware/sessions/$(date +%F)_fabric_run/${prefix}_$(date +%H%M%S)}"
        mkdir -p "$run"; echo "$run" > "$state/last_run"
        git rev-parse HEAD > "$run/commit.txt"; cp experiments/hardware/cell/*.yaml "$run/" 2>/dev/null; mkdir -p "$run/cell"; mv "$run"/*.yaml "$run/cell/"
        cell="$repo/experiments/hardware/cell"
        if [ "$cmd" = warmup-twin ]; then
            cell="$run/cell"; rm -f "$cell/obstacle.yaml"; cp experiments/hardware/cell/fabric_cell.rviz "$cell/"
            python3 - "$cell/twin.yaml" <<'PY' || exit 1
import sys, yaml
path = sys.argv[1]
config = yaml.safe_load(open(path))
config['obstacle'] = dict(config['obstacle'], center=[0.0, 0.0, -50.0], half_extents=[0.1, 0.1, 0.1])
config.setdefault('runtime', {})['warmup_only'] = True
config.setdefault('cbf', {})['enabled'] = False
config.setdefault('stretch_guard', {})['enabled'] = False
open(path, 'w').write('# Empty work area for the warm-up: virtual collider 50 m below the cell (fabric_run.sh warmup-twin).\n'
                      + yaml.safe_dump(config, sort_keys=False))
PY
            echo "Warm-up profile: $cell (CBF OFF, stretch guard OFF; scripted trajectory only). The area under the sheet must be clear."
        elif [ "${NO_OBSTACLE:-0}" = 1 ]; then
            # Baseline run with no obstacle: the detected obstacle is not used and twin.yaml's virtual box goes 50 m
            # below the cell (it would otherwise be a 44 x 44 x 32 cm box at the cell centre). CBF and stretch guard ON.
            cell="$run/cell"; rm -f "$cell/obstacle.yaml"; cp experiments/hardware/cell/fabric_cell.rviz "$cell/"
            python3 - "$cell/twin.yaml" <<'PY' || exit 1
import sys, yaml
path = sys.argv[1]
config = yaml.safe_load(open(path))
config['obstacle'] = dict(config['obstacle'], center=[0.0, 0.0, -50.0], half_extents=[0.1, 0.1, 0.1])
open(path, 'w').write('# No-obstacle baseline: virtual collider 50 m below the cell (NO_OBSTACLE=1 fabric_run.sh).\n'
                      + yaml.safe_dump(config, sort_keys=False))
PY
            echo "No-obstacle run: $cell (no obstacle model; CBF and stretch guard ON). Keep the work area clear."
        fi
        if [ -f "$cell/obstacle.yaml" ]; then   # the obstacle this run uses: its model plus the capture's mesh and picture
            obj="$(cat "$repo/experiments/hardware/cell/obstacle_source.txt" 2>/dev/null)"
            [ -d "$obj" ] && [ -f "$obj/obstacle.yaml" ] && cmp -s "$obj/obstacle.yaml" "$cell/obstacle.yaml" || obj="$(sed -n 's/^  object: //p' "$cell/obstacle.yaml")"
            mkdir -p "$run/cell/obstacle_capture"
            case "$obj" in /*) ;; *) obj="$repo/$obj" ;; esac   # the detector writes absolute paths, older models relative
            for f in obstacle.yaml obstacle.obj obstacle.urdf obstacle.png fit.log; do [ -f "$obj/$f" ] && cp "$obj/$f" "$run/cell/obstacle_capture/"; done
            [ -n "$(ls -A "$run/cell/obstacle_capture")" ] || echo "WARNING: obstacle capture files not found in $obj" >&2
            echo "Obstacle: $(sed -n 's/^  shape: //p' "$cell/obstacle.yaml") from $obj"
        elif [ "${NO_OBSTACLE:-0}" = 1 ]; then echo "Obstacle: none (collider 50 m below the cell)"
        else echo "Obstacle: twin.yaml's virtual box (no detected obstacle in use)"; fi
        cat > "$run/RECORDING.txt" <<REC
Recorded in this run (fabric_run.sh $cmd, $(date '+%F %T')); fabric_run.sh save moves the large files to
/home/taha/fabric_hw_data/$(basename "$(dirname "$run")")_$(basename "$run"):
  cell/               twin.yaml (sheet, CBF, stretch guard), estimator.yaml, grasp.yaml, calibration.yaml, bridge.yaml,
                      obstacle.yaml as used (+ obstacle_capture/: mesh, URDF, picture of the detected obstacle)
  twin.npz            twin sheet state (positions, velocities), the four grips, commands sent
  twin.depth.npz      every camera depth point on the sheet (world frame), for offline scoring
  twin.cbf.jsonl.gz   per CBF frame: clearance, barrier h, model tension and strain, violation, slacks, correction,
                      stretch-guard state, box clearance, nominal and sent commands, timings
  twin.status.jsonl   twin diagnostics (faults, commands on/off, material, frame times, lag)
  estimate.npz        estimated sheet state (positions, velocities, covariance, visibility); estimate.hands.npy
  bag/ (mcap)         measured force/torque at both grips, joint states, bridge/guard/camera diagnostics, deadman,
                      e-stops, TF, gloves, hands, camera observations, all /fabric topics, commands and arming
  compute.log, commit.txt; after the run review.txt and cameras/ (perception logs from the hosts)
REC
        echo "Run folder: $run"
        echo "The twin starts once both hands and both robot grips are measured (the estimator needs ~2 min first)."
        read -r start_bend start_shear extra <<< "${MATERIAL:-1 1}"
        if [ -n "${extra:-}" ] || ! python3 -c "import sys; b, s = map(float, sys.argv[1:]); assert 1/16 <= min(b, s) and max(b, s) <= 16" \
                "$start_bend" "${start_shear:-x}" 2>/dev/null; then
            echo "MATERIAL must be two scales in 1/16-16, \"bend shear\" (e.g. MATERIAL=\"10 0.1\"), not \"$MATERIAL\"" >&2; exit 1; fi
        echo "bend $start_bend shear $start_shear" > "$run/material_start.txt"
        [ "${MATERIAL:-1 1}" != "1 1" ] && echo "Twin and bank start DISTORTED: bend x$start_bend, shear x$start_shear (of twin.yaml)"
        ros_env
        launch=(ros2 launch "$repo/src/fabric_cell/launch/compute.launch.py" cell_dir:="$cell"
                nominal_grips:=true twin:=true bank:="${BANK:-false}" commands:=false rviz:=true record:="$run" record_depth:=true
                start_bend:="$start_bend" start_shear:="$start_shear")
        # warm-up: widen the bank's search to reach a one-decade start error (x19.9 instead of x6.73)
        [ "$cmd" = warmup-twin ] && launch+=(search_span:=8 search_rounds:=4 search_workers:="${BANK_WORKERS:-1}"
                                             search_max_seconds:="${BANK_MAX_SECONDS:-600}")
        if [ "$cmd" != go ]; then "${launch[@]}" 2>&1 | tee "$run/compute.log"; exit; fi
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
    enable)      # the twin refuses when a hand is not seen at that moment; then nothing is armed (2026-11-19 21:50)
        ros_env
        ok=1
        for attempt in 1 2 3; do
            pub_bool /fabric/twin/enable_commands true
            python3 "$here/warmup.py" enabled --timeout 4 && { ok=0; break; }
            [ "$attempt" -lt 3 ] && echo "retrying enable ($attempt/3)"
        done
        if [ "$ok" -ne 0 ]; then
            echo "NOT ENABLED: bridges left disarmed; the robots will not move."; exit 1
        fi
        for r in blue silver; do pub_bool "/oarbot_$r/fabric_cbf/arm" true; done
        sleep 2; python3 "$here/twin_monitor.py" 6 | tail -1
        echo "Armed. Deadman: release, press, hold. Release stops both arms. fabric_run.sh disable to end." ;;
    warmup-auto) # the whole warm-up test for one operator at the sheet, spoken: both runs, waits for them
        ros_env; python3 "$here/warmup_auto.py" "$@" ;;
    warmup)      # warm-up calibration: only with commands on, both bridges armed and the deadman held
        ros_env; python3 "$here/warmup.py" start "$@" ;;
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
                realpath -- "$dir" > "$cell/obstacle_source.txt"   # the selected candidate (a refit has its own mesh/picture)
                echo "Twin obstacle: $dir (experiments/hardware/cell/obstacle.yaml). The next fabric_run.sh go/twin uses it."
                summarize_model ;;
            clear) rm -f "$active" "$cell/obstacle_source.txt" && echo "Twin obstacle: twin.yaml's virtual box" ;;
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
    clocks)
        targets=(oarbot_blue@192.168.1.71 oarbot_silver@192.168.1.110 nuc@192.168.1.90 tablet@192.168.1.99)
        python3 "$here/clock_offsets.py" "${targets[@]}" 2>/dev/null | tee "$state/clocks.txt"
        if [ "${1:-}" = --fix ]; then
            # The lab drop-in (2026-11-17) plus, since 2026-11-21, a 64 s poll cap: timesyncd backed off to 34 min
            # polls while its frequency was ~35 ppm off, so Silver and the NUC swung -22..+28 ms between polls.
            dropin='# Fabric cell (2026-11-17): sync to the compute laptop on the lab LAN first, the campus server second.
# 2026-11-21: poll at most every 64 s (34 min polls let a 35 ppm frequency error drift ~70 ms).
# Revert: delete this file, then: sudo systemctl restart systemd-timesyncd
[Time]
NTP=192.168.1.200 ntp-ipv4.rpi.edu
PollIntervalMinSec=16
PollIntervalMaxSec=64'
            for host in "${targets[@]}"; do
                have="$("$LAB_SSH" "$host" 'cat /etc/systemd/timesyncd.conf.d/fabric-lab.conf 2>/dev/null')"
                server="$(grep "^$host " "$state/clocks.txt" | sed -n 's/.*server \([^ ]*\).*/\1/p')"
                off="$(grep "^$host " "$state/clocks.txt" | awk '{print $2}')"
                limit=5; [[ "$host" == tablet* ]] && limit=10
                if [ "$have" != "$dropin" ] || [ "$server" != 192.168.1.200 ] || awk -v a="${off#[+-]}" -v l="$limit" 'BEGIN {exit !(a > l)}'; then
                    echo "  $host ($off ms, server ${server:-?}): lab timesyncd drop-in written, timesyncd restarted"
                    # password on stdin for sudo, then the file content
                    { cat "$LAB_PW_FILE"; printf '%s\n' "$dropin"; } | "$LAB_SSH" "$host" "sudo -k -S -p '' sh -c 'mkdir -p /etc/systemd/timesyncd.conf.d && cat > /etc/systemd/timesyncd.conf.d/fabric-lab.conf && systemctl restart systemd-timesyncd'"
                fi
            done
            sleep 10; echo "after:"; python3 "$here/clock_offsets.py" "${targets[@]}" 2>/dev/null
        fi ;;
    deadman-watch)
        ros_env; python3 - "${1:-60}" <<'PY'
import sys, time, rclpy
from std_msgs.msg import Bool
rclpy.init(); n = rclpy.create_node('deadman_watch'); st = {}; count = {'deadman': 0, 'e_stop': 0}
def cb(name):
    def f(m):
        count[name] += 1
        if st.get(name) != m.data:
            st[name] = m.data
            print(f"{time.strftime('%H:%M:%S')}  {name}: {'ON' if m.data else 'OFF'}", flush=True)
    return f
for name in count:
    n.create_subscription(Bool, f'/tablet_gui/{name}', cb(name), 10)
end, t0 = time.time() + float(sys.argv[1]), time.time()
while time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.1)
print('rates: ' + ', '.join(f'{k} {v / (time.time() - t0):.1f} Hz' for k, v in count.items()))
PY
        ;;
    bases) python3 "$here/base_check.py" "${1:-10}" ;;
    review) ros_env; python3 "$here/run_review.py" "${1:?review <run_dir>}" ;;
    audit) offline_env; ~/venvs/fabric_ros/bin/python "$here/twin_audit.py" "${1:?audit <run_dir>}" "${@:2}" ;;
    identify) offline_env; fenced ~/venvs/fabric_ros/bin/python "$here/identify_replay.py" "${1:?identify <run_dir> [options]}" "${@:2}" ;;
    grip-apply) offline_env; ~/venvs/fabric_ros/bin/python "$here/grip_point_apply.py" "${1:?grip-apply <result.json> --bag <bag>}" "${@:2}" ;;
    mps)
        mkdir -p "$CUDA_MPS_PIPE_DIRECTORY" "$CUDA_MPS_LOG_DIRECTORY"
        running() { pgrep -f "^nvidia-cuda-mps-control" >/dev/null; }
        case "${1:-status}" in
            start)   running && echo "MPS already running" || { nvidia-cuda-mps-control -d && echo "MPS started"; } ;;
            stop)    echo quit | nvidia-cuda-mps-control 2>/dev/null; sleep 2; echo "MPS stopped" ;;
            restart) echo quit | nvidia-cuda-mps-control 2>/dev/null; sleep 2; nvidia-cuda-mps-control -d && echo "MPS restarted" ;;
            status)  running && echo "MPS running ($CUDA_MPS_PIPE_DIRECTORY)" || echo "MPS not running" ;;
            *) echo "mps start|stop|restart|status" >&2; exit 2 ;;
        esac ;;
    warmup-design) offline_env; ~/venvs/fabric_ros/bin/python "$here/warmup_design.py" "$@" ;;
    twin-compare) offline_env; ~/venvs/fabric_ros/bin/python "$here/twin_compare.py" "$@" ;;
    twin-timing) offline_env; ~/venvs/fabric_ros/bin/python "$here/twin_timing.py" "$@" ;;
    bank-replay) offline_env; ~/venvs/fabric_ros/bin/python "$here/bank_replay.py" "$@" ;;
    warmup-calibrate) offline_env; fenced ~/venvs/fabric_ros/bin/python "$here/warmup_calibrate.py" "${1:?warmup-calibrate <warmup_run> --grips <result.json> --out <dir>}" "${@:2}" ;;
    grip-fit)
        run="${1:?grip-fit <run_dir> [--validate <run>:still ...]}"; shift
        if [ -n "$(twin_pids)" ]; then echo "A twin (compute launch) is running: stop it first (its GPU memory and the workers' do not fit together)" >&2; exit 1; fi
        offline_env
        pgrep -f nvidia-cuda-mps-control >/dev/null || { nvidia-cuda-mps-control -d && echo "MPS started"; }
        # Stage 1: grip points from the sheet near them (corner-weighted, user 2026-11-21: an unweighted fit dropped the
        # hands 10-12 cm to imitate the belly's sag). Stage 2: STRETCH only on the whole sheet with the fitted grips held
        # (user: a still hanging sheet's sag is set by stretch; shear and bending are invisible to it, and today's online
        # fits moved shear x0.06..x16 with no change in error). Both on the still window: place, settle, score.
        # GRIP_FIT_STAGES="2" runs stage 2 alone (on an existing grip_fit/grips/result.json).
        common=(--window still:3 --override cells_x=40 cells_y=40 substeps=32 iterations=10 --max-passes 10 \
                --workers 6 --depth-every 1 --points 128)
        stages="${GRIP_FIT_STAGES:-1 2}"
        { if [[ "$stages" == *1* ]]; then echo "== stage 1: grip points (points within 20 cm of a grip weigh 1, the rest 0.1)"
          fenced ~/venvs/fabric_ros/bin/python "$here/grip_point_search.py" "$run" "${common[@]}" --kinds down,in,side \
              --step 0.02 --min-step 0.005 --near-m 0.2 --far-weight 0.1 --out "$run/grip_fit/grips" "$@" || exit 1; fi
          if [[ "$stages" == *2* ]]; then echo "== stage 2: stretch with the fitted grips held (whole sheet)"
          fenced ~/venvs/fabric_ros/bin/python "$here/grip_point_search.py" "$run" "${common[@]}" --kinds "" --start \
              --material stretch --material-step 0.25 --fix-from "$run/grip_fit/grips/result.json" \
              --out "$run/grip_fit/material" "$@"; fi; } 2>&1 | tee -a "$run/grip_fit.log" ;;
    fit-apply)
        run="${1:?fit-apply <warmup_run_dir>}"; run="${run%/}"
        if [ -n "$(twin_pids)" ]; then echo "A twin (compute launch) is running: stop it first" >&2; exit 1; fi
        name="$(basename "$(dirname "$run")")_$(basename "$run")"; bag="/home/taha/fabric_hw_data/$name/bag"
        [ -d "$bag" ] || { echo "no saved bag for $run ($bag): fabric_run.sh save it first" >&2; exit 1; }
        read -r cx cy ss it <<< "$(python3 -c "import yaml; s=yaml.safe_load(open('$repo/experiments/hardware/cell/twin.yaml'))['sheet']; print(s['cells_x'], s['cells_y'], s['substeps'], s['iterations'])")"
        mkdir -p "$run/fit_apply/cell_before" && cp "$repo"/experiments/hardware/cell/{twin,fusion,calibration}.yaml "$run/fit_apply/cell_before/"
        echo "== fit at the twin's numerics: ${cx}x${cy}, ${ss}x${it}"
        "$0" static-fit "$run" --override cells_x=$cx cells_y=$cy substeps=$ss iterations=$it --tag fit_apply || exit 1
        offline_env
        ~/venvs/fabric_ros/bin/python - "$run" "$repo/experiments/hardware/cell/twin.yaml" <<'PY' || exit 1
import json, re, sys
from pathlib import Path
run, twin = Path(sys.argv[1]), Path(sys.argv[2])
r = json.loads((run / 'fit_apply/result.json').read_text())
# grip-apply reads grip_point_search's format
(run / 'fit_apply/grips_for_apply.json').write_text(json.dumps(dict(run=str(run.resolve()), window=r['window'],
    names=r['grips']['names'], calibrated=r['grips']['values'])))
scale = float(r['stretch_scale'])
text = twin.read_text()
m = re.search(r'\n  stretch: \[([0-9.e+-]+), ([0-9.e+-]+), ([0-9.e+-]+)\]', text)
weft, warp, shear = (float(x) for x in m.groups())
if not 0.3 <= scale <= 3.:
    print(f'stretch x{scale:.3f} relative to twin.yaml is outside x0.3-x3: NOT applied (check the fit; the twin keeps {weft:g})')
else:
    new = f'\n  stretch: [{weft*scale:.1f}, {warp*scale:.1f}, {shear:g}]'
    twin.write_text(text[:m.start()] + new + text[m.end():])
    print(f'twin.yaml stretch weft/warp {weft:g} -> {weft*scale:.1f} (x{scale:.3f}, fitted on {run.name})')
PY
        echo "== grip offsets into the cell (added to the ones the run was recorded with)"
        "$0" grip-apply "$run/fit_apply/grips_for_apply.json" --bag "$bag" --write
        echo "Cell files before: $run/fit_apply/cell_before (revert = copy back). The next twin start uses the fit." ;;
    static-fit)
        if [ -n "$(twin_pids)" ]; then echo "A twin (compute launch) is running: stop it first" >&2; exit 1; fi
        offline_env
        pgrep -f nvidia-cuda-mps-control >/dev/null || { nvidia-cuda-mps-control -d && echo "MPS started"; }
        fenced ~/venvs/fabric_ros/bin/python "$here/static_fit.py" "$@" ;;
    grip-search) offline_env; fenced ~/venvs/fabric_ros/bin/python "$here/grip_point_search.py" "${1:?grip-search <run_dir> --out <dir>}" "${@:2}" ;;
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
