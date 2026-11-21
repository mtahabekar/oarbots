#!/usr/bin/env python3
"""The whole warm-up test, one command, spoken (`fabric_run.sh warmup-auto ["10 0.1" "10 10"]`).

For an operator who holds the sheet and cannot type. Run it, then walk to the sheet: the laptop speaks
(spd-say) every step and waits for you. Per run (one per MATERIAL, default "10 0.1" then "10 10"):

  1. Restores the narrower warm-up pose under the supervised deadman procedure if needed. Stops any
     running twin; starts `fabric_run.sh warmup-twin` with that distorted start (bank on, empty
     work area). 15 s to reach the sheet, then: hold both free corners with the gloves, tags to the overhead
     camera, sheet slightly slack, keep still.
  2. Waits until the twin follows the measured grips and both hands are seen steadily, saying which glove
     is unseen. Turns commands on (formation captured now: hold the sheet as it should stay; retried while a
     hand drops out), arms both bridges.
  3. Deadman operator: "press once" (toggle deadman since 2026-11-21; a press during the motion stops the run and
     the sequence). Starts the warm-up (twin.yaml `warmup`, since 2026-11-21: 24 labelled
     holds, ~194 s; robots first while the hands stay still, then spoken hand moves for the person, robots
     still). If the deadman is released during it, the run is incomplete: commands off, the run is
     stopped, saved and repeated from step 1.
  4. "Press the deadman once to end": commands off, bridges disarmed. The bank fits until convergence (progress spoken).
     Speaks the fit (start -> result, search edge, released), stops the twin, saves, writes review.txt.

Perception on the cameras runs for a set time and then stops silently (2026-11-20 08:50: the twin waited
for hands that no camera reported). Before the first run and while waiting for the hands, no glove frames
from any camera means it is restarted (`fabric_run.sh perception 7200`, no motion).

Ctrl+C at any time: commands off, bridges disarmed, twin stopped and saved. Releasing the deadman always
stops both arms. Uses warmup_grasp.yaml (77 cm robot span), checked again before enabling. Needs the
area under the sheet clear (warmup-twin's
empty-cell profile), a second person at the tablet deadman (the arms move only while it is held).
"""
import argparse
import datetime
import json
import math
import os
import re
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from fabric_state_msgs.msg import GloveObservations, HandStates
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
WARMUP_POSE = REPO / 'experiments/hardware/cell/warmup_grasp.yaml'
sys.path.insert(0, str(HERE))
import warmup  # noqa: E402  (scripts/hw/warmup.py: the status node and the search-edge test)

RUN_SH = str(HERE / 'fabric_run.sh')
ROBOTS = warmup.ROBOTS
HANDS = (('human_left', 'left'), ('human_right', 'right'))
FRESH_SEC = .3       # a hand counts as seen if the fusion reported it with a camera this recently
STEADY_SEC = 3.      # both hands seen this long before commands go on
REACH_SEC = 15.      # the operator's walk to the sheet (user, 2026-11-20)
GLOVE_TOPICS = ('/nuc/fabric/gloves', '/oarbot_blue/azure_kinect/fabric/gloves', '/oarbot_silver/azure_kinect/fabric/gloves')
PERCEPTION_SEC = 7200  # a restarted perception run lasts this long (fabric_run.sh perception); it expires silently


class Abort(Exception):
    pass


class Retry(Exception):
    pass


def say(text, wait=True):
    print(f'{time.strftime("%H:%M:%S")} SAY: {text}', flush=True)
    try:
        call = subprocess.run if wait else subprocess.Popen
        call(['spd-say', '-r', '-10'] + (['-w'] if wait else []) + [text],
             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass


def log(text):
    print(f'{time.strftime("%H:%M:%S")} {text}', flush=True)


def words(x):
    """A scale as spd-say reads it well: 10, 0.1, 0.62."""
    return f'{x:.2g}'


class Cell(warmup.Warmup):
    def __init__(self):
        super().__init__()
        self.seen = {}  # hand_id -> monotonic time last reported with a camera
        self.glove_frame = -1e9  # monotonic time of the newest glove frame from any camera (perception alive)
        self.create_subscription(HandStates, '/fabric/hand_states', self.on_hands, qos_profile_sensor_data)
        for topic in GLOVE_TOPICS:
            self.create_subscription(GloveObservations, topic, self.on_gloves, qos_profile_sensor_data)

    def on_hands(self, msg):
        now = time.monotonic()
        for h in msg.hands:
            if len(h.source_cameras):
                self.seen[h.hand_id] = now

    def on_gloves(self, msg):
        self.glove_frame = time.monotonic()

    def perceiving(self):
        return time.monotonic() - self.glove_frame < 2.

    def unseen(self):
        now = time.monotonic()
        return [name for hand, name in HANDS if now - self.seen.get(hand, -1e9) > FRESH_SEC]

    def following(self):
        return bool(self.twin) and not self.twin.get('fault') and 'following' in self.twin.get('message', '')

    def commands(self, on):
        for _ in range(3):  # latched-free topics: repeat so a late subscriber still gets it
            self.enable_pub.publish(Bool(data=on))
            for r in ROBOTS:
                self.arm_pub[r].publish(Bool(data=on))
            self.spin(.2)


def twin_running():
    return subprocess.run(['pgrep', '-f', 'ros2 launch .*compute.launch.py'], stdout=subprocess.DEVNULL).returncode == 0


def stop_twin():
    if twin_running():
        log('stopping the compute launch (writes its recordings)')
        subprocess.run(['bash', RUN_SH, 'stop-twin'], stdin=subprocess.DEVNULL)


def ensure_perception(node):
    """Glove frames from the cameras, or restart perception on all three (it ends after its set time)."""
    if node.spin(4, node.perceiving):
        return
    say('The cameras stopped detecting. Restarting perception; about thirty seconds.', wait=False)
    log(f'no glove frames from any camera: fabric_run.sh perception {PERCEPTION_SEC}')
    subprocess.run(['bash', RUN_SH, 'perception', str(PERCEPTION_SEC)], stdin=subprocess.DEVNULL)
    if not node.spin(90, node.perceiving):
        raise Abort('perception did not come back on the cameras')
    log('perception publishing again')


def finish_run(node, run, reason):
    """Commands off, twin stopped, run saved and reviewed."""
    node.commands(False)
    stop_twin()
    log(f'saving {run} ({reason})')
    subprocess.run(['bash', RUN_SH, 'save', str(run)], stdin=subprocess.DEVNULL)
    review = subprocess.run(['bash', RUN_SH, 'review', str(run)], stdin=subprocess.DEVNULL,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True).stdout
    (run / 'review.txt').write_text(review)
    (run / 'warmup_auto.txt').write_text(reason + '\n')
    print(review, flush=True)


def slack_cue(problem):
    """Spoken cue for a warm-up hold that would over-stretch a pair with a hand in it (the twin's
    warmup_start_problem), or None when the person cannot fix it. 2026-11-21 10:12: the sheet's Silver-side edge
    started at 0.765-0.805 m against its 0.805 m limit, so "hands down 25 cm" would have stretched it; stepping
    closer to the robots gives the hand cues their room."""
    m = re.search(r'would stretch .*hand.* to ([0-9.]+) m \(start ([0-9.]+) m, stretch-guard limit ([0-9.]+) m\)', problem)
    if not m:
        return None
    after, before, limit = map(float, m.groups())
    excess_cm = 100 * (after - max(before, limit))
    step = int(5 * math.ceil((excess_cm + 3) / 5))      # rounded up to 5 cm, at least 3 cm spare
    return (f'The sheet is too tight for the hand moves. Step about {min(step, 30)} centimetres closer to the robots, '
            'let the sheet hang slack, and hold still.')


def wait_ready(node, run, start_scales):
    """Twin following from the requested start, bridges present, both hands steadily seen, and the warm-up's
    planned holds within the stretch limits for STEADY_SEC (a hand-hold problem is spoken as a cue to step closer,
    not an abort: the twin's corners settle over the first seconds and the person can add slack)."""
    start, last_note, steady_since, last_hint, last_log = time.monotonic(), 0., None, 0., 0.
    last_slack = 0.
    while True:
        node.spin(.25)
        now = time.monotonic()
        if now - start > 420:
            raise Abort('the twin did not start in 7 minutes')
        if now - last_log > 5 and (run / 'compute.log').exists():
            last_log = now
            if 'process has died' in (run / 'compute.log').read_text(errors='ignore'):
                raise Abort('a node of the twin died at start; see compute.log')
        missing = node.unseen()
        if not node.perceiving() and now - start > 10:
            ensure_perception(node)  # expired during the run: restart it (no motion)
        elif missing and node.perceiving() and now - last_hint > 8 and now - start > 5:
            say(f'{" and ".join(missing)} glove not seen. Turn the tag toward the overhead camera.', wait=False)
            last_hint = now
        if node.following() and not missing:
            problem = node.twin.get('warmup_start_problem', 'restart the twin with the updated warm-up geometry check')
            if problem:
                cue = slack_cue(problem)
                if cue is None:
                    raise Abort(problem)
                steady_since = None
                if now - last_slack > 10:
                    log(f'waiting for slack: {problem}')
                    say(cue, wait=False)
                    last_slack = now
                continue
            steady_since = steady_since or now
            if now - steady_since >= STEADY_SEC:
                material = json.loads(node.twin.get('material') or '{}')
                if (material.get('version') != 0 or abs(material.get('bend', 1.) - start_scales[0]) > 1e-6
                        or abs(material.get('shear', 1.) - start_scales[1]) > 1e-6):
                    raise Abort(f'the twin is not at the requested start: {material}')
                absent = [r for r in ROBOTS if r not in node.bridges]
                if absent:
                    log(f'no diagnostics from the {" and ".join(absent)} bridge: starting the bridges (disarmed)')
                    subprocess.run(['bash', RUN_SH, 'bridges', 'start'], stdin=subprocess.DEVNULL)
                    node.spin(10)
                    if any(r not in node.bridges for r in ROBOTS):
                        raise Abort('a bridge is not running')
                return
        else:
            steady_since = None
        if now - last_note > 40 and not node.following():
            state = node.twin.get('message', 'starting') if node.twin else 'starting'
            say(f'Waiting for the twin: {state}. Keep holding still.', wait=False)
            last_note = now


def enable(node):
    if node.twin.get('warmup_only') != 'True':
        raise Abort('the twin is not in warm-up-only mode; restart it with the updated warmup-twin command')
    problem = node.twin.get('warmup_start_problem', 'restart the twin with the updated warm-up geometry check')
    if problem:
        raise Abort(problem)
    if (node.twin.get('cbf_enabled') != 'False' or node.twin.get('stretch_guard_enabled') != 'False'
            or node.twin.get('command_mode') != 'warmup_direct'):
        raise Abort('restart with the updated warmup-twin profile: warm-up requires CBF off and stretch guard off')
    log('warm-up direct trajectory: CBF OFF, stretch guard OFF')
    say('Hold still, the sheet as it should stay. Turning commands on.')
    for attempt in range(8):
        deadline = time.monotonic() + 6
        while node.unseen() and time.monotonic() < deadline:
            node.spin(.1)
        node.enable_pub.publish(Bool(data=True))
        if node.spin(3, node.enabled):
            break
        missing = node.unseen()
        say(f'Commands refused. {" and ".join(missing) + " glove not seen. " if missing else ""}Hold still.', wait=False)
        log(f'enable refused (attempt {attempt + 1}): twin "{node.twin.get("message")}"')
    else:
        raise Abort('the twin refused commands eight times: a hand keeps dropping out')
    for r in ROBOTS:
        node.arm_pub[r].publish(Bool(data=True))
    if not node.spin(6, lambda: all(node.armed(r) for r in ROBOTS)):
        problems = [f'{r}: {node.bridges.get(r)}' for r in ROBOTS if not node.armed(r)]
        raise Abort('bridge not armed: ' + '; '.join(problems))
    log('commands on, both bridges armed')


def deadman_off(node):
    """Toggle deadman (since 2026-11-21): a deadman left on from an earlier step is turned off first, so the
    warm-up starts only on a deliberate press."""
    if node.deadman:
        say('The deadman is still on. Press it once to turn it off.')
        asked = time.monotonic()
        while not node.spin(10, lambda: node.deadman is False):
            if time.monotonic() - asked > 60:
                raise Abort('the deadman was not turned off within a minute')
            say('Deadman: press once to turn it off.', wait=False)


def excite(node):
    deadman_off(node)
    say('Deadman: press once to start the warm-up. Press again only to stop it.')
    asked = time.monotonic()
    while not node.spin(10, lambda: bool(node.deadman)):
        if time.monotonic() - asked > 60:
            raise Abort('the deadman was not pressed within a minute')
        say('Deadman: press once to start.', wait=False)
    node.spin(1)
    if 'hands' in (node.twin.get('warmup_plan') or ''):
        say('Warm-up starting. Keep both hands still until I call a hand move. Move only when I say, '
            'then hold still again. Never pull the sheet tight.')
    else:
        say('Warm-up starting. Keep both hands completely still.')
    if not node.deadman:
        node.commands(False)
        raise Abort('stopped with the deadman before the warm-up started')
    node.window, node.bank = None, []
    # The twin reports its warm-up plan: (move starts s, hold starts s, label) per hold, and the duration.
    try:
        cues = [(float(move), label) for move, _, label in json.loads(node.twin.get('warmup_plan') or '[]')]
    except (ValueError, TypeError):
        cues = []
    limit = float(node.twin.get('warmup_duration_sec', 21.))+20.
    node.start_pub.publish(String(data='warmup_auto'))
    started, peak = time.monotonic(), {r: 0. for r in ROBOTS}
    spoken = set()
    while node.window is None:
        node.spin(.1)
        if node.twin.get('warmup') == 'idle' and node.twin.get('warmup_start_problem'):
            raise Abort(node.twin['warmup_start_problem'])
        t = time.monotonic() - started
        for r in ROBOTS:
            peak[r] = max(peak[r], node.cmd[r])
        if node.deadman is False:   # the second press of the toggle deadman ends the run
            node.commands(False)
            say('Deadman pressed: warm-up stopped. Commands are off.', wait=False)
            raise Abort('stopped with the deadman during the excitation')
        for k, (at, label) in enumerate(cues or [(4., 'slack 2 cm'), (10., 'slack 4 cm'), (15.5, 'return')]):
            if t >= at and k not in spoken:
                spoken.add(k); say(label.replace(' cm', ' centimetres'), wait=False)
        if t > limit:
            raise Abort('the twin did not announce the end of the warm-up')
    # Disarm before speech or any other delay. The twin independently zeros and disables
    # its output at the trajectory boundary, even if this operator process is delayed.
    node.commands(False)
    log('commands off, bridges disarmed')
    log(f'excitation over; peak commanded ' + ', '.join(f'{r} {peak[r]:.1f} cm/s' for r in ROBOTS))
    if max(peak.values()) == 0:
        raise Retry('no robot was commanded during the excitation')
    say('Warm-up motion done. Press the deadman once to end. You can relax; keep the sheet off the floor.')


def check_start_pose(restore=False):
    """Use the dedicated warm-up target; initial restoration uses the supervised deadman path."""
    command = ['bash', str(HERE / 'start_pose.sh'), '--pose', str(WARMUP_POSE)]
    if not restore:
        command.append('--check')
    if subprocess.run(command, stdin=subprocess.DEVNULL).returncode != 0:
        raise Abort('the arms are off the warm-up start pose; use warmup-start with the supervised deadman procedure')


def fit(node, start):
    say('The bank is fitting. It stops when the coefficients and error converge.', wait=False)
    begun, last = time.monotonic(), time.monotonic()

    def done():
        return any(b.get('message', '').startswith(('calibrated', 'calibration skipped', 'calibration stopped')) for b in node.bank)
    while not node.spin(5, done):
        if time.monotonic() - begun > 900:
            raise Abort('the bank did not finish in 15 minutes')
        if time.monotonic() - last > 30:
            progress = next((b['search'] for b in reversed(node.bank) if b.get('message') == 'calibration progress'), {})
            error = progress.get('cost_m')
            say(f'Still fitting. {progress.get("evaluations", 0)} candidate tests.'
                + (f' Best error {error*1000:.1f} millimetres.' if error is not None else ''), wait=False)
            last = time.monotonic()
    skipped = next((b for b in node.bank if b.get('message', '').startswith('calibration skipped')), None)
    if skipped:
        say('The bank skipped the fit: ' + skipped['message'])
        return None
    result = next(b['result'] for b in node.bank if b.get('message') in ('calibrated', 'calibration stopped'))
    if result.get('release_expected'):
        node.spin(3, lambda: bool(node.release) and node.release.get('source') == 'warmup')
    edge = warmup.at_limit((result['bend'], result['shear']), start, float(result.get('reach', warmup.REACH)))
    gain = 100 * (1 - result['cost_m'] / result['start_cost_m']) if result['start_cost_m'] else 0.
    released = bool(node.release) and node.release.get('source') == 'warmup'
    log(f'search stopped: {result.get("stop_reason", "legacy result")}; converged {result.get("converged", False)}; '
        f'{result.get("rounds_completed", "?")} rounds, {result["evaluations"]} evaluations')
    log(f'fit: bend x{result["bend"]:.3f}, shear x{result["shear"]:.3f} from start x{start[0]:g}, x{start[1]:g}; residual '
        f'{result["start_cost_m"] * 1e3:.1f} -> {result["cost_m"] * 1e3:.1f} mm; edge {edge}; released {released}')
    say(('Fit converged. ' if result.get('converged') else f'Search stopped without convergence: {result.get("stop_reason")}. ')
        + f'Bend from {words(start[0])} to {words(result["bend"])}. Shear from {words(start[1])} to '
        f'{words(result["shear"])}. Residual {gain:.0f} percent lower.'
        + (f' Warning: {" and ".join(edge)} at the search edge.' if edge else '')
        + ('' if released else ' Not released.'))
    return dict(result, edge=edge, released=released)


def reusable(node, start):
    """The running twin's run folder if it is a fresh warmup-twin at this start (bank on, no warm-up yet)."""
    if not twin_running():
        return None
    node.twin = {}
    node.spin(5, lambda: bool(node.twin))
    try:
        material = json.loads(node.twin.get('material') or '{}')
        last = Path(os.environ.get('XDG_RUNTIME_DIR', '/tmp'), 'fabric_lab', 'last_run')
        run = Path(last.read_text().strip())
    except (ValueError, OSError):
        return None
    fresh = (node.twin.get('warmup_only') == 'True' and 'warmup_start_problem' in node.twin
             and node.twin.get('warmup_protocol_version') == '3'   # 3: the plan with labels and person cues (2026-11-21)
             and node.twin.get('cbf_enabled') == 'False' and node.twin.get('stretch_guard_enabled') == 'False'
             and node.twin.get('command_mode') == 'warmup_direct'
             and material.get('version') == 0 and node.twin.get('warmup', 'idle') == 'idle'
             and abs(material.get('bend', 0.) - start[0]) < 1e-6 and abs(material.get('shear', 0.) - start[1]) < 1e-6
             and 'fabric_parameter_bank' in node.get_node_names() and (run / 'material_start.txt').exists()
             and run.name.startswith('warmup_'))
    return run if fresh else None


def one_run(node, material, index, total, attempt=1):
    node.commands(False)
    check_start_pose(restore=True)
    start = tuple(float(x) for x in material.split())
    run = reusable(node, start) if attempt == 1 else None
    if run:
        log(f'run {index}/{total}: reusing the running twin at the requested start -> {run}')
    else:
        stop_twin()
        day = datetime.date.today().isoformat()
        run = REPO / 'experiments/hardware/sessions' / f'{day}_fabric_run' / f'warmup_{start[0]:g}_{start[1]:g}_{time.strftime("%H%M%S")}'
        log(f'run {index}/{total}: MATERIAL="{material}" -> {run}')
        subprocess.Popen(['bash', RUN_SH, 'warmup-twin', str(run)],
                         env=dict(os.environ, MATERIAL=material, **({'NOMINAL': '1'} if material.split() == ['1', '1'] else {})),
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
        node.twin = {}
    node.seen, node.bank, node.window, node.release = {}, [], None, None
    say(f'Warm-up run {index} of {total}: bend {words(start[0])}, shear {words(start[1])}. '
        f'You have {int(REACH_SEC)} seconds to take the fabric.')
    t_end = time.monotonic() + REACH_SEC
    while time.monotonic() < t_end - 5:
        node.spin(.2)
    say('Five seconds.', wait=False)
    node.spin(max(0., t_end - time.monotonic()))
    say('Hold both free corners with the gloves, tags toward the overhead camera, the sheet clearly slack: '
        'stand about ten centimetres closer to the robots than where it would be tight. '
        'Keep still. I will tell you each step.')
    try:
        wait_ready(node, run, start)
        # The operator may have spent minutes waiting for visibility. Recheck at the
        # last point before arming, including when reusing an existing twin.
        check_start_pose()
        log(f'twin "{node.twin.get("message")}", material {node.twin.get("material")}')
        enable(node)
        excite(node)
        check_start_pose()
        result = fit(node, start)
    except Retry as why:
        log(f'retry: {why}')
        finish_run(node, run, f'incomplete, repeated: {why}')
        if attempt >= 3:
            raise Abort(f'run {index} incomplete three times ({why})')
        say('Restarting this run. Keep the sheet; the twin restarts in about a minute.')
        return one_run(node, material, index, total, attempt + 1)
    except BaseException as why:
        finish_run(node, run, f'aborted: {why!r}')
        raise
    finish_run(node, run, 'complete')
    check_start_pose()
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('materials', nargs='*', default=['10 0.1', '10 10'],
                    help='starting "bend shear" scales, one run each (default "10 0.1" "10 10")')
    ap.add_argument('--nominal', action='store_true',
                    help='one warm-up at the twin\'s own material ("1 1"): a recording for the still-hold grip/stretch '
                         'fit (fabric_run.sh fit-apply), not a test of the online bank (2026-11-21)')
    a = ap.parse_args()
    if a.nominal:
        a.materials = ['1 1']
    for m in a.materials:
        parts = m.split()
        if len(parts) != 2 or not all(1 / 16 <= float(x) <= 16 for x in parts) or (parts == ['1', '1'] and not a.nominal):
            sys.exit(f'each material must be two distorted scales in 1/16-16, e.g. "10 0.1"; not "{m}"')
    say('Warm-up script started. Go to the sheet; I will call you.', wait=False)
    rclpy.init()
    node = Cell()
    results = []
    try:
        node.commands(False)
        ensure_perception(node)
        for i, m in enumerate(a.materials, 1):
            results.append((m, one_run(node, m, i, len(a.materials))))
            if i < len(a.materials):
                say(f'Run {i} saved. Next run in a moment; you can rest until I call you.')
        fits = [(m, r) for m, r in results if r]
        if len(fits) >= 2:
            shears = [r['shear'] for _, r in fits]
            spread = max(shears) / min(shears)
            valid = all(r.get('converged') and r['released'] for _, r in fits)
            agree = spread <= 1.3 and not any(r['edge'] for _, r in fits) and valid
            say(f'All runs done. Shear ' + ', '.join(words(s) for s in shears)
                + ('. Both fits converged and agree.' if agree else
                   '. A fit did not converge or was not released.' if not valid else '. They do not agree.'))
            log(f'shear fits {shears} (ratio {spread:.2f}); bend fits {[r["bend"] for _, r in fits]}')
        else:
            say('All runs done.')
    except KeyboardInterrupt:
        say('Stopped. Commands are off.')
    except Abort as why:
        say(f'Stopped: {why}. Commands are off.')
        log(f'ABORT: {why}')
    finally:
        node.commands(False)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
