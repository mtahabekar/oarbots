#!/usr/bin/env python3
"""Warm-up calibration on the real cell: start it only when the robots can move, then report the fit.

    fabric_run.sh warmup [--shadow] [--nominal] [--wait 630]   (this script: scripts/hw/warmup.py start ...)
    fabric_run.sh enable                             (uses: scripts/hw/warmup.py enabled)

`start` refuses unless the twin reports commands ENABLED with no fault, both bridges report "Following the
fabric twin" and the tablet deadman is held (it waits up to 30 s for the press). It then publishes
/fabric/twin/warmup, prints the commanded speeds every 2 s through the excitation (holds from twin.yaml
`warmup`; the twin reports the duration), and waits for convergence or a reported search limit. The wait is a timeout,
not a minimum duration; commands are disabled while fitting.

On 2026-11-19 every warm-up ran with commands off: `enable` had been refused by the twin ("a grip is not
measured now") and the excitation was started anyway, so the robots never moved and the bank fitted an
unexcited sheet (shadow run: bend x0.149, shear x6.73 from a 0.7 % cost change, released). `--shadow`
still allows that as a rehearsal of the timing; its fit is not a calibration.

The twin and the bank must start from deliberately WRONG coefficients (`MATERIAL="10 0.1"
fabric_run.sh warmup-twin`): from the nominal ones, a fit that keeps them cannot be told from one that learned
nothing. `start` refuses a nominal start unless --nominal. Use one-decade errors so the change is
observable: MATERIAL="10 0.1", then "10 10". The test passes when both fits bring bend well down from x10
and end at the same shear, inside the search range. The bank's search reaches x19.9 from its start under
warmup-twin (span 8, 4 rounds; x6.73 with the default span 4, 3 rounds); a fit at that edge did not find
a minimum (the 2026-11-19 shadow fit, x0.149 / x6.73 from x1, was at the edge on both axes).

`enabled` waits up to --timeout s for the twin to report commands enabled; exit 0 if it does, else prints the
twin's reason and exits 1.
"""
import argparse
import json
import math
import subprocess
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String

ROBOTS = ('blue', 'silver')


class Warmup(Node):
    def __init__(self):
        super().__init__('warmup_operator')
        self.twin, self.bridges, self.deadman, self.cmd = {}, {}, None, {r: 0. for r in ROBOTS}
        self.bank, self.release, self.window = [], None, None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DiagnosticArray, '/fabric/twin/status', self.on_twin, 10)
        for r in ROBOTS:
            self.create_subscription(DiagnosticArray, f'/oarbot_{r}/diagnostics', lambda m, r=r: self.on_bridge(r, m), 10)
            self.create_subscription(TwistStamped, f'/oarbot_{r}/fabric_cbf/cmd_world', lambda m, r=r: self.on_cmd(r, m), 10)
        self.create_subscription(Bool, '/tablet_gui/deadman', lambda m: setattr(self, 'deadman', m.data), 10)
        self.create_subscription(String, '/fabric/bank/status', lambda m: self.bank.append(json.loads(m.data)), 10)
        self.create_subscription(String, '/fabric/material_parameters', self.on_release, latched)
        self.create_subscription(String, '/fabric/twin/warmup_window', lambda m: setattr(self, 'window', json.loads(m.data)), 5)
        self.start_pub = self.create_publisher(String, '/fabric/twin/warmup', 1)
        self.enable_pub = self.create_publisher(Bool, '/fabric/twin/enable_commands', 5)
        self.arm_pub = {r: self.create_publisher(Bool, f'/oarbot_{r}/fabric_cbf/arm', 5) for r in ROBOTS}

    def on_twin(self, msg):
        for s in msg.status:
            values = {k.key: k.value for k in s.values}
            if 'commands_enabled' in values:
                self.twin = dict(values, message=s.message)

    def on_bridge(self, robot, msg):
        for s in msg.status:
            if s.name.endswith('fabric_cbf_bridge'):
                self.bridges[robot] = s.message

    def on_cmd(self, robot, msg):
        v = msg.twist.linear
        self.cmd[robot] = 100*math.sqrt(v.x*v.x+v.y*v.y+v.z*v.z)

    def on_release(self, msg):
        self.release = json.loads(msg.data)  # the first one is the latched release from before this warm-up

    def spin(self, seconds, until=lambda: False):
        end = time.time()+seconds
        while time.time() < end and not until():
            rclpy.spin_once(self, timeout_sec=.05)
        return until()

    def enabled(self):
        return self.twin.get('commands_enabled') == 'True'

    def armed(self, robot):
        return self.bridges.get(robot) == 'Following the fabric twin'

    def stop_commands(self):
        for _ in range(3):
            self.enable_pub.publish(Bool(data=False))
            for publisher in self.arm_pub.values():
                publisher.publish(Bool(data=False))
            self.spin(.1)


def say(text):
    print(f'{time.strftime("%H:%M:%S")} {text}', flush=True)


def speak(text):
    """Print and speak without waiting (the person at the sheet cannot see the screen)."""
    say(f'CUE: {text}')
    try:
        subprocess.Popen(['spd-say', '-r', '-10', text.replace(' cm', ' centimetres')],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        pass


def cmd_enabled(node, timeout):
    if node.spin(timeout, node.enabled):
        say('twin: robot commands ENABLED')
        return 0
    if not node.twin:
        say('twin: no /fabric/twin/status (is `fabric_run.sh twin` running?)')
    else:
        say(f'twin REFUSED or not yet enabled: commands {node.twin.get("commands_enabled")}, '
            f'"{node.twin.get("warmup_start_problem") or node.twin.get("message")}". A hand must be seen within 0.5 s of the enable: hold both gloves '
            'still in the overhead view (`fabric_run.sh status`) and run enable again.')
    return 1


REACH = 2**2.75   # default Bank.calibrate(span=4, rounds=3); the bank reports its own `reach` in the result
BOUNDS = (1/16, 16.)


def at_limit(fit, start, reach=REACH):
    """Axes where the fit stopped at the search range (start x/÷ reach, or the bounds)."""
    return [name for name, f, s0 in zip(('bend', 'shear'), fit, start)
            if abs(math.log(f/s0)) >= math.log(reach)-1e-3 or f <= BOUNDS[0]*1.001 or f >= BOUNDS[1]*.999]


def cmd_start(node, shadow, wait, nominal=False):
    try:
        if shadow:
            node.stop_commands()
        return run_warmup(node, shadow, wait, nominal)
    finally:
        node.stop_commands()


def run_warmup(node, shadow, wait, nominal=False):
    node.spin(15, lambda: bool(node.twin) and len(node.bridges) == len(ROBOTS))
    if not node.twin:
        say('NO GO: no /fabric/twin/status. Start the twin first (`MATERIAL="10 0.1" fabric_run.sh warmup-twin`).')
        return 2
    if 'fabric_parameter_bank' not in node.get_node_names():
        say('NO GO: the parameter bank is not running, so nothing would be fitted. '
            'Stop the twin and start it with `MATERIAL="10 0.1" fabric_run.sh warmup-twin`.')
        return 2
    try:
        material = json.loads(node.twin.get('material') or '{}')
    except ValueError:
        material = {}
    start = (float(material.get('bend', 1.)), float(material.get('shear', 1.)))
    if int(material.get('version', 0)) > 0:
        say(f'note: the twin runs release {material["version"]} (an earlier fit in this session); the warm-up starts from it, '
            'not from the launch start. For the distorted-start test restart the twin.')
    elif start == (1., 1.) and not nominal:
        say('NO GO: the twin starts at the NOMINAL coefficients, so a fit that keeps them proves nothing. '
            'Restart it distorted: `fabric_run.sh stop-twin`, then `MATERIAL="10 0.1" fabric_run.sh warmup-twin` '
            '(next run "10 10"). --nominal runs from nominal on purpose.')
        return 2
    say(f'starting coefficients: bend x{start[0]:g}, shear x{start[1]:g} of twin.yaml (deliberately wrong)')
    if node.twin.get('warmup', 'idle') != 'idle' and 'started' in node.twin.get('warmup', ''):
        say(f'note: this twin already ran a warm-up ({node.twin["warmup"]}); a new start replaces it.')
    if node.twin.get('fault'):
        say(f'NO GO: twin fault "{node.twin["fault"]}". The person keeps both gloves in the overhead view; '
            'check `fabric_run.sh status` (stale grip a few % at most).')
        return 2
    if not shadow:
        if node.twin.get('warmup_only') != 'True':
            say('NO GO: the twin is not in warm-up-only mode. Restart with the updated `warmup-twin` command.')
            return 2
        problem = node.twin.get('warmup_start_problem', 'restart the twin with the updated warm-up geometry check')
        if problem:
            say('NO GO: ' + problem)
            return 2
        if (node.twin.get('cbf_enabled') != 'False' or node.twin.get('stretch_guard_enabled') != 'False'
                or node.twin.get('command_mode') != 'warmup_direct'):
            say('NO GO: restart with the updated warmup-twin profile (CBF off, stretch guard off).')
            return 2
        if node.twin.get('warmup_protocol_version') != '3':
            say('NO GO: this twin predates the 2026-11-21 warm-up (no plan or person cues). '
                '`fabric_run.sh build-laptop`, then restart it with warmup-twin.')
            return 2
        problems = []
        if not node.enabled():
            problems.append('twin commands are OFF (run `fabric_run.sh enable`; it now reports when the twin refuses)')
        problems += [f'{r} bridge: "{node.bridges.get(r, "no diagnostics")}"' for r in ROBOTS if not node.armed(r)]
        if problems:
            say('NO GO: '+'; '.join(problems)+'. The robots would not move and the fit would be meaningless.')
            return 2
        if not node.deadman:
            say('Waiting up to 30 s: tablet operator, PRESS AND HOLD the deadman for the whole warm-up.')
            if not node.spin(30, lambda: bool(node.deadman)):
                say('NO GO: deadman not held.')
                return 2
    else:
        say('SHADOW warm-up: the robots will not move; the bank result is NOT a calibration (timing rehearsal only).')
    node.spin(.5)
    before = node.release
    node.window, node.bank = None, []
    node.start_pub.publish(String(data='operator'))
    limit = float(node.twin.get('warmup_duration_sec', 21.))+20.
    try:   # (move starts s, hold starts s, label) per hold, from the twin
        cues = [(float(move), label) for move, _, label in json.loads(node.twin.get('warmup_plan') or '[]')]
    except (ValueError, TypeError):
        cues = []
    say(f'warm-up started ({node.twin.get("warmup_profile", "insets 2 and 4 cm")}): the person holds both hands STILL '
        'except when a hand move is called (spoken); the robots move their grips through the holds and return.')
    started, released_deadman, peak = time.time(), None, {r: 0. for r in ROBOTS}
    spoken, printed = set(), 0.
    while node.window is None and time.time()-started < limit:
        node.spin(.25)
        t = time.time()-started
        for k, (at, label) in enumerate(cues):
            if t >= at and k not in spoken:
                spoken.add(k)
                speak(label)
        for r in ROBOTS:
            peak[r] = max(peak[r], node.cmd[r])
        if not shadow and node.deadman is False and released_deadman is None:
            released_deadman = time.time()-started
        if t-printed >= 2.:
            printed = t
            say(f't+{t:4.0f} s  twin "{node.twin.get("message", "")}" | commands '
                + ', '.join(f'{r} {node.cmd[r]:.1f} cm/s' for r in ROBOTS)+f' | deadman {node.deadman}')
    if node.window is None:
        say(f'NO RESULT: the twin did not announce the end of the warm-up within {limit:.0f} s.')
        return 1
    if not shadow:
        node.stop_commands()
    say(f'excitation over: {len(node.window["holds"])} holds; peak commanded '+', '.join(f'{r} {peak[r]:.1f} cm/s' for r in ROBOTS))
    if not shadow and max(peak.values()) == 0:
        say('WARNING: no robot was commanded during the excitation; the fit below has no information.')
    if released_deadman is not None:
        say(f'WARNING: the deadman was released at t+{released_deadman:.0f} s; later holds were not reached. Repeat the warm-up.')
    say(f'waiting for convergence; timeout {wait:.0f} s (not a minimum duration) ...')

    def finished():
        return any(b.get('message', '').startswith(('calibrated', 'calibration skipped', 'calibration stopped')) for b in node.bank)
    node.spin(wait, finished)
    for b in node.bank:
        if b.get('message', '').startswith('calibration skipped'):
            say(f'bank: {b["message"]}')
            return 1
    result = next((b['result'] for b in node.bank if b.get('message') in ('calibrated', 'calibration stopped')), None)
    if result is None:
        say('NO RESULT: the bank did not finish in time. It may still be fitting: `ros2 topic echo /fabric/bank/status`.')
        return 1
    gain = 100*(1-result['cost_m']/result['start_cost_m']) if result['start_cost_m'] else 0.
    say(f'search stopped: {result.get("stop_reason", "legacy result")}; converged {result.get("converged", False)}')
    say(f'bank: bend x{result["bend"]:.3f}, shear x{result["shear"]:.3f}; depth residual {result["start_cost_m"]*1e3:.1f} -> '
        f'{result["cost_m"]*1e3:.1f} mm ({gain:.1f} % lower); candidates differ by {result["sensitivity_m"]*1e3:.1f} mm; '
        f'{result["cases"]} holds; {result["seconds"]:.0f} s; informative {result["informative"]}')
    fit = (result['bend'], result['shear'])
    say(f'moved from the start: bend x{fit[0]/start[0]:.2f}, shear x{fit[1]/start[1]:.2f} '
        f'(the search resolves about +/-10 %; staying near the distorted start means nothing was learned)')
    reach = float(result.get('reach', REACH))
    edge = at_limit(fit, start, reach)
    if edge:
        say(f'WARNING: {" and ".join(edge)} stopped at the edge of the search range (x{reach:.3g}): no minimum was found; '
            'the fit is not a calibration.')
    if result.get('release_expected'):
        node.spin(2, lambda: node.release is not before)
    if node.release is not None and node.release is not before and node.release.get('source') == 'warmup':
        say(f'released to the twin as material version {node.release["version"]}.')
    else:
        say('not released: the twin keeps its coefficients.')
    say('Next: `fabric_run.sh review <run>` after stopping the twin (shows the grip insets actually reached).')
    return 0 if result.get('converged') else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest='cmd', required=True)
    s = sub.add_parser('start')
    s.add_argument('--shadow', action='store_true', help='allow commands off (timing rehearsal; no calibration)')
    s.add_argument('--wait', type=float, default=630., help='maximum seconds to wait for the bank result; returns earlier on completion')
    s.add_argument('--nominal', action='store_true', help='allow a start at the nominal coefficients')
    e = sub.add_parser('enabled')
    e.add_argument('--timeout', type=float, default=5.)
    a = ap.parse_args()
    rclpy.init()
    node = Warmup()
    try:
        code = cmd_start(node, a.shadow, a.wait, a.nominal) if a.cmd == 'start' else cmd_enabled(node, a.timeout)
    except KeyboardInterrupt:
        say('interrupted: warm-up commands disabled and bridges disarmed. Release the deadman.')
        code = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
