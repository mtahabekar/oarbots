#!/usr/bin/env python3
"""Are the Dingo bases where the cell thinks they are? Read-only, from the laptop (repo root).

The twin, the CBF and the identified grip points all place each robot through its static
world -> base_link transform in experiments/hardware/cell/calibration.yaml (bases parked 2026-11-17, Blue
re-parked 2026-11-18). A base that someone pushed since then moves the real grip away from where the twin
puts it, by the same amount everywhere, with no other symptom. This measures each base from its roof marker
with the overhead camera (scripts/hw/park_guide.py `measure`, run on the NUC on the session domain, where
the 1080p frames are local) and compares it with calibration.yaml.

    python3 scripts/hw/base_check.py [seconds]     (fabric_run.sh bases; cell_up.sh step 7)

Needs the NUC session running (the overhead camera on domain 17) and LAB_SSH (scripts/hw/lab_ssh.sh).
Saves the measurement and the annotated overhead view (1280 x 720 JPEG, fetched once; no image stream
crosses the network) under experiments/hardware/sessions/<date>_bases/. Exit status 0 OK, 1 WARN, 2 FAIL.

What is judged is the GRIP shift the base error causes at the grasp pose (translation plus yaw times the
0.65 m from base_link to the grip), because that is what the twin sees:
  OK    <= 10 mm   half of the 2 cm twin-error allowance in the CBF margin (README "Limits in use");
                   the corner measurement repeats to ~2 mm and 0.2 deg (calibration.yaml), ~4 mm at the grip
  WARN  10-20 mm   still inside that allowance, but it eats into it: re-park, or measure with corners
  FAIL  > 20 mm    more than the whole allowance: re-park (park_guide serve + park_drive, lab log 2026-11-18)
                   or, if the base is to stay, measure it for calibration.yaml (and recapture the obstacle
                   background, which is in world coordinates)
A marker partly hidden (Silver's arm can cover Blue's) is located by an edge fit instead of its corners:
accurate to ~6 mm and 0.8 deg (lab log 2026-11-18), i.e. up to ~15 mm at the grip, so a fit-only WARN is
not conclusive. If both bases are off by about the same vector, suspect the overhead camera, not the bases.
"""
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
CELL = REPO / 'experiments' / 'hardware' / 'cell'
WORLD = REPO / 'experiments' / 'hardware' / 'sessions' / '2026-11-17_world' / 'overhead_world.json'
NUC = 'nuc@192.168.1.90'
ROBOTS = ('oarbot_blue', 'oarbot_silver')
# Grip in world at the 77 cm grasp pose (sessions/2026-11-20_warmup_pose/validation.json, grip_world_m):
# the lever arm for a yaw error.
GRIP_WORLD = {'oarbot_blue': (-0.3926, -0.4138), 'oarbot_silver': (0.3773, -0.4231)}
# Edge fit minus corners on Blue's tilted marker plate, 3 frames at 12:44 on 2026-11-18 (x m, y m, yaw deg);
# calibration.yaml holds the corner-equivalent pose, so a fit is corrected by this before comparing.
FIT_BIAS = {'oarbot_blue': (0.0054, -0.0002, -1.12)}
OK_MM, FAIL_MM = 10.0, 20.0
MIN_SAMPLES = 5


def yaw_deg(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def calibrated():
    cal = yaml.safe_load((CELL / 'calibration.yaml').read_text())
    out = {}
    for t in cal['transforms']:
        if t['parent'] == cal.get('world_frame', 'world') and t['child'].endswith('/base_link'):
            out[t['child'].split('/')[0]] = (t['translation'][0], t['translation'][1], yaw_deg(t['rotation_xyzw']))
    return out


def grip_shift(robot, ref, meas):
    """World shift of the grip (mm) when the base is at `meas` instead of `ref` (both x, y, yaw deg)."""
    gx, gy = GRIP_WORLD[robot]
    lx, ly = gx - ref[0], gy - ref[1]                      # lever from the calibrated base to the grip
    d = math.radians(meas[2] - ref[2])
    c, s = math.cos(d), math.sin(d)
    nx = meas[0] + c * lx - s * ly                         # same lever, rotated and moved with the base
    ny = meas[1] + s * lx + c * ly
    return 1000 * math.hypot(nx - gx, ny - gy)


def run(cmd, **kw):
    return subprocess.run(cmd, capture_output=True, text=True, **kw)


def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
    ssh = os.environ.get('LAB_SSH')
    if not ssh:
        sys.exit('LAB_SSH not set: source scripts/hw/lab_ssh.sh; lab_ssh_setup (fabric_run.sh and cell_up.sh do this)')
    ref = calibrated()
    # Stage this checkout's park_guide.py and world definition on the NUC (its ~/fabric_ws may be older).
    stage = f'/dev/shm/base_check_{time.strftime("%Y%m%d_%H%M%S")}'
    tar = subprocess.Popen(['tar', '-cf', '-', '-C', str(REPO / 'scripts' / 'hw'), 'park_guide.py',
                            '-C', str(WORLD.parent), WORLD.name], stdout=subprocess.PIPE)
    st = subprocess.run([ssh, NUC, f"rm -f /dev/shm/park_measure.jpg; mkdir -p '{stage}' && tar -xf - -C '{stage}'"], stdin=tar.stdout,
                        capture_output=True, text=True, timeout=30)
    tar.wait()
    if st.returncode:
        print(f'  FAIL  could not stage park_guide on the NUC: {st.stderr.strip()[-200:]}')
        return 2
    r = run([str(REPO / 'scripts' / 'hw' / 'lab.sh'), 'session', 'nuc', 'exec', 'python3', f'{stage}/park_guide.py',
             'measure', f'{stage}/{WORLD.name}', str(seconds)], timeout=seconds + 90)
    found = {}
    for line in r.stdout.splitlines():
        if line.startswith('{'):
            d = json.loads(line)
            found[d['robot']] = d
    day = time.strftime('%Y-%m-%d')
    out_dir = REPO / 'experiments' / 'hardware' / 'sessions' / f'{day}_bases'
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = time.strftime('%H%M%S')
    jpg = subprocess.run([ssh, NUC, f"cat /dev/shm/park_measure.jpg; rm -rf '{stage}'"], capture_output=True, timeout=30)
    jpg_ok = jpg.returncode == 0 and jpg.stdout[:2] == b'\xff\xd8'
    if jpg_ok:
        (out_dir / f'bases_{tag}.jpg').write_bytes(jpg.stdout)
    worst, report, offsets = 0, {}, {}
    for robot in ROBOTS:
        name = robot.replace('oarbot_', '').capitalize()
        d = found.get(robot)
        if robot not in ref:
            print(f'  FAIL  {name}: no world -> base_link transform in calibration.yaml'); worst = 2; continue
        if d is None:
            msg = r.stderr.strip().splitlines()[-1:] if r.returncode else ''
            print(f'  FAIL  {name}: roof marker not seen in {seconds:.0f} s of overhead frames '
                  f'(NUC session camera running? something on the roof marker?) {msg}')
            worst = 2; continue
        methods = d.get('by_method', {})
        if methods.get('marker', {}).get('n', 0) >= MIN_SAMPLES:
            m, how = methods['marker'], 'corners'
            pose = (m['x'], m['y'], m['yaw_deg'])
        elif methods.get('fit', {}).get('n', 0) >= 1:
            m, how = methods['fit'], 'edge fit (marker partly hidden)'
            b = FIT_BIAS.get(robot, (0.0, 0.0, 0.0))
            pose = (m['x'] - b[0], m['y'] - b[1], m['yaw_deg'] - b[2])
            if robot in FIT_BIAS:
                how += ', tilted-plate bias removed'
        else:                                               # old park_guide without by_method
            m, how = d, '/'.join(d.get('how', []))
            pose = (d['x'], d['y'], d['yaw_deg'])
        c = ref[robot]
        dx, dy = 1000 * (pose[0] - c[0]), 1000 * (pose[1] - c[1])
        dyaw = (pose[2] - c[2] + 180) % 360 - 180
        shift = grip_shift(robot, c, pose)
        level = 0 if shift <= OK_MM else (1 if shift <= FAIL_MM else 2)
        if level == 1 and how.startswith('edge fit'):
            level_note = ' (edge fit only: within its accuracy; clear the marker and rerun to confirm)'
        else:
            level_note = ''
        worst = max(worst, level)
        offsets[robot] = (dx, dy)
        tagtxt = ('OK  ', 'WARN', 'FAIL')[level]
        col = ('32', '33', '31')[level]
        print(f'  \033[{col}m{tagtxt}\033[0m  {name} base: dx {dx:+.1f} mm, dy {dy:+.1f} mm, yaw {dyaw:+.2f} deg from '
              f'calibration.yaml -> grip moved {shift:.1f} mm; {how}, {m["n"]} frames, sd {m["sd_mm"][0]:.1f}/'
              f'{m["sd_mm"][1]:.1f} mm {m["sd_yaw_deg"]:.2f} deg{level_note}')
        report[robot] = {'measured': dict(zip(('x', 'y', 'yaw_deg'), pose)), 'calibration': dict(zip(('x', 'y', 'yaw_deg'), c)),
                         'dx_mm': round(dx, 1), 'dy_mm': round(dy, 1), 'dyaw_deg': round(dyaw, 2),
                         'grip_shift_mm': round(shift, 1), 'method': how, 'level': tagtxt.strip(), 'raw': d}
    if len(offsets) == 2 and worst:
        (bx, by), (sx, sy) = offsets['oarbot_blue'], offsets['oarbot_silver']
        if math.hypot(bx - sx, by - sy) < 0.3 * max(math.hypot(bx, by), math.hypot(sx, sy)):
            print('        both bases off by about the same vector: check the overhead camera mount before moving a base')
    if worst:
        print('        fix: re-park (Dingo motors enabled, deadman; park_guide.py serve + park_drive.py, lab log 2026-11-18,\n'
              '        "Re-parked") or measure the new pose into calibration.yaml and recapture the obstacle background')
    report.update({'time': time.strftime('%Y-%m-%dT%H:%M:%S'), 'seconds': seconds, 'ok_mm': OK_MM, 'fail_mm': FAIL_MM})
    (out_dir / f'bases_{tag}.json').write_text(json.dumps(report, indent=1))
    print(f'        saved {out_dir.relative_to(REPO)}/bases_{tag}.json' + (' and .jpg' if jpg_ok else ''))
    return worst


if __name__ == '__main__':
    sys.exit(main())
