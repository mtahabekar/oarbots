#!/usr/bin/env python3
"""Record one robot's wrist force for the on-site capacity measurements (test guide section 3.4).

One trial per run: robot grip slip, arm margin with known masses (which also checks the sensor's
scale against known loads, gate H5) or the person's comfortable pull. The arm stays still during a
trial, so the change of the raw force from the baseline taken at the start is the added load. No
gravity compensation or calibration enters the result.

Run on the laptop in the session environment (or on the helper), with the session stack running:
    source scripts/hw/laptop_env.sh
    python3 scripts/hw/ft_record.py --robot silver --label slip_edge_1

Keep the sheet still for the first second (the baseline), then load it; Ctrl-C ends the trial.
Prints the change of force while recording and its peak at the end. Writes <label>_<time>.csv
(wrench), <label>_<time>_joints.csv (joint positions and efforts) and <label>_<time>.json (summary).
"""
import argparse
import csv
import json
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

FILTER_SEC = 0.1   # mean window for the filtered peak: the raw signal carries +/-4 N vibration (2026-11-16)
STEADY_SEC = 1.0   # the last second before Ctrl-C: the steady load of a hanging mass


def stamp(msg, now):
    s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
    return s if s > 0 else now


def mean(rows, cols):
    return [sum(r[c] for r in rows) / len(rows) for c in cols] if rows else [math.nan] * len(cols)


class Recorder(Node):
    def __init__(self, args):
        super().__init__('ft_record')
        self.args = args
        self.wrench, self.joints = [], []   # (t, recv, fx, fy, fz, tx, ty, tz); (t, names, pos, eff)
        self.baseline = None
        self.peak = (0., None)               # filtered |dF|, time
        self.start = time.time()
        ns = f'/oarbot_{args.robot}'
        self.create_subscription(WrenchStamped, args.topic or ns + '/force_torque_raw', self.on_wrench,
                                 qos_profile_sensor_data)
        self.create_subscription(JointState, ns + '/joint_states', self.on_joints, 10)
        self.create_timer(0.2, self.report)
        print(f'Waiting for {args.topic or ns + "/force_torque_raw"} ... keep the sheet still for the baseline')

    def on_wrench(self, msg):
        now = time.time()
        f, t = msg.wrench.force, msg.wrench.torque
        self.wrench.append((stamp(msg, now), now, f.x, f.y, f.z, t.x, t.y, t.z))
        if self.baseline is None and self.wrench[-1][0] - self.wrench[0][0] >= self.args.baseline_sec:
            self.baseline = mean(self.wrench, range(2, 8))
            print(f'Baseline over {len(self.wrench)} samples: F = ' + ', '.join(f'{v:.2f}' for v in self.baseline[:3])
                  + ' N.  Load it now; Ctrl-C to end.')

    def on_joints(self, msg):
        now = time.time()
        self.joints.append((stamp(msg, now), list(msg.name), list(msg.position), list(msg.effort)))

    def delta(self, rows):
        m = mean(rows, range(2, 5))
        return [m[i] - self.baseline[i] for i in range(3)]

    def report(self):
        if self.baseline is None:
            if self.wrench == [] and time.time() - self.start > 5:
                print('No wrist force after 5 s: is the session stack running and this laptop on its domain?')
                self.start = math.inf
            return
        t = self.wrench[-1][0]
        recent = [r for r in self.wrench if t - r[0] <= FILTER_SEC]
        d = math.hypot(*self.delta(recent))
        if d > self.peak[0]:
            self.peak = (d, t)
        print(f'  |dF| {d:5.2f} N   peak {self.peak[0]:5.2f} N', flush=True)

    def finish(self):
        if self.baseline is None:
            print('No baseline recorded; nothing written.')
            return
        rows, t0 = self.wrench, self.wrench[0][0]
        base_n = sum(1 for r in rows if r[0] - t0 < self.args.baseline_sec)
        loaded = rows[base_n:]
        filtered = []                      # (t, |dF| of the FILTER_SEC mean ending at t, dF)
        j = 0
        for i, r in enumerate(loaded):
            while loaded[j][0] < r[0] - FILTER_SEC:
                j += 1
            d = self.delta(loaded[j:i + 1])
            filtered.append((r[0], math.hypot(*d), d))
        raw = [(r[0], math.hypot(*(r[2 + k] - self.baseline[k] for k in range(3)))) for r in loaded]
        steady = [r for r in rows if rows[-1][0] - r[0] <= STEADY_SEC]
        gaps = [b[0] - a[0] for a, b in zip(rows, rows[1:])]
        duration = rows[-1][0] - t0
        peak = max(filtered, key=lambda x: x[1]) if filtered else (math.nan, math.nan, [math.nan] * 3)
        summary = {
            'robot': self.args.robot, 'label': self.args.label, 'note': self.args.note,
            'started': datetime.fromtimestamp(t0).isoformat(timespec='seconds'),
            'samples': len(rows), 'duration_s': round(duration, 2),
            'rate_hz': round((len(rows) - 1) / duration, 1) if duration > 0 else None,
            'max_gap_s': round(max(gaps), 3) if gaps else None,
            'baseline_force_n': [round(v, 3) for v in self.baseline[:3]],
            'baseline_torque_nm': [round(v, 4) for v in self.baseline[3:]],
            'peak_filtered_n': round(peak[1], 2), 'peak_filtered_at_s': round(peak[0] - t0, 2),
            'peak_filtered_vector_n': [round(v, 2) for v in peak[2]],
            'peak_raw_n': round(max((x[1] for x in raw), default=math.nan), 2),
            'steady_last_s_n': round(math.hypot(*self.delta(steady)), 2),
            'steady_last_s_vector_n': [round(v, 2) for v in self.delta(steady)],
            'filter_s': FILTER_SEC, 'frame': 'sensor frame of the raw topic',
        }
        if self.joints:
            names = self.joints[0][1]
            base = [r for r in self.joints if r[0] - t0 < self.args.baseline_sec] or self.joints[:1]
            rows_j = [r for r in self.joints if r[1] == names]
            mean_of = lambda rs, k: [sum(r[k][i] for r in rs) / len(rs) for i in range(len(names))]  # noqa: E731
            if all(len(r[2]) == len(names) for r in rows_j):
                p0 = mean_of(base, 2)
                summary['joint_max_drift_rad'] = dict(zip(names, (round(max(abs(r[2][i] - p0[i]) for r in rows_j), 4)
                                                                  for i in range(len(names)))))
            if all(len(r[3]) == len(names) for r in rows_j) and names:
                e0, e1 = mean_of(base, 3), mean_of([r for r in rows_j if rows_j[-1][0] - r[0] <= STEADY_SEC], 3)
                summary['joint_effort_baseline'] = dict(zip(names, (round(v, 3) for v in e0)))
                summary['joint_effort_steady_change'] = dict(zip(names, (round(b - a, 3) for a, b in zip(e0, e1))))
        out = Path(self.args.out)
        out.mkdir(parents=True, exist_ok=True)
        stem = out / f'{self.args.label}_{datetime.fromtimestamp(t0).strftime("%H%M%S")}'
        with open(f'{stem}.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['stamp', 'received', 'fx', 'fy', 'fz', 'tx', 'ty', 'tz'])
            w.writerows(rows)
        with open(f'{stem}_joints.csv', 'w', newline='') as fh:
            w = csv.writer(fh)
            names = self.joints[0][1] if self.joints else []
            w.writerow(['stamp'] + [f'{n}_pos' for n in names] + [f'{n}_eff' for n in names])
            for r in self.joints:
                if r[1] == names:
                    w.writerow([r[0]] + r[2] + r[3])
        Path(f'{stem}.json').write_text(json.dumps(summary, indent=2) + '\n')
        print(f'\n{self.args.label}: peak |dF| {summary["peak_filtered_n"]} N ({FILTER_SEC} s mean; raw '
              f'{summary["peak_raw_n"]} N) at {summary["peak_filtered_at_s"]} s; last {STEADY_SEC:.0f} s '
              f'{summary["steady_last_s_n"]} N; {summary["rate_hz"]} Hz, max gap {summary["max_gap_s"]} s')
        print(f'Wrote {stem}.csv, _joints.csv, .json')


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--robot', required=True, choices=['blue', 'silver'])
    p.add_argument('--label', required=True, help='trial name, e.g. slip_edge_1, mass_400g, person_1')
    p.add_argument('--note', default='', help='free text stored in the summary')
    p.add_argument('--topic', default='', help='wrench topic (default /oarbot_<robot>/force_torque_raw)')
    p.add_argument('--baseline-sec', type=float, default=1.0)
    day = datetime.now().strftime('%Y-%m-%d')
    p.add_argument('--out', default=str(Path(__file__).resolve().parents[2] / 'experiments' / 'hardware' / 'sessions'
                                        / f'{day}_capacity'))
    args = p.parse_args()
    rclpy.init()
    node = Recorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
