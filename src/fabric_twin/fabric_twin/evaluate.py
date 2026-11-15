"""Score the recorded twin against the simulated plant's private truth.

    python -m fabric_twin.evaluate PLANT_TRUTH.npz TWIN.npz OUT.json [--start S] [--end S]

Truth is the plant's cloth interpolated in time to each twin state and bilinearly in material
coordinates to the twin's particles. Reports the twin's surface error, its error at the lowest
part of the sheet (what obstacle clearance depends on), the plant's true clearance from the
obstacle box, and the CBF and feedback figures the twin logged.
"""
import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np

from fabric_sim_cell.core import grid_from_truth, truth_at


def box_clearance(points, center, half):
    d = np.abs(points-center)-half
    return np.linalg.norm(np.maximum(d, 0.), axis=-1)+np.minimum(d.max(axis=-1), 0.)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('truth'); parser.add_argument('twin'); parser.add_argument('output')
    parser.add_argument('--start', type=float, default=0.); parser.add_argument('--end', type=float, default=1e9)
    parser.add_argument('--obstacle', type=float, nargs=6, default=[0., 0., .16, .22, .22, .16],
                        metavar=('CX', 'CY', 'CZ', 'HX', 'HY', 'HZ'))
    parser.add_argument('--particle-radius', type=float, default=.008)
    options = parser.parse_args(args)
    with np.load(options.truth, allow_pickle=False) as t:
        truth = {k: t[k] for k in ('times_ns', 'positions', 'material_uv', 'grid_shape')}
    with np.load(options.twin, allow_pickle=False) as t:
        twin = {k: t[k] for k in t.files}
    grid = grid_from_truth(truth['positions'].astype(float), truth['material_uv'], truth['grid_shape'])
    seconds = twin['times_ns']/1e9
    keep = (seconds >= options.start) & (seconds < options.end)
    aligned = truth_at(twin['times_ns'][keep], truth['times_ns'], grid, twin['uv'].astype(float))
    finite = np.isfinite(aligned).all(axis=(1, 2))
    positions, aligned = twin['positions'][keep][finite].astype(float), aligned[finite]
    error = np.linalg.norm(positions-aligned, axis=2)
    low = np.argsort(aligned[:, :, 2], axis=1)[:, :max(1, aligned.shape[1]//20)]  # lowest 5 % of the true sheet
    low_error = np.take_along_axis(positions[:, :, 2]-aligned[:, :, 2], low, axis=1)
    center, half = np.array(options.obstacle[:3]), np.array(options.obstacle[3:])
    truth_seconds = truth['times_ns']/1e9
    window = (truth_seconds >= options.start) & (truth_seconds < options.end)
    true_clearance = box_clearance(truth['positions'][window].astype(float), center, half).min(axis=1)-options.particle_radius
    report = dict(states=int(len(error)), window_s=[options.start, min(options.end, float(seconds.max()))],
                  surface_error_m=dict(p50=float(np.percentile(error, 50)), p95=float(np.percentile(error, 95)), max=float(error.max())),
                  per_state_p95_m=dict(median=float(np.median(np.percentile(error, 95, axis=1))),
                                       worst=float(np.percentile(error, 95, axis=1).max())),
                  lowest_5_percent_height_error_m=dict(mean=float(low_error.mean()), p95_abs=float(np.percentile(np.abs(low_error), 95))),
                  plant_true_box_clearance_m=dict(min=float(true_clearance.min()), p5=float(np.percentile(true_clearance, 5))),
                  twin_box_clearance_m=float((box_clearance(positions, center, half).min(axis=1)-options.particle_radius).min()),
                  state_lag_s=dict(p50=float(np.percentile((twin['received_ns'][keep]-twin['times_ns'][keep])/1e9, 50))))
    cbf = Path(options.twin).with_suffix('.cbf.jsonl')
    if cbf.exists():
        rows = [json.loads(line) for line in cbf.read_text().splitlines() if line.strip()]
        rows = [r for r in rows if options.start <= r.get('twin_time', -1) < options.end]
        if rows:
            report['cbf'] = dict(updates=len(rows), status=dict(Counter(r.get('status', '') for r in rows)),
                                 min_capsule_clearance_m=float(min(r.get('min_clearance', np.inf) for r in rows)),
                                 controller_ms_p95=float(np.percentile([r.get('controller_ms', 0.) for r in rows], 95)),
                                 max_tension_n=float(max(max(r.get('tension') or [0.]) for r in rows)),
                                 faults=dict(Counter(r.get('fault', '') for r in rows if r.get('fault'))),
                                 max_follower_speed_mps=float(max(np.linalg.norm(r.get('follower_command', [[0, 0, 0]]), axis=1).max() for r in rows)))
    status = Path(options.twin).with_suffix('.status.jsonl')
    if status.exists():
        rows = [json.loads(line) for line in status.read_text().splitlines() if line.strip()]
        values = [r['values'] for r in rows if 'values' in r and 'frame_ms_p95' in r['values']]
        if values:
            report['twin'] = dict(frame_ms_p95=float(values[-1]['frame_ms_p95']), frame_ms_p50=float(values[-1]['frame_ms_p50']),
                                  lag_sec=float(values[-1]['lag_sec']), skipped_frames=int(values[-1]['skipped_frames']),
                                  stale_grip_frames=int(values[-1]['stale_grip_frames']), replaced=int(values[-1]['replaced']),
                                  correction_force_n_p95=float(np.percentile([float(v.get('force_total_n', 0.)) for v in values], 95)),
                                  residual_force_n_last=float(values[-1].get('residual_total_n', 0.)),
                                  material=values[-1]['material'])
        report['material_releases'] = [r['material_release'] for r in rows if 'material_release' in r]
    Path(options.output).write_text(json.dumps(report, indent=2))
    e = report['surface_error_m']
    print(f"twin vs plant truth: {report['states']} states, p50 {e['p50']*1e3:.1f} mm, p95 {e['p95']*1e3:.1f} mm, max {e['max']*1e3:.1f} mm; "
          f"lowest 5% height error mean {report['lowest_5_percent_height_error_m']['mean']*1e3:+.1f} mm; "
          f"plant true clearance min {report['plant_true_box_clearance_m']['min']*100:.1f} cm")
    print('TWIN_EVALUATED', options.output)


if __name__ == '__main__':
    main()
