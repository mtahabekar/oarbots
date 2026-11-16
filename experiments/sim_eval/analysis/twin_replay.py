"""Study S3: the twin replayed offline on one recorded dataset, one switch at a time.

    python twin_replay.py RUN_DIR OUT.npz --correction off|estimator|oracle --scales BEND SHEAR
                          [--grips measured|truth] [--feedback-scale K]

Every condition starts from the plant's true state at t0 and is driven by the same recorded grips
(and, when correcting, the same recorded estimator analyses), so differences between conditions
are caused by the switch alone. Scores current-state error every 4 frames and open-loop forecasts
(0.1, 0.33, 0.5, 1 s) started every second of the held-out validation motion.
"""
import argparse
import json
import time

import numpy as np

from simeval import HORIZONS, Run, replay, twin_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('run'); parser.add_argument('output')
    parser.add_argument('--correction', choices=('off', 'estimator', 'oracle'), default='off')
    parser.add_argument('--scales', type=float, nargs=2, default=(1., 1.))
    parser.add_argument('--grips', choices=('measured', 'truth'), default='measured')
    parser.add_argument('--feedback-scale', type=float, default=1.)
    parser.add_argument('--label', default='')
    parser.add_argument('--t0', type=float, default=12.)
    options = parser.parse_args()
    run = Run(options.run)
    spec, feedback = twin_spec(run.path/'twin.yaml')
    window = run.manifest['validation_window_s']
    started = time.perf_counter()
    result = replay(run, spec, feedback, tuple(options.scales), options.correction, options.grips, t0=options.t0,
                    forecast_every_s=1., forecast_window=window, feedback_scale=options.feedback_scale)
    rows = result['rows']
    keys = ('mean', 'p95', 'max', 'low_dz', 'low_dist')
    forecasts = result['forecasts']
    np.savez(options.output, t=np.array([r['t'] for r in rows]),
             **{f'state_{k}': np.array([r[k] for r in rows]) for k in keys},
             forecast_t=np.array([f['t'] for f in forecasts]),
             **{f'forecast_{h}_{k}': np.array([f[h][k] for f in forecasts]) for h in HORIZONS for k in ('mean', 'p95', 'low_dz')},
             meta=json.dumps(dict(run=run.path.name, manifest=run.manifest, label=options.label, correction=options.correction,
                                  scales=list(options.scales), grips=options.grips, feedback_scale=options.feedback_scale,
                                  info=result['info'], seconds=time.perf_counter()-started)))
    t = np.array([r['t'] for r in rows]); m = np.array([r['mean'] for r in rows])
    inside = (t >= window[0]) & (t <= window[1])
    print(f"{run.path.name} {options.label or options.correction} scales {options.scales}: validation mean error "
          f"{1e3*m[inside].mean():.1f} mm, forecast 1 s {1e3*np.mean([f[60]['mean'] for f in forecasts]):.1f} mm, "
          f"{time.perf_counter()-started:.0f} s", flush=True)


if __name__ == '__main__':
    main()
