"""Study S2c: online adaptation replayed offline on one dataset.

    python adapt_trace.py RUN_DIR OUT.json --inputs STATE GRIPS POINTS [--start BEND SHEAR] [--period 2]

Starting from the given coefficients (default nominal), applies the production bank's bounded
online update (`fabric_twin.bank.Bank.adapt`, as `bank_node` does every `adapt_period_sec`) once per
period, each time on the most recent 2 s case, through the warm-up and the held-out motion.
Records the coefficient trajectory and whether each update judged the data informative.
"""
import argparse
import json
import time

import numpy as np

from simeval import Run, bank_for, make_cases, twin_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('run'); parser.add_argument('output')
    parser.add_argument('--inputs', nargs=3, default=('twin', 'measured', 'depth'))
    parser.add_argument('--start', type=float, nargs=2, default=(1., 1.))
    parser.add_argument('--period', type=float, default=2.)
    parser.add_argument('--window', type=float, nargs=2, default=(17., 140.))
    options = parser.parse_args()
    run = Run(options.run)
    spec, _ = twin_spec(run.path/'twin.yaml')
    bank = bank_for(spec, run)
    scales, trace = tuple(options.start), []
    started = time.perf_counter()
    for t in np.arange(options.window[0], options.window[1], options.period):
        cases = make_cases(run, bank.cloth.uv, [int(t*1e9)], *options.inputs, seed=int(t*10))
        if not cases:
            trace.append(dict(t=float(t), bend=scales[0], shear=scales[1], skipped=True)); continue
        scales, report = bank.adapt(cases, scales)
        trace.append(dict(t=float(t), bend=scales[0], shear=scales[1], moved=report['moved'], informative=report['informative'],
                          sensitivity_mm=[1e3*s for s in report['sensitivity_m']], cost_mm=1e3*report['cost_m']))
    true = run.manifest['true_scales']
    out = dict(run=run.path.name, manifest=run.manifest, inputs=dict(zip(('state', 'grips', 'points'), options.inputs)),
               start=list(options.start), period_s=options.period, trace=trace, seconds=time.perf_counter()-started,
               final=dict(bend=scales[0], shear=scales[1], log_err_bend=float(np.log(scales[0]/true['bend'])),
                          log_err_shear=float(np.log(scales[1]/true['shear']))))
    json.dump(out, open(options.output, 'w'), indent=1)
    print(f"{run.path.name} adapt {'/'.join(options.inputs)}: final bend {scales[0]:.3f} shear {scales[1]:.3f} "
          f"(true {true['bend']:.3f}, {true['shear']:.3f}), {sum(bool(r.get('moved')) for r in trace)} moves of {len(trace)}, "
          f"{out['seconds']:.0f} s", flush=True)


if __name__ == '__main__':
    main()
