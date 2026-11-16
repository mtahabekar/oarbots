"""Study S2b: identification iterated through the twin (fixed-point calibration).

    python iterate_fit.py RUN_DIR OUT.json [--iterations 4] [--holds extended]

The single-pass fit starts each case from the corrected twin's state, whose fold pattern formed
under the twin's current (wrong) coefficients. Here each iteration replays the corrected twin
through the warm-up with the latest estimate, then refits from those states with live grips and
depth: the twin's history is regenerated under ever closer coefficients. With --relaxation r < 1
the update moves only a fraction r of the way (in log space) toward each new fit.
"""
import argparse
import json
import time

import numpy as np

from simeval import Run, bank_for, hold_ends, make_cases, replay, twin_spec

parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
parser.add_argument('run'); parser.add_argument('output')
parser.add_argument('--iterations', type=int, default=4)
parser.add_argument('--holds', default='extended')
parser.add_argument('--correction', default='estimator')
parser.add_argument('--relaxation', type=float, default=1., help='log-space step toward each new fit (1: plain iteration)')
options = parser.parse_args()
run = Run(options.run)
spec, feedback = twin_spec(run.path/'twin.yaml')
bank = bank_for(spec, run)
ends = hold_ends(run, options.holds)
true = run.manifest['true_scales']
scales, history = (1., 1.), []
for iteration in range(options.iterations):
    started = time.perf_counter()
    states = replay(run, spec, feedback, scales, options.correction, 'measured', t0=12., t1=ends[-1]/1e9+.1,
                    sample_every=10**9, keep_states_every=4)['states']
    cases = make_cases(run, bank.cloth.uv, ends, 'twin', 'measured', 'depth', states=states)
    fit = bank.calibrate(cases, scales)
    fit.pop('table')
    r = options.relaxation
    scales = tuple(float(np.exp((1-r)*np.log(a)+r*np.log(b))) for a, b in zip(scales, (fit['bend'], fit['shear'])))
    history.append(dict(iteration=iteration+1, bend=scales[0], shear=scales[1], fit_bend=fit['bend'], fit_shear=fit['shear'],
                        cost_m=fit['cost_m'], relaxation=r,
                        log_err_bend=float(np.log(scales[0]/true['bend'])), log_err_shear=float(np.log(scales[1]/true['shear'])),
                        seconds=time.perf_counter()-started))
    print(f"{run.path.name} iteration {iteration+1}: bend {scales[0]:.3f} shear {scales[1]:.3f} (true {true['bend']}, {true['shear']}) "
          f"cost {fit['cost_m']*1e3:.2f} mm", flush=True)
    json.dump(dict(run=run.path.name, manifest=run.manifest, holds=options.holds, correction=options.correction,
                   relaxation=options.relaxation, history=history),
              open(options.output, 'w'), indent=1)
