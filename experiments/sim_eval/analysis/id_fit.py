"""Study S2: offline material identification on one recorded dataset.

    python id_fit.py RUN_DIR OUT.json --inputs STATE GRIPS POINTS --holds extended|baseline [--landscape]

Runs the production parameter bank (`fabric_twin.bank.Bank.calibrate`, the same search the live
bank runs) on cases whose three inputs are each taken from plant truth or from what the live
system measured, so the effect of each perception input on identification is isolated:

    inputs truth truth truth   identifiability of the warm-up itself (no perception error)
    inputs twin measured depth the live configuration (corrected twin state, measured grips, depth)

With --landscape it also evaluates the cost on a 5 x 5 log grid (bend, shear in 1/4 .. 4).
"""
import argparse
import json
import time

import numpy as np

from simeval import Run, bank_for, hold_ends, landscape, make_cases, twin_spec


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('run'); parser.add_argument('output')
    parser.add_argument('--inputs', nargs=3, default=('twin', 'measured', 'depth'), metavar=('STATE', 'GRIPS', 'POINTS'))
    parser.add_argument('--holds', choices=('extended', 'baseline'), default='extended')
    parser.add_argument('--landscape', action='store_true')
    parser.add_argument('--landscape-only', action='store_true')
    parser.add_argument('--relax', type=float, default=2., help='Case length (s): the free run before each hold end')
    options = parser.parse_args()
    run = Run(options.run)
    spec, _ = twin_spec(run.path/'twin.yaml')
    bank = bank_for(spec, run)
    ends = hold_ends(run, options.holds)
    state, grips, points = options.inputs
    cases = make_cases(run, bank.cloth.uv, ends, state, grips, points, relax_sec=options.relax)
    out = dict(run=run.path.name, manifest=run.manifest, inputs=dict(state=state, grips=grips, points=points),
               holds=options.holds, holds_available=len(ends), cases=len(cases), relax_sec=options.relax,
               points_per_case=[int(len(c.points)) for c in cases])
    started = time.perf_counter()
    if not options.landscape_only:
        result = bank.calibrate(cases, (1., 1.))
        out['fit'] = result
        out['fit_seconds'] = time.perf_counter()-started
        true = run.manifest['true_scales']
        out['log_error'] = dict(bend=float(np.log(result['bend']/true['bend'])), shear=float(np.log(result['shear']/true['shear'])))
        print(f"{run.path.name} {state}/{grips}/{points} {options.holds}: bend {result['bend']:.3f} shear {result['shear']:.3f} "
              f"(true {true['bend']:.3f}, {true['shear']:.3f}) cost {result['cost_m']*1e3:.2f} mm, {len(cases)} cases, "
              f"{out['fit_seconds']:.0f} s", flush=True)
    if options.landscape or options.landscape_only:
        started = time.perf_counter()
        values, table = landscape(bank, cases)
        out['landscape'] = dict(values=values.tolist(), rms_cost_m=table.tolist(), seconds=time.perf_counter()-started)
    with open(options.output, 'w') as f:
        json.dump(out, f, indent=1)


if __name__ == '__main__':
    main()
