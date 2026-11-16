"""Matched-model GPU identification comparison, without cameras, CBF or feedback.

Run with fabric_twin on PYTHONPATH. JSON output is evidence for this synthetic
experiment only; it does not establish recovery with rendered perception.
"""
import argparse
import json
import time
from collections import deque
from pathlib import Path
import numpy as np
from fabric_twin.bank import Bank, Case
from fabric_twin.cloth import TwinCloth, TwinSpec
from fabric_twin.warmup import WarmupScript


def compare():
    spec = TwinSpec(cells_x=16, cells_y=16, device='cuda:0')
    initial = np.array([[-.40, -.42, 1.05], [.40, -.42, 1.05],
                        [-.40, .42, 1.05], [.40, .42, 1.05]])
    results = {}
    for name, options in [('baseline', {}), ('extended', dict(transverse_m=.03, lift_m=.05, lateral_m=.03))]:
        started = time.perf_counter()
        real = TwinCloth(spec, initial)
        real.set_material(2., .5)
        script = WarmupScript(**options)
        script.start(0, initial)
        ends = {round(end*60) for start, end, _ in script.holds() if end-start >= 2.}
        history, cases = deque(maxlen=121), []
        held = initial.copy()
        for frame in range(round(script.duration*60)+1):
            q, v = real.state()
            history.append((q.copy(), v.copy(), held.copy()))
            if frame in ends and len(history) == 121:
                records = list(history)
                cases.append(Case(records[0][0], np.array([r[2] for r in records[1:]]), q.copy(), records[0][1]))
            held[:2] += script.command(round(frame/60*1e9), held[:2])/60
            held = real.within_reach(held)
            real.step(held)
        print(f'{name}: fitting {len(cases)} holds', flush=True)
        result = Bank(spec, initial).calibrate(cases, rounds=1)
        result.pop('table')
        result.update(cases=len(cases), elapsed_sec=time.perf_counter()-started,
                      log_parameter_error=float(np.linalg.norm(np.log(np.array([result['bend'], result['shear']])/[2., .5]))))
        results[name] = result
        print(json.dumps({name: result}), flush=True)
    return results


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.write_text(json.dumps(compare(), indent=2)+'\n')
