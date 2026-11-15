"""Locate the cloth library (`capsule_cbf`) of the fabric simulator checkout.

`capsule_cbf` is the simulator's pure NumPy/SciPy/Newton library: the cloth runtime with
prescribed grip patches, the reduced prediction model and the sensitivity CBF. The twin uses
it unchanged. It is a plain directory in `fabric_sim_isaacsim_newton`, not an installed
package, so this adds that checkout to `sys.path`:

1. the `FABRIC_SIM_ROOT` environment variable, else
2. `<workspace>/src/fabric_sim_isaacsim_newton`, found by walking up from this file (works
   from the source tree and from a colcon install).
"""
import os
import sys
from pathlib import Path


def fabric_sim_root():
    candidates = []
    if os.environ.get('FABRIC_SIM_ROOT'):
        candidates.append(Path(os.environ['FABRIC_SIM_ROOT']))
    for parent in Path(__file__).resolve().parents:
        candidates += [parent / 'src' / 'fabric_sim_isaacsim_newton', parent / 'fabric_sim_isaacsim_newton']
    for candidate in candidates:
        if (candidate / 'capsule_cbf' / 'newton_runtime.py').is_file():
            return candidate
    raise ImportError('capsule_cbf not found: set FABRIC_SIM_ROOT to the fabric_sim_isaacsim_newton checkout')


def use_capsule_cbf():
    root = str(fabric_sim_root())
    if root not in sys.path:
        sys.path.insert(0, root)
    return root
