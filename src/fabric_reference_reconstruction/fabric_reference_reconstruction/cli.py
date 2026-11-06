import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from fabric_state_estimator.data import load_sequence
from fabric_state_estimator.physics import make_physics
from .smoother import ReferenceSmoother, SmootherSpec
from fabric_state_estimator.config import load_config


def main(args=None):
    parser = argparse.ArgumentParser(description='Fit raw sensor residuals and soft nominal dynamics over the whole sequence')
    parser.add_argument('sequence'); parser.add_argument('output')
    parser.add_argument('--max-nfev', type=int)
    parser.add_argument('--association-rounds', type=int)
    parser.add_argument('--config')
    options = parser.parse_args(args)
    mesh, physics_spec, frames, provenance = load_sequence(options.sequence)
    reference, observations = {}, {}
    if options.config:
        configured_mesh, physics_spec, _, _, observations, reference = load_config(options.config)
        if configured_mesh.model_id != mesh.model_id:
            raise ValueError('Configured mesh does not match recorded mesh')
    if options.max_nfev is not None:
        reference['max_nfev'] = options.max_nfev
    if options.association_rounds is not None:
        reference['association_rounds'] = options.association_rounds
    spec = SmootherSpec(**reference)
    result = ReferenceSmoother(mesh, make_physics(mesh, physics_spec), spec, observations).reconstruct(frames)
    report = result.pop('report')
    np.savez_compressed(options.output, **result)
    report.update(schema_version=1, model_id=mesh.model_id, mesh=asdict(mesh.spec), physics=asdict(physics_spec),
                  smoother=asdict(spec), source=provenance)
    Path(str(options.output)+'.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report))
    if not report['converged']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
