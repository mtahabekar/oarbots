"""Deterministic Tier B replay; no ROS graph, devices or robot commands."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np
from .data import load_sequence
from .ensemble import EnsembleObserver, ObserverSpec
from .physics import make_physics
from .config import load_config
from .observations import ObservationBuilder


def replay(mesh, physics, frames, spec=ObserverSpec(), builder=None):
    observer = EnsembleObserver(mesh, physics, spec, builder)
    snapshots = [observer.assimilate(frame) for frame in frames]
    return dict(times_ns=np.array([s['time_ns'] for s in snapshots], np.int64),
        positions=np.stack([s['q'] for s in snapshots]), velocities=np.stack([s['v'] for s in snapshots]),
        covariance=np.stack([s['covariance'] for s in snapshots]), visibility=np.stack([s['visibility'] for s in snapshots]),
        observation_age_sec=np.stack([s['age'] for s in snapshots]), residual_forces=np.stack([s['residual_force'] for s in snapshots])), snapshots


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sequence'); parser.add_argument('output')
    parser.add_argument('--ensemble-size', type=int)
    parser.add_argument('--config')
    parser.add_argument('--seed', type=int)
    namespace = parser.parse_args(args)
    mesh, spec, frames, provenance = load_sequence(namespace.sequence)
    observer_spec, options = ObserverSpec(), {}
    if namespace.config:
        configured_mesh, spec, observer_spec, _, options, _ = load_config(namespace.config)
        if configured_mesh.model_id != mesh.model_id:
            raise ValueError('Configured material mesh does not match recorded mesh')
    if namespace.ensemble_size:
        from dataclasses import replace
        observer_spec = replace(observer_spec, ensemble_size=namespace.ensemble_size)
    if namespace.seed is not None:
        from dataclasses import replace
        observer_spec = replace(observer_spec, seed=namespace.seed)
    states, snapshots = replay(mesh, make_physics(mesh, spec), frames, observer_spec, ObservationBuilder(mesh, **options))
    np.savez_compressed(namespace.output, **states)
    report = {'schema_version': 1, 'model_id': mesh.model_id, 'mesh': asdict(mesh.spec), 'physics': asdict(spec),
              'observer': asdict(observer_spec), 'observations': options,
              'source': provenance, 'frames': len(frames), 'degraded_frames': sum(s['degraded'] for s in snapshots),
              'last_status': {k: snapshots[-1][k] for k in ('degraded', 'reason', 'factors')}}
    Path(str(namespace.output)+'.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({'frames': len(frames), 'degraded_frames': report['degraded_frames']}))


if __name__ == '__main__':
    main()
