import numpy as np
import pytest
from fabric_reference_reconstruction.synthetic import generate
from fabric_reference_reconstruction.smoother import ReferenceSmoother, SmootherSpec
from fabric_state_estimator.physics import make_physics
from fabric_state_estimator.ensemble import EnsembleObserver, ObserverSpec


def test_offline_fit_uses_future_raw_measurements_and_reduces_residuals():
    mesh, spec, frames, truth = generate(frames=3)
    frames[0].views[0].depth = np.empty((0, 6))  # hidden at first capture
    frames[0].views[0].boundary = np.empty((0, 3))
    frames[0].views[0].tracks.clear()
    observer = EnsembleObserver(mesh, make_physics(mesh, spec), ObserverSpec(ensemble_size=8))
    causal = observer.assimilate(frames[0])
    result = ReferenceSmoother(mesh, make_physics(mesh, spec),
        SmootherSpec(max_nfev=12, association_rounds=1)).reconstruct(frames)
    report = result['report']['rounds'][0]
    assert report['measurement_normalized_rms_after'] < report['measurement_normalized_rms_before']
    assert abs(result['positions'][0, 4, 2]-truth[0, 4, 2]) < abs(causal['q'][4, 2]-truth[0, 4, 2])
    assert result['report']['kind'] == 'reference_candidate' and not result['report']['covariance_available']
    assert result['velocities'].shape == result['positions'].shape


def test_offline_requires_surface_data_and_ordered_frames():
    mesh, spec, frames, _ = generate(frames=2)
    smoother = ReferenceSmoother(mesh, make_physics(mesh, spec))
    with pytest.raises(ValueError, match='budget'):
        ReferenceSmoother(mesh, make_physics(mesh, spec), SmootherSpec(max_variables=10)).reconstruct(frames)
    with pytest.raises(ValueError, match='ordered'):
        smoother.reconstruct(frames[::-1])
    for frame in frames:
        frame.views.clear()
    with pytest.raises(ValueError, match='No associated'):
        smoother.reconstruct(frames)
