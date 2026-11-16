"""CPU-only trajectory checks; these do not establish material recovery."""
import numpy as np
import pytest
from fabric_twin.warmup import WarmupScript


def run(script):
    corners = np.array([[0., 0., 1.], [.84, 0., 1.], [0., .84, 1.], [.84, .84, 1.]])
    script.start(0, corners)
    held = corners[:2].copy()
    path = []
    for t in np.arange(0, script.duration, 1/60):
        velocity = script.command(int(t*1e9), held)
        assert np.isfinite(velocity).all()
        assert np.linalg.norm(velocity, axis=1).max() <= 2*script.speed+1e-9
        held += velocity/60
        path.append(held.copy())
    np.testing.assert_allclose(held, corners[:2], atol=.003)
    return np.array(path)


def test_extended_motion_exercises_more_directions_than_baseline():
    baseline = run(WarmupScript())
    script = WarmupScript(transverse_m=.03, lift_m=.05, lateral_m=.03)
    richer = run(script)
    assert np.ptp(baseline[:, 0, 0]) == 0
    assert np.ptp(baseline[:, 0, 2]) == 0
    assert np.ptp(richer[:, 0, 0]) > .05
    assert np.ptp(richer[:, 0, 2]) > .045
    assert np.ptp(richer[:, 1, 2]) > .045
    assert len(script.holds()) > 4
    assert all(end-start >= 2.9 for start, end, _ in script.holds())


@pytest.mark.parametrize('value', [-.01, .2, float('nan')])
def test_invalid_excursions(value):
    with pytest.raises(ValueError):
        WarmupScript(lift_m=value)
