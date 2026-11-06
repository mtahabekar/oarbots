"""Pure helpers of the simulated-cell deployment (no ROS context needed)."""
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from fabric_sim_cell.core import (calibration_transforms, grid_from_truth, hand_covariance, perturbed,
                                  rotation_xyzw, sample_grid, truth_at, xyzw_from_rotation, OracleNoise)

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def plant_grid(n=57, width=0.84, z=1.05):
    xs, ys = np.meshgrid(np.linspace(-width / 2, width / 2, n), np.linspace(-width / 2, width / 2, n))
    rest = np.column_stack((xs.ravel(), ys.ravel(), np.full(n * n, z)))
    uv = (rest[:, :2] - rest[:, :2].min(axis=0)) / np.ptp(rest[:, :2], axis=0)
    return rest, uv


def test_quaternion_round_trip():
    rng = np.random.default_rng(3)
    for _ in range(20):
        q = rng.normal(size=4); q /= np.linalg.norm(q)
        r = rotation_xyzw(q)
        np.testing.assert_allclose(rotation_xyzw(xyzw_from_rotation(r)), r, atol=1e-12)


def test_calibration_file_and_deliberate_errors():
    calibration = yaml.safe_load((CONFIG / 'calibration_exact.yaml').read_text())
    exact = calibration_transforms(calibration)
    children = [t['child'] for t in exact]
    for frame in ('world', 'oarbot_blue/base_link', 'oarbot_silver/base_link', 'oarbot_blue/fabric_grip',
                  'oarbot_silver/fabric_grip', 'nuc/camera_base'):
        assert frame in children
    errors = {'oarbot_blue/fabric_grip': {'translation': [0.005, 0.0, 0.0], 'rotation_rpy_deg': [0, 0, 2.0]}}
    wrong = {t['child']: t for t in calibration_transforms(calibration, errors)}
    grip = wrong['oarbot_blue/fabric_grip']
    np.testing.assert_allclose(grip['translation'], [0.005, 0, 0], atol=1e-12)  # identity parent offset
    angle = math.degrees(2 * math.acos(min(1.0, abs(grip['rotation_xyzw'][3]))))
    assert angle == pytest.approx(2.0, abs=1e-9)
    with pytest.raises(ValueError):
        calibration_transforms(calibration, {'not/a/frame': {'translation': [0, 0, 0]}})
    assert perturbed(exact[0], None) == exact[0]


def test_force_torque_calibration_is_present():
    calibration = yaml.safe_load((CONFIG / 'calibration_exact.yaml').read_text())
    for robot in ('oarbot_blue', 'oarbot_silver'):
        ft = calibration['force_torque'][robot]
        assert ft['end_effector_mass'] > 1.0
        assert len(ft['center_of_mass_vector']) == 3


def test_oracle_covariance_and_noise():
    c = np.asarray(hand_covariance(0.005, 0.05)).reshape(6, 6)
    np.testing.assert_allclose(np.diag(c), [2.5e-5] * 3 + [2.5e-3] * 3)
    p, v = OracleNoise().apply([1, 2, 3], [0, 0, 0])
    np.testing.assert_array_equal(p, [1, 2, 3])
    noisy = OracleNoise(0.01, 0.0, seed=1).apply([0, 0, 0], [0, 0, 0])[0]
    assert 0 < np.linalg.norm(noisy) < 0.1


def test_truth_alignment_hits_plant_particles_exactly():
    rest, uv = plant_grid()
    rng = np.random.default_rng(0)
    order = rng.permutation(len(rest))  # the alignment must not rely on particle order
    positions = np.stack([rest, rest + [0, 0, -0.1]])[:, order]
    grid = grid_from_truth(positions, uv[order], (57, 57))
    u, v = np.meshgrid(np.linspace(0, 1, 15), np.linspace(0, 1, 15))
    mesh_uv = np.column_stack((u.ravel(), v.ravel()))  # every fourth plant particle
    sampled = sample_grid(grid, mesh_uv)
    expected = rest.reshape(57, 57, 3)[::4, ::4].reshape(-1, 3)
    np.testing.assert_allclose(sampled[0], expected, atol=1e-12)
    times = np.array([0, 16_666_667], dtype=np.int64)
    at = truth_at([0, 8_333_333, 16_666_667, 30_000_000], times, grid, mesh_uv)
    np.testing.assert_allclose(at[1][:, 2], 1.05 - 0.05, atol=1e-6)
    assert np.isnan(at[3]).all()  # beyond the recording: no truth, no extrapolation


def test_irregular_truth_is_rejected():
    rest, uv = plant_grid(5)
    with pytest.raises(ValueError):
        grid_from_truth(rest[None], uv + 0.01, (5, 5))


def test_fingers_closed_needs_every_finger_past_the_threshold():
    from fabric_sim_cell.core import fingers_closed
    names = ['j2n6s300_joint_1', 'j2n6s300_joint_finger_1', 'j2n6s300_joint_finger_2', 'j2n6s300_joint_finger_3']
    assert fingers_closed(names, [0.3, 1.3, 1.29, 1.31], 1.0)
    assert not fingers_closed(names, [0.3, 1.3, 0.2, 1.31], 1.0)
    assert not fingers_closed(names[:3], [0.3, 1.3, 1.3], 1.0)  # a finger missing is not closed
    assert not fingers_closed(names, [0.3, 1.3, float('nan'), 1.3], 1.0)
