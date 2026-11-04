"""Least-squares gravity calibration, with moments about the sensor origin."""
import numpy as np


def design_matrix(gravity):
    x, y, z = np.asarray(gravity, dtype=float)
    matrix = np.zeros((6, 10))
    matrix[:3, 0] = (x, y, z)
    # (mass * center_of_mass) cross gravity
    matrix[3:, 1:4] = ((0, z, -y), (-z, 0, x), (y, -x, 0))
    matrix[:, 4:] = np.eye(6)
    return matrix


def solve_calibration(matrix, observations):
    matrix = np.asarray(matrix, dtype=float)
    observations = np.asarray(observations, dtype=float).reshape(-1)
    if matrix.ndim != 2 or matrix.shape[1] != 10 or matrix.shape[0] != observations.size:
        raise ValueError('Calibration dimensions do not match')
    if not np.isfinite(matrix).all() or not np.isfinite(observations).all():
        raise ValueError('Calibration contains non-finite measurements')
    solution, _, rank, singular = np.linalg.lstsq(matrix, observations, rcond=None)
    if rank < 10 or singular[-1] <= 0 or singular[0]/singular[-1] > 1e6:
        raise ValueError('Calibration poses do not adequately excite all parameters')
    mass = solution[0]
    if mass <= 1e-6:
        raise ValueError('Estimated payload mass must be positive; check axes and load')
    return {
        'end_effector_mass': float(mass),
        'center_of_mass_vector': (solution[1:4]/mass).tolist(),
        'force_bias_vector': solution[4:7].tolist(),
        'torque_bias_vector': solution[7:10].tolist(),
    }, float(np.sqrt(np.mean((matrix @ solution - observations)**2)))
