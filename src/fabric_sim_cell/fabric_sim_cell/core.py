"""Pure helpers for running the cell against a simulated plant (no ROS imports).

The simulated plant (`oarbot_simulation`) publishes device topics only. This package holds
what a deployment adds on top: calibration transforms, an explicitly labelled oracle hand
input, a recorder for estimator output, and scoring against the plant's private truth.
"""
import math

import numpy as np


# --------------------------------------------------------------------- transforms
def rotation_xyzw(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(w * w + x * x + y * y + z * z)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def xyzw_from_rotation(r):
    r = np.asarray(r, dtype=float)
    w = math.sqrt(max(0.0, 1.0 + r[0, 0] + r[1, 1] + r[2, 2])) / 2.0
    x = math.copysign(math.sqrt(max(0.0, 1.0 + r[0, 0] - r[1, 1] - r[2, 2])) / 2.0, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] + r[1, 1] - r[2, 2])) / 2.0, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1.0 - r[0, 0] - r[1, 1] + r[2, 2])) / 2.0, r[1, 0] - r[0, 1])
    q = np.array([x, y, z, w])
    return q / np.linalg.norm(q)


def rpy_matrix(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def perturbed(transform, error):
    """Apply a calibration error `{translation: [m], rotation_rpy_deg: [deg]}` in the child frame.

    The error is a right-multiplied rigid offset: what the deployment believes the
    child is, compared with where the plant actually put it.
    """
    if not error:
        return dict(transform)
    r = rotation_xyzw(transform['rotation_xyzw'])
    t = np.asarray(transform['translation'], dtype=float)
    dr = rpy_matrix(*np.radians(error.get('rotation_rpy_deg', [0.0, 0.0, 0.0])))
    dt = np.asarray(error.get('translation', [0.0, 0.0, 0.0]), dtype=float)
    out = dict(transform)
    out['translation'] = (t + r @ dt).tolist()
    out['rotation_xyzw'] = xyzw_from_rotation(r @ dr).tolist()
    return out


def calibration_transforms(calibration, errors=None):
    """Static transforms of a calibration file, with optional named errors applied."""
    errors = errors or {}
    out = []
    for entry in calibration['transforms']:
        for key in ('parent', 'child', 'translation', 'rotation_xyzw'):
            if key not in entry:
                raise ValueError(f'calibration transform is missing {key!r}: {entry}')
        if len(entry['translation']) != 3 or len(entry['rotation_xyzw']) != 4:
            raise ValueError(f'malformed calibration transform: {entry}')
        out.append(perturbed(entry, errors.get(entry['child'])))
    unknown = set(errors) - {e['child'] for e in calibration['transforms']}
    if unknown:
        raise ValueError(f'calibration errors name unknown frames: {sorted(unknown)}')
    return out


# --------------------------------------------------------------------- oracle hands
HANDS = {'glove_left': ('human_left', 20, (0.0, 1.0)), 'glove_right': ('human_right', 21, (1.0, 1.0))}


def hand_covariance(position_std, velocity_std):
    """6x6 position/velocity covariance, row-major, for `HandState.covariance`."""
    return np.diag([position_std ** 2] * 3 + [velocity_std ** 2] * 3).ravel().tolist()


class OracleNoise:
    """Seeded measurement error for the oracle hand input (zero by default)."""

    def __init__(self, position_std=0.0, velocity_std=0.0, seed=0):
        self.position_std, self.velocity_std = float(position_std), float(velocity_std)
        self.rng = np.random.default_rng(seed)

    def apply(self, position, velocity):
        p = np.asarray(position, dtype=float) + self.rng.normal(0.0, self.position_std, 3) * (self.position_std > 0)
        v = np.asarray(velocity, dtype=float) + self.rng.normal(0.0, self.velocity_std, 3) * (self.velocity_std > 0)
        return p, v


# --------------------------------------------------------------------- truth alignment
def grid_from_truth(positions, material_uv, grid_shape):
    """Reorder plant particles (T, N, 3) into a (T, rows, columns, 3) grid by material uv."""
    rows, columns = (int(v) for v in grid_shape)
    uv = np.asarray(material_uv, dtype=float)
    col = np.rint(uv[:, 0] * (columns - 1)).astype(int)
    row = np.rint(uv[:, 1] * (rows - 1)).astype(int)
    if (np.abs(col - uv[:, 0] * (columns - 1)).max() > 1e-3 or np.abs(row - uv[:, 1] * (rows - 1)).max() > 1e-3
            or len(set(zip(row.tolist(), col.tolist()))) != rows * columns):
        raise ValueError('plant particles are not a regular material grid')
    grid = np.empty((positions.shape[0], rows, columns, 3), dtype=float)
    grid[:, row, col] = positions
    return grid


def sample_grid(grid, uv):
    """Bilinear sample of a (T, rows, columns, 3) grid at material coordinates (V, 2)."""
    rows, columns = grid.shape[1], grid.shape[2]
    x = np.clip(np.asarray(uv[:, 0], dtype=float) * (columns - 1), 0, columns - 1)
    y = np.clip(np.asarray(uv[:, 1], dtype=float) * (rows - 1), 0, rows - 1)
    x0, y0 = np.minimum(np.floor(x).astype(int), columns - 2), np.minimum(np.floor(y).astype(int), rows - 2)
    fx, fy = (x - x0)[None, :, None], (y - y0)[None, :, None]
    return ((1 - fx) * (1 - fy) * grid[:, y0, x0] + fx * (1 - fy) * grid[:, y0, x0 + 1]
            + (1 - fx) * fy * grid[:, y0 + 1, x0] + fx * fy * grid[:, y0 + 1, x0 + 1])


def truth_at(times_ns, truth_times_ns, truth_grid, uv):
    """Plant truth at estimate times: linear in time between recorded frames, bilinear in uv.

    Times outside the recorded span get NaN (no truth), never an extrapolation.
    """
    times = np.asarray(times_ns, dtype=np.int64)
    recorded = np.asarray(truth_times_ns, dtype=np.int64)
    sampled = sample_grid(truth_grid, uv)  # (T_truth, V, 3)
    out = np.full((len(times), sampled.shape[1], 3), np.nan)
    inside = (times >= recorded[0]) & (times <= recorded[-1])
    k = np.clip(np.searchsorted(recorded, times[inside], side='right') - 1, 0, len(recorded) - 2)
    span = (recorded[k + 1] - recorded[k]).astype(float)
    w = ((times[inside] - recorded[k]) / np.where(span > 0, span, 1.0))[:, None, None]
    out[inside] = (1 - w) * sampled[k] + w * sampled[k + 1]
    return out


FINGERS = tuple(f'j2n6s300_joint_finger_{i}' for i in (1, 2, 3))


def fingers_closed(names, positions, threshold, fingers=FINGERS):
    """True when every finger joint is present and at or past `threshold` (rad) closed."""
    position = dict(zip(names, positions))
    return all(f in position and np.isfinite(position[f]) and position[f] >= threshold for f in fingers)
