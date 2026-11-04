"""Robust multi-view position fusion and a joint constant-acceleration filter."""
from dataclasses import dataclass
import numpy as np


@dataclass
class Measurement:
    hand_id: str
    tag_id: int
    camera_id: str
    stamp_ns: int
    position: np.ndarray
    covariance: np.ndarray

    def valid(self):
        p, c = np.asarray(self.position), np.asarray(self.covariance)
        return (p.shape == (3,) and c.shape == (3, 3) and np.isfinite(p).all()
                and np.isfinite(c).all() and np.allclose(c, c.T)
                and np.linalg.eigvalsh(c).min() > 0)


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, np.float64)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('TF quaternion must be finite and normalized')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def transform_measurement(measurement, rotation, translation, extrinsic_stddev):
    translation = np.asarray(translation)
    if not np.isfinite(translation).all():
        raise ValueError('Invalid TF translation')
    return Measurement(measurement.hand_id, measurement.tag_id, measurement.camera_id, measurement.stamp_ns,
                       rotation @ measurement.position + translation,
                       rotation @ measurement.covariance @ rotation.T + np.eye(3)*extrinsic_stddev**2)


def fuse_views(measurements, stamp_ns, floor=.01, max_speed=1., spatial_gate=.15, mahalanobis_gate=16.3):
    """Maximum consistent clique; ties between conflicting views are missing data.

    Camera calibration errors are correlated. The final floor is deliberately
    added after fusion so additional views cannot average it away.
    """
    by_camera = {}
    for m in measurements:
        if m.valid() and m.stamp_ns <= stamp_ns:
            if m.camera_id in by_camera:
                return None  # never count a duplicate view as independent evidence
            age = (stamp_ns - m.stamp_ns) / 1e9
            by_camera[m.camera_id] = Measurement(m.hand_id, m.tag_id, m.camera_id, m.stamp_ns,
                m.position, m.covariance + np.eye(3)*(max_speed*age)**2)
    views = list(by_camera.values())
    if not views:
        return None
    from itertools import combinations
    def compatible(a, b):
        delta = a.position-b.position
        return (np.linalg.norm(delta) <= spatial_gate and
                delta @ np.linalg.solve(a.covariance+b.covariance, delta) <= mahalanobis_gate)
    # Cell has two wrists and one overhead; bound the caller's view count.
    if len(views) > 8:
        raise ValueError('At most eight independent camera views supported')
    winners = []
    for size in range(len(views), 0, -1):
        winners = [group for group in combinations(views, size)
                   if all(compatible(a, b) for a, b in combinations(group, 2))]
        if winners:
            break
    if len(winners) != 1:
        return None
    selected = winners[0]
    precision = [np.linalg.inv(m.covariance) for m in selected]
    covariance = np.linalg.inv(sum(precision))
    position = covariance @ sum(info @ m.position for info, m in zip(precision, selected))
    return position, covariance + np.eye(3)*floor**2, selected


class HandFilter:
    def __init__(self, jerk_stddev=3., max_gap=.5, innovation_gate=16.3):
        if not np.isfinite([jerk_stddev, max_gap, innovation_gate]).all() or min(jerk_stddev, max_gap, innovation_gate) <= 0:
            raise ValueError('Invalid hand filter settings')
        self.jerk, self.gap, self.gate = jerk_stddev, max_gap, innovation_gate
        self.x, self.p, self.stamp = None, None, None

    def update(self, position, covariance, stamp_ns):
        if self.stamp is not None and stamp_ns <= self.stamp:
            return None
        if self.x is None or (stamp_ns-self.stamp)/1e9 > self.gap:
            self.x = np.r_[position, np.zeros(6)]
            self.p = np.diag([1.]*3 + [.25]*3 + [1.]*3)
            self.p[:3, :3] = covariance
        else:
            dt = (stamp_ns-self.stamp)/1e9
            transition = np.eye(9)
            transition[:3, 3:6] = np.eye(3)*dt
            transition[:3, 6:] = np.eye(3)*dt**2/2
            transition[3:6, 6:] = np.eye(3)*dt
            temporal = np.array([[dt**5/20, dt**4/8, dt**3/6],
                                 [dt**4/8, dt**3/3, dt**2/2],
                                 [dt**3/6, dt**2/2, dt]])
            x = transition @ self.x
            p = transition @ self.p @ transition.T + np.kron(temporal, np.eye(3))*self.jerk**2
            residual = position - x[:3]
            innovation = p[:3, :3] + covariance
            if residual @ np.linalg.solve(innovation, residual) > self.gate:
                return None  # no fresh state on a rejected measurement
            gain = np.linalg.solve(innovation, p[:3, :]).T
            self.x = x + gain @ residual
            joseph = np.eye(9); joseph[:, :3] -= gain
            self.p = joseph @ p @ joseph.T + gain @ covariance @ gain.T
            self.p = (self.p+self.p.T)/2
        self.stamp = stamp_ns
        return self.x[:6].copy(), self.p[:6, :6].copy()
