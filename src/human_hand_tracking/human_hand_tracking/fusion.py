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


def fuse_views(measurements, stamp_ns, max_speed=1., spatial_gate=.15, mahalanobis_gate=16.3):
    """Maximum consistent clique; ties between conflicting views are missing data.

    The returned covariance excludes the correlated calibration floor; the
    HandFilter adds it to its output so neither extra views nor extra frames
    can average it away.
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
    return position, covariance, selected


def constant_acceleration(dt):
    """Transition and unit-jerk process covariance of (position, velocity, acceleration)."""
    transition = np.eye(9)
    transition[:3, 3:6] = np.eye(3)*dt
    transition[:3, 6:] = np.eye(3)*dt**2/2
    transition[3:6, 6:] = np.eye(3)*dt
    temporal = np.array([[dt**5/20, dt**4/8, dt**3/6],
                         [dt**4/8, dt**3/3, dt**2/2],
                         [dt**3/6, dt**2/2, dt]])
    return transition, np.kron(temporal, np.eye(3))


def propagate(state, covariance, jerk_stddev, dt):
    """Carry a filter state by dt (forward, or backward for dt < 0) with the filter's own model
    and process noise.

    A consumer that extrapolates position + dt*velocity and keeps the old covariance drops
    sigma_j^2 |dt|^5/20 of position variance (91 mm per axis at 0.45 s for sigma_j = 3 m/s^3)
    and the velocity and acceleration uncertainty, which made consumed grips overconfident.
    """
    state, covariance = np.asarray(state, float), np.asarray(covariance, float)
    if state.shape != (9,) or covariance.shape != (9, 9) or not np.isfinite(dt) or jerk_stddev < 0:
        raise ValueError('Expected a 9-state hand filter state, its covariance and a finite interval')
    transition = constant_acceleration(dt)[0]
    return transition @ state, transition @ covariance @ transition.T+constant_acceleration(abs(dt))[1]*jerk_stddev**2


class HandTrack:
    """Recent filter states of one hand, and the grip they imply at any requested time.

    Between two glove detections the grip is interpolated: the earlier state is carried
    forward, the later one backward, and the two are combined by their covariances. Past the
    newest state it can only be extrapolated, and the covariance says how poorly: with glove
    detections 0.2 s apart, the filter's own one-sigma position error is already several
    centimetres at 0.13 s. A consumer that can wait for the next detection should.
    """
    def __init__(self, floor=.01, max_gap=1., history=200):
        self.floor, self.max_gap, self.history, self.states = floor, max_gap, history, []

    def add(self, stamp_ns, state, covariance, jerk_stddev):
        state, covariance = np.asarray(state, float), np.asarray(covariance, float)
        if (state.shape != (9,) or covariance.shape != (9, 9) or not np.isfinite(state).all()
                or not np.isfinite(covariance).all() or not np.isfinite(jerk_stddev) or jerk_stddev < 0):
            raise ValueError('Invalid hand filter state')
        if self.states and stamp_ns <= self.states[-1][0]:
            return False
        self.states.append((int(stamp_ns), state, (covariance+covariance.T)/2, float(jerk_stddev)))
        del self.states[:-self.history]
        return True

    @staticmethod
    def _carry(entry, time_ns):
        stamp, state, covariance, jerk = entry
        # An acceleration estimate from sparse detections is mostly noise; shrink it by its
        # own signal-to-noise ratio so it cannot dominate a long carry.
        state = state.copy()
        power = state[6:] @ state[6:]
        state[6:] *= power/(power+np.trace(covariance[6:, 6:])+1e-12)
        return propagate(state, covariance, jerk, (time_ns-stamp)/1e9)

    def newest(self):
        return self.states[-1][0] if self.states else None

    def at(self, time_ns, max_age):
        """Grip at `time_ns`: dict(position, velocity, covariance 3x3, age_sec, bracketed), or
        None when nothing at or before that time is newer than `max_age`."""
        before = next((s for s in reversed(self.states) if s[0] <= time_ns), None)
        after = next((s for s in self.states if s[0] > time_ns), None)
        if before is None:
            return None
        age = (time_ns-before[0])/1e9
        state, covariance = self._carry(before, time_ns)
        position, velocity, spread = state[:3], state[3:6], covariance[:3, :3]
        bracketed = after is not None and (after[0]-before[0])/1e9 <= self.max_gap
        if bracketed:
            back, back_covariance = self._carry(after, time_ns)
            a, b = np.linalg.inv(spread), np.linalg.inv(back_covariance[:3, :3])
            spread = np.linalg.inv(a+b)
            position = spread @ (a @ position+b @ back[:3])
            weight = np.trace(back_covariance[3:6, 3:6])
            weight = weight/(weight+np.trace(covariance[3:6, 3:6]))
            velocity = weight*velocity+(1-weight)*back[3:6]
            spread = spread+np.eye(3)*self.floor**2  # the shared calibration bias does not average out
        elif age > max_age:
            return None
        return dict(position=position, velocity=velocity, covariance=(spread+spread.T)/2, age_sec=age, bracketed=bracketed)


class HandFilter:
    """Constant-acceleration filter. ``floor`` is a slowly varying bias
    (tag-to-grip offset, hand-eye calibration): it is invisible in the
    innovations, so it is added to the published position covariance rather
    than treated as white measurement noise that filtering would average out."""
    def __init__(self, jerk_stddev=3., max_gap=.5, innovation_gate=16.3, floor=.01):
        if (not np.isfinite([jerk_stddev, max_gap, innovation_gate, floor]).all()
                or min(jerk_stddev, max_gap, innovation_gate) <= 0 or floor < 0):
            raise ValueError('Invalid hand filter settings')
        self.jerk, self.gap, self.gate, self.floor = jerk_stddev, max_gap, innovation_gate, floor
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
            x, p = propagate(self.x, self.p, self.jerk, dt)
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
        covariance = self.p[:6, :6].copy()
        covariance[:3, :3] += np.eye(3)*self.floor**2
        return self.x[:6].copy(), covariance

    def full(self):
        """(state 9, covariance 9x9) with the calibration floor on position, for consumers that
        propagate the grip themselves."""
        covariance = self.p.copy()
        covariance[:3, :3] += np.eye(3)*self.floor**2
        return self.x.copy(), covariance
