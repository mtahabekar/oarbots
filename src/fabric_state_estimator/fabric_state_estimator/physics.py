"""Backend contract: explicit q/v in and out; grip forces are reactions only."""
from dataclasses import asdict, dataclass
import numpy as np


@dataclass(frozen=True)
class PhysicsSpec:
    backend: str = 'spring'
    max_step_sec: float = .002
    gravity: tuple = (0., 0., -9.81)
    spring_stiffness_n_m: float = 8.
    damping_per_sec: float = 1.
    newton_stretch: tuple = (100., 100., 10.)
    newton_bend: tuple = (2e-5, 2e-5, 1e-5)
    device: str = 'cpu'
    iterations: int = 4


def validate_spec(spec):
    if (spec.backend not in ('spring', 'newton_style3d') or spec.iterations < 1
            or len(spec.gravity) != 3 or len(spec.newton_stretch) != 3 or len(spec.newton_bend) != 3
            or not np.isfinite([spec.max_step_sec, spec.spring_stiffness_n_m, spec.damping_per_sec,
                                *spec.gravity, *spec.newton_stretch, *spec.newton_bend]).all()
            or min(spec.max_step_sec, spec.spring_stiffness_n_m, *spec.newton_stretch) <= 0
            or min(spec.damping_per_sec, *spec.newton_bend) < 0):
        raise ValueError('Invalid physics configuration')


class SpringPhysics:
    """Small CPU test plant, not a substitute calibration for Style3D material."""
    def __init__(self, mesh, spec=PhysicsSpec()):
        validate_spec(spec)
        self.mesh, self.spec = mesh, spec
        # Explicit spring stability depends on the smallest lumped mass.
        self.step_limit = min(spec.max_step_sec, .2*np.sqrt(mesh.mass.min()/(6*spec.spring_stiffness_n_m)))

    def warm_up(self, size):
        return None

    def internal_forces(self, q):
        q = np.asarray(q)
        a, b = self.mesh.edges.T
        delta = q[:, b]-q[:, a]
        lengths = np.linalg.norm(delta, axis=-1)
        forces = self.spec.spring_stiffness_n_m*(lengths-self.mesh.lengths)[..., None]*delta/np.maximum(lengths[..., None], 1e-10)
        result = np.zeros((self.mesh.count, len(q), 3))
        np.add.at(result, a, forces.transpose(1, 0, 2))
        np.add.at(result, b, -forces.transpose(1, 0, 2))
        return result.transpose(1, 0, 2)

    def reactions(self, q, loads):
        """Quasi-static force ON the sheet, including pinned-vertex mass."""
        force = self.internal_forces(q) + self.mesh.mass[None, :, None]*np.asarray(self.spec.gravity)
        force += np.asarray(loads)[:, None, :]*(self.mesh.mass/self.mesh.mass.sum())[None, :, None]
        return -force[:, self.mesh.corners]

    def advance(self, q, v, duration, grips, loads=None):
        q, v, grips, loads = validate_inputs(self.mesh, q, v, duration, grips, loads)
        if duration == 0:
            return q, v
        steps = int(np.ceil(duration/self.step_limit)); dt = duration/steps
        start = q[:, self.mesh.corners].copy()
        grip_velocity = (grips-start)/duration
        acceleration = np.asarray(self.spec.gravity)[None, None] + loads[:, None]/self.mesh.mass.sum()
        for step in range(steps):
            q[:, self.mesh.corners] = start+(step+1)/steps*(grips-start)
            force = self.internal_forces(q)
            v += dt*(force/self.mesh.mass[None, :, None] + acceleration - self.spec.damping_per_sec*v)
            q += dt*v
            q[:, self.mesh.corners], v[:, self.mesh.corners] = start+(step+1)/steps*(grips-start), grip_velocity
        if not np.isfinite(q).all() or not np.isfinite(v).all() or np.max(np.abs(v)) > 1e4:
            raise FloatingPointError('Physics diverged; check dimensions, mass, stiffness, steps and grips')
        return q, v


def validate_inputs(mesh, q, v, duration, grips, loads):
    q, v, grips = np.array(q, float, copy=True), np.array(v, float, copy=True), np.asarray(grips, float)
    loads = np.zeros((len(q), 3)) if loads is None else np.asarray(loads, float)
    if (q.ndim != 3 or q.shape[1:] != (mesh.count, 3) or v.shape != q.shape
            or grips.shape != (len(q), 4, 3) or loads.shape != (len(q), 3)
            or not np.isfinite(duration) or not 0 <= duration <= 2.
            or not all(np.isfinite(a).all() for a in (q, v, grips, loads))):
        raise ValueError('Invalid explicit physics state, grip targets, loads or duration (0..2 s)')
    return q, v, grips, loads


def make_physics(mesh, spec):
    validate_spec(spec)
    if spec.backend == 'spring':
        return SpringPhysics(mesh, spec)
    from .newton_backend import NewtonPhysics
    return NewtonPhysics(mesh, spec)
