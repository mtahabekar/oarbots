"""The twin sheet: a Newton Style3D cloth whose four grip patches follow measured grips.

No ROS, Isaac or PhysX. The cloth runtime (prescribed grip patches, captured CUDA graph) is the
simulator's `capsule_cbf.newton_runtime.ClothRuntime`, used unchanged apart from one added
external-force buffer. Material corners follow the estimator's convention:
0 Blue (u, v) = (0, 0), 1 Silver (1, 0), 2 human right (0, 1), 3 human left (1, 1): the person faces the
robots, so their right hand is on Blue's side.

The Newton model is built axis-aligned in a *twin frame*: the world rotated about the vertical
(and mirrored if the corners run the other way round) so that the material u axis lies along x.
The cloth mechanics are unchanged by that rotation or reflection, and the reduced model of the
CBF requires an axis-aligned rest sheet. Everything this class takes and returns is in the
world frame unless a name says `twin`.
"""
from dataclasses import dataclass, field
import numpy as np
import warp as wp
import newton
from newton.solvers import style3d
from scipy.sparse import csr_matrix

from .vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.newton_runtime import ClothRuntime  # noqa: E402


@dataclass(frozen=True)
class Obstacle:
    """A box standing on or above the floor: centre and half extents in metres, yaw in radians (world)."""
    center: tuple = (0., 0., .16)
    half_extents: tuple = (.22, .22, .16)
    yaw: float = 0.


@dataclass(frozen=True)
class TwinSpec:
    width_m: float = .84
    height_m: float = .84
    cells_x: int = 28
    cells_y: int = 28
    areal_density_kg_m2: float = .2
    stretch: tuple = (3.e4, 3.e4, 6.e3)  # Style3D tri_aniso_ke: weft, warp, shear
    bend: tuple = (.002, .001, .0005)    # Style3D edge_aniso_ke
    tri_ka: float = 5.e3
    tri_kd: float = 5.e-5
    edge_kd: float = 1.e-5
    patch_m: float = .03                 # side of each grip patch
    particle_radius: float = .008
    substeps: int = 8
    iterations: int = 6
    linear_iterations: int = 3
    frame_dt: float = 1/60
    contact_ke: float = 4000.
    contact_kd: float = .01
    contact_mu: float = 2.
    contact_margin: float = .012
    max_grip_speed_mps: float = 1.       # a grip target may not jump faster than this
    device: str = 'cuda:0'
    obstacle: Obstacle = field(default_factory=Obstacle)

    def validate(self):
        numbers = [self.width_m, self.height_m, self.areal_density_kg_m2, *self.stretch, *self.bend, self.patch_m,
                   self.particle_radius, self.frame_dt, self.max_grip_speed_mps, *self.obstacle.half_extents]
        if (not np.isfinite(numbers).all() or min(numbers) <= 0 or min(self.cells_x, self.cells_y) < 4
                or min(self.substeps, self.iterations, self.linear_iterations) < 1
                or not np.isfinite([*self.obstacle.center, self.obstacle.yaw]).all()):
            raise ValueError('Invalid twin specification')


@wp.kernel
def add_external(external: wp.array[wp.vec3], forces: wp.array[wp.vec3]):
    i = wp.tid()
    forces[i] = forces[i]+external[i]


class NudgedRuntime(ClothRuntime):
    """`ClothRuntime` with an external per-particle force (N), applied in every substep.

    The force is written after the solver's force buffer is cleared and before the solve, inside
    the captured graph, so changing the buffer's contents needs no new capture.
    """
    def __init__(self, model, *args, **kwargs):
        self.external = wp.zeros(model.particle_count, dtype=wp.vec3, device=model.device)
        super().__init__(model, *args, **kwargs)

    def _advance(self):  # ClothRuntime._advance plus the external force
        a, b = self.state, self.scratch
        for k in range(self.substeps):
            a.clear_forces()
            wp.launch(add_external, self.model.particle_count, inputs=[self.external, a.particle_f], device=self.device)
            elapsed = (k+1)*self.dt
            self._pin(a, elapsed)
            self.pipeline.collide(a, self.contacts)
            self.solver.step(a, b, self.control, self.contacts, self.dt)
            self._pin(b, elapsed)
            a, b = b, a
        if a is not self.state:
            self.state.assign(a)

    def recapture(self):
        """Capture the frame graph again, after the solver's precomputed matrix changed."""
        if self.graph is None:
            return
        saved = self.model.state()
        saved.assign(self.state)
        self._advance()  # allocate anything new before capturing
        self.state.assign(saved); self.scratch.assign(saved)
        with wp.ScopedCapture(device=self.device) as scope:
            self._advance()
        self.graph = scope.graph
        self.state.assign(saved); self.scratch.assign(saved)


def twin_frame(grips):
    """Orthogonal matrix A and origin o with twin = A (world - o), from four grip positions.

    x runs along the horizontal direction from corner 0 to corner 1 (the robots' edge), z is up,
    and y points toward corner 2. If the corners run clockwise seen from above, A includes a
    reflection: the sheet's mechanics are mirror-symmetric, so the mirrored simulation is exact.
    """
    grips = np.asarray(grips, float)
    if grips.shape != (4, 3) or not np.isfinite(grips).all():
        raise ValueError('Four finite grip positions are required')
    x = grips[1]-grips[0]; x[2] = 0.
    if np.linalg.norm(x) < 1e-3:
        raise ValueError('Robot grips coincide horizontally; cannot orient the sheet')
    x /= np.linalg.norm(x)
    y = np.cross([0., 0., 1.], x)
    across = .5*(grips[2]+grips[3])-.5*(grips[0]+grips[1])
    if across @ y < 0:
        y = -y
    origin = grips.mean(axis=0); origin[2] = 0.
    return np.array([x, y, [0., 0., 1.]]), origin


def material_lift(uv, rows, columns):
    """Sparse bilinear weights from an estimator mesh (rows x columns vertices, row-major, u along
    columns) to points with material coordinates `uv` (N, 2)."""
    uv = np.clip(np.asarray(uv, float), 0., 1.)
    x, y = uv[:, 0]*(columns-1), uv[:, 1]*(rows-1)
    i, j = np.minimum(x.astype(int), columns-2), np.minimum(y.astype(int), rows-2)
    a, b = x-i, y-j
    index = np.arange(len(uv))
    weights = np.column_stack(((1-a)*(1-b), a*(1-b), (1-a)*b, a*b))
    columns_index = np.column_stack((j*columns+i, j*columns+i+1, (j+1)*columns+i, (j+1)*columns+i+1))
    return csr_matrix((weights.ravel(), (np.repeat(index, 4), columns_index.ravel())), shape=(len(uv), rows*columns))


def build_model(spec, obstacle_twin=None):
    """Newton model of one axis-aligned sheet, with the ground and (optionally) the obstacle box.

    `obstacle_twin`: (center, half_extents, yaw) in the twin frame, or None for no obstacle.
    Returns (model, rest, corner particles, patch indices, patch corner ids, patch offsets).
    """
    spec.validate()
    builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
    newton.solvers.SolverStyle3D.register_custom_attributes(builder)
    shape = newton.ModelBuilder.ShapeConfig(density=0., mu=.8, ke=5.e5, kd=10.)
    builder.add_ground_plane(cfg=shape)
    if obstacle_twin is not None:
        center, half, yaw = obstacle_twin
        builder.add_shape_box(body=-1, cfg=shape, hx=float(half[0]), hy=float(half[1]), hz=float(half[2]),
                              xform=wp.transform(wp.vec3(*map(float, center)),
                                                 wp.quat_from_axis_angle(wp.vec3(0., 0., 1.), float(yaw))))
    w, h, nx, ny = spec.width_m, spec.height_m, spec.cells_x, spec.cells_y
    # Style3D spreads the given per-particle mass uniformly over the sheet's area.
    style3d.add_cloth_grid(
        builder, pos=wp.vec3(-w/2, -h/2, 1.), rot=wp.quat_identity(), vel=wp.vec3(), dim_x=nx, dim_y=ny,
        cell_x=w/nx, cell_y=h/ny, mass=spec.areal_density_kg_m2*w*h/((nx+1)*(ny+1)),
        tri_aniso_ke=wp.vec3(*spec.stretch), tri_ka=spec.tri_ka, tri_kd=spec.tri_kd,
        edge_aniso_ke=wp.vec3(*spec.bend), edge_kd=spec.edge_kd, particle_radius=spec.particle_radius)
    rest = np.asarray(builder.particle_q, dtype=float)
    tips = np.array([[-w/2, -h/2], [w/2, -h/2], [-w/2, h/2], [w/2, h/2]])
    corners = np.array([np.argmin(np.linalg.norm(rest[:, :2]-tip, axis=1)) for tip in tips])
    patch, ids, offsets = [], [], []
    for corner, tip in enumerate(tips):
        for index in np.flatnonzero(np.all(np.abs(rest[:, :2]-tip) <= spec.patch_m+1e-5, axis=1)):
            patch.append(int(index)); ids.append(corner); offsets.append(rest[index]-rest[corners[corner]])
            builder.particle_flags[int(index)] &= ~int(newton.ParticleFlags.ACTIVE)
    model = builder.finalize(device=spec.device)
    model.set_gravity((0., 0., -9.81))
    model.soft_contact_ke, model.soft_contact_kd, model.soft_contact_mu = spec.contact_ke, spec.contact_kd, spec.contact_mu
    model.soft_contact_margin = spec.contact_margin
    return model, rest, corners, np.asarray(patch), np.asarray(ids), np.asarray(offsets, dtype=float)


class TwinCloth:
    def __init__(self, spec, grips):
        """`grips`: the four measured grip positions (world) the sheet is first placed between."""
        self.spec = spec
        self.A, self.origin = twin_frame(grips)
        o = spec.obstacle
        # Conjugating a yaw by the frame (a rotation or a reflection) is again a yaw.
        yaw = np.array([[np.cos(o.yaw), -np.sin(o.yaw), 0.], [np.sin(o.yaw), np.cos(o.yaw), 0.], [0., 0., 1.]])
        local = self.A @ yaw @ self.A.T
        self.obstacle_twin = (self.to_twin(o.center), np.asarray(o.half_extents, float), float(np.arctan2(local[1, 0], local[0, 0])))
        (self.model, self.rest, self.corner_particles, self.patch, self.patch_corners,
         self.patch_offsets) = build_model(spec, self.obstacle_twin)
        self.runtime = NudgedRuntime(self.model, self.patch, self.patch_corners, self.patch_offsets, spec.substeps,
                                     spec.frame_dt/spec.substeps, spec.iterations, spec.linear_iterations,
                                     capture=wp.get_device(spec.device).is_cuda)
        self.count = len(self.rest)
        self.mass = self.model.particle_mass.numpy().astype(float)
        self.free = np.ones(self.count, bool); self.free[self.patch] = False
        self.uv = (self.rest[:, :2]+[spec.width_m/2, spec.height_m/2])/[spec.width_m, spec.height_m]
        self.nominal_stretch = self.model.style3d.tri_aniso_ke.numpy().copy()
        self.nominal_bend = self.model.edge_bending_properties.numpy().copy()
        self.scales = dict(bend=1., shear=1.)
        self.targets = self.to_twin(grips)
        self.place(grips)

    # ---------------------------------------------------------------- frames
    def to_twin(self, points):
        return (np.asarray(points, float)-self.origin) @ self.A.T

    def to_world(self, points):
        return np.asarray(points, float) @ self.A+self.origin

    def vector_to_twin(self, vectors):
        return np.asarray(vectors, float) @ self.A.T

    def vector_to_world(self, vectors):
        return np.asarray(vectors, float) @ self.A

    # ---------------------------------------------------------------- state
    def state_twin(self):
        return (self.runtime.state.particle_q.numpy().astype(float), self.runtime.state.particle_qd.numpy().astype(float))

    def state(self):
        q, v = self.state_twin()
        return self.to_world(q), self.vector_to_world(v)

    def corners(self):
        """Current world positions of the four corner particles."""
        return self.to_world(self.runtime.state.particle_q.numpy()[self.corner_particles].astype(float))

    def set_state(self, q, v=None):
        """Overwrite every particle (world). Grip patches are then put on the current grip targets."""
        q = self.to_twin(q); v = np.zeros_like(q) if v is None else self.vector_to_twin(v)
        if q.shape != (self.count, 3) or not np.isfinite(q).all() or not np.isfinite(v).all():
            raise ValueError('Invalid twin state')
        q[self.patch] = self.targets[self.patch_corners]+self.patch_offsets
        v[self.patch] = 0.
        for state in (self.runtime.state, self.runtime.scratch):
            state.particle_q.assign(q.astype(np.float32)); state.particle_qd.assign(v.astype(np.float32))

    def place(self, grips, sag_m=0.):
        """A bilinear surface between four grips (world), optionally sagging in the middle, at rest."""
        grips = np.asarray(grips, float)
        u, v = self.uv.T
        q = np.column_stack(((1-u)*(1-v), u*(1-v), (1-u)*v, u*v)) @ grips
        q[:, 2] -= sag_m*16*u*(1-u)*v*(1-v)
        self.targets = self.to_twin(grips)
        self.set_state(q)

    def place_from_mesh(self, positions, rows, columns, grips, velocities=None):
        """Start from an estimated mesh (row-major rows x columns vertices, world), lifted through
        material coordinates; the grip patches go to the measured grips."""
        lift = material_lift(self.uv, rows, columns)
        self.targets = self.to_twin(grips)
        self.set_state(lift @ np.asarray(positions, float), None if velocities is None else lift @ np.asarray(velocities, float))

    def within_reach(self, grips, movable=(2, 3), slack=.002):
        """Grips (world) with the `movable` ones pulled in so no pair is farther apart than the
        sheet between them. The sheet barely stretches, so a measured grip that would stretch it
        is measurement error; millimetres of it against the stretch stiffness are tens of newtons
        of false tension. Robot grips (TF) are not moved."""
        grips = np.array(grips, float)
        w, h = self.spec.width_m, self.spec.height_m
        reach = {(0, 1): w, (2, 3): w, (0, 2): h, (1, 3): h, (0, 3): np.hypot(w, h), (1, 2): np.hypot(w, h)}
        for _ in range(4):
            for (i, j), rest in reach.items():
                delta = grips[j]-grips[i]
                length = np.linalg.norm(delta)
                excess = max(0., length-rest*(1+slack))
                moving = [c for c in (i, j) if c in movable]
                if excess and moving:
                    for c in moving:
                        grips[c] += (1. if c == i else -1.)*excess/len(moving)*delta/length
        return grips

    # ---------------------------------------------------------------- stepping
    def step(self, grips):
        """Advance one frame; each grip patch ends the frame on its measured grip (world)."""
        wanted = self.to_twin(grips)
        if wanted.shape != (4, 3) or not np.isfinite(wanted).all():
            raise ValueError('Four finite grip positions are required')
        command = (wanted-self.targets)/self.spec.frame_dt
        speed = np.linalg.norm(command, axis=1, keepdims=True)
        command *= np.minimum(1., self.spec.max_grip_speed_mps/np.maximum(speed, 1e-12))
        self.runtime.advance(self.targets, command)
        self.targets = self.targets+command*self.spec.frame_dt
        return self.vector_to_world(command)

    def set_external(self, forces):
        """External force on every particle (N, world), held until changed."""
        forces = self.vector_to_twin(forces)
        if forces.shape != (self.count, 3) or not np.isfinite(forces).all():
            raise ValueError('Invalid external force')
        forces[~self.free] = 0.
        self.runtime.external.assign(forces.astype(np.float32))

    # ---------------------------------------------------------------- material
    def set_material(self, bend=1., shear=1.):
        """Scale the nominal bending and shear stiffness (the two adapted coefficients).

        Style3D folds stiffness into a matrix it precomputes, so this rebuilds that matrix and
        captures the frame graph again. Anything that copied the model's stiffness, such as the
        CBF's reduced model, must be rebuilt by the caller.
        """
        if not np.isfinite([bend, shear]).all() or min(bend, shear) <= 0:
            raise ValueError('Material scales must be finite and positive')
        stretch = self.nominal_stretch.copy(); stretch[:, 2] *= shear
        edge = self.nominal_bend.copy(); edge[:, 0] *= bend
        self.model.style3d.tri_aniso_ke.assign(stretch)
        self.model.edge_bending_properties.assign(edge)
        self.runtime.solver._precompute(self.model)
        self.runtime.recapture()
        self.scales = dict(bend=float(bend), shear=float(shear))

    # ---------------------------------------------------------------- obstacle
    def obstacle_clearance(self, q=None):
        """Smallest distance (m) from any particle surface to the obstacle box; negative inside."""
        center, half, yaw = self.obstacle_twin
        points = (self.state_twin()[0] if q is None else self.to_twin(q))-center
        c, s = np.cos(yaw), np.sin(yaw)
        local = points @ np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])
        d = np.abs(local)-half
        distance = np.linalg.norm(np.maximum(d, 0.), axis=1)+np.minimum(d.max(axis=1), 0.)
        return float(distance.min()-self.spec.particle_radius)
