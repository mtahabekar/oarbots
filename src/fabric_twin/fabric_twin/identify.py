"""Identify the twin from a recording: many parameter hypotheses run as parallel worlds, scored on raw depth.

Why this replaces the warm-up bank (2026-11-20 analysis, docs report "Fabric twin: RA-L readiness ..."):

* The bank started every fit case from the corrected twin's state and ran it for 2 s. That state was made
  partly by the nudging force and by the wrong twin, and a 2 s case cannot forget it (start-state bias,
  found in Isaac and again on hardware). Here every hypothesis is *free-run*: it settles between the first
  measured grips and is then driven by the measured grips alone over the whole window (output-error /
  simulation-error identification). Nothing from the twin, the nudger or the estimator enters.
* The bank fitted bending and shear only. On the real sheet those move the twin by under 15 mm while the
  twin sits 40-100 mm above the depth; the error comes from in-plane compliance, the human grip boundary
  (an uncalibrated glove tag-to-pinch offset) and solver convergence. The parameter set here is chosen per
  run, and `sensitivity()` says which of them the recording can inform before anything is fitted.
* The bank's cost clipped residuals at 30 mm, so with 75-80 % of points beyond it the cost was flat. Here a
  residual r enters as s*tanh(r/s) (s = 10 cm by default): linear for small errors, bounded for points that
  are not the sheet, and smooth.
* Batching, and its limit. A 29 x 29 sheet does not fill the GPU, so one Newton model holding N worlds costs
  about the wall time of one sheet (`SheetBatch`). But Style3D solves each step with ONE preconditioned
  conjugate-gradient system over all particles, so its step sizes are global: with the twin's 3 linear
  iterations, a world 10x softer in stretch moved its neighbour by 50 mm (28 x 28, 8 x 6) or 7 mm (20 x 20,
  32 x 10); only ~30 linear iterations removed it (0.14 mm, 7x slower). The same holds for the estimator's
  ensemble. So `Forward` runs every hypothesis in its own single-world model by default (`independent=True`,
  exact, sequential). Batched evaluation is kept for worlds that share all stiffness values and for speed
  studies; per-world PCG reductions (or a solver without global reductions, such as VBD) would make it exact.

The fit uses an ensemble smoother with multiple data assimilation (ES-MDA; Emerick and Reynolds 2013): a
derivative-free, Kalman-type update of the whole parameter ensemble from the cross-covariance between
parameters and predicted residuals, repeated with inflated noise. Parameters move continuously, so the
answer does not have to be one of the starting hypotheses, and the final spread is a usable uncertainty.

The worlds use the twin's own mesh, grip patches and solver settings (TwinSpec), so identified values
are effective values for those numerics. Self-collision is off by default: with Style3D's handler on, two
worlds with identical parameters drifted 8 cm apart within 7 s on the 2026-11-20 warm-up (contact handling
makes the slack sheet chaotic), which would swamp an ensemble's parameter signal. The ground and obstacles
are absent: use warm-up windows or stretches where the sheet touches nothing. No ROS.
"""
from dataclasses import dataclass, replace
import time

import numpy as np
import warp as wp
import newton
from newton.solvers import style3d
from scipy.spatial import cKDTree

from .cloth import inverse_bilinear, twin_frame
from .vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.newton_runtime import move_boundary  # noqa: E402


# ------------------------------------------------------------------------------------------- parameters
@dataclass(frozen=True)
class Parameter:
    """One identified quantity. `log10` parameters are log10 scales of a nominal stiffness; the others are
    offsets in metres."""
    name: str
    prior_sd: float
    lower: float
    upper: float
    log10: bool = True
    description: str = ''


PARAMETERS = {
    'stretch': Parameter('stretch', 1., -3., 1., True, 'log10 scale of the weft/warp stretch stiffness'),
    'shear': Parameter('shear', 1., -3., 1., True, 'log10 scale of the in-plane shear stiffness'),
    'bend': Parameter('bend', .7, -2., 2., True, 'log10 scale of the bending stiffness'),
    'hand_down': Parameter('hand_down', .04, -.12, .12, False, 'human grips lower than measured (m)'),
    'hand_in': Parameter('hand_in', .04, -.12, .12, False,
                         'human grips closer to the robots than measured, horizontally (m)'),
    'robot_down': Parameter('robot_down', .02, -.06, .06, False, 'robot grips lower than TF (m)'),
    'hand_inset': Parameter('hand_inset', .02, 0., .12, False, "material inset of both hands' pinch from the corner (m)"),
    'robot_inset': Parameter('robot_inset', .02, 0., .08, False, "material inset of both robots' pinch from the corner (m)"),
}
# Per-corner grip points (2026-11-20), each corner fitted on its own: where the pinch really is relative to its
# measurement (robot fabric_grip TF, glove estimate) and how far in from the sheet's corner the fabric is pinched.
# Corners: 0 blue (robot), 1 silver (robot), 2 right (the person's right hand, Blue's side), 3 left (Silver's side).
# down: pinch lower than measured; in: pinch toward the opposite edge (robots toward the hands, hands toward the
# robots), horizontal; side: along the robot edge, toward Silver (+) ; inset: along both edges from the corner.
CORNERS = ('blue', 'silver', 'right', 'left')
for _c in CORNERS:
    PARAMETERS[f'down_{_c}'] = Parameter(f'down_{_c}', .03, -.12, .12, False, f'{_c} pinch lower than measured (m)')
    PARAMETERS[f'in_{_c}'] = Parameter(f'in_{_c}', .03, -.12, .12, False, f'{_c} pinch toward the opposite edge (m)')
    PARAMETERS[f'side_{_c}'] = Parameter(f'side_{_c}', .03, -.12, .12, False, f'{_c} pinch along the robot edge toward Silver (m)')
    PARAMETERS[f'inset_{_c}'] = Parameter(f'inset_{_c}', .02, 0., .12, False, f'{_c} pinch inset from the sheet corner (m)')
INSETS = tuple(f'inset_{c}' for c in CORNERS)


def parameter_table(names, values):
    """Values (N, p) as a list of dicts with readable units (scales as multipliers, offsets and insets in m)."""
    rows = []
    for row in np.atleast_2d(values):
        rows.append({name: (float(10**v) if PARAMETERS[name].log10 else float(v)) for name, v in zip(names, row)})
    return rows


def grip_uv(spec, insets=(0., 0., 0., 0.)):
    """Material coordinates (4, 2) of the four pinch points, corner c `insets[c]` m in from its corner along both
    edges (order blue, silver, right, left)."""
    i = np.asarray(insets, float)/spec.width_m
    j = np.asarray(insets, float)/spec.height_m
    return np.array([[i[0], j[0]], [1-i[1], j[1]], [i[2], 1-j[2]], [1-i[3], 1-j[3]]])


def grip_spans(spec, uv):
    """Rest distance (m) of the sheet between every grip pair, from their material coordinates."""
    scale = np.array([spec.width_m, spec.height_m])
    return {(i, j): float(np.linalg.norm((uv[i]-uv[j])*scale)) for i in range(4) for j in range(i+1, 4)}


def grip_weights(uv, points_uv):
    """Bilinear weights (P, 4) that put material point `points_uv` between the four grip points `uv`, exactly at
    the grips and extrapolating over the flaps outside them (cloth.inverse_bilinear)."""
    return inverse_bilinear(uv, points_uv)


# ------------------------------------------------------------------------------------------- batched sheets
class SheetBatch:
    """N independent copies of the twin sheet in one Newton model, each with its own material scales and grip
    offsets. Same mesh, grip patches, Style3D settings and frame rate as `cloth.TwinCloth`."""

    def __init__(self, spec, worlds, grips, device=None, spacing_m=4., self_collision=False, pins_uv=None):
        """`pins_uv` (4, 2): pin one particle per corner, the node nearest each material point, instead of the
        spec's square patches (point grips; see `grip_uv`)."""
        spec.validate()
        self.spec, self.worlds = spec, int(worlds)
        # World w lives at shift[w] in the twin frame, far from the others, so Style3D's self-collision
        # handler (one BVH over all particles) never pairs particles of different worlds.
        self.shift = np.zeros((self.worlds, 1, 3)); self.shift[:, 0, 0] = spacing_m*np.arange(self.worlds)
        self.device = device or spec.device
        self.A, self.origin = twin_frame(grips)
        w, h, nx, ny = spec.width_m, spec.height_m, spec.cells_x, spec.cells_y
        base = newton.ModelBuilder(up_axis=newton.Axis.Z)
        newton.solvers.SolverStyle3D.register_custom_attributes(base)
        style3d.add_cloth_grid(
            base, pos=wp.vec3(-w/2, -h/2, 1.), rot=wp.quat_identity(), vel=wp.vec3(), dim_x=nx, dim_y=ny,
            cell_x=w/nx, cell_y=h/ny, mass=spec.areal_density_kg_m2*w*h/((nx+1)*(ny+1)),
            tri_aniso_ke=wp.vec3(*spec.stretch), tri_ka=spec.tri_ka, tri_kd=spec.tri_kd,
            edge_aniso_ke=wp.vec3(*spec.bend), edge_kd=spec.edge_kd, particle_radius=spec.particle_radius)
        rest = np.asarray(base.particle_q, dtype=float)
        self.count = len(rest)
        tips = np.array([[-w/2, -h/2], [w/2, -h/2], [-w/2, h/2], [w/2, h/2]])
        patch, corner, offsets = [], [], []
        material = (rest[:, :2]+[w/2, h/2])/[w, h]
        if pins_uv is not None:
            self.pins_uv = np.asarray(pins_uv, float)
            for c, target in enumerate(self.pins_uv):
                index = int(np.argmin(np.linalg.norm((material-target)*[w, h], axis=1)))
                patch.append(index); corner.append(c); offsets.append(np.zeros(3))
                base.particle_flags[index] &= ~int(newton.ParticleFlags.ACTIVE)
            self.pins_uv = material[np.array(patch)]       # the nodes actually pinned
        else:
            self.pins_uv = np.array([[0., 0.], [1., 0.], [0., 1.], [1., 1.]])
            for c, tip in enumerate(tips):
                for index in np.flatnonzero(np.all(np.abs(rest[:, :2]-tip) <= spec.patch_m+1e-5, axis=1)):
                    patch.append(int(index)); corner.append(c); offsets.append(rest[index]-rest[np.argmin(np.linalg.norm(rest[:, :2]-tip, axis=1))])
                    base.particle_flags[int(index)] &= ~int(newton.ParticleFlags.ACTIVE)
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        newton.solvers.SolverStyle3D.register_custom_attributes(builder)
        for _ in range(self.worlds):
            builder.add_world(base)
        self.model = model = builder.finalize(device=self.device)
        model.set_gravity((0., 0., -9.81))
        if model.particle_count != self.worlds*self.count:
            raise ValueError('Unexpected particle layout in the batched model')
        self.solver = newton.solvers.SolverStyle3D(model, iterations=spec.iterations, linear_iterations=spec.linear_iterations)
        if not self_collision:
            self.solver.collision = None
        self.state, self.scratch = model.state(), model.state()
        self.control, self.contacts = model.control(), model.contacts()
        patch, corner, offsets = np.array(patch), np.array(corner), np.array(offsets, float)
        k = len(patch)
        self.patch_local, self.patch_corner = patch, corner
        self.indices = wp.array((patch[None]+self.count*np.arange(self.worlds)[:, None]).ravel(), dtype=int, device=self.device)
        self.slots = wp.array((corner[None]+4*np.arange(self.worlds)[:, None]).ravel(), dtype=int, device=self.device)
        self.offsets = wp.array(np.tile(offsets, (self.worlds, 1)).astype(np.float32), dtype=wp.vec3, device=self.device)
        self.patch_offsets = offsets
        self.starts = wp.zeros(4*self.worlds, dtype=wp.vec3, device=self.device)
        self.commands = wp.zeros(4*self.worlds, dtype=wp.vec3, device=self.device)
        self.uv = (rest[:, :2]+[w/2, h/2])/[w, h]
        self.grid = np.lexsort((self.uv[:, 0], self.uv[:, 1])).reshape(ny+1, nx+1)
        self.free = np.ones(self.count, bool); self.free[patch] = False
        self.nominal_stretch = model.style3d.tri_aniso_ke.numpy().copy()
        self.nominal_bend = model.edge_bending_properties.numpy().copy()
        self.dt = spec.frame_dt/spec.substeps
        self.targets = np.zeros((self.worlds, 4, 3))
        self.graph = None
        self.patch_count = k

    # frames
    def to_twin(self, points):
        return (np.asarray(points, float)-self.origin) @ self.A.T

    def to_world(self, points):
        return np.asarray(points, float) @ self.A+self.origin

    # materials
    def set_materials(self, stretch=None, shear=None, bend=None):
        """Per-world multipliers (N,) on the nominal stiffness; None keeps nominal. Rebuilds Style3D's
        precomputed system and recaptures the frame graph."""
        n = self.worlds
        ones = np.ones(n)
        stretch, shear, bend = (ones if x is None else np.broadcast_to(np.asarray(x, float), (n,)) for x in (stretch, shear, bend))
        if not all(np.isfinite(x).all() and (x > 0).all() for x in (stretch, shear, bend)):
            raise ValueError('Material scales must be finite and positive')
        tri = self.nominal_stretch.reshape(n, -1, 3).copy()
        tri[:, :, :2] *= stretch[:, None, None]; tri[:, :, 2] *= shear[:, None]
        edge = self.nominal_bend.reshape(n, -1, self.nominal_bend.shape[-1]).copy()
        edge[:, :, 0] *= bend[:, None]
        self.model.style3d.tri_aniso_ke.assign(tri.reshape(self.nominal_stretch.shape))
        self.model.edge_bending_properties.assign(edge.reshape(self.nominal_bend.shape))
        self.solver._precompute(self.model)
        self.graph = None

    # stepping
    def _pin(self, state, elapsed):
        wp.launch(move_boundary, len(self.indices), inputs=[self.indices, self.slots, self.offsets, self.starts,
                  self.commands, elapsed, state.particle_q, state.particle_qd], device=self.device)

    def _advance(self):
        a, b = self.state, self.scratch
        for k in range(self.spec.substeps):
            a.clear_forces()
            elapsed = (k+1)*self.dt
            self._pin(a, elapsed)
            self.solver.step(a, b, self.control, self.contacts, self.dt)
            self._pin(b, elapsed)
            a, b = b, a
        if a is not self.state:
            self.state.assign(a)

    def _capture(self):
        if not wp.get_device(self.device).is_cuda:
            return
        saved = self.model.state(); saved.assign(self.state)
        self._advance()
        self.state.assign(saved); self.scratch.assign(saved)
        with wp.ScopedCapture(device=self.device) as scope:
            self._advance()
        self.graph = scope.graph
        self.state.assign(saved); self.scratch.assign(saved)

    def place(self, grips, sag_m=.03):
        """Bilinear sheet between each world's four grips (N, 4, 3) or shared (4, 3), world frame, at rest."""
        grips = np.broadcast_to(np.asarray(grips, float), (self.worlds, 4, 3))
        u, v = self.uv.T
        weights = grip_weights(self.pins_uv, self.uv)
        q = np.einsum('pk,nkx->npx', weights, grips)
        q[:, :, 2] -= sag_m*16*u*(1-u)*v*(1-v)
        q = self.to_twin(q)
        self.targets = self.to_twin(grips)
        q[:, self.patch_local] = self.targets[:, self.patch_corner]+self.patch_offsets[None]
        q = q+self.shift
        for state in (self.state, self.scratch):
            state.particle_q.assign(q.reshape(-1, 3).astype(np.float32))
            state.particle_qd.zero_()

    def step(self, grips):
        """One frame; each world's patches end on its grips (N, 4, 3) world, speed-limited as in TwinCloth."""
        wanted = self.to_twin(np.broadcast_to(np.asarray(grips, float), (self.worlds, 4, 3)))
        command = (wanted-self.targets)/self.spec.frame_dt
        speed = np.linalg.norm(command, axis=2, keepdims=True)
        command *= np.minimum(1., self.spec.max_grip_speed_mps/np.maximum(speed, 1e-12))
        self.starts.assign((self.targets+self.shift).reshape(-1, 3).astype(np.float32))
        self.commands.assign(command.reshape(-1, 3).astype(np.float32))
        if self.graph is None and wp.get_device(self.device).is_cuda:
            self._capture()
        if self.graph is not None:
            wp.capture_launch(self.graph)
        else:
            self._advance()
        self.targets = self.targets+command*self.spec.frame_dt

    def positions(self):
        """(N, count, 3) world positions."""
        q = self.state.particle_q.numpy().astype(float).reshape(self.worlds, self.count, 3)
        return self.to_world(q-self.shift)


def within_reach(grips, width, height, movable=(2, 3), slack=.002, rounds=4, spans=None):
    """Vectorized `TwinCloth.within_reach` over worlds: (N, 4, 3) grips with the movable ones pulled in so no
    pair is farther apart than the sheet between them (`spans` {(i, j): m} for inset grips)."""
    grips = np.array(grips, float)
    reach = spans or {(0, 1): width, (2, 3): width, (0, 2): height, (1, 3): height,
                      (0, 3): np.hypot(width, height), (1, 2): np.hypot(width, height)}
    for _ in range(rounds):
        for (i, j), rest in reach.items():
            moving = [c for c in (i, j) if c in movable]
            if not moving:
                continue
            delta = grips[:, j]-grips[:, i]
            length = np.linalg.norm(delta, axis=1)
            excess = np.maximum(0., length-rest*(1+slack))
            if not excess.any():
                continue
            unit = delta/np.maximum(length, 1e-12)[:, None]
            for c in moving:
                grips[:, c] += (1. if c == i else -1.)*(excess/len(moving))[:, None]*unit
    return grips


# ------------------------------------------------------------------------------------------- data
@dataclass
class Window:
    """Measured grips (T, 4, 3) at frame times (T,) ns, and the depth frames scored inside the window:
    `depth_index` (D,) into the grip frames and `points` list of (P_d, 3) world points."""
    frame_ns: np.ndarray
    grips: np.ndarray
    depth_index: np.ndarray
    points: list

    @property
    def observations(self):
        return int(sum(len(p) for p in self.points))


def make_window(corners_ns, corners, depth_ns, depth_points, start_ns, end_ns, depth_every=2, points_per_frame=64,
                skip_sec=1., seed=0):
    """Cut a window from a recording. Grip frames are the twin's 60 Hz corner records; every `depth_every`-th
    depth frame after `skip_sec` is scored with up to `points_per_frame` fixed random points."""
    sel = np.flatnonzero((corners_ns >= start_ns) & (corners_ns <= end_ns))
    if len(sel) < 60:
        raise ValueError('Window holds fewer than 60 grip frames')
    frame_ns, grips = corners_ns[sel], np.asarray(corners[sel], float)
    random = np.random.default_rng(seed)
    index, points = [], []
    chosen = [j for j, t in enumerate(depth_ns) if start_ns+skip_sec*1e9 <= t <= end_ns][::max(1, int(depth_every))]
    for j in chosen:
        k = int(np.searchsorted(frame_ns, depth_ns[j], side='right'))-1
        if k < 0 or depth_ns[j]-frame_ns[k] > .05e9:
            continue
        p = np.asarray(depth_points[j], float)
        if len(p) < 20:
            continue
        if len(p) > points_per_frame:
            p = p[random.choice(len(p), points_per_frame, replace=False)]
        index.append(k); points.append(p)
    if not points:
        raise ValueError('No depth frames inside the window')
    return Window(frame_ns, grips, np.array(index), points)


# ------------------------------------------------------------------------------------------- forward model
def signed_residuals(q, grid, points, gate_m=.15):
    """Signed distance (m) of each point to the sheet along the upward-oriented surface normal at its
    nearest particle (positive = point above the sheet); NaN where the nearest particle is farther than
    `gate_m` (not the sheet, for this hypothesis)."""
    surface = q[grid]
    normal = np.cross(np.gradient(surface, axis=1), np.gradient(surface, axis=0))
    normal /= np.maximum(np.linalg.norm(normal, axis=2, keepdims=True), 1e-12)
    flat, normals = surface.reshape(-1, 3), normal.reshape(-1, 3)
    if normals[:, 2].mean() < 0:
        normals = -normals
    distance, index = cKDTree(flat).query(points)
    residual = np.einsum('ij,ij->i', points-flat[index], normals[index])
    return np.where(distance <= gate_m, residual, np.nan)


class Forward:
    """Maps parameter vectors (N, p) to transformed residuals (N, M) on one window, by free-running N worlds."""

    def __init__(self, spec, window, names, scale_m=.1, settle_frames=120, sag_m=.03, worlds=None, independent=True,
                 point_grips=False):
        unknown = [n for n in names if n not in PARAMETERS]
        if unknown:
            raise ValueError(f'Unknown parameters {unknown}; choose from {sorted(PARAMETERS)}')
        self.spec, self.window, self.names = spec, window, list(names)
        self.scale, self.settle, self.sag = float(scale_m), int(settle_frames), float(sag_m)
        self.batch, self.runs, self.seconds = None, 0, 0.
        self.worlds, self.independent = worlds, bool(independent)
        # point_grips: one pinned particle per corner (at the corner, or inset) instead of the spec's square patch.
        self.point_grips = bool(point_grips)
        self.models = {}                      # (robot inset, hand inset) -> single-world SheetBatch
        self.fixed = {}                       # parameters held at a value (set_fixed), e.g. insets or offsets
        g0 = window.grips[0]
        across = .5*(g0[2]+g0[3])-.5*(g0[0]+g0[1]); across[2] = 0.
        self.inward = -across/np.linalg.norm(across)   # horizontal, from the hands toward the robots
        edge = g0[1]-g0[0]; edge[2] = 0.
        self.side = edge/np.linalg.norm(edge)          # horizontal, along the robot edge from Blue to Silver

    def set_fixed(self, **values):
        """Hold parameters that are not in `names` at given values (metres for offsets and insets)."""
        unknown = [n for n in values if n not in PARAMETERS]
        if unknown:
            raise ValueError(f'Unknown parameters {unknown}')
        self.fixed.update({k: float(v) for k, v in values.items()})

    def _values(self, theta):
        theta = np.atleast_2d(np.asarray(theta, float))
        values = {k: np.full(len(theta), v) for k, v in self.fixed.items()}
        values.update(dict(zip(self.names, theta.T)))
        return values

    def _insets(self, theta_row):
        """Per-corner insets (blue, silver, right, left): a corner's own value, else the shared robot/hand one."""
        values = self._values(theta_row[None])
        shared = ('robot_inset', 'robot_inset', 'hand_inset', 'hand_inset')
        return tuple(float(values[k][0]) if k in values else float(values[g][0]) if g in values else 0.
                     for k, g in zip(INSETS, shared))

    def _batch(self, n, insets=None):
        insets = tuple(round(float(x), 4) for x in (insets or (0., 0., 0., 0.)))
        point = self.point_grips or any(insets)
        if point and n == 1:
            if insets not in self.models:
                self.models[insets] = SheetBatch(self.spec, 1, self.window.grips[0], pins_uv=grip_uv(self.spec, insets))
            return self.models[insets]
        if self.batch is None or self.batch.worlds != n:
            self.batch = SheetBatch(self.spec, n, self.window.grips[0], pins_uv=grip_uv(self.spec, insets) if point else None)
        return self.batch

    def grips(self, theta, k, spans=None):
        """Grips (N, 4, 3) for frame k with each world's offsets applied."""
        g = np.repeat(self.window.grips[k][None], len(np.atleast_2d(theta)), axis=0)
        values = self._values(theta)
        down = {'hand_down': (2, 3), 'hand_down_right': (2,), 'hand_down_left': (3,), 'robot_down': (0, 1),
                'robot_down_blue': (0,), 'robot_down_silver': (1,)}
        for name, corners in down.items():
            if name in values:
                g[:, list(corners), 2] -= values[name][:, None]
        inward = {'hand_in': ((2, 3), 1.), 'hand_in_right': ((2,), 1.), 'hand_in_left': ((3,), 1.), 'robot_in': ((0, 1), -1.)}
        for name, (corners, sign) in inward.items():
            if name in values:
                g[:, list(corners)] += sign*values[name][:, None, None]*self.inward[None, None]
        for c, name in enumerate(CORNERS):
            if f'down_{name}' in values:
                g[:, c, 2] -= values[f'down_{name}']
            if f'in_{name}' in values:   # robots move toward the hands (-inward), hands toward the robots
                g[:, c] += (1. if c >= 2 else -1.)*values[f'in_{name}'][:, None]*self.inward[None]
            if f'side_{name}' in values:
                g[:, c] += values[f'side_{name}'][:, None]*self.side[None]
        return within_reach(g, self.spec.width_m, self.spec.height_m, spans=spans)

    def raw(self, theta):
        """Untransformed signed residuals (N, M) in metres; NaN for points beyond the gate of that world.
        Each row runs in its own model unless `independent` is False (see the module note on coupling)."""
        theta = np.atleast_2d(np.asarray(theta, float))
        if self.independent:
            return np.vstack([self._run(self._batch(1, self._insets(row)), row[None]) for row in theta])
        if any(k in self.names for k in INSETS+('robot_inset', 'hand_inset')):
            raise ValueError('Inset hypotheses need independent runs (one model per inset)')
        return self._run(self._batch(self.worlds or len(theta), self._insets(theta[0])), theta)

    def _run(self, batch, theta):
        n = len(theta)
        if n < batch.worlds:   # pad with copies of the first row
            theta = np.vstack([theta, np.repeat(theta[:1], batch.worlds-n, axis=0)])
        values = self._values(theta)
        batch.set_materials(**{key: 10**values[key] for key in ('stretch', 'shear', 'bend') if key in values})
        spans = grip_spans(self.spec, batch.pins_uv)
        begun = time.perf_counter()
        first = self.grips(theta, 0, spans)
        batch.place(first, self.sag)
        for _ in range(self.settle):
            batch.step(first)
        window = self.window
        out = [[] for _ in range(len(theta))]
        d = 0
        for k in range(len(window.frame_ns)):
            batch.step(self.grips(theta, k, spans))
            while d < len(window.depth_index) and window.depth_index[d] == k:
                q = batch.positions()
                for w in range(len(theta)):
                    out[w].append(signed_residuals(q[w], batch.grid, window.points[d]))
                d += 1
        self.runs += 1
        self.seconds += time.perf_counter()-begun
        return np.array([np.concatenate(o) for o in out])[:n]

    def transform(self, raw):
        """Bounded residual s*tanh(r/s) (s = 10 cm: the hardware twin starts 4-10 cm off, and a smaller s
        saturated most points); a point beyond a world's gate counts as fully saturated (s)."""
        return np.where(np.isfinite(raw), self.scale*np.tanh(np.nan_to_num(raw)/self.scale), self.scale)

    def __call__(self, theta):
        return self.transform(self.raw(theta))


def summary(residuals):
    """Readable statistics of raw residual rows (N, M), in mm."""
    rows = []
    for x in np.atleast_2d(residuals):
        near = np.isfinite(x)
        y = x[near]
        rows.append(dict(median_mm=float(np.median(y)*1e3), mad_mm=float(np.median(np.abs(y-np.median(y)))*1e3),
                         rms_tanh50_mm=float(np.sqrt(np.mean(np.where(near, .05*np.tanh(np.nan_to_num(x)/.05), .05)**2))*1e3),
                         p95_abs_mm=float(np.percentile(np.abs(y), 95)*1e3), over_30mm=float(np.mean(np.abs(y) > .03)),
                         near_sheet=float(near.mean())))
    return rows


# ------------------------------------------------------------------------------------------- analyses
def sensitivity(forward, theta0=None, steps=None, noise_m=.01):
    """Local identifiability from one batched run: the nominal world plus one world per parameter nudged by
    its step. Returns the Jacobian's column norms (RMS change of the transformed residual, mm per step),
    the Cramér-Rao standard deviations with `noise_m` independent noise per point, and the correlations.
    A parameter whose CR std is larger than its prior std is not informed by this window."""
    names = forward.names
    p = len(names)
    theta0 = np.zeros(p) if theta0 is None else np.asarray(theta0, float)
    steps = np.array([.3 if PARAMETERS[n].log10 else .02 for n in names]) if steps is None else np.asarray(steps, float)
    theta = np.vstack([theta0, theta0+np.diag(steps)])
    raw = forward.raw(theta)
    y = forward.transform(raw)
    J = (y[1:]-y[0]).T/steps                     # (M, p) per unit of each parameter
    effect = np.sqrt(np.mean((y[1:]-y[0])**2, axis=1))*1e3
    fisher = J.T @ J/noise_m**2
    try:
        covariance = np.linalg.inv(fisher)
    except np.linalg.LinAlgError:
        covariance = np.linalg.pinv(fisher)
    sd = np.sqrt(np.maximum(np.diag(covariance), 0.))
    corr = covariance/np.maximum(np.outer(sd, sd), 1e-300)
    singular = np.linalg.svd(J/np.array([PARAMETERS[n].prior_sd for n in names]), compute_uv=False)
    return dict(names=names, step=steps.tolist(), effect_mm_per_step=effect.tolist(), cramer_rao_sd=sd.tolist(),
                prior_sd=[PARAMETERS[n].prior_sd for n in names],
                informed=[bool(s < .5*PARAMETERS[n].prior_sd) for s, n in zip(sd, names)],
                correlation=np.round(corr, 3).tolist(), singular_values_prior_scaled=singular.tolist(),
                nominal=summary(raw[:1])[0], nudged=summary(raw[1:]))


def es_mda(forward, members=16, alphas=(9.333, 7., 4., 2.), noise_m=.08, theta0=None, seed=0, progress=None):
    """Ensemble smoother with multiple data assimilation over the parameters.

    The observation is zero residual for every scored depth point. `noise_m` is the noise per point on the
    transformed residual, and it must cover model error, not only depth noise: the points of one frame
    share their errors (calibration, grip offsets, solver), so treating 7000 points as independent at 1 cm
    collapsed the ensemble in one iteration on hardware. About (per-point error) x sqrt(points per frame),
    e.g. 1 cm x 8 = 8 cm, is a reasonable start. Each iteration runs all members in one batch. Returns the iterations' ensembles,
    the posterior mean and std (transformed units), and the residual summary of the posterior mean."""
    if abs(sum(1/a for a in alphas)-1) > 1e-2:
        raise ValueError('ES-MDA inflation factors must satisfy sum(1/alpha) = 1')
    names = forward.names
    prior_sd = np.array([PARAMETERS[n].prior_sd for n in names])
    lower = np.array([PARAMETERS[n].lower for n in names]); upper = np.array([PARAMETERS[n].upper for n in names])
    random = np.random.default_rng(seed)
    center = np.zeros(len(names)) if theta0 is None else np.asarray(theta0, float)
    draws = random.normal(size=(members, len(names)))
    draws -= draws.mean(axis=0)
    draws /= draws.std(axis=0, ddof=1)
    theta = np.clip(center+draws*prior_sd, lower, upper)
    history = []
    for it, alpha in enumerate(alphas):
        y = forward(theta)                                       # (N, M)
        cost = np.sqrt(np.mean(y**2, axis=1))
        history.append(dict(iteration=it, alpha=alpha, mean=theta.mean(axis=0).tolist(), sd=theta.std(axis=0, ddof=1).tolist(),
                            cost_mm_median=float(np.median(cost)*1e3), cost_mm_best=float(cost.min()*1e3),
                            best=theta[int(np.argmin(cost))].tolist()))
        if progress:
            progress(history[-1])
        a_t = (theta-theta.mean(axis=0)).T/np.sqrt(members-1)    # (p, N)
        a_y = (y-y.mean(axis=0)).T/np.sqrt(members-1)            # (M, N)
        r = alpha*noise_m**2
        inner = a_y.T @ a_y/r+np.eye(members)
        perturbed = np.sqrt(alpha)*noise_m*random.normal(size=y.shape)
        innovation = (0.+perturbed-y).T                          # (M, N)
        gain_times = np.linalg.solve(inner, a_y.T @ innovation/r)  # (N, N)
        theta = np.clip(theta+(a_t @ gain_times).T, lower, upper)
    mean, sd = theta.mean(axis=0), theta.std(axis=0, ddof=1)
    final = forward.raw(np.vstack([mean, theta]))
    member_cost = np.sqrt(np.mean(forward.transform(final[1:])**2, axis=1))
    return dict(names=names, mean=mean.tolist(), sd=sd.tolist(), prior_sd=prior_sd.tolist(),
                posterior=theta.tolist(), readable_mean=parameter_table(names, mean)[0],
                mean_summary=summary(final[:1])[0], member_cost_mm=(member_cost*1e3).tolist(), history=history,
                runs=forward.runs, seconds=forward.seconds)
