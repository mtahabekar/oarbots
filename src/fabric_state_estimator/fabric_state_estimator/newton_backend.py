"""Optional Newton 1.0 Style3D adapter, with independent GPU-parallel worlds.

Self/obstacle collision is disabled: the installed Style3D collision handler
does not filter ensemble worlds. Hanging/slack sheets without self-contact only.
No simulator source tree or Isaac imports are required.
"""
import inspect
import numpy as np
from types import FunctionType, SimpleNamespace
from scipy.sparse import coo_matrix
from .physics import validate_inputs

try:
    import warp as wp
    import newton
    from newton.solvers import style3d
except ImportError as error:
    raise ImportError('newton_style3d requires Newton and Warp in the ROS Python environment; see docs/fabric-estimation/hardware.md') from error


@wp.kernel
def pin_corners_linear(corners: wp.array[int], starts: wp.array[wp.vec3], velocity: wp.array[wp.vec3],
                       count: int, elapsed: wp.array[float], q: wp.array[wp.vec3], v: wp.array[wp.vec3]):
    # Prescribed linear corner path: starts + (elapsed/duration)*(targets-starts), with
    # velocity = (targets-starts)/duration precomputed; all per-call data on the device so one
    # captured step can be replayed for any duration.
    n, c = wp.tid()
    i = n*4+c
    q[n*count+corners[c]] = starts[i]+elapsed[0]*velocity[i]
    v[n*count+corners[c]] = velocity[i]


@wp.kernel
def advance_clock(elapsed: wp.array[float], dt: float):
    elapsed[0] = elapsed[0]+dt


@wp.kernel
def uniform_load(loads: wp.array[wp.vec3], weights: wp.array[float], count: int, forces: wp.array[wp.vec3]):
    i = wp.tid()
    forces[i] = weights[i % count]*loads[i // count]


class NewtonPhysics:
    # Captured single steps kept per ensemble size, keyed by step length. Durations that are
    # multiples of the 60 Hz clock all use max_step_sec itself, so one entry usually serves all.
    MAX_GRAPHS = 8

    def __init__(self, mesh, spec):
        self.mesh, self.spec, self.worlds = mesh, spec, {}

    def warm_up(self, size):
        q = np.repeat(self.mesh.rest[None], size, axis=0)
        q[:, :, 2] += 1.
        self.advance(q, np.zeros_like(q), self.spec.max_step_sec, q[:, self.mesh.corners])

    def _world(self, size):
        if size in self.worlds:
            return self.worlds[size]
        base = newton.ModelBuilder(up_axis=newton.Axis.Z)
        newton.solvers.SolverStyle3D.register_custom_attributes(base)
        add_mesh = style3d.add_cloth_mesh
        if not hasattr(np, 'atan2'):
            # Newton 1.0's builder uses NumPy 2's atan2 spelling; ROS Jazzy's
            # OpenCV/cv_bridge use NumPy 1.26. Give only this builder helper a
            # compatibility namespace. Neither NumPy nor installed files change.
            namespace = dict(add_mesh.__globals__)
            helper = namespace['_compute_edge_bending_data']
            helper_namespace = dict(helper.__globals__)
            helper_namespace['np'] = SimpleNamespace(**{**vars(np), 'atan2': np.arctan2, 'pow': np.power})
            compatible = FunctionType(helper.__code__, helper_namespace, helper.__name__, helper.__defaults__)
            compatible.__kwdefaults__ = helper.__kwdefaults__
            namespace['_compute_edge_bending_data'] = compatible
            add_mesh = FunctionType(add_mesh.__code__, namespace, add_mesh.__name__, add_mesh.__defaults__)
            add_mesh.__kwdefaults__ = style3d.add_cloth_mesh.__kwdefaults__
        add_mesh(base, pos=wp.vec3(), rot=wp.quat_identity(), vel=wp.vec3(),
            vertices=self.mesh.rest.tolist(), indices=self.mesh.triangles.ravel().tolist(),
            panel_verts=self.mesh.rest[:, :2].tolist(), density=self.mesh.spec.areal_density_kg_m2,
            tri_aniso_ke=wp.vec3(*self.spec.newton_stretch), edge_aniso_ke=wp.vec3(*self.spec.newton_bend))
        for corner in self.mesh.corners:
            base.particle_flags[int(corner)] &= ~int(newton.ParticleFlags.ACTIVE)
        builder = newton.ModelBuilder(up_axis=newton.Axis.Z)
        newton.solvers.SolverStyle3D.register_custom_attributes(builder)
        for _ in range(size):
            builder.add_world(base)
        model = builder.finalize(device=self.spec.device)
        model.set_gravity(self.spec.gravity)
        if not np.allclose(model.particle_mass.numpy().reshape(size, -1)[0], self.mesh.mass, rtol=1e-4):
            raise ValueError('Newton vertex ordering/mass differs from persistent material mesh')
        solver = newton.solvers.SolverStyle3D(model, iterations=self.spec.iterations)
        # Newton 1.0 needs an explicit private _precompute(builder); 1.6 runs
        # _precompute(model) inside the constructor and must not be repeated.
        parameters = list(inspect.signature(solver._precompute).parameters)
        if parameters and parameters[0] == 'builder':
            solver._precompute(builder)
        solver.collision = None
        record = dict(model=model, solver=solver, a=model.state(), b=model.state(), control=model.control(),
            contacts=model.contacts(), corners=wp.array(self.mesh.corners, dtype=int, device=model.device),
            weights=wp.array(self.mesh.mass/self.mesh.mass.sum(), dtype=float, device=model.device),
            starts=wp.zeros(size*4, dtype=wp.vec3, device=model.device),
            velocity=wp.zeros(size*4, dtype=wp.vec3, device=model.device),
            load=wp.zeros(size, dtype=wp.vec3, device=model.device),
            elapsed=wp.zeros(1, dtype=float, device=model.device), graphs={})
        self.worlds[size] = record
        if not hasattr(self, 'coeff'):
            faces = model.tri_indices.numpy()[:len(self.mesh.triangles)]
            if not np.array_equal(faces, self.mesh.triangles):
                raise ValueError('Newton changed material triangle indices')
            rest = model.tri_poses.numpy()[:len(faces)]
            self.coeff = np.concatenate((-rest.sum(axis=1)[:, None], rest), axis=1)
            self.areas = model.tri_areas.numpy()[:len(faces)]
            self.stretch = model.style3d.tri_aniso_ke.numpy()[:len(faces)]
            edges = model.edge_indices.numpy()
            valid = np.all((edges >= 0) & (edges < self.mesh.count), axis=1)
            edges = edges[valid]
            cot = model.style3d.edge_bending_cot.numpy()[valid]
            weights = np.column_stack((-cot[:, 0]-cot[:, 2], -cot[:, 1]-cot[:, 3], cot[:, 2]+cot[:, 3], cot[:, 0]+cot[:, 1]))
            weights *= (model.edge_bending_properties.numpy()[valid, 0]/model.style3d.edge_rest_area.numpy()[valid])[:, None]
            blocks = weights[:, :, None]*weights[:, None, :]
            rows = np.broadcast_to(edges[:, :, None], blocks.shape)
            cols = np.broadcast_to(edges[:, None, :], blocks.shape)
            self.bend = coo_matrix((blocks.ravel(), (rows.ravel(), cols.ravel())), shape=(self.mesh.count, self.mesh.count)).tocsr()
        return record

    def internal_forces(self, q):
        self._world(len(q))
        positions = q[:, self.mesh.triangles]
        fu = np.einsum('ntix,ti->ntx', positions, self.coeff[:, :, 0])
        fv = np.einsum('ntix,ti->ntx', positions, self.coeff[:, :, 1])
        lu, lv = np.linalg.norm(fu, axis=-1), np.linalg.norm(fv, axis=-1)
        u, v = fu/np.maximum(lu[..., None], 1e-9), fv/np.maximum(lv[..., None], 1e-9)
        a, b = self.coeff[:, :, 0], self.coeff[:, :, 1]
        su = self.stretch[None, :, 0, None]*(lu-1)[..., None]*a
        sv = self.stretch[None, :, 1, None]*(lv-1)[..., None]*b
        shear = self.stretch[None, :, 2]*np.sum(u*v, axis=-1)
        force = -self.areas[None, :, None, None]*(su[..., None]*u[:, :, None] + sv[..., None]*v[:, :, None]
            + shear[..., None, None]*(b[None, :, :, None]*u[:, :, None]+a[None, :, :, None]*v[:, :, None]))
        result = np.zeros((self.mesh.count, len(q), 3))
        np.add.at(result, self.mesh.triangles.ravel(), force.reshape(len(q), -1, 3).transpose(1, 0, 2))
        return result.transpose(1, 0, 2)-np.stack([self.bend @ member for member in q])

    def reactions(self, q, loads):
        force = self.internal_forces(q)+self.mesh.mass[None, :, None]*np.asarray(self.spec.gravity)
        force += loads[:, None]*(self.mesh.mass/self.mesh.mass.sum())[None, :, None]
        return -force[:, self.mesh.corners]

    def _step(self, world, size, dt, a, b):
        """One solver step from state a into b; the elapsed time advances on the device."""
        model = world['model']
        a.clear_forces()
        # Write AFTER force clearing. These are residual external loads,
        # never the measured grip reaction applied a second time.
        wp.launch(uniform_load, model.particle_count, inputs=[world['load'], world['weights'], self.mesh.count, a.particle_f],
                  device=model.device)
        wp.launch(advance_clock, 1, inputs=[world['elapsed'], dt], device=model.device)
        args = [world['corners'], world['starts'], world['velocity'], self.mesh.count, world['elapsed']]
        wp.launch(pin_corners_linear, (size, 4), inputs=args+[a.particle_q, a.particle_qd], device=model.device)
        world['solver'].step(a, b, world['control'], world['contacts'], dt)
        wp.launch(pin_corners_linear, (size, 4), inputs=args+[b.particle_q, b.particle_qd], device=model.device)

    def _step_graphs(self, world, size, dt):
        """Captured a->b and b->a steps of length dt, or None (no CUDA / disabled / not warmed up)."""
        if not self.spec.cuda_graph or not wp.get_device(world['model'].device).is_cuda:
            return None
        key = round(dt, 9)
        if key in world['graphs']:
            return world['graphs'][key]
        if len(world['graphs']) >= self.MAX_GRAPHS:
            world['graphs'].pop(next(iter(world['graphs'])))
        graphs = {}
        for name, (src, dst) in (('ab', (world['a'], world['b'])), ('ba', (world['b'], world['a']))):
            with wp.ScopedCapture(device=world['model'].device) as capture:  # records only
                self._step(world, size, dt, src, dst)
            graphs[name] = capture.graph
        world['graphs'][key] = graphs
        return graphs

    def advance(self, q, v, duration, grips, loads=None):
        q, v, grips, loads = validate_inputs(self.mesh, q, v, duration, grips, loads)
        if duration == 0:
            return q, v
        size = len(q)
        world = self._world(size); model = world['model']
        world['a'].particle_q.assign(q.reshape(-1, 3).astype(np.float32))
        world['a'].particle_qd.assign(v.reshape(-1, 3).astype(np.float32))
        starts = q[:, self.mesh.corners].reshape(-1, 3)
        world['starts'].assign(starts.astype(np.float32))
        world['velocity'].assign(((grips.reshape(-1, 3)-starts)/duration).astype(np.float32))
        world['load'].assign(loads.astype(np.float32))
        world['elapsed'].zero_()
        steps = int(np.ceil(duration/self.spec.max_step_sec)); dt = duration/steps
        a, b = world['a'], world['b']
        b.assign(a)
        # Each solver step is dozens of tiny kernel launches; launching them one by one from
        # Python costs ~4 ms per step whatever the ensemble or mesh size. A captured step,
        # replayed `steps` times (ping-ponging the two state buffers as below), runs the same
        # kernels in a fraction of that. The first step of a new length runs uncaptured, which
        # also loads its kernels and solver buffers before the capture.
        graphs = world['graphs'].get(round(dt, 9))
        for step in range(steps):
            if graphs is None and step == 1:
                graphs = self._step_graphs(world, size, dt)
            if graphs is None:
                self._step(world, size, dt, a, b)
            else:
                wp.capture_launch(graphs['ab' if a is world['a'] else 'ba'])
            a, b = b, a
        if a is not world['a']:
            world['a'].assign(a)
        a = world['a']
        out_q, out_v = a.particle_q.numpy().reshape(q.shape).astype(float), a.particle_qd.numpy().reshape(v.shape).astype(float)
        if not np.isfinite(out_q).all() or not np.isfinite(out_v).all():
            raise FloatingPointError('Newton produced nonfinite states')
        return out_q, out_v
