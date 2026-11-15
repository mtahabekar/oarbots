"""Tier B: local ensemble-transform assimilation without simulator gradients."""
from dataclasses import dataclass
import numpy as np
from .observations import ObservationBuilder

ROBOT, HUMAN = (0, 1), (2, 3)  # material corners held by the robots and by the human hands


def centred_normal(random, members, shape):
    """Standard normal draws with exactly zero ensemble mean and unit ensemble covariance.

    Raw draws from a small ensemble have a nonzero mean and too little spread (0.35-0.47
    sigma offset and 0.85 sigma spread at 16 members for the old fixed seed), which biased
    every prescribed grip for the life of the node.
    """
    draws = random.normal(size=(members,)+tuple(shape))
    draws -= draws.mean(axis=0)
    flat = draws.reshape(members, -1, shape[-1])
    for k in range(flat.shape[1]):
        covariance = flat[:, k].T @ flat[:, k]/(members-1)
        flat[:, k] = flat[:, k] @ np.linalg.inv(np.linalg.cholesky(covariance)).T
    return flat.reshape(draws.shape)


def relax_to_prior_spread(prior, posterior, alpha):
    """Relaxation to prior spread (Whitaker and Hamill 2012): the analysis spread of every
    variable is moved a fraction alpha of the way back to its prior spread."""
    if alpha <= 0:
        return posterior
    before, after = prior.std(axis=0, ddof=1), posterior.std(axis=0, ddof=1)
    factor = np.where(before > after, 1+alpha*(before-after)/np.maximum(after, 1e-12), 1.)
    mean = posterior.mean(axis=0)
    return mean+(posterior-mean)*np.minimum(factor, 1e3)


def gaspari_cohn(distance, radius):
    """Compact material-geodesic taper, zero beyond twice radius."""
    x = np.asarray(distance)/radius
    result = np.zeros_like(x)
    a = x <= 1; b = (x > 1) & (x < 2)
    z = x[a]; result[a] = 1-5*z*z/3+5*z**3/8+z**4/2-z**5/4
    z = x[b]; result[b] = 4-5*z+5*z*z/3+5*z**3/8-z**4/2+z**5/12-2/(3*z)
    return np.clip(result, 0, 1)


def transform_weights(predictions, values, variances, localization, inflation=1.02, huber=3.):
    n = len(predictions)
    anomalies = predictions-predictions.mean(axis=0)
    innovation = values-predictions.mean(axis=0)
    normalized = np.abs(innovation)/np.sqrt(variances+np.var(predictions, axis=0, ddof=1))
    precision = localization/variances*np.minimum(1., huber/np.maximum(normalized, 1e-12))
    information = (n-1)/inflation*np.eye(n)+(anomalies*precision) @ anomalies.T
    eigenvalues, vectors = np.linalg.eigh(information)
    inverse = (vectors/np.maximum(eigenvalues, 1e-10)) @ vectors.T
    mean_weights = inverse @ (anomalies @ (precision*innovation))
    transform = (vectors*np.sqrt((n-1)/np.maximum(eigenvalues, 1e-10))) @ vectors.T
    return mean_weights, transform


def localized_update(mesh, q, v, loads, factors, physics, radius, inflation=1.02, diagnostics=None, huber=3.):
    if not factors:
        return q.copy(), v.copy(), loads.copy(), np.nan
    predictions = np.column_stack([factor.predict(q, v, loads, physics) for factor in factors])
    values, variance = np.array([f.value for f in factors]), np.array([f.variance for f in factors])
    if not np.isfinite(predictions).all():
        raise FloatingPointError('Nonfinite predicted measurements')
    if diagnostics is not None:
        normal = np.array([f.kind == 'normal' for f in factors])
        diagnostics['depth_factors'] = int(normal.sum())
        diagnostics['depth_cameras'] = sorted({f.view.camera_id for f in factors if f.kind == 'normal' and f.view is not None})
        diagnostics['prior_depth_p95_m'] = (float(np.percentile(np.abs(values[normal]-predictions.mean(axis=0)[normal]), 95))
                                               if normal.any() else float('inf'))
    anchors = np.array([f.vertices[np.argmax(f.weights)] for f in factors])
    # Same per-vertex transform as transform_weights(), batched over vertices:
    # a Python loop over vertices cost ~85 ms at 17x17/N=32 per analysis.
    taper = gaspari_cohn(mesh.geodesic[:, anchors], radius)
    active = np.flatnonzero(taper.any(axis=1))
    out_q, out_v = q.copy(), v.copy()
    if len(active):
        n = len(predictions)
        anomalies = predictions-predictions.mean(axis=0)
        innovation = values-predictions.mean(axis=0)
        normalized = np.abs(innovation)/np.sqrt(variance+np.var(predictions, axis=0, ddof=1))
        precision = taper[active]/variance*np.minimum(1., huber/np.maximum(normalized, 1e-12))
        information = (n-1)/inflation*np.eye(n) + (anomalies[None]*precision[:, None, :]) @ anomalies.T
        eigenvalues, vectors = np.linalg.eigh(information)
        eigenvalues = np.maximum(eigenvalues, 1e-10)
        inverse = (vectors/eigenvalues[:, None, :]) @ vectors.transpose(0, 2, 1)
        mean_w = np.einsum('aij,ja->ai', inverse, anomalies @ (precision*innovation).T)
        transform = (vectors*np.sqrt((n-1)/eigenvalues)[:, None, :]) @ vectors.transpose(0, 2, 1)
        state = np.concatenate((q[:, active], v[:, active]), axis=2).transpose(1, 0, 2)  # (A,n,6)
        mean = state.mean(axis=1, keepdims=True); state_anomalies = state-mean
        result = mean + np.einsum('ai,aid->ad', mean_w, state_anomalies)[:, None] + transform @ state_anomalies
        result = result.transpose(1, 0, 2)
        out_q[:, active], out_v[:, active] = result[..., :3], result[..., 3:]
    mean_w, transform = transform_weights(predictions, values, variance, np.ones(len(factors)), inflation, huber)
    load_anomaly = loads-loads.mean(axis=0)
    out_loads = loads.mean(axis=0)+mean_w @ load_anomaly+transform @ load_anomaly
    # Marginal innovation score, not a calibrated chi-square consistency test.
    score = float(np.mean((values-predictions.mean(axis=0))**2/(variance+predictions.var(axis=0, ddof=1))))
    return out_q, out_v, out_loads, score


@dataclass
class ObserverSpec:
    ensemble_size: int = 32
    localization_m: float = .3
    inflation: float = 1.02
    initial_sag_m: float = .08
    initial_position_std_m: float = .04
    initial_velocity_std_mps: float = .08
    residual_force_std_n: float = .05
    residual_force_walk_n_sqrt_sec: float = .01
    velocity_noise_mps_sqrt_sec: float = .02
    stale_after_sec: float = .5
    max_gap_sec: float = .5
    forecast_limit_sec: float = .6
    max_edge_strain: float = .4
    covariance_floor_m: float = .005
    seed: int = 7
    # Fraction of the spread removed by an analysis that is put back (0 = off).
    relaxation: float = 0.
    huber_threshold: float = 3.
    hand_unobserved_after_sec: float = 1.
    # A grip cannot be farther from another grip than the sheet between them allows.
    corner_reach_strain: float = .02


class EnsembleObserver:
    def __init__(self, mesh, physics, spec=ObserverSpec(), builder=None):
        values = [getattr(spec, name) for name in spec.__dataclass_fields__ if name not in ('seed', 'ensemble_size')]
        if (spec.ensemble_size < 4 or not np.isfinite(values).all() or min(values) < 0
                or min(spec.localization_m, spec.inflation, spec.stale_after_sec, spec.max_gap_sec) <= 0):
            raise ValueError('Invalid ensemble observer configuration')
        self.mesh, self.physics, self.spec = mesh, physics, spec
        self.builder = builder or ObservationBuilder(mesh)
        self.random = np.random.default_rng(spec.seed)
        self.q = self.v = self.loads = None
        self.time_ns = None
        self.last_seen = np.full(mesh.count, -1, np.int64)
        self.corner_noise = centred_normal(self.random, spec.ensemble_size, (4, 3))
        self.boundary = {}
        self.status = {'degraded': True, 'reason': 'uninitialized'}
        w, h = mesh.spec.width_m, mesh.spec.height_m
        self.reach = {(0, 2): h, (1, 3): h, (0, 3): np.hypot(w, h), (1, 2): np.hypot(w, h), (2, 3): w}

    def check_corners(self, grips):
        """Reject physically impossible or twisted corner assignments.

        Corner identities come from configuration (which robot/glove holds
        which material corner). A swapped pair gives a bow-tie prior that
        folds the sheet through itself and cannot be repaired by assimilation.
        """
        w, h = self.mesh.spec.width_m, self.mesh.spec.height_m
        stretch = 1+self.spec.max_edge_strain
        for (i, j), rest in (((0, 1), w), ((2, 3), w), ((0, 2), h), ((1, 3), h), ((0, 3), np.hypot(w, h)), ((1, 2), np.hypot(w, h))):
            if np.linalg.norm(grips[i]-grips[j]) > rest*stretch:
                raise ValueError(f'Grips {i} and {j} are farther apart than the material allows; '
                                 'check corner assignment, grip frames and sheet dimensions')
        # Bilinear-patch normal at each corner; a twisted ordering flips one.
        normals = np.array([np.cross(grips[1]-grips[0], grips[2]-grips[0]), np.cross(grips[1]-grips[0], grips[3]-grips[1]),
                            np.cross(grips[3]-grips[2], grips[2]-grips[0]), np.cross(grips[3]-grips[2], grips[3]-grips[1])])
        lengths = np.linalg.norm(normals, axis=1)
        if np.all(lengths > 1e-3*w*h):
            unit = normals/lengths[:, None]
            if np.any(unit @ unit.sum(axis=0) <= 0):
                raise ValueError('Corner assignment is twisted (bow-tie); check which robot/hand holds which material corner')

    def initialize(self, frame):
        grips, covariance = frame.grip_targets()
        self.check_corners(grips)
        n = self.spec.ensemble_size
        self.q = np.repeat(self.mesh.seed(grips, self.spec.initial_sag_m)[None], n, axis=0)
        # Smooth spatial modes, not independent white noise on every mesh node.
        u, v = self.mesh.uv.T
        modes = np.array([np.sin(np.pi*u)*np.sin(np.pi*v), np.sin(2*np.pi*u)*np.sin(np.pi*v),
                          np.sin(np.pi*u)*np.sin(2*np.pi*v)])
        self.q += np.einsum('nkx,kv->nvx', self.random.normal(size=(n, 3, 3)), modes)*self.spec.initial_position_std_m
        self.v = self.random.normal(size=self.q.shape)*self.spec.initial_velocity_std_mps
        self.loads = self.random.normal(size=(n, 3))*self.spec.residual_force_std_n
        self.time_ns = frame.time_ns
        self.q[:, self.mesh.corners] = self._sample_grips(grips, covariance)
        self.v[:, self.mesh.corners] = 0.
        self.builder.bindings.clear()
        self.last_seen[:] = -1

    def _sample_grips(self, grips, covariance):
        return grips[None]+np.einsum('cij,ncj->nci', np.linalg.cholesky(covariance), self.corner_noise)

    def _boundary_targets(self, frame):
        """Each member's four prescribed corner targets at the frame time.

        A measured grip is the measurement plus the member's fixed, centred offset scaled by
        the grip covariance. For a robot that is its calibration uncertainty. For a human hand
        the node supplies the hand filter's estimate at this time, interpolated between glove
        detections or carried past the last one with a covariance that grows by the filter's
        own model. A grip with no measurement holds its place: the analysis still runs.
        """
        corners = self.mesh.corners
        targets = self.q[:, corners].copy()
        grips, covariance = frame.grip_targets(self.q.mean(axis=0)[corners])
        measured = {g.corner for g in frame.grips}
        for corner in measured:
            targets[:, corner] = grips[corner]+self.corner_noise[:, corner] @ np.linalg.cholesky(covariance[corner]).T
        # A grip cannot be farther from another grip than the sheet between them allows. Members
        # sampled beyond that would overstretch the mesh, so the spread of a human grip is scaled
        # down, about its unchanged mean, until every member fits. (Pulling only the offending
        # members inward moved the ensemble mean by centimetres whenever the grip was uncertain.)
        limit = 1+self.spec.corner_reach_strain
        for corner in HUMAN:
            if corner not in measured:
                continue
            offsets = targets[:, corner]-grips[corner]
            for scale in np.linspace(1., 0., 11):
                targets[:, corner] = grips[corner]+scale*offsets
                if all(np.all(np.linalg.norm(targets[:, j]-targets[:, i], axis=1) <= rest*limit)
                       for (i, j), rest in self.reach.items() if corner in (i, j)):
                    break
        return targets

    def _project(self, dt):
        # Bound overextension; compressed edges may represent slack and are not
        # expanded to their rest lengths by an arbitrary geometry projection.
        # Sequential (Gauss-Seidel) passes: measured faster and better
        # converged at 17x17 than a vectorized Jacobi variant.
        # With no edge over the bound every correction below is exactly zero, so the
        # passes change nothing; check that in one vectorized step first (the usual case:
        # the loop alone cost ~17 ms per analysis at 15x15).
        a, b = self.mesh.edges.T
        if not np.any(np.linalg.norm(self.q[:, b]-self.q[:, a], axis=2) > self.mesh.lengths*(1+self.spec.max_edge_strain)):
            return
        before = self.q.copy()
        pinned = set(self.mesh.corners.tolist())
        for _ in range(3):
            for edge, rest in zip(self.mesh.edges, self.mesh.lengths):
                a, b = edge; delta = self.q[:, b]-self.q[:, a]
                lengths = np.linalg.norm(delta, axis=1)
                correction = np.maximum(0., lengths-rest*(1+self.spec.max_edge_strain))[:, None]*delta/np.maximum(lengths[:, None], 1e-9)
                free = int(a not in pinned)+int(b not in pinned)
                if free:
                    if a not in pinned:
                        self.q[:, a] += correction/free
                    if b not in pinned:
                        self.q[:, b] -= correction/free
        self.v += (self.q-before)/max(dt, .02)  # keep q/v corrections coupled

    def assimilate(self, frame):
        if self.time_ns is not None and frame.time_ns <= self.time_ns:
            raise ValueError('Observer rejects duplicate or backward capture times')
        reinitialized = self.q is None or (frame.time_ns-self.time_ns)/1e9 > self.spec.max_gap_sec
        if reinitialized:
            self.initialize(frame)
            dt = .02
        else:
            dt = (frame.time_ns-self.time_ns)/1e9
            self.loads += self.random.normal(size=self.loads.shape)*self.spec.residual_force_walk_n_sqrt_sec*np.sqrt(dt)
            self.q, self.v = self.physics.advance(self.q, self.v, dt, self._boundary_targets(frame), self.loads)
            corner_velocity = self.v[:, self.mesh.corners].copy()
            self.v += self.random.normal(size=self.v.shape)*self.spec.velocity_noise_mps_sqrt_sec*np.sqrt(dt)
            self.v[:, self.mesh.corners] = corner_velocity
        # Grip uncertainty was already sampled into prescribed boundary motion.
        # Do not assimilate the same measurement again as a second likelihood.
        factors = [factor for factor in self.builder.build(self.q.mean(axis=0), frame) if factor.kind != 'grip']
        prior_diagnostics = {}
        prior_q, prior_v = self.q, self.v
        corners = self.mesh.corners
        self.q, self.v, self.loads, score = localized_update(self.mesh, self.q, self.v, self.loads, factors,
            self.physics, self.spec.localization_m, self.spec.inflation, prior_diagnostics, self.spec.huber_threshold)
        self.q = relax_to_prior_spread(prior_q, self.q, self.spec.relaxation)
        self.v = relax_to_prior_spread(prior_v, self.v, self.spec.relaxation)
        # Corners are boundary conditions for the analysis too. Camera evidence must not move
        # them: one depth factor used to move a robot corner by 5 cm, and on replayed
        # teleoperation the camera update dragged the human corners 4-15 cm (p50-p95) away from
        # glove estimates that were within 2-7 mm of the true grip, because the glove hides the
        # sheet's corner. The grips are measured far better than the cameras can place them.
        self.q[:, corners], self.v[:, corners] = prior_q[:, corners], prior_v[:, corners]
        self._project(dt)
        self.time_ns = frame.time_ns
        hand_age = [next((g.age_sec for g in frame.grips if g.corner == corner), None) for corner in HUMAN]
        self.boundary = dict(time_ns=frame.time_ns, reinitialized=reinitialized,
            grips=[dict(corner=g.corner, position=g.position.tolist(), age_sec=g.age_sec,
                        stddev_m=float(np.sqrt(np.trace(g.covariance)/3))) for g in frame.grips],
            hand_age_sec=hand_age,
            corner_mean=self.q[:, corners].mean(axis=0).tolist(),
            corner_std=self.q[:, corners].std(axis=0, ddof=1).mean(axis=1).tolist())
        for factor in factors:
            if factor.kind != 'force':
                self.last_seen[factor.vertices[factor.weights > .05]] = frame.time_ns
        for grip in frame.grips:
            self.last_seen[self.mesh.corners[grip.corner]] = frame.time_ns
        visual = sum(f.kind in ('normal', 'boundary', 'track', 'uv_prior') for f in factors)
        anomaly = np.concatenate((self.q, self.v), axis=2).reshape(len(self.q), -1)
        anomaly -= anomaly.mean(axis=0)
        eigenvalues = np.linalg.eigvalsh(anomaly @ anomaly.T)
        rank = int(np.count_nonzero(eigenvalues > max(1e-12, eigenvalues[-1]*1e-6)))
        max_strain = float(np.max(np.linalg.norm(self.q[:, self.mesh.edges[:, 1]]-self.q[:, self.mesh.edges[:, 0]], axis=2)/self.mesh.lengths)-1)
        violation = max_strain > self.spec.max_edge_strain+.02
        unobserved = any(age is None or age > self.spec.hand_unobserved_after_sec for age in hand_age)
        self.status = {'degraded': visual == 0 or not np.isfinite(score) or score > 25 or rank < 3 or violation or unobserved,
            'hand_age_sec': hand_age,
            'reason': 'no visual factors' if visual == 0 else 'human grip unobserved' if unobserved else 'edge strain bound violated' if violation else 'low ensemble spread' if rank < 3 else 'large innovation' if score > 25 else 'observations assimilated',
            'factors': len(factors), 'visual_factors': visual, 'marginal_innovation_score': score,
            'reinitialized': reinitialized, 'physics_backend': self.physics.spec.backend, 'effective_rank': rank, 'max_edge_strain': max_strain, **prior_diagnostics}
        if not np.isfinite(self.q).all() or not np.isfinite(self.v).all():
            raise FloatingPointError('Nonfinite assimilation state')
        return self.snapshot()

    def snapshot(self, q=None, v=None, state_time_ns=None):
        if self.q is None:
            raise ValueError('Observer has no initialized state')
        q, v = self.q if q is None else q, self.v if v is None else v
        state_time_ns = self.time_ns if state_time_ns is None else state_time_ns
        state = np.concatenate((q, v), axis=2)
        anomalies = state-state.mean(axis=0)
        covariance = np.einsum('nvi,nvj->vij', anomalies, anomalies)/(len(q)-1)
        covariance[:, :3, :3] += np.eye(3)*self.spec.covariance_floor_m**2
        age = np.where(self.last_seen >= 0, (state_time_ns-self.last_seen)/1e9, -1.)
        visibility = np.where(self.last_seen < 0, 0, np.where(age <= self.spec.stale_after_sec, 1, 2)).astype(np.uint8)
        return dict(time_ns=state_time_ns, measurement_ns=self.time_ns, q=q.mean(axis=0), v=v.mean(axis=0),
            covariance=covariance, visibility=visibility, age=age, residual_force=self.loads.mean(axis=0),
            residual_covariance=np.cov(self.loads.T), predicted=state_time_ns != self.time_ns, **self.status)

    def rollout(self, horizons, grip_velocity=None, start=None):
        """Conditional constant-grip-velocity rollout from the analysis.

        Horizons share one chained trajectory, so cost follows the longest
        horizon rather than their sum. ``start=(elapsed_sec, q, v)`` resumes a
        rollout of this same analysis (grip motion is linear, so resuming is
        the same trajectory). Never mutates the analysis.
        Returns (snapshots in input order, (elapsed_sec, q, v) at the end).
        """
        velocity = np.zeros((4, 3)) if grip_velocity is None else np.asarray(grip_velocity, float).reshape(4, 3)
        elapsed, q, v = (0., self.q, self.v) if start is None else start
        base = self.q[:, self.mesh.corners]
        result = [None]*len(horizons)
        for index in np.argsort(horizons, kind='stable'):
            horizon = float(horizons[index])
            if not np.isfinite(horizon) or not 0 <= horizon <= self.spec.forecast_limit_sec:
                raise ValueError('Forecast horizon exceeds configured prediction bound')
            if horizon > elapsed:
                q, v = self.physics.advance(q, v, horizon-elapsed, base+velocity[None]*horizon, self.loads)
                elapsed = horizon
            result[index] = self.snapshot(q, v, self.time_ns+int(round(horizon*1e9)))
        return result, (elapsed, q, v)

    def forecast(self, horizons, grip_velocity=None):
        """Conditional constant-grip-velocity rollouts, never mutate analysis."""
        return self.rollout(horizons, grip_velocity)[0]
