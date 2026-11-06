"""Tier A v1: whole-sequence weak-constraint smoothing, fixed nominal material.

Finite-difference sparse least squares works with either backend; it never
differentiates through Style3D or fits an online-estimated fabric trajectory.
"""
from dataclasses import dataclass
import numpy as np
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix, coo_matrix
from fabric_state_estimator.observations import ObservationBuilder


@dataclass
class SmootherSpec:
    position_model_std_m: float = .02
    velocity_model_std_mps: float = .2
    initial_position_std_m: float = .2
    initial_velocity_std_mps: float = .3
    residual_force_std_n: float = .05
    residual_force_walk_n: float = .02
    huber_threshold: float = 3.
    max_nfev: int = 50
    association_rounds: int = 2
    initial_sag_m: float = .08
    max_variables: int = 12000
    max_jacobian_entries: int = 20000000


def robust_residual(residual, threshold):
    absolute = np.abs(residual)
    return np.sign(residual)*np.sqrt(np.where(absolute <= threshold, residual**2,
                                              2*threshold*absolute-threshold**2))


class ReferenceSmoother:
    def __init__(self, mesh, physics, spec=SmootherSpec(), observation_options=None):
        numeric = [getattr(spec, key) for key in spec.__dataclass_fields__ if key != 'initial_sag_m']
        if not np.isfinite(numeric).all() or min(numeric) <= 0 or spec.initial_sag_m < 0:
            raise ValueError('Invalid offline smoother settings')
        self.mesh, self.physics, self.spec = mesh, physics, spec
        self.options = observation_options or {}

    def reconstruct(self, frames):
        if len(frames) < 2 or any(b.time_ns <= a.time_ns for a, b in zip(frames, frames[1:])):
            raise ValueError('Tier A requires at least two capture-ordered frames')
        intervals = np.diff([frame.time_ns for frame in frames])/1e9
        if np.any(intervals > 2.):
            raise ValueError('Split sequences at capture gaps above two seconds')
        t, count = len(frames), self.mesh.count
        block = count*6+3
        # Dense adjacent-frame simulator coupling makes the v1 finite-difference
        # method expensive. Fail before allocating sparse row lists for a fine
        # or long reconstruction beyond the configured memory/problem budget.
        dynamics_entries = (t-1)*(count*6+3)*2*block
        if t*block > self.spec.max_variables or dynamics_entries > self.spec.max_jacobian_entries:
            raise ValueError('Reference problem exceeds numerical budget; use a coarser mesh at import, '
                             'split the clip, or explicitly raise reference.max_variables/max_jacobian_entries')
        states = np.empty((t, block))
        previous = None
        for i, frame in enumerate(frames):
            grips, _ = frame.grip_targets(previous)
            states[i, :count*3] = self.mesh.seed(grips, self.spec.initial_sag_m).ravel()
            states[i, count*3:count*6] = 0.
            states[i, count*6:] = 0.
            previous = grips
        initial = states[0].copy()
        def unpack(x):
            state = x.reshape(t, block)
            return state[:, :count*3].reshape(t, count, 3), state[:, count*3:count*6].reshape(t, count, 3), state[:, count*6:]
        rounds = []
        for _ in range(self.spec.association_rounds):
            linear_q, _, _ = unpack(states.ravel())
            builder = ObservationBuilder(self.mesh, **self.options)
            factors = [builder.build(q, frame) for q, frame in zip(linear_q, frames)]
            if not any(len(f) > 3*len(frame.grips) for f, frame in zip(factors, frames)):
                raise ValueError('No associated surface/force observations; improve initialization or calibration')
            def measured(x):
                q, v, loads = unpack(x)
                return [np.array([(factor.predict(q[i:i+1], v[i:i+1], loads[i:i+1], self.physics)[0]-factor.value)
                                   /np.sqrt(factor.variance) for factor in group]) for i, group in enumerate(factors)]
            def residual(x):
                q, v, loads = unpack(x)
                pieces = [robust_residual(r, self.spec.huber_threshold) for r in measured(x)]
                pieces += [(q[0].ravel()-initial[:count*3])/self.spec.initial_position_std_m,
                           v[0].ravel()/self.spec.initial_velocity_std_mps,
                           loads.ravel()/self.spec.residual_force_std_n]
                for i, dt in enumerate(intervals):
                    prediction_q, prediction_v = self.physics.advance(q[i:i+1], v[i:i+1], dt,
                        q[i+1:i+2, self.mesh.corners], loads[i:i+1])
                    pieces.extend(((q[i+1]-prediction_q[0]).ravel()/self.spec.position_model_std_m,
                                   (v[i+1]-prediction_v[0]).ravel()/self.spec.velocity_model_std_mps,
                                   (loads[i+1]-loads[i])/self.spec.residual_force_walk_n))
                return np.concatenate(pieces)
            # Actual simulator rollouts can propagate influence across a whole
            # frame: use conservative dense frame blocks, not a false local Jacobian.
            rows = sum(map(len, factors))+count*6+t*3+(t-1)*(count*6+3)
            estimated_entries = dynamics_entries+(sum(map(len, factors))+count*6)*block+t*9
            if estimated_entries > self.spec.max_jacobian_entries:
                raise ValueError('Reference measurement Jacobian exceeds numerical budget; reduce mesh, clip or factor counts')
            sparsity = lil_matrix((rows, t*block), dtype=int)
            row = 0
            for i, group in enumerate(factors):
                sparsity[row:row+len(group), i*block:(i+1)*block] = 1; row += len(group)
            sparsity[row:row+count*6, :block] = 1; row += count*6
            for i in range(t):
                sparsity[row:row+3, i*block+count*6:(i+1)*block] = 1; row += 3
            for i in range(t-1):
                sparsity[row:row+count*6+3, i*block:(i+2)*block] = 1; row += count*6+3
            before = np.concatenate(measured(states.ravel()))
            structure = sparsity.tocsc()
            def numerical_jacobian(x):
                # Absolute steps survive Style3D float32 quantization, including
                # exactly zero initial velocities and loads. Two frame colours
                # are independent because factors span at most adjacent frames.
                baseline = residual(x)
                row_ids, column_ids, entries = [], [], []
                epsilon = np.r_[np.full(count*3, 1e-4), np.full(count*3, 1e-3), np.full(3, 1e-3)]
                for parity in range(2):
                    for local in range(block):
                        columns = np.arange(parity, t, 2)*block+local
                        if not len(columns):
                            continue
                        shifted = x.copy(); shifted[columns] += epsilon[local]
                        difference = (residual(shifted)-baseline)/epsilon[local]
                        for column in columns:
                            rows_for_column = structure.indices[structure.indptr[column]:structure.indptr[column+1]]
                            row_ids.extend(rows_for_column); column_ids.extend([column]*len(rows_for_column))
                            entries.extend(difference[rows_for_column])
                return coo_matrix((entries, (row_ids, column_ids)), shape=structure.shape).tocsr()
            solution = least_squares(residual, states.ravel(), jac=numerical_jacobian,
                max_nfev=self.spec.max_nfev, x_scale='jac', ftol=1e-5, xtol=1e-5, gtol=1e-5)
            states = solution.x.reshape(t, block)
            after = np.concatenate(measured(solution.x))
            rounds.append(dict(success=bool(solution.success), message=solution.message, nfev=solution.nfev,
                cost=float(solution.cost), measurement_normalized_rms_before=float(np.sqrt(np.mean(before**2))),
                measurement_normalized_rms_after=float(np.sqrt(np.mean(after**2))), measurement_factors=len(after)))
        q, v, loads = unpack(states.ravel())
        counts = {}
        for group in factors:
            for factor in group:
                counts[factor.kind] = counts.get(factor.kind, 0)+1
        return dict(times_ns=np.array([f.time_ns for f in frames], np.int64), positions=q, velocities=v,
                    residual_forces=loads, report={'kind': 'reference_candidate', 'material_parameters': 'fixed_nominal',
                    'covariance_available': False, 'factor_counts': counts,
                    'tangential_cues_available': any(counts.get(kind, 0) for kind in ('track', 'boundary', 'uv_prior')),
                    'rounds': rounds, 'converged': rounds[-1]['success']})
