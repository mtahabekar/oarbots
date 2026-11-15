"""Keep the twin on the estimated sheet with a bounded corrective force.

At each estimator analysis the estimated mesh is lifted to the twin's particles through material
coordinates and compared with the twin state stored for that same measurement time. The
difference becomes a spring-damper acceleration on every free particle, weighted by how well
the estimator knows that part of the sheet, and limited in size. It is a force, not a position
overwrite: a jump would break constraints, inject energy and show up in the CBF as prediction
error. The total corrective force is reported; a large, persistent value means the twin's
coefficients or grips are wrong, which is what the parameter adaptation is there to remove.
"""
from dataclasses import dataclass
import numpy as np

from .cloth import material_lift

VISIBILITY_WEIGHT = np.array([.5, 1., .25])  # inferred, observed, stale (FabricState.visibility)


@dataclass(frozen=True)
class FeedbackSpec:
    stiffness_per_s2: float = 25.     # acceleration per metre of position error
    damping_per_s: float = 6.         # acceleration per m/s of velocity error
    # Integral term: a slowly learned residual force field. A proportional force alone leaves
    # the twin wherever its own (possibly wrong) coefficients balance it: with bending x3 and
    # shear x0.4 wrong on a sheet with 3 cm of slack, that was 5-7 cm away. The integral supplies
    # whatever steady force holds the measured shape, so the remaining error goes to zero.
    integral_per_s3: float = 150.     # acceleration per metre of error, per second
    # The estimate has its own error (20-30 mm p95 on replayed teleoperation), part of it
    # systematic, and the twin cannot always reach it. Without a bound and a leak the integral
    # wound up to more than the sheet's weight against a correctly parameterized twin.
    integral_limit_mps2: float = 5.
    integral_leak_sec: float = 5.
    accel_limit_mps2: float = 20.
    trusted_sigma_m: float = .02      # an estimate this uncertain gets half weight
    max_error_m: float = .25          # beyond this the twin is re-placed, not nudged


class Nudger:
    def __init__(self, cloth, rows, columns, spec=FeedbackSpec()):
        values = [spec.stiffness_per_s2, spec.damping_per_s, spec.accel_limit_mps2, spec.trusted_sigma_m, spec.max_error_m,
                  spec.integral_per_s3, spec.integral_limit_mps2, spec.integral_leak_sec]
        if not np.isfinite(values).all() or min(values) < 0 or spec.trusted_sigma_m <= 0:
            raise ValueError('Invalid feedback settings')
        self.cloth, self.spec = cloth, spec
        self.residual = np.zeros((rows*columns, 3))  # learned residual acceleration at the estimator's vertices
        self.lift = material_lift(cloth.uv, rows, columns)
        # The twin sampled at the estimator's vertices. The error is formed there and then
        # lifted: lifting the coarse estimate itself would flatten every curve between its
        # vertices (12 cm apart at 8 x 8) and pull the twin's finer shape toward that.
        u, v = np.meshgrid(np.linspace(0, 1, columns), np.linspace(0, 1, rows))
        nx, ny = cloth.spec.cells_x+1, cloth.spec.cells_y+1
        grid = np.lexsort((cloth.uv[:, 0], cloth.uv[:, 1]))  # particles in row-major material order
        if not np.allclose(cloth.uv[grid].reshape(ny, nx, 2)[0, :, 0], np.linspace(0, 1, nx), atol=1e-6):
            raise ValueError('Twin particles are not a regular material grid')
        order = np.empty(cloth.count, int); order[grid] = np.arange(cloth.count)
        sample = material_lift(np.column_stack((u.ravel(), v.ravel())), ny, nx).tocsc()
        self.sample = sample[:, order].tocsr()

    def reset(self):
        self.residual[:] = 0.

    def force(self, twin_q, twin_v, estimate_q, estimate_v=None, covariance=None, visibility=None, dt=1/15):
        """Corrective force (N, world) and diagnostics, from one estimate and the twin state at
        the estimate's measurement time (all world frame). `dt`: time since the previous estimate."""
        spec = self.spec
        coarse_error = np.asarray(estimate_q, float)-self.sample @ twin_q
        weight = np.ones(len(coarse_error))
        if covariance is not None:
            variance = np.trace(np.asarray(covariance, float)[:, :3, :3], axis1=1, axis2=2)/3
            weight = spec.trusted_sigma_m**2/(spec.trusted_sigma_m**2+variance)
        if visibility is not None:
            weight = weight*VISIBILITY_WEIGHT[np.asarray(visibility, int)]
        self.residual *= np.exp(-min(dt, .5)/spec.integral_leak_sec)
        self.residual += spec.integral_per_s3*min(dt, .5)*weight[:, None]*coarse_error
        norm = np.linalg.norm(self.residual, axis=1, keepdims=True)
        self.residual *= np.minimum(1., spec.integral_limit_mps2/np.maximum(norm, 1e-12))
        acceleration = spec.stiffness_per_s2*weight[:, None]*coarse_error
        if estimate_v is not None:
            acceleration += spec.damping_per_s*weight[:, None]*(np.asarray(estimate_v, float)-self.sample @ twin_v)
        acceleration = self.lift @ (acceleration+self.residual)
        norm = np.linalg.norm(acceleration, axis=1, keepdims=True)
        acceleration *= np.minimum(1., spec.accel_limit_mps2/np.maximum(norm, 1e-12))
        force = self.cloth.mass[:, None]*acceleration
        force[~self.cloth.free] = 0.
        distance = np.linalg.norm(coarse_error, axis=1)
        learned = self.cloth.mass[:, None]*(self.lift @ self.residual)
        return force, dict(error_p50_m=float(np.percentile(distance, 50)), error_p95_m=float(np.percentile(distance, 95)),
                           error_max_m=float(distance.max()), force_total_n=float(np.linalg.norm(force, axis=1).sum()),
                           residual_total_n=float(np.linalg.norm(learned[self.cloth.free], axis=1).sum()),
                           force_net_n=np.round(force.sum(axis=0), 4).tolist(),
                           diverged=bool(np.percentile(distance, 95) > spec.max_error_m))
