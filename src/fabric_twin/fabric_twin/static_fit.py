"""Fast fit of the grip points and the stretch stiffness on a still sheet (2026-11-21). No ROS.

The person holds the sheet still before the warm-up's motion. With nothing moving, a hypothesis (grip offsets, stretch
scale) is scored by placing the sheet at its grip points, letting it settle and comparing it with the depth points
(fabric_twin.identify.Forward on a still window). Two stages, in the user's order:

  1. grip points: down / in / side per corner (12 numbers). Depth points near a grip weigh fully, the rest little
     (corner weighting), so the grips are fitted from the sheet next to them, not from the belly's sag;
  2. stretch (log10 scale), with the grips held: the hanging sheet's sag is set by stretch; shear and bending are
     invisible to a still sheet and are not fitted. One parameter on a non-convex curve (the sag barely changes near
     the nominal stiffness and strongly a decade lower; LM did not move from the flat start, 2026-11-21): a parallel
     bracketing line search instead, `line_search_1d`.

Method: Levenberg-Marquardt on the robust residual vector r = sqrt(w) * s*tanh(e/s) (e: signed point-to-sheet
distance; points off the sheet count as s), Jacobian by forward differences. The p perturbed simulations of an
iteration are independent and run together through `evaluate` (parallel worker processes, each with its own model:
Style3D's global PCG couples batched worlds), and several damping values are tried in one batch. Standard for a
black-box simulator with few parameters (n+1 simulations per iteration). The final Jacobian gives the parameters'
covariance sigma^2 (J^T J)^-1, used as the fit's uncertainty (optimistic: depth residuals are correlated).
"""
import time
from dataclasses import dataclass, field

import numpy as np

ROBUST_SCALE_M = .05
# An RMS change below this (mm) is not detectable against the remaining model error (2026-11-21 warm-up design):
# the line search reports its uncertainty as the half-width where the RMS rises by this much.
DETECTION_MM = 1.


def robust(raw, weights=None, scale=ROBUST_SCALE_M):
    """Residual vector from raw signed distances (m; NaN = point off this hypothesis' sheet)."""
    r = np.where(np.isfinite(raw), scale*np.tanh(np.nan_to_num(raw)/scale), scale)
    return r if weights is None else r*np.sqrt(weights)


def corner_weights(window, near_m=.2, far_weight=.1):
    """Per depth point: 1 within near_m of the nearest measured grip, far_weight beyond."""
    return np.concatenate([
        np.where(np.min(np.linalg.norm(np.asarray(p)[:, None, :]-window.grips[k][None], axis=2), axis=1) < near_m,
                 1., far_weight) for k, p in zip(window.depth_index, window.points)])


@dataclass
class FitResult:
    names: list
    theta: np.ndarray
    sd: np.ndarray
    cost_mm: float
    start_cost_mm: float
    iterations: int
    simulations: int
    seconds: float
    converged: bool
    stop_reason: str
    history: list = field(default_factory=list)

    def values(self):
        return dict(zip(self.names, self.theta.tolist()))

    def as_dict(self):
        return dict(names=self.names, values=self.values(), sd=dict(zip(self.names, self.sd.tolist())),
                    cost_mm=self.cost_mm, start_cost_mm=self.start_cost_mm, iterations=self.iterations,
                    simulations=self.simulations, seconds=round(self.seconds, 1), converged=self.converged,
                    stop_reason=self.stop_reason, history=self.history)


def rms_mm(r):
    return float(np.sqrt(np.mean(r**2))*1e3)


def levenberg_marquardt(evaluate, theta0, names, lower, upper, steps, weights=None, max_iterations=8,
                        dampings=(1e-3, 1e-2, 1e-1, 1.), tolerance=2e-3, max_seconds=None, progress=None):
    """Minimise |r(theta)|^2.

    evaluate(list of theta) -> list of raw residual vectors (same points for every theta), run in parallel.
    steps: finite-difference step per parameter (m for offsets, decades for log10 scales).
    Stops when an iteration lowers the RMS by less than `tolerance` (relative), no damping value improves, or a budget
    runs out. Returns FitResult with the covariance-based standard deviation of each parameter.
    """
    begun = time.perf_counter()
    theta = np.clip(np.asarray(theta0, float), lower, upper)
    lower, upper, steps = (np.asarray(x, float) for x in (lower, upper, steps))
    sims = 0

    def residuals(thetas):
        nonlocal sims
        sims += len(thetas)
        return [robust(raw, weights) for raw in evaluate([list(t) for t in thetas])]

    r = residuals([theta])[0]
    start = cost = rms_mm(r)
    history = [dict(iteration=0, rms_mm=cost, theta=theta.tolist(), seconds=0.)]
    reason, converged, J = 'max_iterations', False, None
    for it in range(1, max_iterations+1):
        # Forward differences, stepping away from a bound when needed.
        sign = np.where(theta+steps > upper, -1., 1.)
        probes = [theta+np.eye(len(theta))[i]*steps[i]*sign[i] for i in range(len(theta))]
        J = np.stack([(rp-r)/(steps[i]*sign[i]) for i, rp in enumerate(residuals(probes))], axis=1)
        A, g = J.T@J, J.T@r
        scale = np.maximum(np.diag(A), 1e-12)
        trials = []
        for mu in dampings:     # several dampings in one parallel batch
            try:
                delta = -np.linalg.solve(A+mu*np.diag(scale), g)
            except np.linalg.LinAlgError:
                continue
            trials.append(np.clip(theta+delta, lower, upper))
        if not trials:
            reason = 'singular'
            break
        results = residuals(trials)
        costs = [rms_mm(x) for x in results]
        k = int(np.argmin(costs))
        history.append(dict(iteration=it, rms_mm=costs[k], theta=trials[k].tolist(), damping=dampings[k],
                            seconds=round(time.perf_counter()-begun, 1)))
        if progress:
            progress(history[-1])
        if costs[k] >= cost:
            reason, converged = 'no_damping_improves', True
            break
        gain = (cost-costs[k])/cost
        theta, r, cost = trials[k], results[k], costs[k]
        if gain < tolerance:
            reason, converged = 'converged', True
            break
        if max_seconds is not None and time.perf_counter()-begun > max_seconds:
            reason = 'time_limit'
            break
    sd = np.full(len(theta), np.nan)
    if J is not None:
        dof = max(1, len(r)-len(theta))
        try:
            sd = np.sqrt(np.diag(np.linalg.inv(J.T@J))*float(r@r)/dof)
        except np.linalg.LinAlgError:
            pass
    return FitResult(list(names), theta, sd, cost, start, len(history)-1, sims, time.perf_counter()-begun,
                     converged, reason, history)


def line_search_1d(evaluate, lower, upper, weights=None, points=6, refinements=2, shrink=3.,
                   name='stretch', progress=None):
    """Parallel bracketing search for one log10 parameter: `points` values across [lower, upper] in one batch, then
    `refinements` finer batches around the best (span divided by `shrink` each time), then a parabola through the best
    three. Robust to a flat start and to non-convexity; ~points*(1+refinements) simulations."""
    begun = time.perf_counter()
    table = {}

    def batch(values):
        values = [float(v) for v in values if round(float(v), 6) not in table]
        for v, raw in zip(values, evaluate([[v] for v in values])):
            table[round(v, 6)] = rms_mm(robust(raw, weights))

    span = (upper-lower)
    batch(np.linspace(lower, upper, points))
    history = []
    for k in range(refinements+1):
        best = min(table, key=table.get)
        history.append(dict(iteration=k, value=best, rms_mm=table[best], seconds=round(time.perf_counter()-begun, 1)))
        if progress:
            progress(history[-1])
        if k == refinements:
            break
        span /= shrink
        batch(np.clip(np.linspace(best-span/2, best+span/2, points), lower, upper))
    xs = np.array(sorted(table)); ys = np.array([table[x] for x in xs])
    i = int(np.argmin(ys))
    value, sd = float(xs[i]), np.nan
    if 0 < i < len(xs)-1:                     # parabola through the best and its neighbours
        x3, y3 = xs[i-1:i+2], ys[i-1:i+2]
        a, b, _ = np.polyfit(x3, y3, 2)
        if a > 0:
            value = float(np.clip(-b/(2*a), x3[0], x3[2]))
            sd = float(np.sqrt(DETECTION_MM/a))   # half-width (decades) where the RMS rises by the detection limit
    start_key = min(table, key=lambda x: abs(x))  # the value nearest the nominal (0) as the start reference
    return FitResult([name], np.array([value]), np.array([sd]), float(ys[i]), float(table[start_key]),
                     len(history), len(table), time.perf_counter()-begun, True,
                     'line_search', history+[dict(table={str(k): v for k, v in sorted(table.items())})])
