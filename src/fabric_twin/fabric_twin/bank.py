"""Calibrate and adapt the twin's bending and shear coefficients against camera depth. No ROS.

A *case* is a short stretch of the real run: the sheet's state at its start (the corrected
twin's state, which is the best available picture of the real sheet), the grip positions over
the stretch, and the cameras' depth points near its end. A candidate pair of coefficient scales
is scored by starting a free-running copy of the sheet from that state, with no corrective
force, driving it with those grips, and measuring how far the cameras' points end up from its
surface. Right coefficients keep the sheet where the cameras see it; wrong ones let it drift.

The coefficients are multipliers on Style3D's bending stiffness and on its shear term, fitted in
log coordinates. They are effective values for this solver, mesh and time step, not material
properties. A taut sheet's shape does not depend on them, so every update first checks that the
candidates actually disagree with each other; if they do not, the data carry no information and
nothing is changed.
"""
from dataclasses import dataclass, replace
import numpy as np
from scipy.spatial import cKDTree

from .cloth import Obstacle, TwinCloth


@dataclass
class Case:
    q: np.ndarray        # (N, 3) sheet state at the start (world), in the twin's particle order
    grips: np.ndarray    # (T, 4, 3) grip positions for each following frame (world)
    points: np.ndarray   # (M, 3) camera depth points on the sheet near the end (world)
    v: np.ndarray = None

    def __post_init__(self):
        self.q, self.grips, self.points = (np.asarray(a, float) for a in (self.q, self.grips, self.points))
        if (self.q.ndim != 2 or self.grips.ndim != 3 or self.grips.shape[1:] != (4, 3) or len(self.grips) < 1
                or self.points.ndim != 2 or len(self.points) < 20
                or not all(np.isfinite(a).all() for a in (self.q, self.grips, self.points))):
            raise ValueError('A case needs a finite start state, grips (T, 4, 3) and at least 20 depth points')


def surface_cost(q, grid, points, gate_m=.08, clip_m=.03):
    """Robust mean squared distance (m^2) from depth points to the sheet, along its normal.

    `grid`: particle indices as a (rows, columns) material grid. Each point is compared with the
    tangent plane at its nearest particle; points farther than `gate_m` from every particle are
    not the sheet and are ignored, and single residuals are clipped at `clip_m`.
    """
    surface = q[grid]
    du = np.gradient(surface, axis=1); dv = np.gradient(surface, axis=0)
    normal = np.cross(du, dv)
    normal /= np.maximum(np.linalg.norm(normal, axis=2, keepdims=True), 1e-12)
    flat, normals = surface.reshape(-1, 3), normal.reshape(-1, 3)
    distance, index = cKDTree(flat).query(points)
    keep = distance < gate_m
    if keep.sum() < 20:
        return np.inf, 0
    residual = np.einsum('ij,ij->i', points[keep]-flat[index[keep]], normals[index[keep]])
    return float(np.mean(np.minimum(residual**2, clip_m**2))), int(keep.sum())


class Bank:
    def __init__(self, spec, grips):
        # A copy of the twin's sheet with the same mesh and solver settings, and no obstacle in reach.
        self.cloth = TwinCloth(replace(spec, obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1))), grips)
        nx, ny = spec.cells_x+1, spec.cells_y+1
        self.grid = np.lexsort((self.cloth.uv[:, 0], self.cloth.uv[:, 1])).reshape(ny, nx)
        self.evaluations = 0

    def cost(self, scales, cases):
        """Mean robust squared distance (m^2) over the cases for one (bend, shear) pair, and the
        end states it produced (to measure how much candidates differ)."""
        cloth = self.cloth
        cloth.set_material(*scales)
        cloth.set_external(np.zeros((cloth.count, 3)))
        costs, ends = [], []
        for case in cases:
            cloth.targets = cloth.to_twin(case.grips[0])
            cloth.set_state(case.q, case.v)
            for grips in case.grips:
                cloth.step(grips)
            q = cloth.state()[0]
            costs.append(surface_cost(q, self.grid, case.points)[0])
            ends.append(q)
        self.evaluations += 1
        return float(np.mean(costs)), ends

    def calibrate(self, cases, start=(1., 1.), span=4., rounds=3, bounds=(1/16, 16.)):
        """Coarse-to-fine search in log scale around `start`.

        Returns dict(bend, shear, cost_m, start_cost_m, sensitivity_m, table). `sensitivity_m`
        is the largest RMS difference between any candidate's end shape and the best one's: if
        it is below the depth noise, the cases cannot tell the candidates apart.
        """
        table, best, best_cost, best_ends = {}, tuple(float(s) for s in start), None, None
        factor = float(span)

        def evaluate(scales):
            key = tuple(round(float(np.log(s)), 6) for s in scales)
            if key not in table:
                table[key] = self.cost(scales, cases)
            return table[key]
        start_cost, _ = evaluate(best)
        sensitivity = 0.
        for round_index in range(rounds):
            steps = (1/factor, 1/np.sqrt(factor), 1., np.sqrt(factor), factor) if round_index == 0 else (1/factor, 1., factor)
            for b in steps:
                for s in steps:
                    scales = tuple(float(np.clip(x, *bounds)) for x in (best[0]*b, best[1]*s))
                    cost, ends = evaluate(scales)
                    if best_cost is None or cost < best_cost:
                        candidate, best_cost, best_ends = scales, cost, ends
            best = candidate
            factor = factor**.25 if round_index == 0 else np.sqrt(factor)
        for cost, ends in table.values():
            sensitivity = max(sensitivity, max(float(np.sqrt(np.mean(np.sum((a-b)**2, axis=1)))) for a, b in zip(ends, best_ends)))
        return dict(bend=best[0], shear=best[1], cost_m=float(np.sqrt(best_cost)), start_cost_m=float(np.sqrt(start_cost)),
                    sensitivity_m=sensitivity, evaluations=len(table),
                    table=[dict(bend=float(np.exp(k[0])), shear=float(np.exp(k[1])), cost_m=float(np.sqrt(v[0])))
                           for k, v in sorted(table.items())])

    def adapt(self, cases, scales, delta=.2, step=.08, min_sensitivity_m=.0015, min_gain=.03, bounds=(1/16, 16.)):
        """One bounded online update of (bend, shear) from recent cases.

        Compares the current pair with each coefficient scaled up and down by exp(delta). A
        coefficient moves by exp(step) toward its better side only if the candidates' end shapes
        differ by more than `min_sensitivity_m` RMS (the data are informative) and that side
        lowers the cost by more than `min_gain` (relative). Returns (new scales, report).
        """
        scales = tuple(float(s) for s in scales)
        base, base_ends = self.cost(scales, cases)
        new, report = list(scales), dict(cost_m=float(np.sqrt(base)), moved=[], sensitivity_m=[])
        for axis, name in enumerate(('bend', 'shear')):
            trial = {}
            for sign in (-1., 1.):
                candidate = list(scales); candidate[axis] = float(np.clip(scales[axis]*np.exp(sign*delta), *bounds))
                trial[sign] = self.cost(tuple(candidate), cases)
            sensitivity = max(float(np.sqrt(np.mean(np.sum((a-b)**2, axis=1)))) for sign in trial
                              for a, b in zip(trial[sign][1], base_ends))
            report['sensitivity_m'].append(sensitivity)
            better = min(trial, key=lambda sign: trial[sign][0])
            if sensitivity >= min_sensitivity_m and trial[better][0] < base*(1-min_gain):
                new[axis] = float(np.clip(scales[axis]*np.exp(better*step), *bounds))
                report['moved'].append(name)
        report['informative'] = bool(max(report['sensitivity_m']) >= min_sensitivity_m)
        return tuple(new), report
