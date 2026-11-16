"""sim_eval: loaders, truth alignment, offline twin replay, forecasts and identification cases.

Everything here replays the recorded inputs of one plant run (study S1 datasets). The robots in
those runs were driven only by the warm-up script and by formation following of the measured
hands, never by the twin's state, so the recorded grips, depth and estimates are exogenous: a
module replayed offline sees exactly the inputs it saw live, and two conditions that differ in one
switch see identical inputs. That is what isolates one module's contribution from the others.

The twin, its corrective force and the parameter bank are the production classes
(`fabric_twin.cloth.TwinCloth`, `fabric_twin.feedback.Nudger`, `fabric_twin.bank.Bank`), driven
the way `fabric_twin.node` drives them.
"""
import json
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np
import yaml

from fabric_sim_cell.core import sample_grid
from fabric_twin.bank import Bank, Case
from fabric_twin.cloth import Obstacle, TwinCloth, TwinSpec
from fabric_twin.feedback import FeedbackSpec, Nudger

ESTIMATOR_ROWS = ESTIMATOR_COLUMNS = 8
HORIZONS = (6, 20, 30, 60)  # forecast horizons in 60 Hz frames: 0.1, 0.33, 0.5 and 1 s


def twin_spec(config_path, device='cuda:0'):
    """TwinSpec and FeedbackSpec from a twin YAML (the same fields `fabric_twin.node` reads)."""
    value = yaml.safe_load(Path(config_path).read_text()) or {}
    sheet = dict(value.get('sheet', {}))
    for key in ('stretch', 'bend'):
        if key in sheet:
            sheet[key] = tuple(sheet[key])
    obstacle = Obstacle(**{k: tuple(v) if isinstance(v, list) else v for k, v in value.get('obstacle', {}).items()})
    return TwinSpec(obstacle=obstacle, device=device, **sheet), FeedbackSpec(**value.get('feedback', {}))


def regular_grid(positions, uv, shape):
    """Plant particles (T, N, 3) as a float32 (T, rows, columns, 3) grid by material uv."""
    rows, columns = (int(v) for v in shape)
    col = np.rint(uv[:, 0]*(columns-1)).astype(int)
    row = np.rint(uv[:, 1]*(rows-1)).astype(int)
    grid = np.empty((positions.shape[0], rows, columns, 3), np.float32)
    grid[:, row, col] = positions
    return grid


class Run:
    """One recorded plant run: manifest, plant truth, the twin's recorded inputs and outputs."""

    def __init__(self, path):
        self.path = Path(path)
        self.manifest = json.loads((self.path/'manifest.json').read_text())
        with np.load(self.path/'truth.npz') as t:
            self.truth_ns = t['times_ns'].astype(np.int64)
            self.truth_corners = t['corners'].astype(float)
            self.truth_positions = t['positions']  # float32 (T, N, 3), plant particle order
            self.truth_uv = t['material_uv'].astype(float)
            self.truth_shape = tuple(int(v) for v in t['grid_shape'])
        self.grid = regular_grid(self.truth_positions, self.truth_uv, self.truth_shape)
        with np.load(self.path/'twin.npz') as t:
            self.twin = {k: t[k] for k in t.files}
        self.depth = None
        if (self.path/'twin.depth.npz').exists():
            with np.load(self.path/'twin.depth.npz') as d:
                self.depth = {k: d[k] for k in d.files}
            self.depth['offsets'] = np.concatenate(([0], np.cumsum(self.depth['counts'])))
        self.estimate = None
        if (self.path/'estimate.npz').exists():
            with np.load(self.path/'estimate.npz') as e:
                self.estimate = {k: e[k] for k in e.files}
        self.status = [json.loads(line) for line in (self.path/'twin.status.jsonl').read_text().splitlines() if line.strip()]
        self._truth_at_uv = {}

    # ---------------------------------------------------------------- warm-up
    def warmup_window(self):
        """(start_ns, holds) as the twin announced them to the parameter bank."""
        for row in self.status:
            if row.get('topic') == '/fabric/twin/warmup_window':
                return int(row['data']['start_ns']), [tuple(h) for h in row['data']['holds']]
        raise ValueError(f'{self.path.name}: no warm-up window was recorded')

    # ---------------------------------------------------------------- truth
    def truth_at_uv(self, uv):
        """Plant truth (T_truth, V, 3) sampled bilinearly at material coordinates `uv` (cached)."""
        key = (len(uv), float(np.asarray(uv).sum()))
        if key not in self._truth_at_uv:
            out = np.empty((len(self.truth_ns), len(uv), 3), np.float32)
            for start in range(0, len(self.truth_ns), 500):
                out[start:start+500] = sample_grid(self.grid[start:start+500], np.asarray(uv, float))
            self._truth_at_uv[key] = out
        return self._truth_at_uv[key]

    def frame(self, time_ns):
        """Index of the truth frame nearest `time_ns`."""
        i = min(max(int(np.searchsorted(self.truth_ns, time_ns)), 1), len(self.truth_ns)-1)
        return i-1 if abs(self.truth_ns[i-1]-time_ns) <= abs(self.truth_ns[i]-time_ns) else i

    def truth_velocity(self, truth_uv_positions, k):
        a, b = max(k-1, 0), min(k+1, len(self.truth_ns)-1)
        return (truth_uv_positions[b].astype(float)-truth_uv_positions[a])/((self.truth_ns[b]-self.truth_ns[a])/1e9)

    # ---------------------------------------------------------------- grips
    def grips(self, source, times_ns):
        """Four grips (len(times), 4, 3): 'truth' (plant corner particles) or 'measured' (the
        grips the live twin was stepped with: robot TF and the glove filter, after its reach check)."""
        times = np.asarray(times_ns, np.int64)
        if source == 'truth':
            stamps, values = self.truth_ns, self.truth_corners
        elif source == 'measured':
            stamps, values = self.twin['corners_ns'], self.twin['corners']
        else:
            raise ValueError(source)
        out = np.empty((len(times), 4, 3))
        for c in range(4):
            for a in range(3):
                out[:, c, a] = np.interp(times, stamps, values[:, c, a])
        return out

    # ---------------------------------------------------------------- depth
    def depth_points(self, start_ns, end_ns):
        d = self.depth
        keep = np.flatnonzero((d['stamps_ns'] >= start_ns) & (d['stamps_ns'] <= end_ns))
        if not len(keep):
            return np.zeros((0, 3))
        return np.concatenate([d['points'][d['offsets'][i]:d['offsets'][i+1]] for i in keep]).astype(float)


# -------------------------------------------------------------------- the twin, replayed
def oracle_estimates(run, measurement_ns, sigma_m=.005):
    """Perfect-perception stand-in for the estimator: plant truth at the estimator's 8 x 8 material
    vertices, at the real estimator's own measurement times, all vertices marked observed."""
    u, v = np.meshgrid(np.linspace(0, 1, ESTIMATOR_COLUMNS), np.linspace(0, 1, ESTIMATOR_ROWS))
    uv = np.column_stack((u.ravel(), v.ravel()))
    truth = run.truth_at_uv(uv)
    covariance = np.zeros((len(uv), 6, 6)); covariance[:, :3, :3] = np.eye(3)*sigma_m**2
    covariance[:, 3:, 3:] = np.eye(3)*(.05)**2
    out = []
    for stamp in measurement_ns:
        k = run.frame(stamp)
        out.append((int(stamp), truth[k].astype(float), run.truth_velocity(truth, k), covariance,
                    np.ones(len(uv), np.uint8), False))
    return out


def recorded_estimates(run):
    """The live estimator's analyses (not its predictions), as the twin received them."""
    e = run.estimate
    keep = np.flatnonzero(~e['predicted'])
    return [(int(e['measurement_ns'][i]), e['positions'][i], e['velocities'][i], e['covariance'][i],
             e['visibility'][i], bool(e['degraded'][i])) for i in keep]


def replay(run, spec, feedback, scales=(1., 1.), correction='off', grips='measured', t0=12., t1=None,
           forecast_every_s=None, forecast_window=None, sample_every=4, oracle_sigma_m=.005, feedback_scale=1.,
           keep_states_every=0):
    """Drive a twin with recorded grips (and optionally recorded corrections) over [t0, t1] s.

    correction: 'off' (grip-driven simulation), 'estimator' (the live estimator's analyses) or
    'oracle' (plant truth at the estimator's vertices and times). All conditions start from the
    plant's true state at t0, so they differ only in the switch under test.
    Returns per-sampled-frame errors against truth and, if requested, open-loop forecast errors.
    """
    k0 = run.frame(int(t0*1e9))
    k1 = len(run.truth_ns)-1 if t1 is None else run.frame(int(t1*1e9))
    frames = np.arange(k0, k1+1)
    times = run.truth_ns[frames]
    G = run.grips(grips, times)
    cloth = TwinCloth(spec, G[0])
    if tuple(scales) != (1., 1.):
        cloth.set_material(*scales)
    truth = run.truth_at_uv(cloth.uv)
    cloth.targets = cloth.to_twin(G[0])
    cloth.set_state(truth[k0].astype(float), run.truth_velocity(truth, k0))
    nudger, queue = None, deque()
    if correction != 'off':
        if feedback_scale != 1.:
            feedback = replace(feedback, stiffness_per_s2=feedback.stiffness_per_s2*feedback_scale,
                               damping_per_s=feedback.damping_per_s*feedback_scale,
                               integral_per_s3=feedback.integral_per_s3*feedback_scale)
        nudger = Nudger(cloth, ESTIMATOR_ROWS, ESTIMATOR_COLUMNS, feedback)
        estimates = recorded_estimates(run)
        if correction == 'oracle':
            estimates = oracle_estimates(run, [e[0] for e in estimates], oracle_sigma_m)
        queue = deque(sorted((e for e in estimates if e[0] >= times[0]), key=lambda e: e[0]))
    forecaster = None
    origins = set()
    if forecast_every_s:
        lo, hi = forecast_window or (t0, times[-1]/1e9)
        for t in np.arange(lo, hi-max(HORIZONS)/60, forecast_every_s):
            origins.add(int(np.searchsorted(times, int(t*1e9))))
        forecaster = TwinCloth(spec, G[0])
        if tuple(scales) != (1., 1.):
            forecaster.set_material(*scales)
    history = deque(maxlen=90)
    kept_t, kept_q, kept_v = [], [], []
    rows, forecasts, last_analysis, info = [], [], None, dict(applied=0, replaced=0, force_total_n=[])
    for i, k in enumerate(frames):
        g = cloth.within_reach(G[i])
        cloth.step(g)
        q, v = cloth.state()
        history.append((times[i], q, v))
        while queue and queue[0][0] <= times[i]:
            stamp, eq, ev, cov, vis, degraded = queue.popleft()
            match = min(history, key=lambda h: abs(h[0]-stamp))
            if abs(match[0]-stamp) > 1.5e9*spec.frame_dt or (last_analysis is not None and stamp <= last_analysis):
                continue
            dt = 1/15 if last_analysis is None else min(.5, max(1e-3, (stamp-last_analysis)/1e9))
            last_analysis = stamp
            force, report = nudger.force(match[1], match[2], eq, ev, cov, vis, dt)
            if report['diverged'] and not degraded:
                cloth.place_from_mesh(eq, ESTIMATOR_ROWS, ESTIMATOR_COLUMNS, cloth.to_world(cloth.targets), ev)
                nudger.reset(); cloth.set_external(np.zeros((cloth.count, 3))); history.clear()
                info['replaced'] += 1
                continue
            cloth.set_external(force)
            info['applied'] += 1; info['force_total_n'].append(report['force_total_n'])
        if i % sample_every == 0:
            rows.append(errors(q, truth[k]) | dict(t=times[i]/1e9))
        if keep_states_every and i % keep_states_every == 0:
            kept_t.append(times[i]); kept_q.append(q.astype(np.float32)); kept_v.append(v.astype(np.float32))
        if i in origins and i+max(HORIZONS) < len(frames):
            forecaster.targets = forecaster.to_twin(g)
            forecaster.set_state(q, v)
            out = dict(t=times[i]/1e9)
            for h in range(1, max(HORIZONS)+1):
                forecaster.step(forecaster.within_reach(G[i+h]))
                if h in HORIZONS:
                    out[h] = errors(forecaster.state()[0], truth[frames[i+h]])
            forecasts.append(out)
    info['force_total_n'] = float(np.percentile(info['force_total_n'], 95)) if info['force_total_n'] else 0.
    out = dict(rows=rows, forecasts=forecasts, info=info)
    if keep_states_every:
        out['states'] = (np.array(kept_t), np.array(kept_q), np.array(kept_v))
    return out


def errors(q, true):
    """Per-state error summary of a twin state (V, 3) against truth at the same particles."""
    true = true.astype(float)
    distance = np.linalg.norm(q-true, axis=1)
    low = np.argsort(true[:, 2])[:max(1, len(true)//20)]  # the lowest 5 % of the true sheet
    return dict(mean=float(distance.mean()), p95=float(np.percentile(distance, 95)), max=float(distance.max()),
                low_dz=float((q[low, 2]-true[low, 2]).mean()), low_dist=float(distance[low].mean()))


# -------------------------------------------------------------------- identification
def hold_ends(run, which='extended'):
    """End times (ns) of the warm-up holds. 'baseline': the subset a 2/4 cm inset warm-up has
    (settle, 2 cm, 4 cm and the final return), taken from the same run."""
    start, holds = run.warmup_window()
    ends = [start+int(end*1e9) for _, end, _ in holds]
    if which == 'baseline':
        ends = [ends[0], ends[1], ends[2], ends[-1]]
    return ends


def make_cases(run, cloth_uv, ends, state='truth', grips='truth', points='truth', relax_sec=2., points_sec=1.,
               max_points=2500, seed=0, states=None):
    """Bank cases with each input taken from plant truth or from what the live system measured.

    state  'truth': the plant sheet at the case start | 'twin': the live corrected twin state
    grips  'truth': plant corners                    | 'measured': the grips the live twin used
    points 'truth': plant particles (the true surface) | 'depth': the cameras' gated depth points
    """
    rng = np.random.default_rng(seed)
    truth = run.truth_at_uv(cloth_uv)
    cases = []
    for end in ends:
        begin = end-int(relax_sec*1e9)
        if state == 'truth':
            k = run.frame(begin)
            q, v, t_state = truth[k].astype(float), run.truth_velocity(truth, k), run.truth_ns[k]
        else:  # 'twin': the live twin's recorded states, or `states` = (times, q, v) from an offline replay
            st, sq, sv = states if states is not None else (run.twin['times_ns'], run.twin['positions'], run.twin['velocities'])
            j = int(np.argmin(np.abs(st-begin)))
            if abs(st[j]-begin) > .2e9:
                continue
            q, v, t_state = sq[j].astype(float), sv[j].astype(float), st[j]
        if grips == 'truth':
            keep = (run.truth_ns > t_state) & (run.truth_ns <= end)
            G = run.truth_corners[keep]
        else:
            keep = (run.twin['corners_ns'] > t_state) & (run.twin['corners_ns'] <= end)
            G = run.twin['corners'][keep].astype(float)
        if points == 'truth':
            keep = np.flatnonzero((run.truth_ns >= end-int(points_sec*1e9)) & (run.truth_ns <= end))
            P = run.truth_positions[keep].reshape(-1, 3).astype(float)
        else:
            P = run.depth_points(end-int(points_sec*1e9), end)
        if len(P) > max_points:
            P = P[rng.choice(len(P), max_points, replace=False)]
        if len(G) < 30 or len(P) < 20:
            continue
        cases.append(Case(q, G, P, v))
    return cases


def bank_for(spec, run):
    """A parameter bank with the twin's mesh and solver, oriented like the twin of this run."""
    return Bank(spec, run.truth_corners[run.frame(int(15e9))])


def landscape(bank, cases, values=2.**np.arange(-2, 2.01, 1.)):
    """J(bend, shear) on a log grid: RMS of the bank's robust depth cost (m)."""
    table = np.full((len(values), len(values)), np.nan)
    for i, b in enumerate(values):
        for j, s in enumerate(values):
            table[i, j] = np.sqrt(bank.cost((float(b), float(s)), cases)[0])
    return values, table
