"""Headless closed loop: a 'real' sheet, a stand-in estimator, the twin, feedback and the CBF.

No ROS and no Isaac. A second cloth with different coefficients plays the real sheet. Its two
leader corners follow a scripted human motion; its two follower corners are driven by the
commands the CBF computes from the twin alone. The twin sees only what the real system
provides: grip positions (delayed) and a coarse, noisy, delayed estimate of the sheet. Used by
the tests and for tuning; the same classes run in the ROS node.
"""
from collections import deque
from dataclasses import dataclass
import numpy as np

from .cbf import TwinCbf, load_settings
from .cloth import TwinCloth, TwinSpec, material_lift
from .feedback import FeedbackSpec, Nudger
from .vendor import fabric_sim_root


@dataclass
class LoopSpec:
    seconds: float = 12.
    leader_velocity: tuple = (0., 0., -.1)   # scripted human motion while `moving`
    move_start_s: float = 2.
    move_seconds: float = 5.
    real_scales: tuple = (1., 1.)            # (bend, shear) of the real sheet relative to the twin
    grip_delay_frames: int = 6               # twin runs this far behind the real sheet
    estimate_period_frames: int = 4          # 15 Hz analyses
    estimate_delay_frames: int = 6
    estimate_noise_m: float = .005
    estimate_shape: tuple = (8, 8)
    feedback: bool = True
    cbf: bool = True                         # False: followers copy the leaders, unfiltered
    cbf_period: int = 4
    follower_lag_s: float = .1               # first-order arm response
    max_speed: float = .1
    margin: float = None
    cbf_config: str = ''
    seed: int = 0


def run(spec=LoopSpec(), twin_spec=TwinSpec(), grips=None, feedback_spec=FeedbackSpec(), settings=None):
    random = np.random.default_rng(spec.seed)
    w, h = twin_spec.width_m, twin_spec.height_m
    if grips is None:  # sheet centred over the default obstacle, 1.05 m up, as in the simulator
        grips = np.array([[-w/2, -h/2, 1.05], [w/2, -h/2, 1.05], [-w/2, h/2, 1.05], [w/2, h/2, 1.05]])
    real, twin = TwinCloth(twin_spec, grips), TwinCloth(twin_spec, grips)
    real.set_material(*spec.real_scales)
    for _ in range(90):  # both hang before anything starts
        real.step(grips); twin.step(grips)
    settings = settings or load_settings(spec.cbf_config or str(fabric_sim_root()/'capsule_cbf'/'config.json'))
    cbf = TwinCbf(twin, settings, spec.cbf_period, spec.max_speed, spec.margin)
    rows, columns = spec.estimate_shape
    nudger = Nudger(twin, rows, columns, feedback_spec)
    u, v = np.meshgrid(np.linspace(0, 1, columns), np.linspace(0, 1, rows))
    sample = material_lift(np.column_stack((u.ravel(), v.ravel())), twin_spec.cells_y+1, twin_spec.cells_x+1)
    dt = twin_spec.frame_dt
    real_grips, command, applied = grips.copy(), np.zeros((2, 3)), np.zeros((2, 3))
    real_history, twin_history = deque(maxlen=64), deque(maxlen=64)
    log = dict(time=[], error_p95=[], clearance_real=[], clearance_twin=[], status=[], nudge_n=[], cbf_ms=[], tension=[])
    frames = int(round(spec.seconds/dt))
    for frame in range(frames):
        t = frame*dt
        moving = spec.move_start_s <= t < spec.move_start_s+spec.move_seconds
        leaders = np.tile(spec.leader_velocity if moving else (0., 0., 0.), (2, 1)).astype(float)
        # --- the real sheet: human corners scripted, robot corners follow the commanded velocity
        applied += (command-applied)*min(1., dt/max(spec.follower_lag_s, dt))
        real_grips[2:] += leaders*dt
        real_grips[:2] += applied*dt
        real.step(real_grips)
        q_real, v_real = real.state()
        real_history.append((frame, real_grips.copy(), leaders.copy(), q_real, v_real))
        # --- the twin sees the grips `grip_delay_frames` late
        if len(real_history) > spec.grip_delay_frames:
            seen = real_history[-1-spec.grip_delay_frames]
            twin.step(seen[1])
            q_twin, v_twin = twin.state()
            twin_history.append((seen[0], q_twin, v_twin))
            # --- stand-in estimator: coarse, noisy, delayed samples of the real sheet
            if spec.feedback and frame % spec.estimate_period_frames == 0:
                source = frame-spec.estimate_delay_frames-spec.grip_delay_frames
                truth = next((r for r in real_history if r[0] == source), None)
                match = next((s for s in twin_history if s[0] == source), None)
                if truth is not None and match is not None:
                    estimate = sample @ truth[3]+random.normal(0., spec.estimate_noise_m, (rows*columns, 3))
                    force, info = nudger.force(match[1], match[2], estimate, sample @ truth[4])
                    twin.set_external(force)
                    log['nudge_n'].append(info['force_total_n'])
            # --- the CBF on the twin's state, with the leader velocity the twin saw
            if frame % spec.cbf_period == 0 and frame >= 90:
                if spec.cbf:
                    command, debug = cbf.update(seen[2])
                    log['status'].append(debug['status']); log['cbf_ms'].append(debug.get('controller_ms', 0.))
                    log['tension'].append(max(debug.get('tension', [0.])))
                else:
                    command = seen[2].copy()
            error = np.linalg.norm(q_twin-seen[3], axis=1)
            log['time'].append(t); log['error_p95'].append(np.percentile(error, 95))
            log['clearance_twin'].append(twin.obstacle_clearance())
        log['clearance_real'].append(real.obstacle_clearance())
    return {k: np.asarray(v) for k, v in log.items()}
