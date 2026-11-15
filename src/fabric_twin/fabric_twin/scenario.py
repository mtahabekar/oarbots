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

from .cbf import TwinCbf, default_settings_path, load_settings
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
    settings = settings or load_settings(spec.cbf_config or str(default_settings_path()))
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


# ------------------------------------------------------------------ the two-hand carry
def hand_path(segments, start_s, gather_m=0., gather_start_s=None):
    """Offsets (callable t -> (2, 3)) of the two hands from their start, both hands moving together.

    segments: ('move', dx, dy, dz, mean_speed) with a cosine speed profile, or ('hold', seconds).
    gather_m: each hand first moves this far toward the other (a person holds the sheet slack).
    """
    knots, t = [], start_s
    for segment in segments:
        if segment[0] == 'hold':
            t += segment[1]
            continue
        delta = np.array(segment[1:4], float)
        duration = max(np.linalg.norm(delta)/segment[4]*1.5, 1e-6)
        knots.append((t, duration, delta)); t += duration
    gather_start_s = start_s-2.5 if gather_start_s is None else gather_start_s

    def offsets(time_s):
        common = np.zeros(3)
        for begin, duration, delta in knots:
            common += .5*(1-np.cos(np.pi*np.clip((time_s-begin)/duration, 0., 1.)))*delta
        inward = .5*(1-np.cos(np.pi*np.clip((time_s-gather_start_s)/2., 0., 1.)))*gather_m
        return np.array([common+[inward, 0., 0.], common-[inward, 0., 0.]])
    return offsets, t


CARRY = (('move', 0., 0., -.55, .08), ('hold', 3.), ('move', .08, 0., 0., .04), ('hold', 3.),
         ('move', -.08, 0., 0., .04), ('hold', 2.), ('move', 0., 0., .3, .08))


@dataclass
class CarrySpec:
    segments: tuple = CARRY
    start_s: float = 4.
    gather_m: float = .02
    seconds: float = 35.
    real_scales: tuple = (1., 1.)
    grip_delay_s: float = .15          # the twin runs this far behind
    follower_lag_s: float = .15        # first-order arm response to a velocity command
    cbf: bool = True                   # False: the follow command goes out unfiltered
    cbf_overrides: dict = None         # changes to the CBF settings, e.g. {'follower_max_accel': 1.}
    cbf_period: int = 4
    margin: float = .04
    guard: object = 'default'          # a StretchSpec, 'default', or None for no stretch guard
    formation_slack_m: float = .04
    formation_lateral_slack_m: float = .02
    formation_gain_per_s: float = 2.
    feedback: bool = True
    cbf_config: str = ''
    forces: bool = False               # give the guard the real sheet's pull on each robot (an ideal F/T)
    force_noise_n: float = .5
    seed: int = 0


def carry(spec=CarrySpec(), twin_spec=TwinSpec(), feedback_spec=FeedbackSpec()):
    """The rehearsal carry, headless: true grip tension and obstacle clearance of the real sheet.

    The real sheet's robot grips follow the commanded velocities through a first-order lag; its
    hand grips follow the scripted path. The twin sees both `grip_delay_s` late, is corrected by a
    noisy coarse estimate, and the CBF (and the stretch guard) compute the commands from the twin
    and from the newest measured grips, as the node does.
    """
    from dataclasses import replace as _replace
    from .stretch import StretchGuard, StretchSpec
    random = np.random.default_rng(spec.seed)
    w, h = twin_spec.width_m, twin_spec.height_m
    start = np.array([[-w/2, -h/2, 1.05], [w/2, -h/2, 1.05], [-w/2, h/2, 1.05], [w/2, h/2, 1.05]])
    real, twin = TwinCloth(twin_spec, start), TwinCloth(twin_spec, start)
    real.set_material(*spec.real_scales)
    from .vendor import use_capsule_cbf
    use_capsule_cbf()
    from capsule_cbf.cloth_model import ClothMechanics
    mechanics = ClothMechanics.from_newton(real.model)
    patches = [real.patch[real.patch_corners == c] for c in range(4)]
    settings = load_settings(spec.cbf_config or str(default_settings_path()))
    if spec.cbf_overrides:
        settings = _replace(settings, **spec.cbf_overrides)
    if not spec.cbf:
        settings = _replace(settings, controller='none')
    offsets, motion_end = hand_path(spec.segments, spec.start_s, spec.gather_m)
    dt = twin_spec.frame_dt
    delay = int(round(spec.grip_delay_s/dt))
    real_grips, applied, command = start.copy(), np.zeros((2, 3)), np.zeros((2, 3))
    history = deque(maxlen=delay+2)
    cbf = guard = nudger = formation = None
    rows, columns = 8, 8
    u, v = np.meshgrid(np.linspace(0, 1, columns), np.linspace(0, 1, rows))
    sample = material_lift(np.column_stack((u.ravel(), v.ravel())), twin_spec.cells_y+1, twin_spec.cells_x+1)
    log = dict(time=[], tension=[], clearance=[], twin_error=[], status=[], guard=[], distance=[], grips=[], command=[], guard_time=[], correction=[])
    for frame in range(int(round(spec.seconds/dt))):
        t = frame*dt
        hands = start[2:]+offsets(t)
        hand_velocity = (offsets(t+dt)-offsets(t))/dt
        applied += (command-applied)*min(1., dt/max(spec.follower_lag_s, dt))
        real_grips[:2] += applied*dt
        real_grips[2:] = hands
        real.step(real_grips)
        history.append((t, real_grips.copy(), hand_velocity))
        q_real = real.state_twin()[0]
        log['time'].append(t)
        log['tension'].append([np.linalg.norm(mechanics.resultant(q_real, p)) for p in patches])
        log['clearance'].append(real.obstacle_clearance())
        log['distance'].append([np.linalg.norm(real_grips[j]-real_grips[i]) for i, j in ((0, 2), (1, 3), (0, 1), (0, 3), (1, 2), (2, 3))])
        log['grips'].append(real_grips.copy()); log['command'].append(applied.copy())
        if len(history) <= delay:
            continue
        seen = history[0]
        twin.step(seen[1])
        if cbf is None and t >= 1.5:  # both sheets have hung for a while: start control
            cbf = TwinCbf(twin, settings, spec.cbf_period, .1, spec.margin)
            nudger = Nudger(twin, rows, columns, feedback_spec)
            guard = (None if spec.guard is None else StretchGuard(w, h, StretchSpec() if spec.guard == 'default' else spec.guard))
            offset = seen[1][:2]-seen[1][2:]
            flat = offset.copy(); flat[:, 2] = 0.
            formation = offset-spec.formation_slack_m*flat/np.linalg.norm(flat, axis=1, keepdims=True)
            across = seen[1][1]-seen[1][0]; across[2] = 0.; across /= np.linalg.norm(across)
            formation[0] += spec.formation_lateral_slack_m*across
            formation[1] -= spec.formation_lateral_slack_m*across
        if cbf is None:
            continue
        if spec.feedback and frame % 4 == 0:
            q_twin, v_twin = twin.state()
            estimate = sample @ real.state()[0]
            force, _ = nudger.force(q_twin, v_twin, estimate+random.normal(0., .005, estimate.shape))
            twin.set_external(force)
        log['twin_error'].append(float(np.percentile(np.linalg.norm(twin.state()[0]-real.state()[0], axis=1), 95)))
        if frame % spec.cbf_period == 0:
            # Follow command from the newest measurements, then the CBF on the delayed twin.
            velocity = np.clip(hand_velocity, -.5, .5)
            nominal = velocity+spec.formation_gain_per_s*(hands+formation-real_grips[:2])
            speed = np.linalg.norm(nominal, axis=1, keepdims=True)
            nominal *= np.minimum(1., .1/np.maximum(speed, 1e-12))
            proposed, debug = cbf.update(seen[2], nominal)
            log['status'].append(debug['status']); log['correction'].append(debug.get('correction', 0.))
            if guard is not None:
                pulls = None
                if spec.forces:
                    pulls = [real.vector_to_world(mechanics.resultant(q_real, patches[k]))+random.normal(0., spec.force_noise_n, 3)
                             for k in range(2)]
                command, report = guard.filter(real_grips[:2], hands, hand_velocity, proposed, pulls)
                log['guard'].append(report['status']); log['guard_time'].append(t)
                cbf.applied(command, seen[2])
            else:
                command = proposed
    out = {k: np.asarray(v) for k, v in log.items()}
    out['motion_end_s'] = motion_end
    return out
