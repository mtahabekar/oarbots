"""Keep the real sheet slack: a model-free stretch guard on the robot grip velocities.

Why this exists. The sensitivity CBF limits tension with rows predicted by its linear sheet
model, tightened by conformal margins learned from its own prediction errors. Run on the twin,
those predictions are poor: the twin's robot corners follow the *measured* arms, which lag the
commanded velocities, and on a nearly taut sheet (about 9e4 N/m at a held corner) a centimetre of
lag is tens of newtons. In the closed-loop rehearsal the learned tension margin grew to 28-30 N,
larger than the 20 N limit, so the tension rows were infeasible on most updates and were relaxed.
The CBF then kept obstacle clearance by spreading and raising the robots until the sheet was
taut (robot-to-robot 0.83-0.85 m on a 0.84 m sheet) and the true grip tension reached 60 N.

That limit (20 N, inherited from the simulator) also had no physical basis. What actually bounds
the pull at a grip here is the arm's remaining payload (a Jaco2 is rated 1.6 kg continuous at
mid-range and this cell already adds about 0.95 kg beyond the stock hand), the grip's slip force
on dry carbon and what the person holds comfortably: a few newtons, to be measured on site. So
the twin's CBF config drops the model-based tension (and strain) rows, this guard keeps the sheet
in the slack regime, where the pull is set by the sheet's weight and motion rather than by an
unknown stretch stiffness, and the measured wrist force is the check.

What it does. A dry woven sheet barely stretches, so its tension is governed by geometry: while
every pair of grips is closer than the sheet between them, only the sheet's weight and motion
load the grips. This guard enforces that directly, with measured positions only (robot grips from
TF, human grips from the glove filter), as a control-barrier condition on each pair distance d:

    d/dt d  <=  alpha (d_max - d),     d_max = rest length - slack

for the two robot-to-hand edges, the robot-to-robot edge and the two diagonals through a robot.
It is the last layer before the command leaves the twin and takes priority over obstacle
clearance, which is the CBF's own stretch-first ordering. If the hands outrun the robots so that
no command satisfies every pair, the least-violating command is used and reported.

Optionally it also closes the loop on the measured wrist force: when the cloth's pull on a robot
exceeds `force_soft_n`, the robot is moved along that pull (toward the sheet) in proportion to
the excess. That uses no model at all and also covers what geometry cannot see, such as the sheet
lying on the obstacle or a grip offset error larger than the slack.
"""
from dataclasses import dataclass
import numpy as np

from .vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.qp import safety_qp  # noqa: E402

ROBOTS, HANDS = (0, 1), (2, 3)


@dataclass(frozen=True)
class StretchSpec:
    enabled: bool = True
    edge_slack_m: float = .03       # each edge kept at least this much shorter than the sheet
    diagonal_slack_m: float = .03
    alpha_per_s: float = 2.         # how fast a pair may approach its limit
    max_speed_mps: float = .1
    force_feedback: bool = False    # needs calibrated wrist forces (gates H5/H6 on hardware)
    force_soft_n: float = 6.        # cloth pull above which the robot yields
    force_gain_mps_per_n: float = .004
    force_max_speed_mps: float = .05
    force_max_age_sec: float = .2

    def validate(self):
        values = [self.edge_slack_m, self.diagonal_slack_m, self.alpha_per_s, self.max_speed_mps, self.force_soft_n,
                  self.force_gain_mps_per_n, self.force_max_speed_mps, self.force_max_age_sec]
        if not np.isfinite(values).all() or min(values) < 0 or self.alpha_per_s <= 0 or self.max_speed_mps <= 0:
            raise ValueError('Invalid stretch guard settings')


class StretchGuard:
    def __init__(self, width_m, height_m, spec=StretchSpec(), spans=None):
        """`spans` {(i, j): m}: the sheet's rest length between grip pairs when the pinches are inset from the
        corners (calibrated grip points, cloth.TwinCloth.grip_spans). Default: corner-to-corner."""
        spec.validate()
        self.spec = spec
        diagonal = float(np.hypot(width_m, height_m))
        rest = {(0, 2): height_m, (1, 3): height_m, (0, 1): width_m, (0, 3): diagonal, (1, 2): diagonal}
        if spans:
            rest.update({pair: float(spans[pair]) for pair in rest if pair in spans})
        # (i, j, d_max): corners 0, 1 robots (v = 0 edge), 2, 3 hands; 0-2 and 1-3 run along v.
        self.pairs = [(0, 2, rest[0, 2]-spec.edge_slack_m), (1, 3, rest[1, 3]-spec.edge_slack_m),
                      (0, 1, rest[0, 1]-spec.edge_slack_m), (0, 3, rest[0, 3]-spec.diagonal_slack_m),
                      (1, 2, rest[1, 2]-spec.diagonal_slack_m)]

    def force_term(self, forces):
        """Velocity (2, 3) that yields to the cloth's measured pull on each robot (world, N)."""
        s, out = self.spec, np.zeros((2, 3))
        if forces is None:
            return out
        for k, force in enumerate(forces):
            if force is None:
                continue
            force = np.asarray(force, float)
            magnitude = np.linalg.norm(force)
            excess = magnitude-s.force_soft_n
            if excess > 0:
                out[k] = min(s.force_gain_mps_per_n*excess, s.force_max_speed_mps)*force/magnitude
        return out

    def filter(self, robots, hands, hand_velocity, command, forces=None):
        """The robot grip velocities (2, 3, world) closest to `command` that keep the sheet slack.

        robots, hands: measured grip positions (2, 3); hand_velocity: (2, 3); forces: per robot the
        cloth's pull on it (3,) in the world frame, or None. Returns (velocities, report).
        """
        s = self.spec
        command = np.asarray(command, float).reshape(2, 3)
        if not s.enabled:
            return command, dict(enabled=False)
        points = np.vstack((np.asarray(robots, float), np.asarray(hands, float)))
        hand_velocity = np.asarray(hand_velocity, float).reshape(2, 3)
        if not np.isfinite(points).all() or not np.isfinite(hand_velocity).all():
            return np.zeros((2, 3)), dict(enabled=True, status='invalid input')
        yielding = self.force_term(forces) if s.force_feedback else np.zeros((2, 3))
        target = command+yielding
        speed = np.linalg.norm(target, axis=1, keepdims=True)
        target = target*np.minimum(1., s.max_speed_mps/np.maximum(speed, 1e-12))
        rows, rhs, report = [], [], dict(enabled=True, pairs={})
        for i, j, d_max in self.pairs:
            delta = points[j]-points[i]
            d = float(np.linalg.norm(delta))
            n = delta/max(d, 1e-9)
            # d/dt d = n . (v_j - v_i); require it <= alpha (d_max - d), as rows a u >= b on u = (v_0, v_1).
            row, known = np.zeros(6), 0.
            for corner, sign in ((i, -1.), (j, 1.)):
                if corner in ROBOTS:
                    row[3*corner:3*corner+3] -= sign*n
                else:
                    known += sign*n @ hand_velocity[corner-2]
            rows.append(row)
            rhs.append(known-s.alpha_per_s*(d_max-d))
            report['pairs'][f'{i}{j}'] = round(d_max-d, 4)  # remaining slack, m (negative: over the limit)
        bound = np.full(6, s.max_speed_mps)
        u, status, slacks, violation = safety_qp(np.clip(target.ravel(), -bound, bound), np.array(rows), np.array(rhs),
                                                 np.ones(len(rows), int), -bound, bound, hard_tolerance=0.)
        u = u.reshape(2, 3)
        # The arms' speed limit is a norm; the QP box is per axis. When keeping the sheet slack
        # needs more than the limit (the hands outrun the robots), the command is scaled down and
        # the remaining violation is reported: the guard cannot outrun the human.
        speed = np.linalg.norm(u, axis=1, keepdims=True)
        if np.any(speed > s.max_speed_mps):
            u *= np.minimum(1., s.max_speed_mps/np.maximum(speed, 1e-12))
            status = 'speed_limited'
            violation = float(np.max(np.array(rhs)-np.array(rows) @ u.ravel(), initial=0.))
        report.update(status=status, violation_mps=round(float(violation), 4),
                      change_mps=round(float(np.linalg.norm(u-command)), 4),
                      min_slack_m=round(min(report['pairs'].values()), 4),
                      force_yield_mps=np.round(np.linalg.norm(yielding, axis=1), 4).tolist())
        return u, report
