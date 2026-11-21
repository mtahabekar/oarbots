"""Warm-up excitation: the robots move the sheet through measured shapes while the person holds still.

A sheet held taut at four corners has almost the same shape whatever its bending and shear
stiffness, so a taut sheet tells the calibration nothing. The script moves the two robot grips
through a list of hold targets and returns:

    slack <x> cm        both grips toward the hands by each of `insets_m` (the deepest inset is the base
                        of the excursions below)
    narrow              both grips toward each other by `transverse_m` (slack along the robot edge)
    lift Blue / Silver  one grip up by `lift_m` (twists the sheet)
    shift toward Silver / Blue   both grips sideways by `lateral_m` (shears the sheet: the robot edge moves
                        along itself relative to the hands)
    return              back to the start

Then, optionally, the person moves the hands on spoken cues while the robots hold still (`person_moves_m`):
the robots' workspace allows only about 10 cm, while the person can move the sheet's free edge 15-20 cm, which
is what made the recorded carry runs informative about shear and bending (scripts/hw/warmup_design.py, 2026-11-21):

    hands up / hands down       the hand edge up or down by `up` / `down`
    hands toward the robots     both hands toward the robots by `toward` (the base of the next cues)
    hands together / apart      each hand toward the other by `together`
    hands to your left / right  both hands sideways by `side` (left = Silver's side)
    return                      back to the start

These moves only ever slacken the sheet. The plan records where the hands are expected to be, but the
calibration uses the measured gloves, so the person need not be exact; holding still at each hold matters.
Every excursion holds `hold_sec` and returns ("back"). It produces a nominal velocity for the two robot grips;
the twin node decides whether it is sent (direct warm-up) or filtered.
"""
PERSON = ('up', 'down', 'toward', 'together', 'side')
import numpy as np

ROBOTS = ('Blue', 'Silver')


class WarmupScript:
    def __init__(self, insets_m=(.02, .04), speed_mps=.03, hold_sec=5., settle_sec=3., gain_per_s=1.5, transverse_m=0., lift_m=0., lateral_m=0.,
                 person_moves_m=None, person_speed_mps=.1, person_reaction_sec=2.):
        insets = [float(x) for x in insets_m]
        if (not insets or not np.isfinite(insets+[speed_mps, hold_sec, settle_sec, gain_per_s]).all()
                or min(insets) <= 0 or max(insets) > .2 or speed_mps <= 0 or speed_mps > .1 or min(hold_sec, settle_sec) < 0):
            raise ValueError('Warm-up insets must be 0-0.2 m and the speed 0-0.1 m/s')
        self.insets, self.speed, self.hold, self.settle, self.gain = insets, speed_mps, hold_sec, settle_sec, gain_per_s
        extra = np.array([transverse_m, lift_m, lateral_m], float)
        if not np.isfinite(extra).all() or np.any(extra < 0) or np.any(extra > .1):
            raise ValueError('Additional excursions must be finite and between 0 and 0.1 m')
        self.extra = extra
        person = {k: float(v) for k, v in (person_moves_m or {}).items()}
        unknown = set(person)-set(PERSON)
        if (unknown or not np.isfinite(list(person.values())+[person_speed_mps, person_reaction_sec]).all()
                or any(v < 0 or v > .25 for v in person.values()) or not 0 < person_speed_mps <= .3 or person_reaction_sec < 0):
            raise ValueError(f'person_moves_m takes {PERSON} between 0 and 0.25 m; person speed 0-0.3 m/s')
        self.person, self.person_speed, self.reaction = person, float(person_speed_mps), float(person_reaction_sec)
        self.start_ns = None
        # Durations do not depend on where the grips are (only directions do), so the plan is known before start.
        self._set(*self._profile(np.array([[0., 0., 1.], [1., 0., 1.], [0., 1., 1.], [1., 1., 1.]])))

    def _profile(self, corners):
        """Knot times (K,), robot displacements (K, 2, 3), hold labels and the inward unit vectors for start corners."""
        corners = np.asarray(corners, float)
        direction = corners[2:]-corners[:2]
        direction[:, 2] = 0.
        across = corners[1]-corners[0]
        across[2] = 0.
        length = np.linalg.norm(direction, axis=1, keepdims=True)
        if not np.isfinite(corners).all() or np.any(length < 1e-6) or np.linalg.norm(across) < 1e-6:
            raise ValueError('Warm-up requires finite, separated grips')
        direction, across = direction/length, across/np.linalg.norm(across)
        targets = [(inset*direction, f'slack {inset*100:g} cm') for inset in self.insets]
        slack = targets[-1][0]
        transverse, lift, lateral = self.extra
        if transverse:
            targets += [(slack+transverse*np.array([across, -across]), f'narrow {transverse*100:g} cm'), (slack, 'back')]
        if lift:
            for robot, name in enumerate(ROBOTS):
                pose = slack.copy()
                pose[robot, 2] += lift
                targets += [(pose, f'lift {name} {lift*100:g} cm'), (slack, 'back')]
        if lateral:
            targets += [(slack+lateral*across, f'shift toward Silver {lateral*100:g} cm'),
                        (slack-lateral*across, f'shift toward Blue {lateral*100:g} cm'),
                        (slack, 'back')]
        targets.append((np.zeros((2, 3)), 'return'))
        zero = np.zeros((2, 3))
        moves = [(zero, target, label) for target, label in targets]
        p = self.person
        if p:
            # Hands: 0 = right (opposite Blue), 1 = left (opposite Silver). Toward the robots is -direction; the
            # right hand moves toward the left along +across (Blue to Silver).
            up, toward = np.array([0., 0., 1.]), -direction
            if p.get('up'):
                moves += [(np.array([up, up])*p['up'], zero, f'hands up {p["up"]*100:g} cm'), (zero, zero, 'back')]
            if p.get('down'):
                moves += [(-np.array([up, up])*p['down'], zero, f'hands down {p["down"]*100:g} cm'), (zero, zero, 'back')]
            base = toward*p.get('toward', 0.)
            if p.get('toward'):
                moves.append((base, zero, f'hands toward the robots {p["toward"]*100:g} cm'))
            if p.get('together'):
                moves += [(base+p['together']*np.array([across, -across]), zero, f'hands together {p["together"]*100:g} cm each'),
                          (base, zero, 'apart')]
            if p.get('side'):
                moves += [(base+p['side']*across, zero, f'hands to your left {p["side"]*100:g} cm'),
                          (base-p['side']*across, zero, f'hands to your right {p["side"]*100:g} cm'),
                          (base, zero, 'back')]
            moves.append((zero, zero, 'return'))
        times, poses, hands, labels = [0., self.settle], [zero]*2, [zero]*2, ['start']
        for hand, pose, label in moves:
            ramp = np.linalg.norm(pose-poses[-1], axis=1).max()/self.speed
            reach = np.linalg.norm(hand-hands[-1], axis=1).max()
            if reach > 0:   # the person hears the cue, then moves
                ramp = max(ramp, self.reaction+reach/self.person_speed)
            times += [times[-1]+ramp, times[-1]+ramp+self.hold]
            poses += [pose.copy(), pose.copy()]
            hands += [hand.copy(), hand.copy()]
            labels.append(label)
        return np.array(times), np.array(poses), labels, direction, np.array(hands)

    def _set(self, times, offsets, labels, direction, hands):
        self.times, self.offsets, self.labels, self.direction, self.hand_offsets = times, offsets, labels, direction, hands
        self.duration = float(times[-1])

    def hold_targets(self, corners):
        """Robot displacements (H, 2, 3) of every hold if the script started at `corners` (pre-motion checks)."""
        return self._profile(corners)[1][::2]

    def holds(self):
        """(start s, end s, inset m) of every hold, relative to the start of the script: the settle at the start, then
        one per target. The inset is the mean displacement toward the hands (excursions sit at the deepest inset)."""
        out = []
        for k in range(0, len(self.times)-1, 2):
            if self.times[k+1] > self.times[k]:
                inset = float(np.mean(np.sum(self.offsets[k]*self.direction, axis=1)))
                out.append((float(self.times[k]), float(self.times[k+1]), round(inset, 6)))
        return out

    def hand_targets(self, corners):
        """Expected hand displacements (H, 2, 3) of every hold if the script started at `corners`."""
        return self._profile(corners)[4][::2]

    def hand_offset(self, t):
        """Where the person's hands are asked to be (2, 3), relative to the start, t seconds into the script."""
        return np.array([np.interp(t, self.times, self.hand_offsets[:, h, c]) for h in range(2) for c in range(3)]).reshape(2, 3)

    def plan(self):
        """Every hold: start and end (s), label, robot displacements (2, 3) and expected hand displacements, m."""
        return [dict(start=float(self.times[k]), end=float(self.times[k+1]), label=self.labels[k//2],
                     target=np.round(self.offsets[k], 4).tolist(), hands=np.round(self.hand_offsets[k], 4).tolist())
                for k in range(0, len(self.times)-1, 2)]

    def start(self, time_ns, corners, label=''):
        corners = np.asarray(corners, float)
        self._set(*self._profile(corners))
        self.origin, self.start_ns, self.label = corners[:2].copy(), int(time_ns), label

    def active(self, time_ns):
        return self.start_ns is not None and 0 <= (time_ns-self.start_ns)/1e9 <= self.duration

    def window(self):
        return None if self.start_ns is None else (self.start_ns, self.start_ns+int(self.duration*1e9))

    def offset(self, t):
        """Displacement (2, 3) of the robot grips from where they started, t seconds into the script."""
        return np.array([np.interp(t, self.times, self.offsets[:, r, c]) for r in range(2) for c in range(3)]).reshape(2, 3)

    def command(self, time_ns, held):
        """Nominal velocity (2, 3) for the robot grips: follow the profile from where they are."""
        t = (time_ns-self.start_ns)/1e9
        target = self.origin+self.offset(t)
        velocity = (self.offset(t+.02)-self.offset(t))/.02+self.gain*(target-np.asarray(held, float))
        speed = np.linalg.norm(velocity, axis=1, keepdims=True)
        return velocity*np.minimum(1., 2*self.speed/np.maximum(speed, 1e-12))

    def summary(self):
        moves = [f'{n} {v*100:g} cm' for n, v in zip(('narrow', 'lift', 'shift'), self.extra) if v]
        moves += [f'hands {n} {v*100:g} cm' for n, v in self.person.items() if v]
        return (f'insets {self.insets} m' + (', ' + ', '.join(moves) if moves else '')
                + f' at {self.speed} m/s, {self.duration:.0f} s')

    def describe(self):
        if self.start_ns is None:
            return 'idle'
        return f'{self.summary()}, started at {self.start_ns/1e9:.2f} s'
