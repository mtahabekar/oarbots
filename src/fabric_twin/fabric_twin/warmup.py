"""Warm-up excitation: the robots give the sheet measured amounts of slack.

A sheet held taut at four corners has almost the same shape whatever its bending and shear
stiffness, so a taut sheet tells the calibration nothing. This script moves the two robot
grips toward the human grips by a few centimetres in steps, holds at each step, and returns.
The human keeps both hands still. It produces a nominal velocity for the two robot grips; the
CBF still filters that command, and the command guard still limits it.
"""
import numpy as np


class WarmupScript:
    def __init__(self, insets_m=(.02, .04), speed_mps=.03, hold_sec=5., settle_sec=3., gain_per_s=1.5, transverse_m=0., lift_m=0., lateral_m=0.):
        insets = [float(x) for x in insets_m]
        if (not insets or not np.isfinite(insets+[speed_mps, hold_sec, settle_sec, gain_per_s]).all()
                or min(insets) <= 0 or max(insets) > .2 or speed_mps <= 0 or speed_mps > .1 or min(hold_sec, settle_sec) < 0):
            raise ValueError('Warm-up insets must be 0-0.2 m and the speed 0-0.1 m/s')
        self.insets, self.speed, self.hold, self.settle, self.gain = insets, speed_mps, hold_sec, settle_sec, gain_per_s
        extra = np.array([transverse_m, lift_m, lateral_m], float)
        if not np.isfinite(extra).all() or np.any(extra < 0) or np.any(extra > .1):
            raise ValueError('Additional excursions must be finite and between 0 and 0.1 m')
        self.extra = extra
        self.start_ns = None
        # Knots (time s, inset m) of the piecewise-linear profile: settle, then ramp and hold at
        # each inset, then return and hold.
        knots, t, level = [(0., 0.), (settle_sec, 0.)], settle_sec, 0.
        for inset in insets+[0.]:
            t += abs(inset-level)/speed_mps; knots.append((t, inset))
            t += hold_sec; knots.append((t, inset)); level = inset
        self.knots = np.array(knots)
        self.duration = float(self.knots[-1, 0])

    def holds(self):
        """(start s, end s, inset m) of every hold, relative to the start of the script."""
        if getattr(self, 'offsets', None) is not None:
            return [(a[0], b[0], float(i)) for i, (a, b) in enumerate(zip(self.knots[:-1], self.knots[1:]))
                    if b[0] > a[0] and np.array_equal(self.offsets[i], self.offsets[i+1])]
        return [(a[0], b[0], a[1]) for a, b in zip(self.knots[:-1], self.knots[1:]) if a[1] == b[1] and b[0] > a[0]]

    def start(self, time_ns, corners, label=''):
        corners = np.asarray(corners, float)
        direction = corners[2:]-corners[:2]
        direction[:, 2] = 0.
        self.direction = direction/np.linalg.norm(direction, axis=1, keepdims=True)
        if not np.isfinite(corners).all() or not np.isfinite(self.direction).all():
            raise ValueError('Warm-up requires finite, separated grips')
        self.origin, self.start_ns, self.label = corners[:2].copy(), int(time_ns), label
        self.offsets = None
        if np.any(self.extra):
            transverse, lift, lateral = self.extra
            across = corners[1]-corners[0]
            across[2] = 0.
            if np.linalg.norm(across) < 1e-6:
                raise ValueError('Robot grips must be separated horizontally')
            across /= np.linalg.norm(across)
            zero = np.zeros((2, 3))
            slack = self.insets[-1]*self.direction
            targets = [inset*self.direction for inset in self.insets]
            if transverse:
                targets.append(slack + transverse*np.array([across, -across]))
                targets.append(slack.copy())
            if lift:
                for robot in (0, 1):
                    pose = slack.copy(); pose[robot, 2] += lift
                    targets.extend([pose, slack.copy()])
            if lateral:
                targets.extend([slack+lateral*across, slack-lateral*across, slack.copy()])
            targets.append(zero)
            times, poses = [0., self.settle], [zero, zero]
            for pose in targets:
                duration = np.linalg.norm(pose-poses[-1], axis=1).max()/self.speed
                times.extend([times[-1]+duration, times[-1]+duration+self.hold])
                poses.extend([pose.copy(), pose.copy()])
            self.offsets = np.array(poses)
            self.knots = np.column_stack((times, np.arange(len(times))))
            self.duration = times[-1]

    def active(self, time_ns):
        return self.start_ns is not None and 0 <= (time_ns-self.start_ns)/1e9 <= self.duration

    def window(self):
        return None if self.start_ns is None else (self.start_ns, self.start_ns+int(self.duration*1e9))

    def command(self, time_ns, held):
        """Nominal velocity (2, 3) for the robot grips: follow the inset profile from where they are."""
        t = (time_ns-self.start_ns)/1e9
        if self.offsets is not None:
            def offset(at):
                return np.array([np.interp(at, self.knots[:, 0], self.offsets[:, r, c])
                                 for r in range(2) for c in range(3)]).reshape(2, 3)
            target = self.origin+offset(t)
            velocity = (offset(t+.02)-offset(t))/.02+self.gain*(target-np.asarray(held, float))
            speed = np.linalg.norm(velocity, axis=1, keepdims=True)
            return velocity*np.minimum(1., 2*self.speed/np.maximum(speed, 1e-12))
        inset = np.interp(t, self.knots[:, 0], self.knots[:, 1])
        rate = (np.interp(t+.02, self.knots[:, 0], self.knots[:, 1])-inset)/.02
        target = self.origin+inset*self.direction
        velocity = rate*self.direction+self.gain*(target-np.asarray(held, float))
        speed = np.linalg.norm(velocity, axis=1, keepdims=True)
        return velocity*np.minimum(1., 2*self.speed/np.maximum(speed, 1e-12))

    def describe(self):
        if self.start_ns is None:
            return 'idle'
        return f'insets {self.insets} m at {self.speed} m/s, {self.duration:.0f} s, started at {self.start_ns/1e9:.2f} s'
