"""Warm-up excitation: the robots give the sheet measured amounts of slack.

A sheet held taut at four corners has almost the same shape whatever its bending and shear
stiffness, so a taut sheet tells the calibration nothing. This script moves the two robot
grips toward the human grips by a few centimetres in steps, holds at each step, and returns.
The human keeps both hands still. It produces a nominal velocity for the two robot grips; the
CBF still filters that command, and the command guard still limits it.
"""
import numpy as np


class WarmupScript:
    def __init__(self, insets_m=(.02, .04), speed_mps=.03, hold_sec=5., settle_sec=3., gain_per_s=1.5):
        insets = [float(x) for x in insets_m]
        if (not insets or not np.isfinite(insets+[speed_mps, hold_sec, settle_sec, gain_per_s]).all()
                or min(insets) <= 0 or max(insets) > .2 or speed_mps <= 0 or speed_mps > .1 or min(hold_sec, settle_sec) < 0):
            raise ValueError('Warm-up insets must be 0-0.2 m and the speed 0-0.1 m/s')
        self.insets, self.speed, self.hold, self.settle, self.gain = insets, speed_mps, hold_sec, settle_sec, gain_per_s
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
        return [(a[0], b[0], a[1]) for a, b in zip(self.knots[:-1], self.knots[1:]) if a[1] == b[1] and b[0] > a[0]]

    def start(self, time_ns, corners, label=''):
        corners = np.asarray(corners, float)
        direction = corners[2:]-corners[:2]
        direction[:, 2] = 0.
        self.direction = direction/np.linalg.norm(direction, axis=1, keepdims=True)
        self.origin, self.start_ns, self.label = corners[:2].copy(), int(time_ns), label

    def active(self, time_ns):
        return self.start_ns is not None and 0 <= (time_ns-self.start_ns)/1e9 <= self.duration

    def window(self):
        return None if self.start_ns is None else (self.start_ns, self.start_ns+int(self.duration*1e9))

    def command(self, time_ns, held):
        """Nominal velocity (2, 3) for the robot grips: follow the inset profile from where they are."""
        t = (time_ns-self.start_ns)/1e9
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
