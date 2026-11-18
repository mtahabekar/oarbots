"""Deterministic watchdog state machine; times are monotonic seconds."""
from math import isfinite, sqrt


def limited(values, linear_limit, angular_limit):
    if len(values) != 6 or not all(isfinite(v) for v in values):
        raise ValueError('Velocity must have six finite components')
    result = []
    for vector, limit in ((values[:3], linear_limit), (values[3:], angular_limit)):
        norm = sqrt(sum(v*v for v in vector))
        scale = min(1.0, limit / norm) if norm else 1.0
        result.extend(v * scale for v in vector)
    return tuple(result)


# 'fabric_cbf': arm velocities from the fabric twin's sensitivity CBF, through fabric_cbf_bridge.
MODES = ('disabled', 'teleop', 'admittance', 'fabric_cbf')


class Guard:
    def __init__(self, timeout=0.25, command_timeout=0.15, hold_fingers=None):
        if not all(isfinite(v) and v > 0 for v in (timeout, command_timeout)):
            raise ValueError('Timeouts must be finite and positive')
        if hold_fingers is not None and not (isfinite(hold_fingers) and 0. <= hold_fingers <= 100.):
            raise ValueError('hold_fingers must be a closure percentage in 0..100')
        self.hold_fingers = None if hold_fingers is None else float(hold_fingers)
        self.timeout, self.command_timeout = timeout, command_timeout
        self.deadman, self.estop = False, True
        self.deadman_time = self.estop_time = float('-inf')
        self.mode, self.base_enabled = 'disabled', False
        self.mode_time = self.base_time = float('-inf')
        self.armed = self.release_seen = False
        self.commands = {}

    def deadman_update(self, value, now):
        self.deadman, self.deadman_time = bool(value), now
        # A fresh release with a fresh disengaged stop is required after loss/stop.
        if not value and not self.estop and 0 <= now-self.estop_time <= self.timeout:
            self.release_seen = True
        if not value:
            self.armed = False
            self.commands.clear()

    def estop_update(self, value, now):
        self.estop, self.estop_time = bool(value), now
        if value:
            self.trip()

    def trip(self):
        self.armed = self.release_seen = False
        self.commands.clear()

    def mode_update(self, mode, now):
        if mode not in MODES:
            mode = 'disabled'
        if mode != self.mode:
            self.commands.clear()
        self.mode, self.mode_time = mode, now

    def base_update(self, enabled, now):
        if enabled != self.base_enabled:
            self.commands.pop('base', None)
        self.base_enabled, self.base_time = bool(enabled), now

    def permitted(self, now):
        if self.estop or not all(0 <= now-t <= self.timeout for t in (self.deadman_time, self.estop_time)):
            self.trip()
            return False
        if not self.deadman:
            self.armed = False
            return False
        if self.release_seen:
            self.armed = True
        return self.armed

    def command(self, source, values, now, linear_limit, angular_limit):
        if not self.permitted(now):
            return
        try:
            self.commands[source] = (limited(values, linear_limit, angular_limit), now)
        except ValueError:
            self.trip()

    def finger_output(self, now):
        """Finger closure (percent) to send with this tick's arm command, or None to leave the fingers as they
        are. With hold_fingers set, the fingers are driven to it only while the deadman is held: releasing it
        stops them where they are, and the driver's own start-up (which opens them) is never fought."""
        return self.hold_fingers if self.hold_fingers is not None and self.permitted(now) else None

    def output(self, now):
        zero = (0.,)*6
        if not self.permitted(now):
            return zero, zero
        def get(source):
            values, stamp = self.commands.get(source, (zero, float('-inf')))
            return values if 0 <= now-stamp <= self.command_timeout else zero
        arm = get(self.mode) if self.mode != 'disabled' and 0 <= now-self.mode_time <= self.timeout else zero
        base = get('base') if self.base_enabled and 0 <= now-self.base_time <= self.timeout else zero
        return arm, base
