"""Pure logic of the fabric-CBF bridge: frame conversion and independent stops. No ROS.

The fabric twin publishes a world-frame velocity for this robot's grip. The bridge runs on the
robot's own computer, next to the command guard, and turns that into the arm command the guard
accepts. It also decides, from what this computer measures itself, whether the arm may follow
the twin at all. A stop latches: it is cleared only by an explicit re-arm, never by the
condition going away.
"""
from math import isfinite, sqrt


def norm(v):
    return sqrt(sum(x*x for x in v))


class CbfBridge:
    def __init__(self, max_speed=.1, command_timeout=.1, force_limit_n=6., force_timeout=.25,
                 workspace_min=(-1e9, -1e9, -1e9), workspace_max=(1e9, 1e9, 1e9), pose_timeout=.25):
        values = [max_speed, command_timeout, force_limit_n, force_timeout, pose_timeout, *workspace_min, *workspace_max]
        if (not all(isfinite(v) for v in values) or min(max_speed, command_timeout, force_limit_n, force_timeout, pose_timeout) <= 0
                or any(a >= b for a, b in zip(workspace_min, workspace_max))):
            raise ValueError('Invalid fabric CBF bridge limits')
        self.max_speed, self.command_timeout = max_speed, command_timeout
        self.force_limit, self.force_timeout, self.pose_timeout = force_limit_n, force_timeout, pose_timeout
        self.workspace = tuple(workspace_min), tuple(workspace_max)
        self.command = self.force = self.baseline = self.pose = None
        self.tripped = 'not armed'

    # ---------------------------------------------------------------- inputs (times: monotonic seconds)
    def command_update(self, linear_world, sent_age, now):
        """A world-frame grip velocity from the twin, `sent_age` seconds old by its own stamp."""
        if len(linear_world) != 3 or not all(isfinite(v) for v in linear_world) or not isfinite(sent_age):
            self.trip('invalid command from the twin')
            return
        if not -.05 <= sent_age <= self.command_timeout:
            return  # a late command is not a command; the output falls to zero when none is fresh
        self.command = (tuple(linear_world), now)

    def force_update(self, force, now):
        """Compensated wrist force (N), any fixed frame: only the change in magnitude is used."""
        if len(force) != 3 or not all(isfinite(v) for v in force):
            self.trip('invalid wrist force')
            return
        self.force = (tuple(force), now)
        if self.baseline is not None and norm([a-b for a, b in zip(force, self.baseline)]) > self.force_limit:
            self.trip(f'wrist force changed by more than {self.force_limit:.0f} N')

    def pose_update(self, tool_world, rotation_command_from_world, now):
        """Tool position in the world and the 3x3 rotation taking world vectors into the frame the
        arm driver expects (row-major nested lists)."""
        self.pose = (tuple(tool_world), [tuple(row) for row in rotation_command_from_world], now)
        low, high = self.workspace
        if not all(a <= x <= b for x, a, b in zip(tool_world, low, high)):
            self.trip('tool left the workspace box')

    # ---------------------------------------------------------------- arming
    def trip(self, reason):
        if not self.tripped:
            self.tripped = reason

    def rearm(self, now):
        """Arm, taking the current wrist force as the baseline. Refused without fresh inputs."""
        if self.force is None or now-self.force[1] > self.force_timeout:
            self.tripped = 'cannot arm: no fresh wrist force'
        elif self.pose is None or now-self.pose[2] > self.pose_timeout:
            self.tripped = 'cannot arm: no fresh tool pose'
        else:
            low, high = self.workspace
            if not all(a <= x <= b for x, a, b in zip(self.pose[0], low, high)):
                self.tripped = 'cannot arm: tool outside the workspace box'
            else:
                self.baseline, self.tripped = self.force[0], ''
        return not self.tripped

    def disarm(self):
        self.tripped, self.baseline = 'not armed', None

    # ---------------------------------------------------------------- output
    def output(self, now):
        """Six arm velocity components in the driver's frame (angular always zero)."""
        zero = (0.,)*6
        if self.tripped:
            return zero
        if self.force is None or now-self.force[1] > self.force_timeout:
            self.trip('wrist force went stale')
            return zero
        if self.pose is None or now-self.pose[2] > self.pose_timeout:
            self.trip('tool pose went stale')
            return zero
        if self.command is None or now-self.command[1] > self.command_timeout:
            return zero  # no fresh command: hold still, stay armed
        linear, rotation = self.command[0], self.pose[1]
        speed = norm(linear)
        scale = min(1., self.max_speed/speed) if speed else 1.
        rotated = tuple(scale*sum(rotation[i][j]*linear[j] for j in range(3)) for i in range(3))
        return rotated+(0., 0., 0.)
