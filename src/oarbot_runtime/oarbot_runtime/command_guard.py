"""Run on each robot helper, next to the driver. This is not a hardware E-stop."""
from math import isfinite
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
from rclpy.clock import Clock, ClockType
from geometry_msgs.msg import TwistStamped
from kinova_msgs.msg import PoseVelocity
from std_msgs.msg import Bool, String
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from oarbot_runtime.guard import Guard

FIELDS = ('twist_linear_x', 'twist_linear_y', 'twist_linear_z',
          'twist_angular_x', 'twist_angular_y', 'twist_angular_z')


class CommandGuard(Node):
    def __init__(self):
        super().__init__('command_guard')
        if self.get_namespace() == '/':
            raise ValueError('Run command_guard in the robot namespace')
        self.ambiguous = False
        self.guard = Guard(self.declare_parameter('heartbeat_timeout_sec', 0.25).value,
                           self.declare_parameter('command_timeout_sec', 0.15).value)
        self.limits = [self.declare_parameter(name, value).value for name, value in (
            ('max_arm_linear_mps', 0.1), ('max_arm_angular_rps', 0.3),
            ('max_base_linear_mps', 0.15), ('max_base_angular_rps', 0.3))]
        if not all(isfinite(v) and v > 0 for v in self.limits):
            raise ValueError('Speed limits must be finite and positive')
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.guard.command_timeout))
        self.arm_pub = self.create_publisher(PoseVelocity, 'kinova/j2n6s300_driver/in/cartesian_velocity', qos)
        self.base_pub = self.create_publisher(TwistStamped, 'dingo/cmd_vel', qos)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(Bool, '/tablet_gui/deadman', lambda m: self.guard.deadman_update(m.data, monotonic()), 1)
        self.create_subscription(Bool, '/tablet_gui/e_stop', lambda m: self.guard.estop_update(m.data, monotonic()), 1)
        self.create_subscription(String, 'control_mode', lambda m: self.guard.mode_update(m.data, monotonic()), 1)
        self.create_subscription(Bool, 'base_enable', lambda m: self.guard.base_update(m.data, monotonic()), 1)
        for source in ('teleop', 'admittance'):
            self.create_subscription(PoseVelocity, f'commands/{source}/arm',
                lambda m, source=source: self.guard.command(source, tuple(getattr(m, f) for f in FIELDS), monotonic(), *self.limits[:2]), qos)
        self.create_subscription(TwistStamped, 'commands/teleop/base', self.base_command, qos)
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.01, self.tick, clock=self.steady_clock)
        self.create_timer(0.5, self.diagnostics, clock=self.steady_clock)

    def base_command(self, msg):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if not 0 <= age <= self.guard.command_timeout:
            self.guard.commands.pop('base', None)
            return
        v = msg.twist
        expected_frame = self.get_namespace().strip('/') + '/base_link'
        if msg.header.frame_id != expected_frame or not all(isfinite(x) for x in (
                v.linear.x, v.linear.y, v.linear.z, v.angular.x, v.angular.y, v.angular.z)):
            self.guard.commands.pop('base', None)
            return
        # Dingo commands are planar and expressed in its base frame.
        values = (v.linear.x, v.linear.y, 0., 0., 0., v.angular.z)
        self.guard.command('base', values, monotonic(), *self.limits[2:])

    def tick(self):
        if self.ambiguous:
            self.guard.trip()
        arm, base = self.guard.output(monotonic())
        self.arm_pub.publish(PoseVelocity(**dict(zip(FIELDS, arm))))
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_namespace().strip('/') + '/base_link'
        msg.twist.linear.x, msg.twist.linear.y = base[:2]
        msg.twist.angular.z = base[5]
        self.base_pub.publish(msg)

    def diagnostics(self):
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Multiple controllers on a selected input are a configuration fault, not last-writer-wins.
        inputs = ('control_mode', 'base_enable', 'commands/teleop/arm', 'commands/admittance/arm', 'commands/teleop/base')
        self.ambiguous = any(self.count_publishers(topic) > 1 for topic in inputs)
        if self.ambiguous:
            self.guard.trip()
        permitted = self.guard.permitted(monotonic()) and not self.ambiguous
        msg.status = [DiagnosticStatus(level=DiagnosticStatus.OK if permitted else DiagnosticStatus.WARN,
            name=self.get_namespace() + '/command_guard', hardware_id=self.get_namespace(),
            message='Multiple command publishers; resolve controller conflict' if self.ambiguous else
                    'Enabled' if permitted else 'Stopped; fresh heartbeat and deadman release/press required',
            values=[KeyValue(key='mode', value=self.guard.mode)])]
        self.diagnostics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CommandGuard()
    try:
        rclpy.spin(node)
    finally:
        node.guard.trip()
        node.tick()
        node.destroy_node()
        rclpy.try_shutdown()
