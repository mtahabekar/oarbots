"""Run on each robot helper, in the robot namespace, next to command_guard.

Subscribes to this robot's world-frame grip velocity from the fabric twin
(`fabric_cbf/cmd_world`), the compensated wrist force and TF, and publishes the arm command
`commands/fabric_cbf/arm`, which command_guard forwards only in control mode `fabric_cbf` with
the deadman held. Arm with `fabric_cbf/arm` (std_msgs/Bool true); any stop needs a new arm.
This is not a hardware E-stop.
"""
from time import monotonic

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped, WrenchStamped
from kinova_msgs.msg import PoseVelocity
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener

from oarbot_runtime.cbf_bridge import CbfBridge

FIELDS = ('twist_linear_x', 'twist_linear_y', 'twist_linear_z', 'twist_angular_x', 'twist_angular_y', 'twist_angular_z')


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
            [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
            [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]


class FabricCbfBridge(Node):
    def __init__(self):
        super().__init__('fabric_cbf_bridge')
        if self.get_namespace() == '/':
            raise ValueError('Run fabric_cbf_bridge in the robot namespace')
        prefix = self.get_namespace().strip('/')
        param = lambda name, default: self.declare_parameter(name, default).value  # noqa: E731
        self.world = param('world_frame', 'world')
        # The frame the Kinova driver interprets Cartesian velocities in. Verify per robot (gate
        # H3): Silver's arm base is mounted 180 degrees about Z relative to its chassis.
        self.command_frame = param('command_frame', prefix+'/j2n6s300_link_base')
        self.tool_frame = param('tool_frame', prefix+'/fabric_grip')
        self.bridge = CbfBridge(param('max_speed_mps', .1), param('command_timeout_sec', .1), param('force_limit_n', 6.),
                                param('force_timeout_sec', .25), param('workspace_min', [-10., -10., -10.]),
                                param('workspace_max', [10., 10., 10.]), param('pose_timeout_sec', .25),
                                param('require_force', True), param('force_window_sec', .25), param('workspace_margin_m', .02))
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.bridge.command_timeout))
        self.publisher = self.create_publisher(PoseVelocity, 'commands/fabric_cbf/arm', qos)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_subscription(TwistStamped, 'fabric_cbf/cmd_world', self.command, 10)
        self.create_subscription(WrenchStamped, 'force_torque_calibrated', self.force, qos_profile_sensor_data)
        self.create_subscription(Bool, 'fabric_cbf/arm', self.arm, 1)
        self.create_timer(.01, self.tick)
        self.create_timer(.5, self.diagnostics)

    def command(self, msg):
        if msg.header.frame_id != self.world:
            self.bridge.trip('command is not in the world frame')
            return
        age = (self.get_clock().now()-Time.from_msg(msg.header.stamp)).nanoseconds/1e9
        v = msg.twist.linear
        self.bridge.command_update((v.x, v.y, v.z), age, monotonic())

    def force(self, msg):
        f = msg.wrench.force
        self.bridge.force_update((f.x, f.y, f.z), monotonic())

    def arm(self, msg):
        if msg.data:
            self.get_logger().warn('armed' if self.bridge.rearm(monotonic()) else self.bridge.tripped)
        else:
            self.bridge.disarm()

    def tick(self):
        try:
            tool = self.tf.lookup_transform(self.world, self.tool_frame, Time()).transform.translation
            frame = self.tf.lookup_transform(self.command_frame, self.world, Time()).transform.rotation
            self.bridge.pose_update((tool.x, tool.y, tool.z), rotation_matrix(frame), monotonic())
        except TransformException:
            pass  # the pose then goes stale and the bridge stops
        self.publisher.publish(PoseVelocity(**dict(zip(FIELDS, self.bridge.output(monotonic())))))

    def diagnostics(self):
        tripped = self.bridge.tripped
        self.diagnostics_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[
            DiagnosticStatus(level=DiagnosticStatus.WARN if tripped else DiagnosticStatus.OK,
                             name=self.get_namespace()+'/fabric_cbf_bridge', hardware_id=self.get_namespace(),
                             message=tripped or 'Following the fabric twin',
                             values=[KeyValue(key='command_frame', value=self.command_frame),
                                     KeyValue(key='force_limit_n', value=str(self.bridge.force_limit)),
                                     KeyValue(key='require_force', value=str(self.bridge.require_force))])]))


def main(args=None):
    rclpy.init(args=args)
    node = FabricCbfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.bridge.disarm()
        node.tick()
        node.destroy_node()
        rclpy.try_shutdown()
