"""Run on each robot helper, in the robot namespace, next to command_guard.

Subscribes to this robot's world-frame grip velocity from the fabric twin
(`fabric_cbf/cmd_world`), the compensated wrist force and TF, and publishes the arm command
`commands/fabric_cbf/arm`, which command_guard forwards only in control mode `fabric_cbf` with
the deadman held. Arm with `fabric_cbf/arm` (std_msgs/Bool true); any stop needs a new arm.
This is not a hardware E-stop.

`output: joint` sends joint velocities on `commands/joint/arm` instead (the guard forwards them under
the same conditions). The bridge's limits and stops are unchanged; its safe grip velocity is converted
with the robot's own model (`robot_description`, `joint_states`, the grip offset from TF), holding the
tool orientation it had at arming. Use it where Kinova's Cartesian control is refused: at the fabric
grasp pose both arms report CartesianFaultState 1 and ignore Cartesian velocity (2026-11-18).
"""
from time import monotonic

import rclpy
import rclpy.executors
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped, WrenchStamped
from math import degrees, radians

import numpy as np
from kinova_msgs.msg import JointVelocity, PoseVelocity
from rclpy.qos import DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformException, TransformListener

from oarbot_runtime.arm_kinematics import Chain, GripRates
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
        self.output_mode = param('output', 'cartesian')
        if self.output_mode not in ('cartesian', 'joint'):
            raise ValueError("output must be 'cartesian' or 'joint'")
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, lifespan=Duration(seconds=self.bridge.command_timeout))
        if self.output_mode == 'joint':
            # The chain runs from the command frame (the arm base the twin's velocity is rotated into)
            # to the end effector; the grip offset (end effector -> tool frame) comes from TF.
            self.end_effector = param('end_effector_frame', prefix+'/j2n6s300_end_effector')
            self.max_joint_rate = radians(param('max_joint_dps', 10.))
            self.orientation_gain = param('orientation_gain_per_s', 1.)
            self.ik_damping = param('ik_damping', .01)
            self.grip = self.joints = None
            self.joint_pub = self.create_publisher(JointVelocity, 'commands/joint/arm', qos)
            self.create_subscription(JointState, 'joint_states', self.joint_states, 10)
            self.create_subscription(String, 'robot_description', self.description,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        else:
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

    def description(self, msg):
        try:
            chain = Chain(msg.data, self.command_frame, self.end_effector)
        except Exception as error:  # a description without this chain: stay without a model
            self.get_logger().error(f'robot_description: {error}')
            return
        self.grip = GripRates(chain, self.max_joint_rate, self.orientation_gain, self.ik_damping)

    def joint_states(self, msg):
        if self.grip is None:
            return
        position = dict(zip(msg.name, msg.position))
        if all(name in position for name in self.grip.chain.names):
            self.joints = (np.array([position[name] for name in self.grip.chain.names]), monotonic())

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
        if self.output_mode == 'joint':
            self.joint_tick()
        else:
            self.publisher.publish(PoseVelocity(**dict(zip(FIELDS, self.bridge.output(monotonic())))))

    def joint_tick(self):
        now = monotonic()
        out = self.bridge.output(now)
        rates = np.zeros(6)
        if self.bridge.tripped or self.grip is None:
            if self.grip is not None:
                self.grip.reset()  # the next arming holds the orientation it starts from
        elif self.joints is None or now-self.joints[1] > self.bridge.pose_timeout:
            self.bridge.trip('joint states went stale' if self.joints else 'no joint states')
        elif any(out[:3]):
            try:
                offset = self.tf.lookup_transform(self.end_effector, self.tool_frame, Time()).transform
                tip = np.eye(4)
                tip[:3, :3] = np.array(rotation_matrix(offset.rotation))
                tip[:3, 3] = [offset.translation.x, offset.translation.y, offset.translation.z]
                rates = self.grip.rates(self.joints[0], out[:3], tip)
            except TransformException:
                self.bridge.trip(f'no transform {self.end_effector} -> {self.tool_frame}')
        self.joint_pub.publish(JointVelocity(**{f'joint{i}': float(degrees(v)) for i, v in enumerate(rates, 1)}))

    def diagnostics(self):
        tripped = self.bridge.tripped
        self.diagnostics_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[
            DiagnosticStatus(level=DiagnosticStatus.WARN if tripped else DiagnosticStatus.OK,
                             name=self.get_namespace()+'/fabric_cbf_bridge', hardware_id=self.get_namespace(),
                             message=tripped or 'Following the fabric twin',
                             values=[KeyValue(key='output', value=self.output_mode),
                                     KeyValue(key='command_frame', value=self.command_frame),
                                     KeyValue(key='force_limit_n', value=str(self.bridge.force_limit)),
                                     KeyValue(key='require_force', value=str(self.bridge.require_force))])]))


def main(args=None):
    rclpy.init(args=args)
    node = FabricCbfBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.bridge.disarm()
        try:
            node.tick()  # a last zero, if the context still allows publishing
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
