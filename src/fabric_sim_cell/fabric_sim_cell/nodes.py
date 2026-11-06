"""ROS nodes the simulated cell adds to the normal stack.

* `calibration_publisher`: the deployment's static calibration (one TF authority), with
  optional deliberate errors for sensitivity studies.
* `oracle_hands`: an explicitly labelled ideal hand input. It turns the plant's mocap of the
  leader grips into `/fabric/hand_states`, bypassing glove detection. Use it to isolate the
  estimator from perception; never report it as hand tracking.
* `grasp_gate`: passes hand states to the estimator only while both robots report closed
  fingers (Kinova driver joint states), so estimation starts when the four corners are held, as
  an operator would start it on the real cell. Before that the estimator would pin its robot
  corners at the approaching tools, far from the cloth's real corners.
* `recorder`: writes `/fabric/estimated_state` in the estimator's replay NPZ format.
"""
import json
from pathlib import Path

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, TransformStamped, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

from fabric_sim_cell.core import HANDS, OracleNoise, calibration_transforms, fingers_closed, hand_covariance

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def nanoseconds(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class CalibrationPublisher(Node):
    def __init__(self):
        super().__init__('sim_cell_calibration')
        path = self.declare_parameter('calibration_file', '').value
        errors_path = self.declare_parameter('errors_file', '').value
        calibration = yaml.safe_load(Path(path).read_text())
        errors = yaml.safe_load(Path(errors_path).read_text()).get('errors', {}) if errors_path else {}
        transforms = calibration_transforms(calibration, errors)
        msgs = []
        for t in transforms:
            m = TransformStamped(header=Header(stamp=Time(), frame_id=t['parent']), child_frame_id=t['child'])
            m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = map(float, t['translation'])
            (m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z,
             m.transform.rotation.w) = map(float, t['rotation_xyzw'])
            msgs.append(m)
        self.publisher = self.create_publisher(TFMessage, '/tf_static', LATCHED)
        self.publisher.publish(TFMessage(transforms=msgs))
        self.get_logger().info(f'published {len(msgs)} calibration transforms from {path}'
                               + (f' with deliberate errors on {sorted(errors)}' if errors else ''))


class OracleHands(Node):
    def __init__(self):
        super().__init__('oracle_hands')
        from fabric_state_msgs.msg import HandState, HandStates
        self.HandState, self.HandStates = HandState, HandStates
        self.world = self.declare_parameter('world_frame', 'world').value
        self.covariance = hand_covariance(self.declare_parameter('position_std_m', 0.005).value,
                                          self.declare_parameter('velocity_std_mps', 0.05).value)
        self.noise = OracleNoise(self.declare_parameter('noise_position_std_m', 0.0).value,
                                 self.declare_parameter('noise_velocity_std_mps', 0.0).value,
                                 self.declare_parameter('seed', 0).value)
        self.period_ns = int(1e9 / self.declare_parameter('rate_hz', 15.0).value)
        self.latest, self.last_publish = {}, -1
        self.publisher = self.create_publisher(HandStates, '/fabric/hand_states', qos_profile_sensor_data)
        self.subscriptions_ = [self.create_subscription(Odometry, f'/sim/mocap/{name}',
                                                        lambda m, n=name: self.mocap(n, m), qos_profile_sensor_data)
                               for name in HANDS]
        self.get_logger().warn('ORACLE hand input: leader grips come from the plant, not from glove detection')

    def mocap(self, name, msg):
        if msg.header.frame_id != self.world:
            self.get_logger().error(f'mocap {name} is in {msg.header.frame_id!r}, expected {self.world!r}')
            return
        self.latest[name] = msg
        stamps = {nanoseconds(m.header.stamp) for m in self.latest.values()}
        if len(self.latest) < len(HANDS) or len(stamps) != 1:
            return
        stamp = stamps.pop()
        if self.last_publish >= 0 and stamp - self.last_publish < self.period_ns:
            return
        self.last_publish = stamp
        hands = []
        for key, m in sorted(self.latest.items()):
            hand_id, tag, uv = HANDS[key]
            p, v = m.pose.pose.position, m.twist.twist.linear
            position, velocity = self.noise.apply([p.x, p.y, p.z], [v.x, v.y, v.z])
            hands.append(self.HandState(hand_id=hand_id, tag_id=tag, position=Point(x=position[0], y=position[1], z=position[2]),
                                        velocity=Vector3(x=velocity[0], y=velocity[1], z=velocity[2]),
                                        covariance=self.covariance, source_cameras=['oracle_mocap'],
                                        source_stamps=[m.header.stamp], holding_state=self.HandState.HOLDING,
                                        holding_is_assumed=True, grip_uv=list(uv)))
        self.publisher.publish(self.HandStates(header=Header(stamp=next(iter(self.latest.values())).header.stamp,
                                                             frame_id=self.world), hands=hands))


class GraspGate(Node):
    """Forward `/fabric/hand_states_ungated` to `/fabric/hand_states` while both grippers are closed."""

    def __init__(self):
        super().__init__('grasp_gate')
        from fabric_state_msgs.msg import HandStates
        from sensor_msgs.msg import JointState
        self.threshold = self.declare_parameter('closed_rad', 1.0).value
        robots = self.declare_parameter('robots', ['oarbot_blue', 'oarbot_silver']).value
        self.closed = {robot: False for robot in robots}
        self.open_since_start, self.forwarded = True, 0
        self.publisher = self.create_publisher(HandStates, '/fabric/hand_states', qos_profile_sensor_data)
        self.create_subscription(HandStates, '/fabric/hand_states_ungated', self.hands, qos_profile_sensor_data)
        self.subscriptions_ = [self.create_subscription(
            JointState, f'/{robot}/kinova/j2n6s300_driver/out/joint_state',
            lambda msg, r=robot: self.fingers(r, msg), qos_profile_sensor_data) for robot in robots]

    def fingers(self, robot, msg):
        closed = fingers_closed(msg.name, msg.position, self.threshold)
        if closed != self.closed[robot]:
            self.get_logger().info(f'{robot} gripper {"closed" if closed else "open"} at {msg.header.stamp.sec}.'
                                   f'{msg.header.stamp.nanosec:09d} s')
        self.closed[robot] = closed

    def hands(self, msg):
        if all(self.closed.values()):
            if self.open_since_start:
                self.get_logger().info('both grippers closed: hand states now reach the estimator')
                self.open_since_start = False
            self.publisher.publish(msg)
            self.forwarded += 1


class Recorder(Node):
    """Collects estimator states; writes the replay NPZ format on exit and every few seconds."""

    def __init__(self):
        super().__init__('sim_cell_recorder')
        from diagnostic_msgs.msg import DiagnosticArray
        from fabric_state_msgs.msg import FabricState
        self.output = Path(self.declare_parameter('output', 'estimate.npz').value)
        self.states, self.status = [], []
        # Calibrated wrist forces, rotated to world at their own stamps, to check the F/T chain.
        self.forces = {name: [] for name in ('oarbot_blue', 'oarbot_silver')}
        if self.declare_parameter('record_forces', False).value:
            from geometry_msgs.msg import WrenchStamped
            from tf2_ros import Buffer, TransformListener
            self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
            for name in self.forces:
                self.create_subscription(WrenchStamped, f'/{name}/force_torque_calibrated',
                                         lambda m, n=name: self.force(n, m), qos_profile_sensor_data)
        self.create_subscription(FabricState, '/fabric/estimated_state', self.state, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/fabric/estimator_status', self.diagnostics, 10)
        self.create_timer(5.0, self.save)

    def state(self, msg):
        self.states.append(dict(
            time=nanoseconds(msg.header.stamp), measurement=nanoseconds(msg.measurement_stamp), predicted=msg.predicted,
            received=self.get_clock().now().nanoseconds, degraded=msg.degraded,
            q=np.array([[p.x, p.y, p.z] for p in msg.positions]), v=np.array([[p.x, p.y, p.z] for p in msg.velocities]),
            covariance=np.asarray(msg.covariance, dtype=float), visibility=np.asarray(msg.visibility, dtype=np.uint8),
            age=np.asarray(msg.observation_age_sec, dtype=float),
            residual=[msg.residual_force.x, msg.residual_force.y, msg.residual_force.z]))

    def force(self, name, msg):
        from rclpy.time import Time as RosTime
        from tf2_ros import TransformException
        try:
            t = self.tf.lookup_transform('world', msg.header.frame_id, RosTime.from_msg(msg.header.stamp))
        except TransformException:
            return
        q = t.transform.rotation
        x, y, z, w = q.x, q.y, q.z, q.w
        r = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                      [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                      [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        f = msg.wrench.force
        self.forces[name].append([nanoseconds(msg.header.stamp), *(r @ [f.x, f.y, f.z])])

    def diagnostics(self, msg):
        for status in msg.status:
            self.status.append(dict(stamp=self.get_clock().now().nanoseconds, level=int.from_bytes(status.level, 'little')
                                    if isinstance(status.level, bytes) else int(status.level), message=status.message,
                                    values={v.key: v.value for v in status.values}))

    def save(self):
        if not self.states:
            return
        count = len(self.states[0]['q'])
        states = [s for s in self.states if len(s['q']) == count]
        self.output.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.output.with_suffix('.tmp.npz')
        np.savez(tmp, times_ns=np.array([s['time'] for s in states], dtype=np.int64),
                 measurement_ns=np.array([s['measurement'] for s in states], dtype=np.int64),
                 received_ns=np.array([s['received'] for s in states], dtype=np.int64),
                 degraded=np.array([s['degraded'] for s in states], dtype=bool),
                 predicted=np.array([s['predicted'] for s in states], dtype=bool),
                 positions=np.stack([s['q'] for s in states]), velocities=np.stack([s['v'] for s in states]),
                 covariance=np.stack([s['covariance'].reshape(count, 6, 6) for s in states]),
                 visibility=np.stack([s['visibility'] for s in states]),
                 observation_age_sec=np.stack([s['age'] for s in states]),
                 residual_forces=np.array([s['residual'] for s in states]))
        tmp.replace(self.output)
        for name, rows in self.forces.items():
            if rows:
                np.save(self.output.with_name(f'{self.output.stem}.{name}_force_world.npy'), np.asarray(rows))
        self.output.with_suffix('.status.json').write_text(json.dumps(self.status[-200:], indent=1))


def _run(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if isinstance(node, Recorder):
            node.save()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - the signal handler may already have shut the context down
            pass


def calibration_main():
    _run(CalibrationPublisher)


def oracle_hands_main():
    _run(OracleHands)


def recorder_main():
    _run(Recorder)


def grasp_gate_main():
    _run(GraspGate)
