"""Parameter bank node: warm-up calibration and slow online adaptation of the twin's coefficients.

It listens to the twin (its corrected state and the grips it used), to the cameras' compact
depth observations, and to TF, and keeps the last minute of each. When the twin announces that
a warm-up excitation has finished, it fits the bending and shear scales to the holds of that
warm-up (`bank.Bank.calibrate`) and publishes a versioned release on
`/fabric/material_parameters`, which the twin applies to its sheet and its CBF model. With
`adapt_period_sec` > 0 it then repeats a small bounded update from the most recent seconds.

It never commands anything and runs at its own pace, in its own process.
"""
from collections import deque
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from fabric_state_estimator.config import load_config
from fabric_state_estimator.ros_adapters import transform_matrix
from fabric_state_msgs.msg import FabricObservations

from .bank import Bank, Case
from .node import load_twin_config

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def nanoseconds(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class ParameterBankNode(Node):
    def __init__(self):
        super().__init__('fabric_parameter_bank')
        share = Path(get_package_share_directory('fabric_twin'))
        self.spec, _, _, runtime, *_ = load_twin_config(self.declare_parameter('config_file', str(share/'config'/'twin.yaml')).value)
        estimator = self.declare_parameter('estimator_config', '').value
        topics = load_config(estimator)[3].observation_topics if estimator else [
            '/oarbot_blue/azure_kinect/fabric/observations', '/oarbot_silver/azure_kinect/fabric/observations', '/nuc/fabric/observations']
        self.world = runtime['world_frame']
        self.relax_sec = float(self.declare_parameter('relax_sec', 2.0).value)
        self.points_sec = float(self.declare_parameter('points_sec', 1.0).value)
        self.max_points = int(self.declare_parameter('max_points', 2500).value)
        self.adapt_period = float(self.declare_parameter('adapt_period_sec', 0.0).value)
        self.min_sensitivity = float(self.declare_parameter('min_sensitivity_m', 0.0015).value)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.states, self.corners, self.depth, self.pending = deque(maxlen=900), deque(maxlen=3600), deque(maxlen=2700), deque(maxlen=300)
        self.bank, self.scales, self.version, self.request = None, (1., 1.), 0, None
        self.random = np.random.default_rng(0)
        self.release_pub = self.create_publisher(String, '/fabric/material_parameters', LATCHED)
        self.status_pub = self.create_publisher(String, '/fabric/bank/status', 10)
        self.create_subscription(PointCloud2, '/fabric/twin/state', self.state, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/fabric/twin/corners', self.corner, 100)
        self.create_subscription(String, '/fabric/twin/warmup_window', self.warmup, 5)
        for topic in topics:
            self.create_subscription(FabricObservations, topic, self.observation, qos_profile_sensor_data)
        self.create_timer(.2, self.work)
        self.last_adapt = None
        self.get_logger().info(f'parameter bank ready; depth from {len(topics)} cameras')

    # ---------------------------------------------------------------- buffers
    def state(self, msg):
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, -1).astype(float)
        self.states.append((nanoseconds(msg.header.stamp), data[:, :3], data[:, 3:6]))

    def corner(self, msg):
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, -1).astype(float)
        if data.shape == (4, 3):
            self.corners.append((nanoseconds(msg.header.stamp), data))

    def observation(self, msg):
        if msg.depth_source == 'raw_registered_quality_gated' and msg.depth:
            points = np.array([[d.position.x, d.position.y, d.position.z] for d in msg.depth if d.quality > 0])
            if len(points):
                self.pending.append((msg.header.frame_id, msg.header.stamp, points))

    def resolve_depth(self):
        """Move depth samples into the world frame once TF for their capture time exists."""
        for _ in range(len(self.pending)):
            frame, stamp, points = self.pending.popleft()
            try:
                matrix = transform_matrix(self.tf.lookup_transform(self.world, frame, Time.from_msg(stamp)))
            except (TransformException, ValueError):
                if (self.get_clock().now().nanoseconds-nanoseconds(stamp))/1e9 < 1.:
                    self.pending.append((frame, stamp, points))
                continue
            self.depth.append((nanoseconds(stamp), points @ matrix[:3, :3].T+matrix[:3, 3]))

    def warmup(self, msg):
        try:
            self.request = json.loads(msg.data)
        except ValueError:
            self.get_logger().error('ignored a malformed warm-up window')

    # ---------------------------------------------------------------- cases
    def case(self, end_ns):
        """The stretch ending at `end_ns`: twin state `relax_sec` earlier, the grips since, and the
        depth points of the last `points_sec`. None when any of the three is missing."""
        start_ns = end_ns-int(self.relax_sec*1e9)
        state = min(self.states, key=lambda s: abs(s[0]-start_ns), default=None)
        if state is None or abs(state[0]-start_ns) > .2e9:
            return None
        grips = [g for stamp, g in self.corners if state[0] < stamp <= end_ns]
        points = [p for stamp, p in self.depth if end_ns-int(self.points_sec*1e9) <= stamp <= end_ns]
        if len(grips) < 30 or not points:
            return None
        points = np.concatenate(points)
        if len(points) > self.max_points:
            points = points[self.random.choice(len(points), self.max_points, replace=False)]
        try:
            return Case(state[1], np.stack(grips), points, state[2])
        except ValueError:
            return None

    def ensure_bank(self):
        if self.bank is None and self.corners:
            self.bank = Bank(self.spec, self.corners[-1][1])
        return self.bank is not None

    def release(self, scales, source, **details):
        self.scales, self.version = tuple(float(s) for s in scales), self.version+1
        payload = dict(version=self.version, bend=self.scales[0], shear=self.scales[1], source=source,
                       stamp_ns=self.get_clock().now().nanoseconds, mesh=[self.spec.cells_x, self.spec.cells_y],
                       substeps=self.spec.substeps, **details)
        self.release_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(f'material release {self.version} ({source}): bend x{self.scales[0]:.3f}, shear x{self.scales[1]:.3f}')

    # ---------------------------------------------------------------- work
    def work(self):
        self.resolve_depth()
        if self.request is not None:
            request, self.request = self.request, None
            self.calibrate(request)
        elif self.adapt_period > 0 and self.version > 0:
            now = self.get_clock().now().nanoseconds
            if self.last_adapt is None or (now-self.last_adapt)/1e9 >= self.adapt_period:
                self.last_adapt = now
                self.adapt()

    def calibrate(self, request):
        if not self.ensure_bank():
            self.status('calibration skipped: no twin data yet')
            return
        cases = [c for c in (self.case(int(request['start_ns']+end*1e9)) for _, end, _ in request['holds']) if c is not None]
        if len(cases) < 2:
            self.status(f'calibration skipped: only {len(cases)} of {len(request["holds"])} holds have state, grips and depth')
            return
        started = time.perf_counter()
        result = self.bank.calibrate(cases, self.scales)
        table = result.pop('table')
        result.update(cases=len(cases), seconds=round(time.perf_counter()-started, 1),
                      informative=bool(result['sensitivity_m'] >= self.min_sensitivity))
        self.status('calibrated', result=result, table=table)
        if result['informative']:
            self.release((result['bend'], result['shear']), 'warmup', **{k: result[k] for k in (
                'cost_m', 'start_cost_m', 'sensitivity_m', 'cases', 'seconds', 'evaluations')})
        else:
            self.get_logger().warn(f'warm-up was not informative (candidates differ by {result["sensitivity_m"]*1e3:.1f} mm); coefficients kept')

    def adapt(self):
        if not self.ensure_bank() or not self.states:
            return
        case = self.case(self.states[-1][0])
        if case is None:
            self.status('adaptation paused: missing state, grips or depth')
            return
        scales, report = self.bank.adapt([case], self.scales, min_sensitivity_m=self.min_sensitivity)
        self.status('adapted' if report['moved'] else 'adaptation paused: ' + (
            'no better neighbour' if report['informative'] else 'sheet too taut to tell coefficients apart'), report=report)
        if report['moved']:
            self.release(scales, 'online', cost_m=report['cost_m'], sensitivity_m=max(report['sensitivity_m']))

    def status(self, message, **details):
        self.status_pub.publish(String(data=json.dumps(dict(message=message, version=self.version, bend=self.scales[0],
                                                            shear=self.scales[1], **details))))
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = ParameterBankNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
