"""Tier B observation-time ROS orchestrator; output only, no command publishers."""
from collections import deque
import heapq
from pathlib import Path
from time import monotonic
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Point32, WrenchStamped
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from fabric_state_msgs.msg import FabricModel, FabricState, FabricForecasts, FabricObservations, HandStates
from .config import load_config
from .data import Frame, Grip, Force
from .ensemble import EnsembleObserver
from .observations import ObservationBuilder
from .physics import make_physics
from .ros_adapters import nanoseconds, transform_matrix, view_from_message, state_message


class FabricObserverNode(Node):
    def __init__(self):
        super().__init__('fabric_state_estimator')
        default = str(Path(get_package_share_directory('fabric_state_estimator'))/'config'/'estimator.yaml')
        path = self.declare_parameter('config_file', default).value
        self.mesh, physics, observer, self.runtime, options, _ = load_config(path)
        r = self.runtime
        numeric = [r.robot_position_std_m, r.hand_max_age_sec, r.max_capture_age_sec, r.max_wait_sec,
                   r.reorder_wait_sec, r.prediction_limit_sec, r.publish_rate_hz, r.forecast_rate_hz,
                   r.force_stddev_n, r.force_max_age_sec]
        if (not np.isfinite(numeric).all() or min(numeric) <= 0 or r.max_wait_sec >= r.max_capture_age_sec
                or r.prediction_limit_sec > observer.forecast_limit_sec or len(r.robot_grip_frames) != 2
                or not r.observation_topics or len(set(r.observation_topics)) != len(r.observation_topics)
                or len(r.force_topics) != 2 or r.force_on_sheet_sign not in (-1., 1.)
                or any(h < 0 or h+r.prediction_limit_sec > observer.forecast_limit_sec for h in r.forecast_horizons_sec)):
            raise ValueError('Invalid observer runtime timing, topics, grip frames or force calibration sign')
        self.observer = EnsembleObserver(self.mesh, make_physics(self.mesh, physics), observer,
                                        ObservationBuilder(self.mesh, **options))
        self.get_logger().info('Preparing physics backend before accepting live captures')
        self.observer.physics.warm_up(observer.ensemble_size)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.hand_history = deque(maxlen=200)
        self.force_history = [deque(maxlen=200), deque(maxlen=200)]
        self.queue, self.sequence, self.last_camera, self.arrivals = [], 0, {}, {}
        self.grip_velocity, self.prediction, self.last_forecast = np.zeros((4, 3)), None, -np.inf
        self.last_success, self.failure, self.last_latency_ms = None, 'Waiting for observations, grips and capture-time TF', 0.
        self.state_pub = self.create_publisher(FabricState, '/fabric/estimated_state', qos_profile_sensor_data)
        self.forecast_pub = self.create_publisher(FabricForecasts, '/fabric/predicted_states', qos_profile_sensor_data)
        self.model_pub = self.create_publisher(FabricModel, '/fabric/model', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub = self.create_publisher(DiagnosticArray, '/fabric/estimator_status', 10)
        self.subscriptions_ = [self.create_subscription(FabricObservations, topic, self.observations, qos_profile_sensor_data)
                               for topic in r.observation_topics]
        self.subscriptions_.append(self.create_subscription(HandStates, r.hand_topic, self.hands, qos_profile_sensor_data))
        if r.use_forces:
            for i, topic in enumerate(r.force_topics):
                self.subscriptions_.append(self.create_subscription(WrenchStamped, topic,
                    lambda msg, index=i: self.force_history[index].append(msg), qos_profile_sensor_data))
        self.publish_model()
        self.create_timer(.01, self.flush)
        self.create_timer(1/r.publish_rate_hz, self.predict)
        self.create_timer(.5, self.diagnostics)

    def publish_model(self):
        spec = self.mesh.spec
        self.model_pub.publish(FabricModel(header=Header(frame_id=self.runtime.world_frame), model_id=self.mesh.model_id,
            rows=spec.rows, columns=spec.columns, width_m=spec.width_m, height_m=spec.height_m,
            areal_density_kg_m2=spec.areal_density_kg_m2,
            rest_positions=[Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in self.mesh.rest],
            material_uv=[Point32(x=float(p[0]), y=float(p[1])) for p in self.mesh.uv],
            triangles=self.mesh.triangles.ravel().tolist(), corner_indices=self.mesh.corners.tolist()))

    def hands(self, msg):
        if msg.header.frame_id == self.runtime.world_frame and nanoseconds(msg.header.stamp) > 0:
            self.hand_history.append(msg)
        else:
            self.failure = 'Hand states must use the configured common world frame'

    def observations(self, msg):
        capture = nanoseconds(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds-capture)/1e9
        if (not msg.camera_id or not msg.header.frame_id or capture <= self.last_camera.get(msg.camera_id, 0)
                or not -.01 <= age <= self.runtime.max_capture_age_sec):
            self.failure = 'Rejected duplicate, backward, stale or future capture'
            return
        self.last_camera[msg.camera_id] = capture; self.arrivals[msg.camera_id] = monotonic()
        heapq.heappush(self.queue, (capture, self.sequence, monotonic(), msg)); self.sequence += 1
        if len(self.queue) > 80:
            heapq.heappop(self.queue); self.failure = 'Observation queue overflow'

    @staticmethod
    def latest(history, capture):
        eligible = [m for m in history if nanoseconds(m.header.stamp) <= capture]
        return max(eligible, key=lambda m: nanoseconds(m.header.stamp)) if eligible else None

    def grips_at(self, capture):
        """Measured grips and grip velocities at one capture time.

        Each hand is taken from the newest causal HandStates message that
        contains it: fusion omits hands a camera group did not see, so one
        message rarely holds both hands when cameras are not synchronized.
        The material corner comes from the hand's configured grip_uv.
        """
        r = self.runtime
        grips, velocity = [], np.zeros((4, 3))
        for corner, frame in enumerate(r.robot_grip_frames):
            matrix = transform_matrix(self.tf.lookup_transform(r.world_frame, frame, Time(nanoseconds=capture)))
            grips.append(Grip(corner, matrix[:3, 3], np.eye(3)*r.robot_position_std_m**2))
            try:  # forecast input only; a long baseline keeps localization jitter out
                earlier = transform_matrix(self.tf.lookup_transform(r.world_frame, frame, Time(nanoseconds=capture-100_000_000)))
                velocity[corner] = (matrix[:3, 3]-earlier[:3, 3])/.1
            except TransformException:
                pass
        corner_uv = np.round(self.mesh.uv[self.mesh.corners]).astype(int)
        found = set()
        for msg in sorted((m for m in self.hand_history if nanoseconds(m.header.stamp) <= capture),
                          key=lambda m: nanoseconds(m.header.stamp), reverse=True):
            age = (capture-nanoseconds(msg.header.stamp))/1e9
            if age > r.hand_max_age_sec:
                break
            for hand in msg.hands:
                if hand.hand_id in found or hand.holding_state != hand.HOLDING:
                    continue
                matches = np.flatnonzero(np.all(np.isclose(corner_uv, hand.grip_uv, atol=1e-6), axis=1))
                if len(matches) != 1 or matches[0] < 2:
                    raise ValueError(f'{hand.hand_id} grip_uv {list(hand.grip_uv)} is not a free material corner (robots hold corners 0,1)')
                corner = int(matches[0]); found.add(hand.hand_id)
                p, v = hand.position, hand.velocity
                covariance = np.asarray(hand.covariance).reshape(6, 6)
                propagation = np.column_stack((np.eye(3), np.eye(3)*age))
                velocity[corner] = [v.x, v.y, v.z]
                grips.append(Grip(corner, np.array([p.x, p.y, p.z])+age*velocity[corner],
                                  propagation @ covariance @ propagation.T))
            if len(found) == 2:
                break
        if {g.corner for g in grips} != {0, 1, 2, 3}:
            raise ValueError('Missing causal, fresh human grip measurements for both corners')
        return grips, velocity

    def forces_at(self, capture):
        forces = []
        if not self.runtime.use_forces:
            return forces
        for corner, history in enumerate(self.force_history):
            msg = self.latest(history, capture)
            if msg is None or (capture-nanoseconds(msg.header.stamp))/1e9 > self.runtime.force_max_age_sec:
                continue
            try:
                matrix = transform_matrix(self.tf.lookup_transform(self.runtime.world_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp)))
                f = msg.wrench.force
                force = self.runtime.force_on_sheet_sign*(matrix[:3, :3] @ [f.x, f.y, f.z])
                forces.append(Force(corner, force, self.runtime.force_stddev_n))
            except (TransformException, ValueError):
                continue  # visual inference may proceed with a degraded force status
        return forces

    def flush(self):
        while self.queue:
            capture, _, arrival, msg = self.queue[0]
            age = (self.get_clock().now().nanoseconds-capture)/1e9
            if self.observer.time_ns is not None and capture <= self.observer.time_ns or age > self.runtime.max_capture_age_sec:
                heapq.heappop(self.queue); self.failure = 'Late or expired capture dropped'
                continue
            if monotonic()-arrival < self.runtime.reorder_wait_sec:
                return
            # Watermark of every camera that has actually been live recently.
            waiting = any(monotonic()-self.arrivals[c] < self.runtime.max_capture_age_sec and stamp < capture
                          for c, stamp in self.last_camera.items())
            if waiting and age < self.runtime.max_wait_sec:
                return
            try:
                # One analysis per synchronized camera group: with wired sync,
                # three cameras cost one ensemble update instead of three. Each
                # view keeps its own stamp/TF; capture skew inflates its variance.
                tolerance = int(self.runtime.camera_sync_tolerance_sec*1e9)
                group = [item for item in self.queue if item[0]-capture <= tolerance]
                if len({item[3].camera_id for item in group}) != len(group):
                    group = [item for item in self.queue if item[0] == capture]
                capture = max(item[0] for item in group)
                views = [view_from_message(item[3], transform_matrix(self.tf.lookup_transform(self.runtime.world_frame,
                    item[3].header.frame_id, Time.from_msg(item[3].header.stamp)))) for item in group]
                grips, grip_velocity = self.grips_at(capture)
                frame = Frame(capture, views, grips, self.forces_at(capture))
            except (TransformException, ValueError) as error:
                self.failure = str(error)
                if age < self.runtime.max_wait_sec:
                    return
                heapq.heappop(self.queue)
                continue
            members = {id(item) for item in group}
            self.queue = [item for item in self.queue if id(item) not in members]
            heapq.heapify(self.queue)
            try:
                snapshot = self.observer.assimilate(frame)
                self.last_latency_ms = (self.get_clock().now().nanoseconds-capture)/1e6
                if self.last_latency_ms > self.runtime.max_capture_age_sec*1000:
                    self.failure = 'Assimilation exceeded capture-age deadline; state not published'
                    continue
                snapshot['degraded'] = snapshot['degraded'] or self.last_latency_ms > 100
                snapshot['degraded'] = snapshot['degraded'] or self.runtime.use_forces and len(frame.forces) != 2
                # Measured hand velocities and long-baseline robot TF velocity;
                # differencing successive grip targets amplified localization jitter.
                self.grip_velocity = grip_velocity
                self.prediction = None
                self.state_pub.publish(state_message(snapshot, self.mesh, self.observer.physics.spec.backend, self.runtime.world_frame))
                self.last_success, self.failure = monotonic(), ''
                if self.runtime.use_forces and len(frame.forces) != 2:
                    self.failure = 'One or both optional force factors missing'
            except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
                self.failure = str(error)
                self.observer.q = None  # never publish a partially updated invalid ensemble
                self.observer.time_ns = None

    def predict(self):
        if self.observer.q is None:
            return
        age = (self.get_clock().now().nanoseconds-self.observer.time_ns)/1e9
        if not 0 <= age <= self.runtime.prediction_limit_sec:
            return
        try:
            # Resume this analysis' rollout instead of re-simulating from the
            # analysis on every tick (four independent rollouts per tick cost
            # ~1 s of GPU time per 66 ms tick with Newton at 17x17).
            start = self.prediction if self.prediction is not None and self.prediction[0] <= age else None
            (snapshot,), self.prediction = self.observer.rollout([age], self.grip_velocity, start)
            snapshot['degraded'] = snapshot['degraded'] or age > .1 or bool(self.failure)
            self.state_pub.publish(state_message(snapshot, self.mesh, self.observer.physics.spec.backend, self.runtime.world_frame))
            if monotonic()-self.last_forecast < 1/self.runtime.forecast_rate_hz:
                return
            self.last_forecast = monotonic()
            snapshots, _ = self.observer.rollout([age+h for h in self.runtime.forecast_horizons_sec],
                                                 self.grip_velocity, self.prediction)
            for forecast in snapshots:
                forecast['degraded'] = forecast['degraded'] or snapshot['degraded']
            self.forecast_pub.publish(FabricForecasts(header=Header(stamp=Time(nanoseconds=snapshot['time_ns']).to_msg(),
                frame_id=self.runtime.world_frame), assumption='constant measured grip velocity; fixed residual loads; no self/obstacle contact',
                horizons_sec=self.runtime.forecast_horizons_sec, states=[state_message(s, self.mesh,
                    self.observer.physics.spec.backend, self.runtime.world_frame) for s in snapshots]))
        except (ValueError, FloatingPointError) as error:
            self.failure = str(error)

    def diagnostics(self):
        fresh = self.last_success is not None and monotonic()-self.last_success <= self.runtime.max_capture_age_sec
        degraded = not fresh or self.observer.status['degraded'] or bool(self.failure) or self.last_latency_ms > 100
        self.status_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[DiagnosticStatus(
            level=DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK, name=self.get_fully_qualified_name(),
            hardware_id=self.mesh.model_id, message=self.failure or self.observer.status['reason'],
            values=[KeyValue(key='capture_to_analysis_ms', value=str(self.last_latency_ms)),
                    KeyValue(key='physics', value=self.observer.physics.spec.backend),
                    KeyValue(key='contact_assumption', value='all four prescribed corners; no self/obstacle collision')])]))


def main(args=None):
    rclpy.init(args=args)
    node = FabricObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.try_shutdown()
