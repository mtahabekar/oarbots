"""Tier B observation-time ROS orchestrator; output only, no command publishers."""
from collections import Counter, deque
import json
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
from .monitoring import MonitorPolicy, MonitorSpec
from .ros_adapters import nanoseconds, transform_matrix, view_from_message, state_message
from human_hand_tracking.fusion import HandTrack


class AwaitingHands(Exception):
    """A glove detection after this capture is still expected; analyse a little later."""


class FabricObserverNode(Node):
    def __init__(self):
        super().__init__('fabric_state_estimator')
        default = str(Path(get_package_share_directory('fabric_state_estimator'))/'config'/'estimator.yaml')
        path = self.declare_parameter('config_file', default).value
        self.mesh, physics, observer, self.runtime, options, _ = load_config(path)
        r = self.runtime
        numeric = [r.robot_position_std_m, r.hand_max_age_sec, r.max_capture_age_sec, r.max_wait_sec,
                   r.reorder_wait_sec, r.prediction_limit_sec, r.publish_rate_hz,
                   r.force_stddev_n, r.force_max_age_sec]
        if (not np.isfinite(numeric).all() or min(numeric) <= 0 or r.max_wait_sec >= r.max_capture_age_sec
                or not np.isfinite(r.forecast_rate_hz) or r.forecast_rate_hz < 0
                or r.prediction_limit_sec > observer.forecast_limit_sec or len(r.robot_grip_frames) != 2
                or not r.observation_topics or len(set(r.observation_topics)) != len(r.observation_topics)
                or len(r.force_topics) != 2 or r.force_on_sheet_sign not in (-1., 1.)
                or any(h < 0 or h+r.prediction_limit_sec > observer.forecast_limit_sec for h in r.forecast_horizons_sec)):
            raise ValueError('Invalid observer runtime timing, topics, grip frames or force calibration sign')
        self.monitor = MonitorPolicy(MonitorSpec(**r.monitoring))
        self.observer = EnsembleObserver(self.mesh, make_physics(self.mesh, physics), observer,
                                        ObservationBuilder(self.mesh, **options))
        self.get_logger().info('Preparing physics backend before accepting live captures')
        self.observer.physics.warm_up(observer.ensemble_size)
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.hand_tracks, self.drops = {}, Counter()
        self.boundary_log = open(r.boundary_log, 'a') if r.boundary_log else None
        self.force_history = [deque(maxlen=200), deque(maxlen=200)]
        self.queue, self.sequence, self.last_camera, self.arrivals = [], 0, {}, {}
        self.intervals = {}  # camera -> recent capture intervals (ns), for its capture period
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
        stamp = nanoseconds(msg.header.stamp)
        if msg.header.frame_id != self.runtime.world_frame or stamp <= 0:
            self.failure = 'Hand states must use the configured common world frame'
            return
        corner_uv = np.round(self.mesh.uv[self.mesh.corners]).astype(int)
        for hand in msg.hands:
            if hand.holding_state != hand.HOLDING:
                continue
            matches = np.flatnonzero(np.all(np.isclose(corner_uv, hand.grip_uv, atol=1e-6), axis=1))
            if len(matches) != 1 or matches[0] < 2:
                self.failure = f'{hand.hand_id} grip_uv {list(hand.grip_uv)} is not a free material corner (robots hold corners 0,1)'
                continue
            p, v, a = hand.position, hand.velocity, hand.acceleration
            if len(hand.filter_covariance) == 81:
                covariance = np.asarray(hand.filter_covariance).reshape(9, 9)
            else:  # a publisher without the full filter state: acceleration unknown
                covariance = np.eye(9)
                covariance[:6, :6] = np.asarray(hand.covariance).reshape(6, 6)
            try:
                track = self.hand_tracks.setdefault(hand.hand_id, (int(matches[0]), HandTrack(floor=self.runtime.hand_floor_m)))[1]
                track.add(stamp, [p.x, p.y, p.z, v.x, v.y, v.z, a.x, a.y, a.z], covariance,
                          hand.jerk_stddev or self.runtime.hand_jerk_stddev)
            except ValueError:
                self.drops['invalid hand state'] += 1

    def observations(self, msg):
        capture = nanoseconds(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds-capture)/1e9
        if (not msg.camera_id or not msg.header.frame_id or capture <= self.last_camera.get(msg.camera_id, 0)
                or not -.01 <= age <= self.runtime.max_capture_age_sec):
            self.failure = 'Rejected duplicate, backward, stale or future capture'
            return
        if msg.camera_id in self.last_camera:
            self.intervals.setdefault(msg.camera_id, deque(maxlen=10)).append(capture-self.last_camera[msg.camera_id])
        self.last_camera[msg.camera_id] = capture; self.arrivals[msg.camera_id] = self.get_clock().now().nanoseconds
        heapq.heappush(self.queue, (capture, self.sequence, monotonic(), msg)); self.sequence += 1
        if len(self.queue) > 80:
            heapq.heappop(self.queue); self.failure = 'Observation queue overflow'

    def may_still_deliver(self, camera, stamp, capture):
        """Whether `camera` (newest capture `stamp`) can still send a capture at or before `capture`.

        Only a recently live camera can. Its next capture comes about one capture period after
        `stamp` (the period is the shortest of its recent intervals; 80 % of it absorbs jitter),
        so a 5 Hz camera that just delivered is not waited for by the 15 Hz captures in between.
        Until a camera's period is known, any earlier stamp means waiting, as before.
        """
        if stamp >= capture or (self.get_clock().now().nanoseconds-self.arrivals[camera])/1e9 >= self.runtime.max_capture_age_sec:
            return False
        intervals = self.intervals.get(camera)
        if not intervals or len(intervals) < 3:
            return True
        tolerance = int(self.runtime.camera_sync_tolerance_sec*1e9)
        return stamp + 0.8*min(intervals) <= capture + tolerance

    @staticmethod
    def latest(history, capture):
        eligible = [m for m in history if nanoseconds(m.header.stamp) <= capture]
        return max(eligible, key=lambda m: nanoseconds(m.header.stamp)) if eligible else None

    def grips_at(self, capture, may_wait=False):
        """Grips and grip velocities at one capture time.

        Robot grips come from TF at the capture time. Each human grip comes from that hand's
        recent filter states (`HandTrack`): interpolated when a glove detection after the
        capture has already arrived, otherwise extrapolated with a covariance that grows by
        the filter's own model. While `may_wait` and a live hand has no later detection yet,
        this raises AwaitingHands so the analysis runs slightly later on an interpolated grip
        instead of now on an extrapolated one. A hand not seen for `hand_coast_sec` is left
        out and the analysis still runs; only a new or restarted ensemble needs both hands, at
        most `hand_max_age_sec` old, to place its corners.
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
        starting = self.observer.q is None or (capture-self.observer.time_ns)/1e9 > self.observer.spec.max_gap_sec
        for corner, track in self.hand_tracks.values():
            grip = track.at(capture, r.hand_max_age_sec if starting else r.hand_coast_sec)
            if grip is None:
                continue
            if may_wait and not grip['bracketed']:
                raise AwaitingHands()
            velocity[corner] = grip['velocity']
            grips.append(Grip(corner, grip['position'], grip['covariance'], age_sec=grip['age_sec']))
        if starting and {g.corner for g in grips} != {0, 1, 2, 3}:
            raise ValueError('Missing causal, fresh human grip measurements for both corners (needed to start)')
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
                self.drops[self.failure] += 1
                continue
            if monotonic()-arrival < self.runtime.reorder_wait_sec:
                return
            # Watermark of every camera that has actually been live recently.
            waiting = any(self.may_still_deliver(c, stamp, capture) for c, stamp in self.last_camera.items())
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
                grips, grip_velocity = self.grips_at(capture, may_wait=age < self.runtime.hand_wait_sec)
                frame = Frame(capture, views, grips, self.forces_at(capture))
            except AwaitingHands:
                return
            except (TransformException, ValueError) as error:
                self.failure = str(error)
                if age < self.runtime.max_wait_sec:
                    return
                heapq.heappop(self.queue)
                self.drops[type(error).__name__+': '+self.failure[:60]] += 1
                continue
            members = {id(item) for item in group}
            self.queue = [item for item in self.queue if id(item) not in members]
            heapq.heapify(self.queue)
            try:
                snapshot = self.observer.assimilate(frame)
                self.last_latency_ms = (self.get_clock().now().nanoseconds-capture)/1e6
                if self.boundary_log is not None:
                    self.boundary_log.write(json.dumps(dict(self.observer.boundary, latency_ms=self.last_latency_ms,
                        cameras=[view.camera_id for view in views], factors=self.observer.status.get('factors'))) + '\n')
                    self.boundary_log.flush()
                if self.last_latency_ms > self.runtime.max_capture_age_sec*1000:
                    self.failure = 'Assimilation exceeded capture-age deadline; state not published'
                    continue
                snapshot['degraded'] = snapshot['degraded'] or self.last_latency_ms > 100
                snapshot['degraded'] = snapshot['degraded'] or self.runtime.use_forces and len(frame.forces) != 2
                # Monitoring assesses capture-time agreement, not the optional live-display
                # 100 ms latency target. The capture-age deadline and policy watchdog still apply.
                self.monitor.update(capture, self.observer.status, snapshot.get('depth_cameras', []))
                # Measured hand velocities and long-baseline robot TF velocity;
                # differencing successive grip targets amplified localization jitter.
                self.grip_velocity = grip_velocity
                self.prediction = None
                self.state_pub.publish(state_message(snapshot, self.mesh, self.observer.physics.spec.backend, self.runtime.world_frame))
                self.last_success, self.failure = self.get_clock().now().nanoseconds, ''
                if self.runtime.use_forces and len(frame.forces) != 2:
                    self.failure = 'One or both optional force factors missing'
            except (ValueError, FloatingPointError, np.linalg.LinAlgError) as error:
                self.failure = str(error)
                self.drops[type(error).__name__+': '+self.failure[:60]] += 1
                self.monitor.reset()
                self.observer.q = None  # never publish a partially updated invalid ensemble
                self.observer.time_ns = None

    def predict(self):
        if self.observer.q is None or not self.runtime.publish_predictions:
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
            # Neither the demo nor recorder consumes forecasts. Avoid an entire unused rollout.
            # A subscribed consumer retains the existing contract; zero explicitly disables forecasts.
            if (self.runtime.forecast_rate_hz == 0 or not self.forecast_pub.get_subscription_count()
                    or (self.get_clock().now().nanoseconds-self.last_forecast)/1e9 < 1/self.runtime.forecast_rate_hz):
                return
            self.last_forecast = self.get_clock().now().nanoseconds
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
        fresh = (self.last_success is not None and
                 0 <= (self.get_clock().now().nanoseconds-self.last_success)/1e9 <= self.runtime.max_capture_age_sec)
        degraded = not fresh or self.observer.status['degraded'] or bool(self.failure) or self.last_latency_ms > 100
        now = self.get_clock().now().nanoseconds
        mode = self.monitor.mode(now)
        # A current diagnostic must never keep throttling cameras after analyses have stopped.
        monitor_values = [KeyValue(key='monitor_mode', value=mode),
                          KeyValue(key='measurement_ns', value=str(self.observer.time_ns or 0)),
                          KeyValue(key='prior_depth_p95_m', value=str(self.observer.status.get('prior_depth_p95_m', float('inf')))),
                          KeyValue(key='marginal_innovation_score', value=str(self.observer.status.get('marginal_innovation_score', float('inf'))))]
        self.status_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[DiagnosticStatus(
            level=DiagnosticStatus.WARN if degraded else DiagnosticStatus.OK, name=self.get_fully_qualified_name(),
            hardware_id=self.mesh.model_id, message=self.failure or self.observer.status['reason'],
            values=monitor_values+[KeyValue(key='capture_to_analysis_ms', value=str(self.last_latency_ms)),
                    KeyValue(key='hand_age_sec', value=json.dumps(self.observer.status.get('hand_age_sec'))),
                    KeyValue(key='dropped_captures', value=json.dumps(dict(self.drops))),
                    KeyValue(key='physics', value=self.observer.physics.spec.backend),
                    KeyValue(key='contact_assumption', value='four prescribed corners (robot TF, propagated glove estimate); no self/obstacle collision')])]))


def main(args=None):
    rclpy.init(args=args)
    node = FabricObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.try_shutdown()
