"""Central capture-time tagged-hand fusion. Does not infer grasp or contact."""
import heapq
from time import monotonic
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformListener, TransformException
from fabric_state_msgs.msg import GloveObservations, HandState, HandStates
from .fusion import Measurement, HandFilter, fuse_views, rotation_matrix, transform_measurement


def stamp_ns(stamp):
    return stamp.sec*1_000_000_000 + stamp.nanosec


class HandFusion(Node):
    def __init__(self):
        super().__init__('hand_fusion')
        param = lambda name, default: self.declare_parameter(name, default).value
        topics = param('observation_topics', [
            '/oarbot_blue/azure_kinect/fabric/gloves', '/oarbot_silver/azure_kinect/fabric/gloves', '/nuc/fabric/gloves'])
        self.world = param('world_frame', 'world')
        self.max_age = param('max_age_sec', .25)
        self.reorder_delay = param('reorder_delay_sec', .03)
        self.sync_tolerance = param('sync_tolerance_sec', .015)
        self.extrinsic_noise = param('extrinsic_stddev_m', .01)
        self.fusion_floor = param('fusion_floor_m', .01)
        self.max_speed = param('hand_max_speed_mps', 1.)
        self.spatial_gate = param('view_spatial_gate_m', .15)
        jerk = param('jerk_stddev', 3.)
        gap = param('filter_max_gap_sec', .5)
        tags = param('glove_tag_ids', [20, 21])
        uvs = param('human_corner_uv', [0., 1., 1., 1.])
        numeric = [self.max_age, self.reorder_delay, self.sync_tolerance, self.extrinsic_noise,
                   self.fusion_floor, self.max_speed, self.spatial_gate]
        if (not topics or len(topics) > 8 or len(set(topics)) != len(topics) or not self.world
                or not np.isfinite(numeric).all() or min(numeric) <= 0
                or self.sync_tolerance > self.reorder_delay or self.reorder_delay >= self.max_age
                or len(tags) != 2 or len(set(tags)) != 2 or len(uvs) != 4
                or not np.isfinite(uvs).all() or min(uvs) < 0 or max(uvs) > 1):
            raise ValueError('Invalid camera topics, timing, calibration or hand identity settings')
        self.tags = dict(zip(('human_left', 'human_right'), tags))
        self.uvs = dict(zip(self.tags, np.asarray(uvs).reshape(2, 2)))
        self.filters = {hand: HandFilter(jerk, gap) for hand in self.tags}
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.publisher = self.create_publisher(HandStates, '/fabric/hand_states', qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, '/fabric/hand_tracking_status', 10)
        self.subscriptions_ = [self.create_subscription(GloveObservations, topic, self.observations,
                                                      qos_profile_sensor_data) for topic in topics]
        self.queue, self.sequence, self.last_camera = [], 0, {}
        self.last_group, self.last_arrival, self.last_count = 0, None, 0
        self.failure = 'Waiting for glove observations and capture-time TF'
        self.create_timer(.01, self.flush)
        self.create_timer(.5, self.diagnostics)

    def observations(self, msg):
        ns = stamp_ns(msg.header.stamp)
        age = (self.get_clock().now().nanoseconds - ns)/1e9
        if (not msg.camera_id or not msg.header.frame_id or ns <= self.last_camera.get(msg.camera_id, 0)
                or not -.01 <= age <= self.max_age):
            self.failure = 'Rejected stale, duplicate, future or unidentified camera sample'
            return
        self.last_camera[msg.camera_id] = ns
        heapq.heappush(self.queue, (ns, self.sequence, monotonic(), msg))
        self.sequence += 1
        if len(self.queue) > 80:
            heapq.heappop(self.queue)
            self.failure = 'Observation queue overflow'

    def flush(self):
        while self.queue and monotonic()-self.queue[0][2] >= self.reorder_delay:
            first = heapq.heappop(self.queue)
            group = [first]
            while self.queue and self.queue[0][0]-first[0] <= self.sync_tolerance*1e9:
                group.append(heapq.heappop(self.queue))
            latest = max(item[0] for item in group)
            if first[0] <= self.last_group:
                self.failure = 'Late group dropped; filters never run backward'
                continue
            if (self.get_clock().now().nanoseconds - first[0])/1e9 > self.max_age:
                self.failure = 'Queued observations expired'
                continue
            self.last_group = latest
            measurements = {hand: [] for hand in self.tags}
            failures = []
            for _, _, _, msg in group:
                try:
                    transform = self.tf.lookup_transform(self.world, msg.header.frame_id,
                                                         Time.from_msg(msg.header.stamp))
                    q, t = transform.transform.rotation, transform.transform.translation
                    rotation = rotation_matrix([q.x, q.y, q.z, q.w])
                    seen = set()
                    view_measurements = []
                    for hand in msg.hands:
                        if self.tags.get(hand.hand_id) != hand.tag_id or hand.hand_id in seen:
                            raise ValueError('Unexpected or duplicate glove identity')
                        seen.add(hand.hand_id)
                        p = hand.position
                        m = Measurement(hand.hand_id, hand.tag_id, msg.camera_id, stamp_ns(msg.header.stamp),
                                        np.array([p.x, p.y, p.z]), np.asarray(hand.covariance).reshape(3, 3))
                        if not m.valid():
                            raise ValueError('Invalid hand position covariance')
                        view_measurements.append(transform_measurement(m, rotation,
                            [t.x, t.y, t.z], self.extrinsic_noise))
                    for m in view_measurements:
                        measurements[m.hand_id].append(m)
                except (TransformException, ValueError) as error:
                    failures.append(str(error))
            output = HandStates(header=Header(stamp=Time(nanoseconds=latest).to_msg(), frame_id=self.world))
            for hand, views in measurements.items():
                fused = fuse_views(views, latest, self.fusion_floor, self.max_speed, self.spatial_gate)
                if fused is None:
                    if views:
                        failures.append(hand + ': conflicting views')
                    continue
                position, covariance, selected = fused
                filtered = self.filters[hand].update(position, covariance, latest)
                if filtered is None:
                    failures.append(hand + ': temporal innovation rejected')
                    continue
                x, p = filtered
                output.hands.append(HandState(hand_id=hand, tag_id=self.tags[hand],
                    position=Point(x=float(x[0]), y=float(x[1]), z=float(x[2])),
                    velocity=Vector3(x=float(x[3]), y=float(x[4]), z=float(x[5])),
                    covariance=p.ravel().tolist(), source_cameras=[m.camera_id for m in selected],
                    source_stamps=[Time(nanoseconds=m.stamp_ns).to_msg() for m in selected],
                    holding_state=HandState.HOLDING, holding_is_assumed=True, grip_uv=self.uvs[hand].tolist()))
            self.publisher.publish(output)
            self.last_arrival, self.last_count = monotonic(), len(output.hands)
            self.failure = '; '.join(failures)

    def diagnostics(self):
        fresh = self.last_arrival is not None and monotonic()-self.last_arrival <= self.max_age
        self.diagnostics_pub.publish(DiagnosticArray(header=Header(stamp=self.get_clock().now().to_msg()), status=[
            DiagnosticStatus(level=DiagnosticStatus.OK if fresh and self.last_count == 2 and not self.failure else DiagnosticStatus.WARN,
                name=self.get_fully_qualified_name(), hardware_id='tagged_gloves',
                message=self.failure or ('Both hands observed; corner holding is assumed' if fresh and self.last_count == 2
                                        else 'One or both grip observations missing or stale'),
                values=[KeyValue(key='fresh', value=str(fresh)), KeyValue(key='holding_is_assumed', value='true')])]))


def main(args=None):
    rclpy.init(args=args)
    node = HandFusion()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
