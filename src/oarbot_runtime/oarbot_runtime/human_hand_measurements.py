"""Expose selected Azure SDK hand markers as timestamped position measurements.

Covariance is a configured measurement-noise assumption, NOT SDK confidence.
Each camera requires explicit body selection; IDs are not shared between cameras.
"""
from math import isfinite
from time import monotonic
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus


class HumanHands(Node):
    def __init__(self):
        super().__init__('human_hand_measurements')
        self.body_id = self.declare_parameter('body_id', -1).value
        self.stddev = self.declare_parameter('position_stddev_m', 0.05).value
        self.max_age = self.declare_parameter('max_age_sec', 0.25).value
        if self.body_id < -1 or not all(isfinite(v) and v > 0 for v in (self.stddev, self.max_age)):
            raise ValueError('Invalid body ID or measurement noise/age')
        self.last = {}
        self.hand_publishers = {joint: self.create_publisher(PoseWithCovarianceStamped, 'human/' + name + '/pose', qos_profile_sensor_data)
                           for joint, name in ((8, 'left_hand'), (15, 'right_hand'))}
        self.create_subscription(MarkerArray, 'body_tracking_data', self.markers, qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)
        self.create_timer(0.5, self.diagnostics)

    def markers(self, msg):
        if self.body_id < 0:
            return
        for marker in msg.markers:
            body, joint = divmod(marker.id, 100)
            if body != self.body_id or joint not in self.hand_publishers or marker.action != Marker.ADD:
                continue
            stamp = rclpy.time.Time.from_msg(marker.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9
            p = marker.pose.position
            if (not marker.header.frame_id or stamp.nanoseconds == 0 or not -0.01 <= age <= self.max_age or
                    not all(isfinite(v) for v in (p.x, p.y, p.z))):
                continue
            if joint in self.last and stamp.nanoseconds <= self.last[joint][0]:
                continue
            output = PoseWithCovarianceStamped()
            output.header = marker.header
            output.pose.pose.position = p
            output.pose.pose.orientation.w = 1.0
            for i in (0, 7, 14):
                output.pose.covariance[i] = self.stddev**2
            for i in (21, 28, 35):
                output.pose.covariance[i] = 1e6  # orientation is not measured by this adapter
            self.hand_publishers[joint].publish(output)
            self.last[joint] = (stamp.nanoseconds, monotonic())

    def diagnostics(self):
        fresh = all(j in self.last and monotonic()-self.last[j][1] <= self.max_age for j in self.hand_publishers)
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [DiagnosticStatus(level=DiagnosticStatus.OK if fresh else DiagnosticStatus.WARN, name=self.get_fully_qualified_name(), hardware_id=str(self.body_id),
            message='Both hands measured; configured covariance, SDK confidence unavailable' if fresh else
                    'Select body_id' if self.body_id < 0 else 'One or both hand observations missing or stale')]
        self.diagnostics_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HumanHands()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
