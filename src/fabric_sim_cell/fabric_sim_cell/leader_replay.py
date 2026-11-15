"""Replay recorded human (leader) corner motion into the simulated plant.

    ros2 run fabric_sim_cell leader_replay --ros-args -p truth:=/path/to/truth.npz

The plant's private truth file stores the four corner positions at every cloth frame. The two
leader corners are where the human hands were. This node differentiates them and publishes
the resulting velocities on `/fabric/leader_{left,right}/cmd_vel`, keyed to the plant's
`/clock`, so an earlier teleoperated run can be repeated without a keyboard. It reads only
the corner tracks: nothing derived from the cloth itself reaches the plant or the estimator.
"""
import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

LEADERS = (2, 3)  # corner order: follower_left, follower_right, leader_left, leader_right


def leader_velocities(times_ns, corners):
    """Per-frame leader velocities (T-1, 2, 3) and the frame start times (s)."""
    times = np.asarray(times_ns, dtype=np.int64)
    corners = np.asarray(corners, dtype=float)
    if corners.ndim != 3 or corners.shape[1:] != (4, 3) or len(times) != len(corners) or len(times) < 2:
        raise ValueError('Expected corner tracks (T, 4, 3) with matching times')
    dt = np.diff(times) / 1e9
    if np.any(dt <= 0):
        raise ValueError('Truth times must increase')
    return np.diff(corners[:, LEADERS], axis=0) / dt[:, None, None], times[:-1] / 1e9


class LeaderReplay(Node):
    def __init__(self):
        super().__init__('leader_replay')
        path = self.declare_parameter('truth', '').value
        self.scale = float(self.declare_parameter('speed_scale', 1.0).value)
        if not path:
            raise ValueError('leader_replay needs truth:=/path/to/truth.npz')
        with np.load(path, allow_pickle=False) as truth:
            self.velocity, self.start = leader_velocities(truth['times_ns'], truth['corners'])
        self.publishers_ = [self.create_publisher(TwistStamped, f'/fabric/leader_{side}/cmd_vel', 10)
                            for side in ('left', 'right')]
        self.create_subscription(Clock, '/clock', self.clock, qos_profile_sensor_data)
        self.get_logger().info(f'replaying {len(self.velocity)} leader frames from {path}')

    def clock(self, msg):
        now = msg.clock.sec + msg.clock.nanosec * 1e-9
        index = int(np.searchsorted(self.start, now + 1e-6, side='right')) - 1
        velocity = self.velocity[index] * self.scale if 0 <= index < len(self.velocity) else np.zeros((2, 3))
        for publisher, value in zip(self.publishers_, velocity):
            out = TwistStamped()
            out.header.stamp = msg.clock
            out.header.frame_id = 'world'
            out.twist.linear.x, out.twist.linear.y, out.twist.linear.z = (float(v) for v in value)
            publisher.publish(out)


def main():
    rclpy.init()
    node = LeaderReplay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
