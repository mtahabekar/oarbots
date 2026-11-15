"""Record the twin's state, status and CBF diagnostics for scoring.

    ros2 run fabric_twin twin_recorder --ros-args -p output:=/path/twin.npz

Writes `twin.npz` (times_ns, positions, velocities, uv, per-frame particles in the twin's order),
`twin.status.jsonl` (the twin's diagnostics) and `twin.cbf.jsonl` (the CBF debug stream).
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


def nanoseconds(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class TwinRecorder(Node):
    def __init__(self):
        super().__init__('twin_recorder')
        self.output = Path(self.declare_parameter('output', 'twin.npz').value)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.states, self.commands = [], []
        self.status = self.output.with_suffix('.status.jsonl').open('w')
        self.cbf = self.output.with_suffix('.cbf.jsonl').open('w')
        self.create_subscription(PointCloud2, '/fabric/twin/state', self.state, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/fabric/twin/status', self.diagnostics, 10)
        self.create_subscription(String, '/fabric/twin/cbf_debug', lambda m: self.cbf.write(m.data+'\n'), 50)
        self.create_subscription(String, '/fabric/material_parameters', lambda m: self.status.write(
            json.dumps({'material_release': json.loads(m.data)})+'\n'), 10)
        for index, robot in enumerate(('blue', 'silver')):
            self.create_subscription(TwistStamped, f'/oarbot_{robot}/fabric_cbf/cmd_world',
                                     lambda m, i=index: self.commands.append(
                                         [nanoseconds(m.header.stamp), i, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]), 50)
        self.create_timer(5., self.save)

    def state(self, msg):
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, -1)
        self.states.append((nanoseconds(msg.header.stamp), self.get_clock().now().nanoseconds, data.copy()))

    def diagnostics(self, msg):
        for status in msg.status:
            self.status.write(json.dumps({'stamp_ns': nanoseconds(msg.header.stamp), 'message': status.message,
                                          'values': {v.key: v.value for v in status.values}})+'\n')

    def save(self):
        self.status.flush(); self.cbf.flush()
        if not self.states:
            return
        count = self.states[0][2].shape[0]
        states = [s for s in self.states if s[2].shape[0] == count]
        data = np.stack([s[2] for s in states])
        tmp = self.output.with_suffix('.tmp.npz')
        np.savez(tmp, times_ns=np.array([s[0] for s in states], np.int64), received_ns=np.array([s[1] for s in states], np.int64),
                 positions=data[:, :, :3], velocities=data[:, :, 3:6], uv=data[0, :, 6:8],
                 commands=np.asarray(self.commands, float).reshape(-1, 5))
        tmp.replace(self.output)


def main():
    rclpy.init()
    node = TwinRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:  # noqa: BLE001 - the signal handler may already have shut the context down
            pass
