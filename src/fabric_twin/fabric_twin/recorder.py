"""Record the twin's state, status and CBF diagnostics for scoring.

    ros2 run fabric_twin twin_recorder --ros-args -p output:=/path/twin.npz

Writes `twin.npz` (times_ns, positions, velocities, uv, per-frame particles in the twin's order;
`corners_ns`, `corners`: the four grips the twin was driven with, every frame),
`twin.status.jsonl` (the twin's diagnostics, warm-up windows, material releases and parameter-bank
status), `twin.cbf.jsonl` (the CBF debug stream) and, with `depth` true, `twin.depth.npz`: every
quality-gated camera depth point the parameter bank would use, in the world frame at its capture
time (`stamps_ns`, `camera`, `counts`, `points`). With the twin's grips and states these are the
complete inputs of the parameter bank, so its calibration can be repeated offline.
"""
import json
from pathlib import Path

import numpy as np
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from fabric_state_msgs.msg import FabricObservations
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

DEPTH_TOPICS = ['/oarbot_blue/azure_kinect/fabric/observations', '/oarbot_silver/azure_kinect/fabric/observations',
                '/nuc/fabric/observations']


def nanoseconds(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


class TwinRecorder(Node):
    def __init__(self):
        super().__init__('twin_recorder')
        self.output = Path(self.declare_parameter('output', 'twin.npz').value)
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.states, self.commands, self.corners = [], [], []
        self.depth, self.pending = [], []
        self.status = self.output.with_suffix('.status.jsonl').open('w')
        self.cbf = self.output.with_suffix('.cbf.jsonl').open('w')
        self.create_subscription(PointCloud2, '/fabric/twin/state', self.state, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, '/fabric/twin/status', self.diagnostics, 10)
        self.create_subscription(String, '/fabric/twin/cbf_debug', lambda m: self.cbf.write(m.data+'\n'), 50)
        self.create_subscription(String, '/fabric/material_parameters', lambda m: self.status.write(
            json.dumps({'material_release': json.loads(m.data)})+'\n'), 10)
        self.create_subscription(PointCloud2, '/fabric/twin/corners', self.corner, 100)
        for topic in ('/fabric/twin/warmup_window', '/fabric/bank/status'):
            self.create_subscription(String, topic, lambda m, t=topic: self.status.write(
                json.dumps({'topic': t, 'stamp_ns': self.get_clock().now().nanoseconds, 'data': json.loads(m.data)})+'\n'), 10)
        if self.declare_parameter('depth', False).value:
            self.world = self.declare_parameter('world_frame', 'world').value
            self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
            for camera, topic in enumerate(self.declare_parameter('depth_topics', DEPTH_TOPICS).value):
                self.create_subscription(FabricObservations, topic, lambda m, c=camera: self.observation(c, m),
                                         qos_profile_sensor_data)
            self.create_timer(.2, self.resolve_depth)
        for index, robot in enumerate(('blue', 'silver')):
            self.create_subscription(TwistStamped, f'/oarbot_{robot}/fabric_cbf/cmd_world',
                                     lambda m, i=index: self.commands.append(
                                         [nanoseconds(m.header.stamp), i, m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]), 50)
        self.create_timer(5., self.save)

    def state(self, msg):
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, -1)
        self.states.append((nanoseconds(msg.header.stamp), self.get_clock().now().nanoseconds, data.copy()))

    def corner(self, msg):
        data = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(msg.width, -1)
        if data.shape[0] == 4:
            self.corners.append((nanoseconds(msg.header.stamp), data[:, :3].copy()))

    def observation(self, camera, msg):
        """The same selection as the parameter bank: quality-gated raw depth points."""
        if msg.depth_source == 'raw_registered_quality_gated' and msg.depth:
            points = np.array([[d.position.x, d.position.y, d.position.z] for d in msg.depth if d.quality > 0], np.float32)
            if len(points):
                self.pending.append((camera, msg.header.frame_id, msg.header.stamp, points))

    def resolve_depth(self):
        keep = []
        for camera, frame, stamp, points in self.pending:
            try:
                t = self.tf.lookup_transform(self.world, frame, Time.from_msg(stamp)).transform
            except TransformException:
                if (self.get_clock().now().nanoseconds-nanoseconds(stamp))/1e9 < 1.:
                    keep.append((camera, frame, stamp, points))
                continue
            x, y, z, w = t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
            world = points.astype(float) @ rotation.T+[t.translation.x, t.translation.y, t.translation.z]
            self.depth.append((nanoseconds(stamp), camera, world.astype(np.float32)))
        self.pending = keep

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
                 commands=np.asarray(self.commands, float).reshape(-1, 5),
                 corners_ns=np.array([c[0] for c in self.corners], np.int64),
                 corners=np.array([c[1] for c in self.corners], float).reshape(-1, 4, 3))
        tmp.replace(self.output)
        if self.depth:
            depth = sorted(self.depth, key=lambda d: d[0])
            tmp = self.output.with_suffix('.depth.tmp.npz')
            np.savez(tmp, stamps_ns=np.array([d[0] for d in depth], np.int64), camera=np.array([d[1] for d in depth], np.int8),
                     counts=np.array([len(d[2]) for d in depth], np.int64), points=np.concatenate([d[2] for d in depth]))
            tmp.replace(self.output.with_suffix('.depth.npz'))


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
