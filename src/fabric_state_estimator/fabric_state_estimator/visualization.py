"""Optional RViz adapter; display labels describe observations, not accuracy."""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from builtin_interfaces.msg import Duration
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from fabric_state_msgs.msg import FabricModel, FabricState


def mesh_marker(model, state):
    if model.model_id != state.model_id or len(state.positions) != len(model.rest_positions):
        raise ValueError('Model/state material identity mismatch')
    if len(state.visibility) != len(state.positions) or len(model.triangles) % 3:
        raise ValueError('Invalid mesh labels or triangle array')
    faces = np.asarray(model.triangles).reshape(-1, 3)
    if faces.max(initial=0) >= len(state.positions):
        raise ValueError('Mesh triangle vertex outside state')
    edges = np.unique(np.sort(faces[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2), axis=1), axis=0)
    marker = Marker(header=state.header, ns='fabric', id=0, type=Marker.LINE_LIST, action=Marker.ADD,
                    lifetime=Duration(sec=0, nanosec=500_000_000))
    marker.pose.orientation.w = 1.; marker.scale.x = .002
    palette = ((.2, .45, 1.), (.2, .85, .3), (1., .6, .1))
    for vertex in edges.ravel():
        marker.points.append(state.positions[int(vertex)])
        label = int(state.visibility[int(vertex)])
        color = (1., .2, .2) if state.degraded else palette[min(label, 2)]
        marker.colors.append(ColorRGBA(r=color[0], g=color[1], b=color[2], a=1.))
    return marker


class MeshDisplay(Node):
    def __init__(self):
        super().__init__('fabric_mesh_display')
        self.model = None
        self.create_subscription(FabricModel, '/fabric/model', self.set_model,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(FabricState, '/fabric/estimated_state', self.state, qos_profile_sensor_data)
        self.publisher = self.create_publisher(MarkerArray, '/fabric/mesh_markers', 10)

    def set_model(self, model):
        self.model = model

    def state(self, state):
        if self.model is not None:
            try:
                self.publisher.publish(MarkerArray(markers=[mesh_marker(self.model, state)]))
            except ValueError as error:
                self.get_logger().warning(str(error), throttle_duration_sec=5.)


def main(args=None):
    rclpy.init(args=args)
    node = MeshDisplay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node(); rclpy.try_shutdown()
