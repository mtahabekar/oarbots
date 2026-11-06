"""Small ROS conversion boundary; numerical modules never import ROS."""
import numpy as np
from scipy.spatial.transform import Rotation
from fabric_perception.core import Camera
from .data import View


def nanoseconds(stamp):
    return int(stamp.sec)*1_000_000_000+int(stamp.nanosec)


def transform_matrix(transform):
    t, q = transform.transform.translation, transform.transform.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w])
    if not np.isfinite(quaternion).all() or abs(np.linalg.norm(quaternion)-1) > .01:
        raise ValueError('Invalid TF quaternion')
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_quat(quaternion).as_matrix()
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def view_from_message(message, world_from_camera):
    info = message.camera_info
    if info.distortion_model not in ('', 'plumb_bob', 'rational_polynomial'):
        raise ValueError('Unsupported camera distortion model')
    if message.depth_source != 'raw_registered_quality_gated' and message.depth:
        raise ValueError('This estimator requires raw gated depth; learned repairs need explicit provenance support')
    depth = [[d.u, d.v, d.position.x, d.position.y, d.position.z, d.axial_variance]
             for d in message.depth if d.quality > 0]
    boundary = [[p.x, p.y, c] for contour in message.boundaries if not contour.is_hole
                for p, c in zip(contour.pixels, contour.confidence)]
    if any(len(contour.pixels) != len(contour.confidence) for contour in message.boundaries):
        raise ValueError('Boundary pixels/confidence length mismatch')
    tracks = [dict(id=int(t.id), first_ns=nanoseconds(t.first_stamp), u=t.u, v=t.v) for t in message.tracks]
    uv = [[p.u_pixel, p.v_pixel, *p.material_uv, p.covariance[0], p.covariance[3], p.confidence] for p in message.uv_priors]
    return View(message.camera_id, Camera(info.width, info.height, info.k, info.d), world_from_camera,
                depth, boundary, tracks, uv, nanoseconds(message.header.stamp))


def state_message(snapshot, mesh, physics_backend, world_frame):
    from rclpy.time import Time
    from std_msgs.msg import Header
    from geometry_msgs.msg import Point, Vector3
    from fabric_state_msgs.msg import FabricState
    return FabricState(header=Header(stamp=Time(nanoseconds=snapshot['time_ns']).to_msg(), frame_id=world_frame),
        measurement_stamp=Time(nanoseconds=snapshot['measurement_ns']).to_msg(), model_id=mesh.model_id,
        physics_backend=physics_backend, predicted=snapshot['predicted'], degraded=bool(snapshot['degraded']),
        positions=[Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in snapshot['q']],
        velocities=[Vector3(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in snapshot['v']],
        covariance=snapshot['covariance'].ravel().tolist(), visibility=snapshot['visibility'].tolist(),
        observation_age_sec=snapshot['age'].tolist(),
        residual_force=Vector3(x=float(snapshot['residual_force'][0]), y=float(snapshot['residual_force'][1]),
                              z=float(snapshot['residual_force'][2])),
        residual_force_covariance=snapshot['residual_covariance'].ravel().tolist())
