#!/usr/bin/env python3
"""Measure stationary clip points from annotated RGB views and the measured cell TF."""

import argparse
import json
from pathlib import Path
from time import monotonic

import cv2
import numpy as np
import yaml

from fabric_perception.core import Camera, depth_metres
from human_hand_tracking.fusion import rotation_matrix

CAMERAS = {'overhead': 'nuc', 'blue_wrist': 'oarbot_blue', 'silver_wrist': 'oarbot_silver'}
TOOLS = {colour: f'oarbot_{colour}/j2n6s300_end_effector' for colour in ('blue', 'silver')}


def matrix(translation, quaternion):
    result = np.eye(4)
    result[:3, :3] = rotation_matrix(quaternion)
    result[:3, 3] = translation
    return result


def snapshot(calibration):
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    data = yaml.safe_load(calibration.read_text())
    world = data['world_frame']
    surveyed = {row['child']: matrix(row['translation'], row['rotation_xyzw'])
                for row in data['transforms'] if row['parent'] == world}
    requests = {f'{ns}/rgb_camera_link': ('nuc/camera_base' if ns == 'nuc' else f'{ns}/base_link')
                for ns in CAMERAS.values()}
    requests.update({frame: f'oarbot_{colour}/base_link' for colour, frame in TOOLS.items()})
    rclpy.init()
    node = Node('grip_calibration_tf_snapshot')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    try:
        deadline = monotonic() + 12
        while monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            transforms = {}
            for child, parent in requests.items():
                try:
                    msg = buffer.lookup_transform(parent, child, Time())
                except TransformException:
                    continue
                t, q = msg.transform.translation, msg.transform.rotation
                stamp = msg.header.stamp
                ns = stamp.sec * 1_000_000_000 + stamp.nanosec
                if ns and abs(node.get_clock().now().nanoseconds - ns) > 500_000_000:
                    continue
                transforms[child] = {
                    'world_from_frame': (surveyed[parent] @ matrix((t.x, t.y, t.z), (q.x, q.y, q.z, q.w))).tolist(),
                    'tf_stamp_ns': ns,
                }
            if len(transforms) == len(requests):
                return {'world_frame': world, 'snapshot_ns': node.get_clock().now().nanoseconds,
                        'frames': transforms}
        missing = ', '.join(frame for frame in requests if frame not in transforms)
        raise RuntimeError(f'Missing fresh transform for {missing}')
    finally:
        del listener
        node.destroy_node()
        rclpy.try_shutdown()


def triangulate(origins, directions):
    """Least-squares point and perpendicular distance to every observation ray."""
    perpendicular = np.eye(3) - directions[:, :, None] * directions[:, None, :]
    rhs = np.einsum('nij,nj->ni', perpendicular, origins)
    point, _, rank, _ = np.linalg.lstsq(perpendicular.reshape(-1, 3), rhs.ravel(), rcond=None)
    if rank != 3 or np.any(np.sum((point - origins) * directions, axis=1) <= 0):
        raise ValueError('Contact point requires distinct, forward-facing camera rays')
    return point, np.linalg.norm(np.einsum('nij,nj->ni', perpendicular, point - origins), axis=1)


def read_camera(path):
    info = json.loads(path.read_text())
    return info['frame_id'], Camera(info['width'], info['height'], info['k'], info['d'])


def depth_patch(path, uv, camera):
    raw = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if raw is None or raw.shape != (camera.height, camera.width):
        raise ValueError(f'Missing or mismatched registered depth image {path}')
    x, y = np.rint(uv).astype(int)
    patch = depth_metres(raw[max(0, y-2):y+3, max(0, x-2):x+3], '16UC1')
    return patch[patch > 0]


def fit_depth(snapshot_path, annotation_path):
    """Provisional contact coordinates from a visible tip in one registered RGB-D view."""
    frames = json.loads(snapshot_path.read_text())['frames']
    annotations = json.loads(annotation_path.read_text())
    results = {}
    for colour, views in annotations.items():
        tool_from_world = np.linalg.inv(frames[TOOLS[colour]]['world_from_frame'])
        points = []
        for view in views:
            frame, camera = read_camera(annotation_path.parent / view['camera_info'])
            uv = np.asarray(view['pixel_xy'], dtype=float)
            if uv.shape != (2,) or not np.all((uv >= 0) & (uv < [camera.width, camera.height])):
                raise ValueError('Contact pixel is outside the RGB image')
            samples = depth_patch(annotation_path.parent / view['depth_png'], uv, camera)
            if not samples.size:
                raise ValueError(f'No registered depth samples at {uv.tolist()}')
            depth = float(np.median(samples))
            world_from_camera = np.asarray(frames[frame]['world_from_frame'])
            world = world_from_camera @ np.r_[camera.unproject([uv], [depth])[0], 1.]
            tool = (tool_from_world @ world)[:3]
            points.append({
                'camera_frame': frame, 'pixel_xy': uv.tolist(), 'depth_png': view['depth_png'],
                'depth_samples': int(samples.size), 'depth_m_median': depth,
                'depth_m_min_max': [float(samples.min()), float(samples.max())],
                'clip_world_m': world[:3].tolist(), 'clip_in_tool_m': tool.tolist(),
            })
        results[colour] = {'status': 'provisional_single_view', 'observations': points}
    return results


def fit(snapshot_path, annotation_path):
    frames = json.loads(snapshot_path.read_text())['frames']
    annotations = json.loads(annotation_path.read_text())
    results = {}
    for colour, placements in annotations.items():
        tool_from_world = np.linalg.inv(frames[TOOLS[colour]]['world_from_frame'])
        estimates = []
        for placement in placements:
            views, origins, directions = [], [], []
            for view in placement:
                info_path = annotation_path.parent / view['camera_info']
                frame, camera = read_camera(info_path)
                uv = np.asarray(view['pixel_xy'], dtype=float)
                if uv.shape != (2,) or not np.all((uv >= 0) & (uv < [camera.width, camera.height])):
                    raise ValueError('Contact pixel is outside the RGB image')
                world_from_camera = np.asarray(frames[frame]['world_from_frame'])
                ray = world_from_camera[:3, :3] @ camera.unproject([uv], [1.])[0]
                origins.append(world_from_camera[:3, 3])
                directions.append(ray / np.linalg.norm(ray))
                views.append((view, frame, camera, uv, world_from_camera))
            if len({frame for _, frame, *_ in views}) < 2:
                raise ValueError('Annotate at least two camera views for each placement')
            point, ray_error = triangulate(np.array(origins), np.array(directions))
            reprojections = []
            for (view, frame, camera, uv, world_from_camera), error in zip(views, ray_error):
                optical = np.linalg.solve(world_from_camera, np.r_[point, 1.])[:3]
                projected, _ = cv2.projectPoints(optical, np.zeros(3), np.zeros(3), camera.k, camera.distortion)
                result = {'camera_frame': frame, 'pixel_xy': uv.tolist(),
                          'ray_error_mm': float(error * 1000),
                          'reprojection_px': float(np.linalg.norm(projected.ravel() - uv)),
                          'triangulated_depth_m': float(optical[2])}
                if view.get('depth_png'):
                    samples = depth_patch(annotation_path.parent / view['depth_png'], uv, camera)
                    result['local_measured_depth_m'] = float(np.median(samples)) if len(samples) else None
                    result['depth_samples'] = int(samples.size)
                reprojections.append(result)
            estimates.append({'clip_world_m': point.tolist(),
                              'clip_in_tool_m': (tool_from_world @ np.r_[point, 1.])[:3].tolist(),
                              'views': reprojections})
        points = np.asarray([entry['clip_in_tool_m'] for entry in estimates])
        if not len(points):
            raise ValueError(f'No annotated placements for {colour}')
        centre = np.median(points, axis=0)
        results[colour] = {'candidate_tool_to_grip_m': centre.tolist(),
                           'placement_residuals_mm': (np.linalg.norm(points - centre, axis=1) * 1000).tolist(),
                           'placements': estimates}
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    actions = parser.add_subparsers(dest='action', required=True)
    capture = actions.add_parser('snapshot', help='Read the stationary tool and camera TF')
    capture.add_argument('calibration', type=Path)
    solve = actions.add_parser('fit', help='Triangulate annotated contact points')
    solve.add_argument('snapshot', type=Path)
    solve.add_argument('annotations', type=Path)
    solve.add_argument('--depth-only', action='store_true', help='Report provisional positions from single-view RGB-D')
    args = parser.parse_args()
    if args.action == 'snapshot':
        result = snapshot(args.calibration)
    else:
        result = (fit_depth if args.depth_only else fit)(args.snapshot, args.annotations)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
