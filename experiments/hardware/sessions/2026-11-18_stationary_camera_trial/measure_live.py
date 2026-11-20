#!/usr/bin/env python3
"""Run the perception-only candidate and log observation age on the camera host."""

import argparse
import json
from pathlib import Path
from time import monotonic

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor

from fabric_perception.node import FabricCamera, stamp_ns
from glove_record import NAMESPACES, Recorder


class ObservationLog:
    def __init__(self, node, output):
        self.clock = node.get_clock()
        self.publisher = node.observations_pub
        self.output = output
        self.ages = []

    def publish(self, message):
        self.publisher.publish(message)
        stamp = stamp_ns(message.header.stamp)
        age = (self.clock.now().nanoseconds - stamp) / 1e6
        self.ages.append(age)
        self.output.write(json.dumps({
            'stamp_ns': stamp,
            'publication_age_ms': round(age, 3),
            'rgbd_skew_ms': round((stamp_ns(message.depth_stamp) - stamp) / 1e6, 3),
            'valid_depth_fraction': message.valid_depth_fraction,
            'depth_points': len(message.depth),
            'boundary_points': sum(len(contour.pixels) for contour in message.boundaries),
            'supported_boundary_points': sum(sum(weight > 0 for weight in contour.confidence)
                                             for contour in message.boundaries),
        }) + '\n')


def set_workspace_calibration(camera, file, root):
    """Add this camera host's world anchor to its local buffer."""
    settings = yaml.safe_load(file.read_text())
    record = next(t for t in settings['transforms']
                  if t['parent'] == camera.workspace_frame and t['child'] == root)
    tf = TransformStamped()
    tf.header.frame_id, tf.child_frame_id = record['parent'], record['child']
    t, q = tf.transform.translation, tf.transform.rotation
    t.x, t.y, t.z = record['translation']
    q.x, q.y, q.z, q.w = record['rotation_xyzw']
    camera.tf.set_transform_static(tf, 'fabric_workspace_calibration')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', choices=NAMESPACES, required=True)
    parser.add_argument('--cell', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--seconds', type=float, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    remaps = ['--ros-args', '-r', '__node:=fabric_perception',
              '-r', f'__ns:={NAMESPACES[args.camera]}', '-p', f'camera_id:={args.camera}',
              '--params-file', str(args.cell / f'perception_{args.camera}.yaml'),
              '--params-file', str(args.cell / 'perception_workspace.yaml')]
    # Initialize the candidate with the same parameter order as camera_host.launch.py.
    rclpy.init(args=remaps)
    camera = FabricCamera()
    if camera.workspace.box is None and camera.workspace.depth_range is None:
        raise RuntimeError(f'Workspace overlay was not loaded for {camera.get_fully_qualified_name()}')
    if camera.workspace.box is not None:
        # The camera driver and robot state publisher already supply the rest of the chain.
        namespace = NAMESPACES[args.camera].lstrip('/')
        root = 'nuc/camera_base' if args.camera == 'overhead' else f"{namespace.split('/')[0]}/base_link"
        set_workspace_calibration(camera, args.cell / 'calibration.yaml', root)
    glove_log = Recorder(args.camera, str(args.out / 'gloves.jsonl'), use_global_arguments=False)
    executor = SingleThreadedExecutor()
    executor.add_node(camera)
    executor.add_node(glove_log)
    with (args.out / 'observations.jsonl').open('w', buffering=1) as output:
        fabric_log = ObservationLog(camera, output)
        camera.observations_pub = fabric_log
        started = monotonic()
        try:
            while rclpy.ok() and monotonic() - started < args.seconds:
                executor.spin_once(timeout_sec=.05)
        finally:
            executor.shutdown()
            glove_log.out.close()
            summary = {
                'camera': args.camera,
                'observations': len(fabric_log.ages),
                'publication_age_ms_p50_p95': np.percentile(fabric_log.ages, [50, 95]).round(2).tolist()
                if fabric_log.ages else [],
                'glove_messages': len(glove_log.rows),
                'diagnostics': glove_log.diag,
                'last_reason': camera.drop_reason,
            }
            (args.out / 'summary.json').write_text(json.dumps(summary, indent=2) + '\n')
            print(json.dumps({key: val for key, val in summary.items() if key != 'diagnostics'}), flush=True)
            glove_log.destroy_node()
            camera.destroy_node()
            rclpy.try_shutdown()
    return 0 if fabric_log.ages else 1


if __name__ == '__main__':
    raise SystemExit(main())
