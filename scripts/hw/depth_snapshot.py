#!/usr/bin/env python3
"""Median depth image of one Azure Kinect, with its intrinsics and pose, for obstacle detection.

Runs on the camera's own host (no images cross the network), in the session environment. `fabric_run.sh
obstacle` streams this file to each host:  lab.sh session <host> exec python3 - <prefix> <root_frame> <seconds>
< depth_snapshot.py  and reads the result (a compressed .npz, ~1 MB) from the stdout line DEPTHNPZ:<base64>.
  prefix      /nuc  or  /oarbot_blue/azure_kinect
  root_frame  a frame the laptop can place in the world from calibration.yaml: nuc/camera_base or
              oarbot_<robot>/base_link (the arm moves the wrist camera; its pose is part of the snapshot)
Saved: depth (float32 m, per-pixel median of the valid samples, 0 where under half the frames were valid),
valid fraction, K, D, distortion model, size, frame_id, T_root_depth (4x4), frames, stamp.
"""
import base64
import io
import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener


def matrix(t):
    q, p = t.rotation, t.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    T = np.eye(4)
    T[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                 [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                 [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def main():
    prefix, root, seconds = sys.argv[1].rstrip('/'), sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 2.
    rclpy.init()
    node = rclpy.create_node('depth_snapshot')
    tf = Buffer()
    TransformListener(tf, node)
    frames, info = [], []
    node.create_subscription(CameraInfo, f'{prefix}/depth/camera_info', lambda m: info.append(m), qos_profile_sensor_data)

    def on_image(m):
        if m.encoding != '16UC1':
            raise SystemExit(f'unexpected depth encoding {m.encoding}')
        frames.append((m.header.frame_id, np.frombuffer(bytes(m.data), np.uint16).reshape(m.height, m.width).copy()))
    node.create_subscription(Image, f'{prefix}/depth/image_raw', on_image, qos_profile_sensor_data)
    end = time.time()+seconds+10.
    while time.time() < end and (len(frames) < max(5, int(15*seconds)) or not info):
        rclpy.spin_once(node, timeout_sec=.05)
    if not frames or not info:
        raise SystemExit(f'no depth frames from {prefix}')
    frame_id = frames[0][0]
    T = None
    while T is None and time.time() < end+5:
        try:
            T = matrix(tf.lookup_transform(root, frame_id, Time()).transform)
        except Exception:
            rclpy.spin_once(node, timeout_sec=.05)
    if T is None:
        raise SystemExit(f'no transform {root} -> {frame_id}')
    stack = np.stack([f for _, f in frames]).astype(np.float32)/1000.
    stack[stack <= 0] = np.nan
    valid = np.mean(np.isfinite(stack), axis=0)
    depth = np.nanmedian(stack, axis=0)
    depth[(valid < .5) | ~np.isfinite(depth)] = 0.
    i = info[-1]
    buf = io.BytesIO()
    np.savez_compressed(buf, depth=depth.astype(np.float32), valid=valid.astype(np.float32), K=np.array(i.k).reshape(3, 3),
                        D=np.array(i.d), model=i.distortion_model, size=(i.width, i.height), frame_id=frame_id, root=root,
                        T_root_depth=T, frames=len(frames), stamp=time.time(), prefix=prefix)
    sys.stdout.write('DEPTHNPZ:'+base64.b64encode(buf.getvalue()).decode()+'\n')
    sys.stdout.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
