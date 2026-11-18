#!/usr/bin/env python3
"""How old an Azure Kinect's frames are when they arrive, against its own IMU (gate H9/H12).

Subscribes to `<prefix>/rgb/camera_info`, `<prefix>/rgb/image_raw` and `<prefix>/imu` for a few
seconds and prints each stream's age at arrival (receive time minus header stamp). The IMU shares the
camera's device clock and the driver's device-to-wall-clock offset, so its age is the reference:
image age minus IMU age is the frame delay, independent of that offset. Run it on the machine the
camera is plugged into, in the session's environment (`fabric_session.sh exec`), so the network does
not add to the result.

    python3 scripts/hw/frame_age.py /nuc [seconds]
    python3 scripts/hw/frame_age.py /oarbot_silver/azure_kinect

2026-11-17: stock NUC (5 fps) colour 1.14 s, IMU -0.02 s; our session (15 fps) colour 0.062 s, IMU -0.023 s.
"""
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    prefix, seconds = sys.argv[1].rstrip('/'), float(sys.argv[2]) if len(sys.argv) > 2 else 6.0
    rclpy.init()
    node = rclpy.create_node('frame_age')
    ages = {'camera_info': [], 'image': [], 'imu': []}

    def record(key):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ages[key].append(time.time() - stamp)
        return callback

    node.create_subscription(CameraInfo, prefix + '/rgb/camera_info', record('camera_info'), qos_profile_sensor_data)
    node.create_subscription(Image, prefix + '/rgb/image_raw', record('image'), qos_profile_sensor_data)
    node.create_subscription(Imu, prefix + '/imu', record('imu'), qos_profile_sensor_data)
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    med = {}
    for key, values in ages.items():
        values.sort()
        if values:
            med[key] = values[len(values) // 2]
            print(f'{key:12s} {len(values):5d} msgs ({len(values) / seconds:6.1f} Hz)  age min/median/max '
                  f'{values[0]:+.3f} / {med[key]:+.3f} / {values[-1]:+.3f} s')
        else:
            print(f'{key:12s} no messages')
    if 'image' in med and 'imu' in med:
        print(f'frame delay (image - IMU, median): {med["image"] - med["imu"]:.3f} s')
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
