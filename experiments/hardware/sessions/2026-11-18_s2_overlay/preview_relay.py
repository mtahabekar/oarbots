#!/usr/bin/env python3
"""Send an overhead Camera display feed while keeping raw RGB on the NUC."""

import argparse
from copy import deepcopy
from time import monotonic

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class Preview(Node):
    def __init__(self):
        super().__init__('fabric_overhead_preview')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, '/nuc/rgb/camera_info', self.save_info, qos)
        self.create_subscription(Image, '/nuc/rgb/image_raw', self.send_frame, qos)
        self.image_pub = self.create_publisher(Image, '/nuc/preview/rgb/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, '/nuc/preview/rgb/camera_info', 1)
        self.info = None
        self.bridge = CvBridge()
        self.next_frame = 0.0
        self.frames = 0

    def save_info(self, info):
        self.info = info

    def send_frame(self, image):
        now = monotonic()
        if self.info is None or now < self.next_frame:
            return
        info = deepcopy(self.info)
        if (
            info.header.frame_id != image.header.frame_id
            or (info.width, info.height) != (image.width, image.height)
            or max(info.binning_x, info.binning_y) > 1
            or info.roi.width or info.roi.height
        ):
            raise ValueError('Expected matching, full-frame, unbinned overhead RGB calibration')
        # Take every third pixel from (0, 0). Scale both image-plane rows of
        # each camera matrix to the same pixel coordinates as the preview.
        frame = self.bridge.imgmsg_to_cv2(image, 'bgr8')[::3, ::3]
        preview = self.bridge.cv2_to_imgmsg(frame, 'bgr8', header=image.header)
        info.width, info.height = preview.width, preview.height
        info.k = [v / 3 for v in info.k[:6]] + list(info.k[6:])
        info.p = [v / 3 for v in info.p[:8]] + list(info.p[8:])
        info.header = image.header
        self.info_pub.publish(info)
        self.image_pub.publish(preview)
        self.next_frame = now + 1 / 5
        self.frames += 1
        if self.frames == 1:
            self.get_logger().info(
                f'Preview {info.width}x{info.height}, up to 5 Hz, from {image.width}x{image.height}'
            )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, required=True)
    args = parser.parse_args()
    rclpy.init()
    preview = Preview()
    end = monotonic() + args.seconds
    try:
        while rclpy.ok() and monotonic() < end:
            rclpy.spin_once(preview, timeout_sec=0.1)
    finally:
        print(f'Published {preview.frames} preview pairs', flush=True)
        preview.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
