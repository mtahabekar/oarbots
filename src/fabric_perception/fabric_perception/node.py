"""Synchronized per-camera ROS adapter; processing stays next to the camera."""
from collections import OrderedDict
import hashlib
from pathlib import Path
from time import monotonic
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge, CvBridgeError
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Point, Point32
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import CameraInfo, Image
from fabric_state_msgs.msg import (Boundary2D, DepthObservation, FabricObservations,
                                   GloveObservation, GloveObservations, PointTrack, UVPrior)
from human_hand_tracking.tags import GloveDetector
from .core import Camera, DepthProcessor, PointTracker, Segmenter, UVRegressor, boundaries, depth_metres


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def time_msg(ns):
    return Time(sec=int(ns // 1_000_000_000), nanosec=int(ns % 1_000_000_000))


def model_source(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as model:
        for chunk in iter(lambda: model.read(1024 * 1024), b''):
            digest.update(chunk)
    return 'onnx:sha256:' + digest.hexdigest()


class FabricCamera(Node):
    def __init__(self):
        super().__init__('fabric_camera')
        param = lambda name, default: self.declare_parameter(name, default).value
        self.camera_id = param('camera_id', 'camera')
        self.max_age = param('max_age_sec', .25)
        self.sync_slop = param('sync_slop_sec', .015)
        self.publish_debug = param('publish_debug_images', False)
        self.require_robot_mask = param('require_robot_mask', False)
        self.boundary_budget = param('max_boundary_points', 1200)
        if (not self.camera_id or not np.isfinite([self.max_age, self.sync_slop]).all()
                or not 0 < self.sync_slop < self.max_age or self.boundary_budget < 3):
            raise ValueError('Camera ID and valid synchronization/freshness settings required')
        backend = param('segmentation_backend', 'hsv')
        model = param('segmentation_model', '')
        size = param('model_input_size', [640, 384])
        self.segmenter = Segmenter(backend, model, size,
            param('cloth_max_value', 85), param('cloth_max_saturation', 110),
            param('min_component_area', 300), param('segmentation_confidence', .65),
            param('workspace_roi', [0., 0., 1., 1.]))
        self.segment_source = model_source(model) if backend == 'onnx' else 'hsv_baseline_uncalibrated'
        uv_model = param('uv_model', '')
        self.uv = UVRegressor(uv_model, size, param('uv_confidence', .7), param('uv_stddev', .05))
        self.uv_source = model_source(uv_model) if uv_model else 'unavailable'
        self.depth = DepthProcessor(param('min_depth_m', .2), param('max_depth_m', 3.),
            param('depth_edge_threshold_m', .03), param('depth_noise_floor_m', .003),
            param('depth_noise_scale', .002), param('sample_stride', 8), param('max_depth_samples', 1500))
        self.tracker = PointTracker(param('max_tracks', 200), param('track_max_gap_sec', .2),
                                    param('track_fb_threshold_px', 1.))
        self.gloves = GloveDetector(param('glove_tag_ids', [20, 21]), param('glove_tag_size_m', .04),
            param('tag_to_grip_offsets_m', [0., 0., 0., 0., 0., 0.]),
            param('glove_hsv_lower', [35, 80, 60]), param('glove_hsv_upper', [90, 255, 255]),
            param('glove_tag_dictionary', 'DICT_4X4_50'), param('tag_max_reprojection_px', 2.),
            param('tag_min_color_support', .08), param('tag_pixel_stddev', .7),
            param('tag_position_floor_m', .01))
        self.bridge = CvBridge()
        self.info, self.last_stamp, self.calibration = None, 0, None
        self.robot_masks = OrderedDict()
        self.pending_images = OrderedDict()
        self.last_arrival, self.summary, self.drop_reason = None, {}, 'Waiting for synchronized images and calibration'
        self.observations_pub = self.create_publisher(FabricObservations, 'fabric/observations', qos_profile_sensor_data)
        self.hands_pub = self.create_publisher(GloveObservations, 'fabric/gloves', qos_profile_sensor_data)
        self.labels_pub = self.create_publisher(Image, 'fabric/labels', qos_profile_sensor_data)
        self.depth_valid_pub = self.create_publisher(Image, 'fabric/depth_valid', qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, 'fabric/diagnostics', 10)
        self.create_subscription(CameraInfo, 'rgb/camera_info', self.camera_info, qos_profile_sensor_data)
        self.create_subscription(Image, 'fabric/robot_mask', self.robot_mask, qos_profile_sensor_data)
        self.rgb_sub = Subscriber(self, Image, 'rgb/image_raw', qos_profile=qos_profile_sensor_data)
        self.depth_sub = Subscriber(self, Image, 'depth_to_rgb/image_raw', qos_profile=qos_profile_sensor_data)
        self.synchronizer = ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 5, self.sync_slop)
        self.synchronizer.registerCallback(self.images)
        self.create_timer(.5, self.diagnostics)

    def camera_info(self, msg):
        self.info = msg

    def robot_mask(self, msg):
        capture = stamp_ns(msg.header.stamp)
        self.robot_masks[capture] = msg
        while len(self.robot_masks) > 10:
            self.robot_masks.popitem(last=False)
        pending = self.pending_images.pop(capture, None)
        if pending is not None:
            self.images(*pending)

    def images(self, rgb, depth):
        start = monotonic()
        capture = stamp_ns(rgb.header.stamp)
        age = (self.get_clock().now().nanoseconds - capture) / 1e9
        depth_age = (self.get_clock().now().nanoseconds - stamp_ns(depth.header.stamp)) / 1e9
        if capture <= self.last_stamp or not -.01 <= age <= self.max_age or not -.01 <= depth_age <= self.max_age:
            self.drop_reason = 'Rejected old, duplicate, out-of-order or future image'
            return
        info = self.info
        if (info is None or not rgb.header.frame_id or
                rgb.header.frame_id != depth.header.frame_id or rgb.header.frame_id != info.header.frame_id or
                (rgb.width, rgb.height) != (depth.width, depth.height) or
                (rgb.width, rgb.height) != (info.width, info.height)):
            self.drop_reason = 'Need RGB calibration and depth registered into the same RGB optical frame and resolution'
            return
        if (info.distortion_model not in ('plumb_bob', 'rational_polynomial', '') or
                info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset or
                info.roi.width not in (0, info.width) or info.roi.height not in (0, info.height)):
            self.drop_reason = 'Unsupported distortion model or cropped/binned calibration'
            return
        mask_msg = self.robot_masks.pop(capture, None)
        if self.require_robot_mask and mask_msg is None:
            self.pending_images[capture] = (rgb, depth)
            while len(self.pending_images) > 5:
                self.pending_images.popitem(last=False)
            self.drop_reason = 'Waiting for exact-capture robot exclusion mask'
            return
        try:
            camera = Camera(info.width, info.height, info.k, info.d)
            bgr = self.bridge.imgmsg_to_cv2(rgb, 'bgr8')
            metres = depth_metres(self.bridge.imgmsg_to_cv2(depth, 'passthrough'), depth.encoding)
            robot_mask = None
            if mask_msg is not None:
                if (mask_msg.encoding != 'mono8' or mask_msg.header.frame_id != rgb.header.frame_id or
                        (mask_msg.width, mask_msg.height) != (rgb.width, rgb.height)):
                    raise ValueError('Robot mask must be mono8 in this RGB capture frame/resolution')
                robot_mask = self.bridge.imgmsg_to_cv2(mask_msg, 'mono8')
            signature = (info.width, info.height, tuple(info.k), tuple(info.d))
            if self.calibration is not None and signature != self.calibration:
                self.tracker.gray = self.tracker.points = None
            self.calibration = signature
            glove_mask, hands = self.gloves.detect(bgr, camera.k, camera.distortion)
            sheet, labels, confidence, occlusion, roi = self.segmenter.infer(bgr, glove_mask, robot_mask)
            valid, variance, pixels, points, fraction = self.depth.process(metres, sheet, camera)
            output = FabricObservations(header=rgb.header, camera_id=self.camera_id, camera_info=info,
                depth_stamp=depth.header.stamp, segmentation_source=self.segment_source,
                depth_source='raw_registered_quality_gated', uv_source=self.uv_source,
                valid_depth_fraction=fraction)
            for contour, weights, hole in boundaries(sheet, confidence, occlusion, roi, self.boundary_budget):
                output.boundaries.append(Boundary2D(is_hole=hole,
                    pixels=[Point32(x=float(x), y=float(y)) for x, y in contour], confidence=weights.tolist()))
            for pixel, point in zip(pixels, points):
                x, y = pixel
                output.depth.append(DepthObservation(u=float(x), v=float(y),
                    position=Point32(x=float(point[0]), y=float(point[1]), z=float(point[2])),
                    axial_variance=float(variance[y, x]), quality=.5))
            # UV is image evidence even where carbon ToF fails completely.
            sy, sx = np.nonzero(sheet[::self.depth.stride, ::self.depth.stride])
            uv_pixels = np.column_stack((sx, sy)) * self.depth.stride
            if len(uv_pixels) > self.depth.limit:
                uv_pixels = uv_pixels[np.linspace(0, len(uv_pixels)-1, self.depth.limit, dtype=int)]
            for pixel, uv in self.uv.infer(bgr, uv_pixels):
                output.uv_priors.append(UVPrior(u_pixel=float(pixel[0]), v_pixel=float(pixel[1]),
                    material_uv=uv[:2].tolist(), confidence=float(uv[2]),
                    covariance=[self.uv.stddev**2, 0., 0., self.uv.stddev**2]))
            for identity, first, previous, point, old, error in self.tracker.update(bgr, sheet, capture):
                x, y = np.rint(point).astype(int)
                track = PointTrack(id=identity, first_stamp=time_msg(first), previous_stamp=time_msg(previous),
                    u=float(point[0]), v=float(point[1]), previous_u=float(old[0]), previous_v=float(old[1]),
                    forward_backward_error=error, has_depth=bool(valid[y, x]))
                if track.has_depth:
                    position = camera.unproject([point], [metres[y, x]])[0]
                    track.position = Point32(x=float(position[0]), y=float(position[1]), z=float(position[2]))
                    track.axial_variance = float(variance[y, x])
                output.tracks.append(track)
            glove_output = GloveObservations(header=rgb.header, camera_id=self.camera_id)
            for hand in hands:
                glove_output.hands.append(GloveObservation(hand_id=hand.hand_id, tag_id=hand.tag_id,
                    position=Point(x=float(hand.position[0]), y=float(hand.position[1]), z=float(hand.position[2])),
                    covariance=hand.covariance.ravel().tolist(), reprojection_error_px=hand.reprojection_error,
                    color_support=hand.color_support))
            publication_age = (self.get_clock().now().nanoseconds - capture)/1e9
            if publication_age > self.max_age:
                self.last_stamp = capture
                self.drop_reason = 'Processing exceeded capture-to-publication age limit'
                return
            self.observations_pub.publish(output)
            self.hands_pub.publish(glove_output)  # empty arrays explicitly report no detections
            if self.publish_debug:
                for publisher, data in ((self.labels_pub, labels), (self.depth_valid_pub, np.uint8(valid)*255)):
                    msg = self.bridge.cv2_to_imgmsg(data, 'mono8'); msg.header = rgb.header
                    publisher.publish(msg)
            self.last_stamp, self.last_arrival, self.drop_reason = capture, monotonic(), ''
            self.summary = {'sheet_pixels': int(np.count_nonzero(sheet)), 'depth_valid_fraction': fraction,
                'tracks': len(output.tracks), 'gloves': len(hands), 'processing_ms': (monotonic()-start)*1000,
                'capture_age_ms': publication_age*1000, 'segmentation': self.segment_source, 'UV': self.uv_source,
                'robot_exclusion': 'mask' if robot_mask is not None else 'model' if self.segmenter.net is not None else 'ROI only'}
        except (ValueError, cv2.error, CvBridgeError) as error:
            self.drop_reason = str(error)
            self.get_logger().warning(self.drop_reason, throttle_duration_sec=5.)

    def diagnostics(self):
        fresh = self.last_arrival is not None and monotonic()-self.last_arrival <= self.max_age
        degraded = self.segmenter.net is None or not self.summary.get('sheet_pixels') or not self.summary.get('depth_valid_fraction')
        message = self.drop_reason or ('Classical baseline; validate colour/ROI/noise calibration' if degraded else 'Observations available')
        if not fresh:
            message = self.drop_reason or 'Synchronized camera observations stale'
        self.diagnostics_pub.publish(DiagnosticArray(header=self.get_clock_header(), status=[DiagnosticStatus(
            level=DiagnosticStatus.WARN if not fresh or degraded or self.drop_reason else DiagnosticStatus.OK,
            name=self.get_fully_qualified_name(), hardware_id=self.camera_id, message=message,
            values=[KeyValue(key=k, value=str(v)) for k, v in self.summary.items()])]))

    def get_clock_header(self):
        from std_msgs.msg import Header
        return Header(stamp=self.get_clock().now().to_msg())


def main(args=None):
    rclpy.init(args=args)
    node = FabricCamera()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
