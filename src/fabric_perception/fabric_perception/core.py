"""Camera-local observations. All pixel coordinates refer to raw RGB resolution.

Classical methods are deliberately labelled baselines. Point identities are not
material coordinates; learned UV predictions remain priors. Invalid ToF returns
are never silently completed or spatially smoothed across folds.
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class Camera:
    width: int
    height: int
    k: np.ndarray
    distortion: np.ndarray

    def __post_init__(self):
        self.k = np.asarray(self.k, np.float64).reshape(3, 3)
        self.distortion = np.asarray(self.distortion, np.float64)
        if (min(self.width, self.height) <= 0 or not np.isfinite(self.k).all()
                or not np.isfinite(self.distortion).all() or min(self.k[0, 0], self.k[1, 1]) <= 0
                or not np.allclose(self.k[2], [0, 0, 1])):
            raise ValueError('Invalid camera calibration')
        if self.distortion.size not in (0, 4, 5, 8, 12, 14):
            raise ValueError('Unsupported OpenCV distortion coefficients')

    def unproject(self, pixels, depths):
        points = np.asarray(pixels, np.float64).reshape(-1, 1, 2)
        rays = cv2.undistortPoints(points, self.k, self.distortion).reshape(-1, 2)
        return np.column_stack((rays, np.ones(len(rays)))) * np.asarray(depths)[:, None]


def depth_metres(image, encoding):
    if encoding == '16UC1' and image.dtype == np.uint16:
        return image.astype(np.float32) * .001
    if encoding == '32FC1' and image.dtype == np.float32:
        return image.copy()
    raise ValueError('Registered depth must be 16UC1 millimetres or 32FC1 metres')


class Segmenter:
    """HSV baseline or explicit ONNX RGB -> four-class logits contract."""
    def __init__(self, backend='hsv', model_path='', input_size=(640, 384),
                 cloth_max_value=85, cloth_max_saturation=110,
                 min_component_area=300, confidence_threshold=.65, roi=()):
        if backend not in ('hsv', 'onnx'):
            raise ValueError('segmentation_backend must be hsv or onnx')
        if (not 0 <= cloth_max_value <= 255 or not 0 <= cloth_max_saturation <= 255
                or min_component_area <= 0 or not 0 < confidence_threshold <= 1
                or len(input_size) != 2 or min(input_size) <= 0):
            raise ValueError('Invalid segmentation settings')
        if roi and (len(roi) != 4 or not np.isfinite(roi).all() or
                    not (0 <= roi[0] < roi[2] <= 1 and 0 <= roi[1] < roi[3] <= 1)):
            raise ValueError('ROI must be normalized [left, top, right, bottom]')
        self.backend, self.size, self.roi = backend, tuple(input_size), roi
        self.max_v, self.max_s, self.min_area = cloth_max_value, cloth_max_saturation, min_component_area
        self.threshold = confidence_threshold
        self.net = cv2.dnn.readNetFromONNX(model_path) if backend == 'onnx' else None

    def infer(self, bgr, glove_mask, robot_mask=None, hsv=None):
        """`hsv`: the caller's HSV conversion of `bgr`, if it already has one (the glove detector's)."""
        h, w = bgr.shape[:2]
        roi = np.zeros((h, w), np.uint8)
        x0, y0, x1, y1 = self.roi or (0., 0., 1., 1.)
        roi[int(y0*h):int(y1*h), int(x0*w):int(x1*w)] = 255
        if self.net is None:
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV) if hsv is None else hsv
            sheet = cv2.inRange(hsv, (0, 0, 0), (179, self.max_s, self.max_v))
            labels = np.zeros((h, w), np.uint8)
            confidence = np.full((h, w), .5, np.float32)  # heuristic, not calibrated probability
        else:
            self.net.setInput(cv2.dnn.blobFromImage(bgr, 1/255., self.size, swapRB=True))
            logits = np.asarray(self.net.forward())
            if logits.ndim != 4 or logits.shape[:2] != (1, 4) or not np.isfinite(logits).all():
                raise ValueError('Segmentation ONNX must output finite [1,4,H,W] logits')
            scores = logits[0] - logits[0].max(axis=0, keepdims=True)
            probs = np.exp(scores); probs /= probs.sum(axis=0, keepdims=True)
            probs = cv2.resize(probs.transpose(1, 2, 0), (w, h), interpolation=cv2.INTER_LINEAR)
            labels, confidence = probs.argmax(axis=2).astype(np.uint8), probs.max(axis=2)
            labels[confidence < self.threshold] = 4
            glove_mask = cv2.bitwise_or(glove_mask, np.uint8(labels == 2) * 255)
            predicted_robot = np.uint8(labels == 3) * 255
            robot_mask = predicted_robot if robot_mask is None else cv2.bitwise_or(robot_mask, predicted_robot)
            sheet = np.uint8(labels == 1) * 255
        robot_mask = np.zeros((h, w), np.uint8) if robot_mask is None else robot_mask
        occlusion = cv2.bitwise_or(glove_mask, robot_mask)
        occlusion = cv2.dilate(occlusion, np.ones((7, 7), np.uint8))
        sheet[(roi == 0) | (occlusion > 0)] = 0
        count, components, stats, _ = cv2.connectedComponentsWithStats(sheet)
        keep = np.zeros(count, np.uint8)
        keep[1:] = np.uint8(stats[1:, cv2.CC_STAT_AREA] >= self.min_area) * 255
        sheet = keep[components]  # keep disconnected visible pieces; do not bridge occlusions
        labels[labels == 1] = 0
        labels[sheet > 0] = 1
        labels[glove_mask > 0] = 2
        labels[robot_mask > 0] = 3
        labels[roi == 0] = 0
        return sheet, labels, confidence, occlusion, roi


def boundaries(sheet, confidence, occlusion, roi, max_points=1200):
    if max_points < 3:
        raise ValueError('Boundary sample budget must be at least three')
    # Dense pixel chains, resampled at uniform arc length. Polygon vertices are
    # not used: a straight edge would collapse to two points, and a vertex at a
    # fabric/occluder junction cannot say which adjacent edge is observed.
    contours, hierarchy = cv2.findContours(sheet, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if hierarchy is None:
        return []
    # Occluder and workspace edges do not observe the fabric's true boundary.
    invalid = cv2.dilate(occlusion, np.ones((9, 9), np.uint8)) > 0
    invalid |= cv2.erode(roi, np.ones((5, 5), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0) == 0
    candidates = []
    for contour, tree in zip(contours, hierarchy[0]):
        points = contour[:, 0]
        if len(points) < 3:
            continue
        steps = np.linalg.norm(np.diff(points, axis=0, append=points[:1]), axis=1)
        candidates.append((points, np.r_[0., np.cumsum(steps)], bool(tree[3] >= 0)))
    candidates = sorted(candidates, key=lambda item: item[1][-1], reverse=True)[:max_points//3]
    extra = max_points-3*len(candidates)
    total = max(1., sum(arc[-1] for _, arc, _ in candidates))
    output = []
    for points, arc, hole in candidates:
        count = min(len(points), 3 + int(extra*arc[-1]/total))
        index = np.searchsorted(arc, np.arange(count)*arc[-1]/count, side='right') - 1
        samples = points[np.clip(index, 0, len(points)-1)]
        x, y = samples.T
        weights = confidence[y, x].astype(np.float32)
        weights[invalid[y, x]] = 0.
        output.append((samples, weights, hole))
    return output


class DepthProcessor:
    def __init__(self, min_depth=.2, max_depth=3., edge_threshold=.03,
                 noise_floor=.003, noise_scale=.002, stride=8, max_samples=1500):
        if (not np.isfinite([min_depth, max_depth, edge_threshold, noise_floor, noise_scale]).all()
                or not 0 < min_depth < max_depth or min(edge_threshold, noise_floor) <= 0
                or noise_scale < 0 or stride < 1 or max_samples < 1):
            raise ValueError('Invalid depth settings')
        self.minimum, self.maximum, self.edge = min_depth, max_depth, edge_threshold
        self.floor, self.scale, self.stride, self.limit = noise_floor, noise_scale, stride, max_samples

    def process(self, depth, sheet, camera):
        valid = np.isfinite(depth) & (depth >= self.minimum) & (depth <= self.maximum)
        values = np.where(valid, depth, 0).astype(np.float32)
        local_min = cv2.erode(values, np.ones((3, 3), np.uint8))
        local_max = cv2.dilate(values, np.ones((3, 3), np.uint8))
        interior = cv2.erode(sheet, np.ones((5, 5), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0) > 0
        valid &= interior & (local_min > 0) & ((local_max - local_min) <= self.edge)
        variance = values * values  # (floor + scale*z^2)^2, in place: one full-frame buffer
        variance *= self.scale
        variance += self.floor
        variance *= variance
        grid = np.zeros_like(valid); grid[::self.stride, ::self.stride] = True
        y, x = np.nonzero(valid & grid)
        if len(x) > self.limit:
            indices = np.linspace(0, len(x)-1, self.limit, dtype=int)
            x, y = x[indices], y[indices]
        pixels = np.column_stack((x, y))
        points = camera.unproject(pixels, values[y, x]) if len(x) else np.empty((0, 3))
        fraction = np.count_nonzero(valid) / max(1, np.count_nonzero(sheet))
        return valid, variance, pixels, points, float(fraction)


class PointTracker:
    """Forward/backward checked KLT baseline with camera-local persistent IDs."""
    def __init__(self, max_tracks=200, max_gap=.2, fb_threshold=1.0):
        if max_tracks < 1 or not np.isfinite([max_gap, fb_threshold]).all() or min(max_gap, fb_threshold) <= 0:
            raise ValueError('Invalid point tracking settings')
        self.limit, self.gap, self.threshold = max_tracks, max_gap, fb_threshold
        self.gray = self.points = None
        self.tracks, self.stamp, self.next_id = [], None, 0

    def update(self, bgr, sheet, stamp_ns):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        mask = cv2.erode(sheet, np.ones((11, 11), np.uint8), borderType=cv2.BORDER_CONSTANT, borderValue=0)
        output, kept_points, kept_tracks = [], [], []
        previous_stamp = self.stamp
        if (self.gray is not None and self.gray.shape == gray.shape and self.points is not None
                and previous_stamp < stamp_ns <= previous_stamp + self.gap * 1e9):
            forward, status, _ = cv2.calcOpticalFlowPyrLK(self.gray, gray, self.points, None)
            if forward is None:
                forward = self.points.copy()
                status = np.zeros((len(forward), 1), np.uint8)
            backward, back_status, _ = cv2.calcOpticalFlowPyrLK(gray, self.gray, forward, None)
            if backward is None:
                backward = self.points.copy()
                back_status = np.zeros((len(backward), 1), np.uint8)
            for old, new, back, ok, back_ok, track in zip(self.points[:, 0], forward[:, 0], backward[:, 0],
                                                        status[:, 0], back_status[:, 0], self.tracks):
                if not ok or not back_ok or not np.isfinite(np.r_[new, back]).all():
                    continue
                x, y = np.rint(new).astype(int)
                error = float(np.linalg.norm(back-old))
                if not (0 <= x < gray.shape[1] and 0 <= y < gray.shape[0] and mask[y, x] and error <= self.threshold):
                    continue
                output.append((track[0], track[1], previous_stamp, new, old, error))
                kept_points.append(new); kept_tracks.append(track)
        for point in kept_points:
            cv2.circle(mask, tuple(np.rint(point).astype(int)), 10, 0, -1)
        capacity = self.limit - len(kept_points)
        if capacity:
            new = cv2.goodFeaturesToTrack(gray, capacity, .02, 10, mask=mask, blockSize=5)
            if new is not None:
                for point in new[:, 0]:
                    track = (self.next_id, stamp_ns); self.next_id += 1
                    kept_points.append(point); kept_tracks.append(track)
                    output.append((track[0], stamp_ns, stamp_ns, point, point, 0.))
        self.gray, self.stamp, self.tracks = gray, stamp_ns, kept_tracks
        self.points = np.asarray(kept_points, np.float32).reshape(-1, 1, 2) if kept_points else None
        return output


class UVRegressor:
    """Optional trained RGB -> [u,v,confidence] prior; no geometric UV fallback."""
    def __init__(self, model_path='', input_size=(640, 384), threshold=.7, stddev=.05):
        if not 0 < threshold <= 1 or not np.isfinite(stddev) or stddev <= 0:
            raise ValueError('Invalid UV prior confidence/noise')
        self.net = cv2.dnn.readNetFromONNX(model_path) if model_path else None
        self.size, self.threshold, self.stddev = tuple(input_size), threshold, stddev

    def infer(self, bgr, pixels):
        if self.net is None or len(pixels) == 0:
            return []
        self.net.setInput(cv2.dnn.blobFromImage(bgr, 1/255., self.size, swapRB=True))
        result = np.asarray(self.net.forward())
        if (result.ndim != 4 or result.shape[:2] != (1, 3) or not np.isfinite(result).all()
                or result.min() < 0 or result.max() > 1):
            raise ValueError('UV ONNX must output normalized finite [1,3,H,W] u,v,confidence')
        uv = cv2.resize(result[0].transpose(1, 2, 0), (bgr.shape[1], bgr.shape[0]))
        return [(pixel, uv[int(pixel[1]), int(pixel[0])].copy()) for pixel in pixels
                if uv[int(pixel[1]), int(pixel[0]), 2] >= self.threshold]
