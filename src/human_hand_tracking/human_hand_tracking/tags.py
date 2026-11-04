"""Metric tagged-glove observations. No bare-hand detector or MANO is needed.

Tag axes follow OpenCV's IPPE_SQUARE convention; offsets must be calibrated
in that frame. A glove's colour establishes support, never left/right identity.
"""
from dataclasses import dataclass
import cv2
import numpy as np


@dataclass
class TagMeasurement:
    hand_id: str
    tag_id: int
    position: np.ndarray
    covariance: np.ndarray
    reprojection_error: float
    color_support: float


class GloveDetector:
    def __init__(self, tag_ids=(20, 21), tag_size_m=0.04,
                 grip_offsets=(0., 0., 0., 0., 0., 0.),
                 hsv_lower=(35, 80, 60), hsv_upper=(90, 255, 255),
                 dictionary='DICT_4X4_50', max_reprojection_px=2.0,
                 min_color_support=0.08, pixel_stddev=0.7, position_floor_m=0.01):
        if (len(tag_ids) != 2 or len(set(tag_ids)) != 2 or
                len(grip_offsets) != 6 or not np.isfinite(grip_offsets).all()):
            raise ValueError('Two unique glove IDs and two finite 3D tag-to-grip offsets required')
        if (not np.isfinite([tag_size_m, max_reprojection_px, pixel_stddev, position_floor_m]).all()
                or min(tag_size_m, max_reprojection_px, pixel_stddev, position_floor_m) <= 0
                or not 0 <= min_color_support <= 1):
            raise ValueError('Invalid tag size, reprojection gate, colour support or noise')
        self.lower, self.upper = np.array(hsv_lower), np.array(hsv_upper)
        for bound in (self.lower, self.upper):
            if bound.shape != (3,) or np.any(bound < 0) or np.any(bound > (179, 255, 255)):
                raise ValueError('HSV bounds must have H in [0,179], S,V in [0,255]')
        if np.any(self.lower > self.upper):
            raise ValueError('HSV lower must not exceed upper (use a non-wrapping glove hue)')
        if not dictionary.startswith('DICT_') or not hasattr(cv2.aruco, dictionary):
            raise ValueError('Unknown ArUco dictionary')
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
        if any(not 0 <= i < len(self.dictionary.bytesList) for i in tag_ids):
            raise ValueError('Tag ID outside dictionary')
        self.parameters = (cv2.aruco.DetectorParameters_create() if
                           hasattr(cv2.aruco, 'DetectorParameters_create') else cv2.aruco.DetectorParameters())
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.identities = dict(zip(tag_ids, ('human_left', 'human_right')))
        self.offsets = dict(zip(tag_ids, np.asarray(grip_offsets).reshape(2, 3)))
        s = tag_size_m / 2
        self.object_points = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
        self.max_error, self.min_support = max_reprojection_px, min_color_support
        self.pixel_stddev, self.floor = pixel_stddev, position_floor_m

    def detect(self, bgr, k, distortion):
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower, self.upper)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            corners, ids, _ = cv2.aruco.ArucoDetector(self.dictionary, self.parameters).detectMarkers(bgr)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(bgr, self.dictionary, parameters=self.parameters)
        measurements = []
        if ids is None:
            return mask, measurements
        # Cover black/white tag pixels too, including tags whose pose is rejected.
        for quad, tag in zip(corners, ids.ravel()):
            if int(tag) in self.identities:
                cv2.fillConvexPoly(mask, np.rint(quad[0]).astype(np.int32), 255)
        for quad, tag in zip(corners, ids.ravel()):
            tag = int(tag)
            if tag not in self.identities or np.count_nonzero(ids == tag) != 1:
                continue  # duplicate IDs cannot supply a persistent identity
            points = np.asarray(quad[0], np.float64)
            center = points.mean(axis=0)
            radius = max(2, int(np.linalg.norm(points[0] - points[2])))
            x, y = np.rint(center).astype(int)
            crop = hsv[max(0, y-radius):y+radius+1, max(0, x-radius):x+radius+1]
            support = float(np.mean(cv2.inRange(crop, self.lower, self.upper) > 0))
            if support < self.min_support:
                continue
            measurement = self.from_corners(tag, points, k, distortion, support)
            if measurement is not None:
                measurements.append(measurement)
        return mask, measurements

    def from_corners(self, tag, points, k, distortion, support=1.0):
        """Pose and first-order pixel-noise covariance, with a calibration floor."""
        result = cv2.solvePnPGeneric(self.object_points, points, k, distortion,
                                    flags=cv2.SOLVEPNP_IPPE_SQUARE)
        candidates = []
        for rvec, tvec in zip(result[1], result[2]):
            if not np.isfinite(np.r_[rvec.ravel(), tvec.ravel()]).all() or tvec[2, 0] <= 0:
                continue
            rotation = cv2.Rodrigues(rvec)[0]
            if np.min((self.object_points @ rotation.T + tvec.ravel())[:, 2]) <= 0:
                continue
            projected, jacobian = cv2.projectPoints(self.object_points, rvec, tvec, k, distortion)
            error = float(np.sqrt(np.mean(np.sum((projected[:, 0] - points)**2, axis=1))))
            grip = rotation @ self.offsets[tag] + tvec.ravel()
            candidates.append((error, grip, rvec, jacobian[:, :6]))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.max_error:
            return None
        error, grip, rvec, jacobian = candidates[0]
        # A planar tag can have two plausible poses. Do not select a distant grip
        # solely because its image residual is marginally smaller.
        if len(candidates) > 1 and candidates[1][0] < max(0.5, error * 1.5):
            if np.linalg.norm(candidates[1][1] - grip) > 0.03:
                return None
        pose_cov = np.linalg.pinv(jacobian.T @ jacobian, rcond=1e-12) * self.pixel_stddev**2
        derivative = np.zeros((3, 6))
        eps = 1e-5
        for i in range(3):
            delta = np.zeros((3, 1)); delta[i] = eps
            derivative[:, i] = ((cv2.Rodrigues(rvec + delta)[0] -
                                 cv2.Rodrigues(rvec - delta)[0]) @ self.offsets[tag]) / (2 * eps)
        derivative[:, 3:] = np.eye(3)
        covariance = derivative @ pose_cov @ derivative.T + np.eye(3) * self.floor**2
        if not np.isfinite(covariance).all():
            return None
        return TagMeasurement(self.identities[tag], tag, grip, covariance, error, support)
