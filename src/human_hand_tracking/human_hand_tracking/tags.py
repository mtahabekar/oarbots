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
                 min_color_support=0.08, pixel_stddev=0.7, position_floor_m=0.01,
                 min_glove_area_px=100, depth_gate_m=0.005, max_range_correction=0.25, roi_upscale=1):
        # With registered depth (detect(..., depth=)): an ambiguous planar pose is resolved by
        # the candidate whose tilt matches the measured depth within depth_gate_m (mean, after
        # removing the offset), and the tag's range is rescaled to the measured depth; a tag
        # whose range would change by more than max_range_correction (fraction) is rejected.
        # roi_upscale: each glove region is enlarged this many times (cubic) before the ArUco
        # search; tags only ~17 px wide are otherwise often missed.
        self.depth_gate = float(depth_gate_m)
        self.max_range_correction = float(max_range_correction)
        if int(roi_upscale) != roi_upscale or not 1 <= roi_upscale <= 4:
            raise ValueError('roi_upscale must be an integer 1..4')
        self.roi_upscale = int(roi_upscale)
        if (len(tag_ids) != 2 or len(set(tag_ids)) != 2 or
                len(grip_offsets) != 6 or not np.isfinite(grip_offsets).all()):
            raise ValueError('Two unique glove IDs and two finite 3D tag-to-grip offsets required')
        if (not np.isfinite([tag_size_m, max_reprojection_px, pixel_stddev, position_floor_m]).all()
                or min(tag_size_m, max_reprojection_px, pixel_stddev, position_floor_m) <= 0
                or not 0 <= min_color_support <= 1 or min_glove_area_px < 1):
            raise ValueError('Invalid tag size, reprojection gate, colour support, glove area or noise')
        self.min_area = int(min_glove_area_px)
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

    def detect(self, bgr, k, distortion, depth=None, hsv=None):
        """`depth`: optional registered depth (m, same pixels as `bgr`) to resolve ambiguous tag
        poses; `hsv`: the caller's HSV conversion of `bgr`, if it already has one."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV) if hsv is None else hsv
        mask = cv2.inRange(hsv, self.lower, self.upper)
        corners, ids = self.markers(bgr, mask)
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
            measurement = self.from_corners(tag, points, k, distortion, support, depth)
            if measurement is not None:
                measurements.append(measurement)
        return mask, measurements

    def _aruco(self, image):
        if hasattr(cv2.aruco, 'ArucoDetector'):
            corners, ids, _ = cv2.aruco.ArucoDetector(self.dictionary, self.parameters).detectMarkers(image)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(image, self.dictionary, parameters=self.parameters)
        return list(corners), ([] if ids is None else [int(i) for i in ids.ravel()])

    def markers(self, bgr, mask):
        """Search for tags only near glove-coloured regions.

        Full-frame 1080p detection dominates per-frame cost. A tag without
        glove colour nearby fails the support gate anyway, so this loses
        nothing. Overlapping boxes are merged so one tag is never reported
        twice (which the duplicate-ID rule would reject).
        """
        h, w = mask.shape
        _, _, stats, _ = cv2.connectedComponentsWithStats(mask)
        boxes = []
        for x, y, bw, bh, area in stats[1:]:
            if area >= self.min_area:
                pad = max(48, int(bw), int(bh))
                boxes.append([max(0, x-pad), max(0, y-pad), min(w, x+bw+pad), min(h, y+bh+pad)])
        merged = True
        while merged:
            merged = False
            for i in range(len(boxes)):
                for j in range(i+1, len(boxes)):
                    a, b = boxes[i], boxes[j]
                    if a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]:
                        boxes[i] = [min(a[0], b[0]), min(a[1], b[1]), max(a[2], b[2]), max(a[3], b[3])]
                        del boxes[j]
                        merged = True
                        break
                if merged:
                    break
        if len(boxes) > 8:  # cluttered colour mask: fall back to one full-frame search
            boxes = [[0, 0, w, h]]
        corners, ids = [], []
        s = self.roi_upscale
        for x0, y0, x1, y1 in boxes:
            crop = np.ascontiguousarray(bgr[y0:y1, x0:x1])
            if s > 1:
                crop = cv2.resize(crop, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
            found, tags = self._aruco(crop)
            # Pixel centres: upscaled coordinate u maps back to (u + 0.5)/s - 0.5.
            corners += [(quad + 0.5) / s - 0.5 + np.float32([x0, y0]) if s > 1 else quad + np.float32([x0, y0])
                        for quad in found]
            ids += tags
        return corners, (np.array(ids, np.int32).reshape(-1, 1) if ids else None)

    def depth_fit(self, rvec, tvec, points, depth):
        """How one candidate pose agrees with registered depth over the tag.

        Returns (shape residual m, range scale): the mean |predicted - measured| depth after
        removing each side's mean (the tag's tilt, independent of any range bias), and the
        median measured/predicted depth ratio. Samples sit 30 % in from the detected corners
        and at the centre, off the tag's edges; each is the median of the valid pixels of a
        3 x 3 window. (nan, nan) if fewer than four samples have depth.
        """
        rotation = cv2.Rodrigues(rvec)[0]
        centre = points.mean(axis=0)
        pixels = np.vstack((centre + 0.7 * (points - centre), centre))
        model = np.vstack((0.7 * self.object_points, np.zeros((1, 3))))
        predicted = (model @ rotation.T + tvec.ravel())[:, 2]
        h, w = depth.shape[:2]
        pairs = []
        for (u, v), z in zip(pixels, predicted):
            x, y = int(round(u)), int(round(v))
            window = depth[max(0, y-1):min(h, y+2), max(0, x-1):min(w, x+2)].ravel()
            window = window[np.isfinite(window) & (window > 0)]
            if len(window):
                pairs.append((z, float(np.median(window))))
        if len(pairs) < 4:
            return float('nan'), float('nan')
        z, measured = np.array(pairs).T
        shape = float(np.mean(np.abs((measured - measured.mean()) - (z - z.mean()))))
        return shape, float(np.median(measured / z))

    def from_corners(self, tag, points, k, distortion, support=1.0, depth=None):
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
            candidates.append((error, grip, rvec, jacobian[:, :6], tvec))
        candidates.sort(key=lambda item: item[0])
        if not candidates or candidates[0][0] > self.max_error:
            return None
        chosen = candidates[0]
        error, grip, rvec, jacobian, tvec = chosen
        # A planar tag can have two plausible poses. Do not select a distant grip
        # solely because its image residual is marginally smaller. Small tags (tens of
        # pixels) are ambiguous most of the time; the tilt seen in registered depth tells
        # the two apart.
        if len(candidates) > 1 and candidates[1][0] < max(0.5, error * 1.5):
            if np.linalg.norm(candidates[1][1] - grip) > 0.03:
                if depth is None:
                    return None
                shapes = [self.depth_fit(c[2], c[4], points, depth)[0] for c in candidates[:2]]
                if not np.isfinite(shapes).all():
                    return None
                best = int(np.argmin(shapes))
                if shapes[best] > self.depth_gate or shapes[1 - best] < 2 * shapes[best]:
                    return None  # depth does not clearly prefer one pose
                chosen = candidates[best]
                error, grip, rvec, jacobian, tvec = chosen
        if depth is not None:
            # Corner localisation on a small tag is biased inward (it then looks smaller and
            # farther: 2-5 % at ~35 px, 10-15 % at ~15 px on the simulated renders). The
            # measured range at the tag replaces the PnP range along the same ray. A correction
            # beyond the bound means the depth is not the tag's: no position rather than a
            # range known to be biased.
            _, scale = self.depth_fit(rvec, tvec, points, depth)
            if not np.isfinite(scale) or abs(scale - 1) > self.max_range_correction:
                return None
            rotation = cv2.Rodrigues(rvec)[0]
            grip = rotation @ self.offsets[tag] + scale * tvec.ravel()
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
