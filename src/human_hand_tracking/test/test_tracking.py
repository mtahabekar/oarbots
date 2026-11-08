import cv2
import numpy as np
import pytest
from human_hand_tracking.tags import GloveDetector
from human_hand_tracking.fusion import (Measurement, HandFilter, fuse_views, rotation_matrix,
                                       transform_measurement)


def calibration():
    return np.array([[700., 0., 320.], [0., 700., 240.], [0., 0., 1.]])


def test_tag_pose_offset_and_positive_covariance():
    detector = GloveDetector(tag_size_m=.06, grip_offsets=[.01, -.015, .02, 0., 0., 0.])
    rvec = np.array([[3.0], [.12], [.15]])
    tvec = np.array([[.06], [.02], [.7]])
    points = cv2.projectPoints(detector.object_points, rvec, tvec, calibration(), None)[0][:, 0]
    result = detector.from_corners(20, points, calibration(), np.zeros(5))
    assert result is not None and result.hand_id == 'human_left'
    np.testing.assert_allclose(result.position, cv2.Rodrigues(rvec)[0] @ [.01, -.015, .02] + tvec.ravel(), atol=1e-6)
    assert np.linalg.eigvalsh(result.covariance).min() >= .01**2 - 1e-10


def draw_tag(image, dictionary, identity, left, top, size=80):
    marker = (cv2.aruco.drawMarker(dictionary, identity, size) if hasattr(cv2.aruco, 'drawMarker')
              else cv2.aruco.generateImageMarker(dictionary, identity, size))
    # A white quiet border around the printed marker is needed for detection.
    image[top-16:top+size+16, left-16:left+size+16] = 255
    image[top:top+size, left:left+size] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)


def test_rendered_same_colour_gloves_have_tag_identity_and_missing_stays_missing():
    detector = GloveDetector(tag_size_m=.06)
    image = np.zeros((480, 640, 3), np.uint8); image[:] = (0, 220, 0)
    draw_tag(image, detector.dictionary, 21, 100, 180)
    draw_tag(image, detector.dictionary, 20, 420, 180)
    mask, hands = detector.detect(image, calibration(), np.zeros(5))
    assert {hand.hand_id for hand in hands} == {'human_left', 'human_right'}
    assert next(hand for hand in hands if hand.hand_id == 'human_left').position[0] > 0
    assert mask[200, 110]  # black marker pixels are excluded as well as glove colour
    image[:] = (0, 220, 0)
    assert detector.detect(image, calibration(), np.zeros(5))[1] == []


def test_duplicate_tag_or_absent_colour_support_cannot_supply_identity():
    detector = GloveDetector()
    image = np.zeros((480, 640, 3), np.uint8); image[:] = (0, 220, 0)
    draw_tag(image, detector.dictionary, 20, 100, 180)
    draw_tag(image, detector.dictionary, 20, 420, 180)
    assert detector.detect(image, calibration(), np.zeros(5))[1] == []
    image[:] = 255
    draw_tag(image, detector.dictionary, 20, 100, 180)
    assert detector.detect(image, calibration(), np.zeros(5))[1] == []


def measurement(camera='a', x=1., stamp=1_000_000_000, covariance=None):
    return Measurement('human_left', 20, camera, stamp, np.array([x, 0., 1.]),
                       np.eye(3)*.01**2 if covariance is None else covariance)


def test_capture_transform_rotates_covariance_and_adds_extrinsic_noise():
    m = measurement(covariance=np.diag([.01, .02, .03]))
    rotation = rotation_matrix([0., 0., np.sqrt(.5), np.sqrt(.5)])
    transformed = transform_measurement(m, rotation, [1, 2, 3], .02)
    np.testing.assert_allclose(transformed.position, [1., 3., 4.], atol=1e-10)
    np.testing.assert_allclose(np.diag(transformed.covariance), [.0204, .0104, .0304])
    with pytest.raises(ValueError):
        rotation_matrix([0., 0., 0., 0.])


def test_robust_view_consensus_and_correlated_floor():
    views = [measurement('a', 1), measurement('b', 1.01), measurement('c', 2)]
    position, covariance, selected = fuse_views(views, views[0].stamp_ns)
    assert len(selected) == 2 and position[0] == pytest.approx(1.005)
    assert covariance[0, 0] == pytest.approx(.01**2/2)  # floor is applied by HandFilter output
    assert fuse_views([views[0], views[2]], views[0].stamp_ns) is None
    assert fuse_views([views[0], views[0]], views[0].stamp_ns) is None
    assert fuse_views([], views[0].stamp_ns) is None
    invalid = measurement(covariance=np.diag([-1, 1, 1]))
    assert fuse_views([invalid], views[0].stamp_ns) is None


def test_hand_filter_joint_velocity_rejection_dropout_and_reacquisition():
    tracker = HandFilter()
    covariance = np.eye(3)*.005**2
    for i in range(30):
        result = tracker.update(np.array([.1*i*.05, 0., 1.]), covariance, int(1e9 + i*.05e9))
        assert result is not None
    state, uncertainty = result
    assert state[3] == pytest.approx(.1, abs=.015)
    assert np.linalg.eigvalsh(uncertainty).min() > 0
    last_stamp = tracker.stamp
    assert tracker.update(np.array([10., 10., 10.]), covariance, last_stamp+50_000_000) is None
    assert tracker.stamp == last_stamp
    assert tracker.update(np.zeros(3), covariance, last_stamp-1) is None
    reacquired, uncertainty = tracker.update(np.array([1., 2., 3.]), covariance, last_stamp+1_000_000_000)
    np.testing.assert_allclose(reacquired, [1., 2., 3., 0., 0., 0.])
    assert uncertainty[3, 3] >= .25


def test_calibration_floor_is_not_averaged_away_over_time():
    floor = .01
    tracker = HandFilter(floor=floor)
    for i in range(200):
        _, uncertainty = tracker.update(np.zeros(3), np.eye(3)*.005**2, int(1e9 + i*.033e9))
    # Many frames shrink the random part, never the shared calibration bias.
    assert np.linalg.eigvalsh(uncertainty[:3, :3]).min() >= floor**2
    assert np.linalg.eigvalsh(tracker.p[:3, :3]).max() < floor**2


def test_glove_region_search_matches_full_frame_and_skips_tags_without_glove_colour():
    detector = GloveDetector(tag_size_m=.06)
    image = np.full((1080, 1920, 3), 230, np.uint8)
    image[300:520, 300:520] = (0, 220, 0)  # one glove, tag on its back
    draw_tag(image, detector.dictionary, 20, 370, 370)
    draw_tag(image, detector.dictionary, 21, 1400, 600)  # on white: no glove nearby
    k = np.array([[910., 0., 960.], [0., 910., 540.], [0., 0., 1.]])
    mask, hands = detector.detect(image, k, np.zeros(5))
    assert [hand.hand_id for hand in hands] == ['human_left']
    # Same corners as an unrestricted search, so cropping does not bias the pose.
    full, ids = detector._aruco(image)
    reference = detector.from_corners(20, full[ids.index(20)][0].astype(np.float64), k, np.zeros(5))
    np.testing.assert_allclose(hands[0].position, reference.position, atol=1e-6)
    # Overlapping glove regions are merged, so one tag is never seen twice.
    image[300:520, 520:700] = (0, 220, 0)
    image[300:520, 515:525] = 230
    assert [hand.hand_id for hand in detector.detect(image, k, np.zeros(5))[1]] == ['human_left']


def plane_depth(detector, rvec, tvec, k, shape):
    """Ideal registered depth of the tag's plane (and beyond it, of the same plane)."""
    rotation = cv2.Rodrigues(rvec)[0]
    normal, origin = rotation[:, 2], tvec.ravel()
    v, u = np.mgrid[0:shape[0], 0:shape[1]]
    rays = np.stack(((u - k[0, 2]) / k[0, 0], (v - k[1, 2]) / k[1, 1], np.ones(shape)), axis=-1)
    return (normal @ origin / (rays @ normal)).astype(np.float32)  # z of the ray's plane hit


def test_ambiguous_small_tag_is_resolved_by_depth_and_range_corrected():
    """A distant, nearly frontal tag has two poses; without depth it is rejected, with depth
    the one matching the measured tilt wins, and a range-biased (shrunken) corner set is
    moved back to the measured depth."""
    k = np.array([[1700., 0., 1023.5], [0., 1700., 767.5], [0., 0., 1.]])
    detector = GloveDetector(tag_size_m=.08, grip_offsets=[0., -.06, -.0155, 0., -.06, -.0155])
    rvec = cv2.Rodrigues(cv2.Rodrigues(np.array([np.pi, 0., 0.]))[0] @ cv2.Rodrigues(np.array([.25, .1, 0.]))[0])[0]
    tvec = np.array([[.3], [.2], [2.2]])
    truth = cv2.Rodrigues(rvec)[0] @ [0., -.06, -.0155] + tvec.ravel()
    points = cv2.projectPoints(detector.object_points, rvec, tvec, k, None)[0][:, 0]
    candidates = cv2.solvePnPGeneric(detector.object_points, points, k, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    grips = [cv2.Rodrigues(r)[0] @ [0., -.06, -.0155] + t.ravel() for r, t in zip(candidates[1], candidates[2])]
    assert len(grips) == 2 and np.linalg.norm(grips[0] - grips[1]) > .03  # genuinely ambiguous
    depth = plane_depth(detector, rvec, tvec, k, (1536, 2048))
    resolved = detector.from_corners(21, points, k, np.zeros(5), depth=depth)
    assert resolved is not None and np.linalg.norm(resolved.position - truth) < .005
    # Corners localised 0.8 px inward (as on anti-aliased renders) read the tag 2-3 % too far.
    centre = points.mean(axis=0)
    shrunk = centre + (points - centre) * (1 - .8 / np.linalg.norm(points[0] - centre))
    corrected = detector.from_corners(21, shrunk, k, np.zeros(5), depth=depth)
    assert corrected is not None and np.linalg.norm(corrected.position - truth) < .01
    flat = detector.from_corners(21, shrunk, k, np.zeros(5), depth=np.full((1536, 2048), np.nan, np.float32))
    assert flat is None  # no depth: still ambiguous, still rejected


def test_depth_that_is_not_the_tag_rejects_rather_than_passing_a_biased_range():
    k = np.array([[1700., 0., 1023.5], [0., 1700., 767.5], [0., 0., 1.]])
    detector = GloveDetector(tag_size_m=.08, grip_offsets=[0., -.06, -.0155, 0., -.06, -.0155])
    rvec, tvec = np.array([[np.pi], [0.], [0.]]), np.array([[.3], [.2], [2.2]])
    points = cv2.projectPoints(detector.object_points, rvec, tvec, k, None)[0][:, 0]
    behind = plane_depth(detector, rvec, tvec + [[0.], [0.], [.8]], k, (1536, 2048))  # 36 % farther
    assert detector.from_corners(21, points, k, np.zeros(5), depth=behind) is None
    assert detector.from_corners(21, points, k, np.zeros(5)) is not None  # without depth: PnP alone
