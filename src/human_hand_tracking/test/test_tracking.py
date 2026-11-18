import cv2
import numpy as np
import pytest
from human_hand_tracking.tags import GloveDetector
from human_hand_tracking.fusion import (Measurement, HandFilter, fuse_views, rotation_matrix,
                                       transform_measurement, propagate, HandTrack)


def calibration():
    return np.array([[700., 0., 320.], [0., 700., 240.], [0., 0., 1.]])


def test_tag_pose_offset_and_positive_covariance():
    detector = GloveDetector(tag_size_m=.06, grip_offsets=[.01, -.015, .02, 0., 0., 0.])
    rvec = np.array([[3.0], [.12], [.15]])
    tvec = np.array([[.06], [.02], [.7]])
    points = cv2.projectPoints(detector.object_points, rvec, tvec, calibration(), None)[0][:, 0]
    result = detector.from_corners(21, points, calibration(), np.zeros(5))  # tag 21: the left hand
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
    # Identity comes from the tag, not the colour: tag 21 (left hand) was drawn on the image's left.
    assert next(hand for hand in hands if hand.hand_id == 'human_left').position[0] < 0
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
    return Measurement('human_left', 21, camera, stamp, np.array([x, 0., 1.]),
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
    assert [hand.hand_id for hand in hands] == ['human_right']  # tag 20: the right hand
    # Same corners as an unrestricted search, so cropping does not bias the pose.
    full, ids = detector._aruco(image)
    reference = detector.from_corners(20, full[ids.index(20)][0].astype(np.float64), k, np.zeros(5))
    np.testing.assert_allclose(hands[0].position, reference.position, atol=1e-6)
    # Overlapping glove regions are merged, so one tag is never seen twice.
    image[300:520, 520:700] = (0, 220, 0)
    image[300:520, 515:525] = 230
    assert [hand.hand_id for hand in detector.detect(image, k, np.zeros(5))[1]] == ['human_right']


def test_unscaled_fallback_supplies_only_glove_tags_the_enlarged_search_missed():
    k = np.array([[910., 0., 960.], [0., 910., 540.], [0., 0., 1.]])
    image = np.full((1080, 1920, 3), 230, np.uint8)
    image[300:520, 300:520] = (0, 220, 0)
    draw_tag(image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 20, 370, 370)
    image[300:520, 1300:1520] = (0, 220, 0)
    draw_tag(image, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 21, 1370, 370)
    plain = GloveDetector(tag_size_m=.06, roi_upscale=2)
    both = GloveDetector(tag_size_m=.06, roi_upscale=2, roi_fallback_unscaled=True)
    # Both searches find both tags: the enlarged search's corners are kept, nothing is added twice.
    reference = {h.tag_id: h.position for h in plain.detect(image, k, np.zeros(5))[1]}
    found = {h.tag_id: h.position for h in both.detect(image, k, np.zeros(5))[1]}
    assert set(found) == {20, 21}
    for tag in found:
        np.testing.assert_allclose(found[tag], reference[tag], atol=1e-9)
    # The enlarged search misses tag 20 (as glare did on the real overhead camera): only the
    # fallback recovers it, from the original-size crop.
    for detector in (plain, both):
        search = detector._aruco
        def without_20_when_enlarged(crop, search=search):
            corners, ids = search(crop)
            if crop.shape[0] > 1000:  # enlarged crop: (220 px glove + 2 x 220 px padding) x 2
                keep = [i for i, tag in enumerate(ids) if tag != 20]
                return [corners[i] for i in keep], [ids[i] for i in keep]
            return corners, ids
        detector._aruco = without_20_when_enlarged
    assert [h.tag_id for h in plain.detect(image, k, np.zeros(5))[1]] == [21]
    assert sorted(h.tag_id for h in both.detect(image, k, np.zeros(5))[1]) == [20, 21]
    with pytest.raises(ValueError):
        GloveDetector(roi_upscale=0)


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


def test_depth_grid_keeps_a_tag_whose_depth_has_holes_at_the_five_default_samples():
    """Registered depth on the real wrist cameras has holes on the tags; a grid uses the depth that is
    there, the five default samples miss it. Depth over one quarter only still does not count."""
    k = np.array([[915., 0., 960.], [0., 915., 540.], [0., 0., 1.]])
    rvec = cv2.Rodrigues(cv2.Rodrigues(np.array([np.pi, 0., 0.]))[0] @ cv2.Rodrigues(np.array([.3, .2, 0.]))[0])[0]
    tvec = np.array([[.05], [.02], [.6]])
    five = GloveDetector(tag_size_m=.08)
    grid = GloveDetector(tag_size_m=.08, depth_grid=7)
    points = cv2.projectPoints(five.object_points, rvec, tvec, k, None)[0][:, 0]
    depth = plane_depth(five, rvec, tvec, k, (1080, 1920))
    centre = points.mean(axis=0)
    for u, v in np.vstack((centre + .7 * (points - centre), centre)):   # the five default samples
        x, y = int(round(u)), int(round(v))
        depth[y-4:y+5, x-4:x+5] = 0                                        # a hole at each
    assert five.from_corners(21, points, k, np.zeros(5), depth=depth) is None
    kept = grid.from_corners(21, points, k, np.zeros(5), depth=depth)
    plain = grid.from_corners(21, points, k, np.zeros(5))
    assert kept is not None and np.linalg.norm(kept.position - plain.position) < .002   # exact corners: same range
    # Depth only in one quarter of the tag: the tilt is not constrained, so no depth fit.
    quarter = np.zeros_like(depth)
    x0, y0 = np.floor(points.min(axis=0)).astype(int)
    quarter[y0:int(centre[1]), x0:int(centre[0])] = depth[y0:int(centre[1]), x0:int(centre[0])]
    rv, tv = cv2.solvePnPGeneric(grid.object_points, points, k, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)[1:3]
    assert np.isnan(grid.depth_fit(rv[0], tv[0], points, quarter)[0])
    with pytest.raises(ValueError):
        GloveDetector(depth_grid=2)


def test_consumer_propagation_uses_the_filter_model_and_adds_its_process_noise():
    state = np.r_[[1., 2., 3.], [.1, 0., 0.], [0., .5, 0.]]
    covariance = np.diag([.005**2]*3+[.05**2]*3+[.3**2]*3)
    for tau in (.1, .2, .45):
        out, p = propagate(state, covariance, 3., tau)
        np.testing.assert_allclose(out[:3], [1+.1*tau, 2+.25*tau**2, 3.])  # acceleration is not dropped
        frozen = np.sqrt(.005**2+(tau*.05)**2+(tau**2/2*.3)**2)
        # The jerk term alone is 91 mm per axis at 0.45 s for sigma_j = 3 m/s^3.
        np.testing.assert_allclose(np.sqrt(p[0, 0]-frozen**2), 3*np.sqrt(tau**5/20), rtol=1e-9)
    np.testing.assert_allclose(propagate(state, covariance, 3., 0.)[1], covariance)
    filt = HandFilter(jerk_stddev=3., floor=.01)
    filt.update(np.zeros(3), np.eye(3)*.005**2, 1_000_000_000)
    filt.update(np.array([.01, 0., 0.]), np.eye(3)*.005**2, 1_100_000_000)
    full, full_covariance = filt.full()
    assert full.shape == (9,) and full_covariance.shape == (9, 9)
    np.testing.assert_allclose(np.diag(full_covariance)[:3], np.diag(filt.p)[:3]+.01**2)


def test_hand_track_interpolates_between_detections_and_extrapolates_honestly():
    # A hand that accelerates: x(t) = 0.5 * 0.8 * t^2, detected every 0.2 s.
    track, filt = HandTrack(floor=.01), HandFilter(jerk_stddev=3., floor=.01)
    for k in range(8):
        t = k*.2
        assert filt.update(np.array([.4*t*t, 0., 1.]), np.eye(3)*.003**2, int(1e9+t*1e9)) is not None
        assert track.add(int(1e9+t*1e9), *filt.full(), 3.)
    assert not track.add(int(1e9), *filt.full(), 3.)  # never runs backward
    query = int(1e9+1.1e9)  # midway between the detections at 1.0 s and 1.2 s
    between = track.at(query, 1.)
    assert between['bracketed'] and between['age_sec'] == pytest.approx(.1)
    assert abs(between['position'][0]-.4*1.1**2) < .005
    causal = HandTrack(floor=.01); causal.states = [s for s in track.states if s[0] <= query]
    ahead = causal.at(query, 1.)
    assert not ahead['bracketed']
    # Past the last detection the stated uncertainty is larger than between two detections.
    assert np.trace(ahead['covariance']) > np.trace(between['covariance'])
    assert abs(ahead['position'][0]-.4*1.1**2) < 3*np.sqrt(ahead['covariance'][0, 0])
    late = track.at(int(1e9+1.4e9+.5e9), 1.)  # 0.5 s after the newest detection
    assert not late['bracketed'] and np.sqrt(late['covariance'][0, 0]) > .05
    assert track.at(int(1e9+1.4e9+2e9), 1.) is None and track.at(int(.5e9), 1.) is None
