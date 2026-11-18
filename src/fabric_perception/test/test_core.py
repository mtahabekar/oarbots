import cv2
import numpy as np
import pytest
from fabric_perception.core import (Camera, DepthProcessor, DepthWorkspace, PointTracker, Segmenter,
                                    UVRegressor, boundaries, depth_metres)


def camera(width=160, height=120, distortion=()):
    return Camera(width, height, [[200., 0., width/2], [0., 210., height/2], [0., 0., 1.]], distortion)


def test_segmentation_excludes_gloves_robots_and_workspace_without_bridging():
    image = np.full((120, 160, 3), 220, np.uint8)
    image[20:100, 10:150] = 40
    gloves = np.zeros((120, 160), np.uint8); gloves[30:80, 60:90] = 255
    robots = np.zeros_like(gloves); robots[:, 130:] = 255
    segmenter = Segmenter(min_component_area=30, roi=[.05, .05, .95, .95])
    sheet, labels, confidence, occlusion, roi = segmenter.infer(image, gloves, robots)
    assert sheet[50, 40] and not sheet[50, 70] and not sheet[50, 135]
    assert not sheet[29, 59]  # dilated hand margin
    assert labels[50, 70] == 2 and labels[50, 135] == 3
    contours = boundaries(sheet, confidence, occlusion, roi)
    assert any(hole for _, _, hole in contours)
    assert any(np.any(weights == 0) for _, weights, _ in contours)
    assert any(np.any(weights > 0) for _, weights, _ in contours)
    bounded = boundaries(sheet, confidence, occlusion, roi, max_points=6)
    assert sum(len(c) for c, _, _ in bounded) <= 6


def test_boundary_samples_are_dense_and_confidence_is_local_to_each_edge():
    image = np.full((1080, 1920, 3), 220, np.uint8)
    image[200:850, 300:1600] = 40
    robots = np.zeros(image.shape[:2], np.uint8); robots[600:, 1200:] = 255
    sheet, _, confidence, occlusion, roi = Segmenter().infer(image, np.zeros_like(robots), robots)
    (contour, weights, hole), = boundaries(sheet, confidence, occlusion, roi, max_points=1200)
    assert not hole and 1000 <= len(contour) <= 1200
    spacing = np.linalg.norm(np.diff(contour, axis=0), axis=1)
    assert spacing.max() < 6  # no straight edge collapses to its two end vertices
    x, y = contour.T
    true_bottom = (y == 849) & (x < 1180)
    true_right = (x == 1599) & (y < 580)
    beside_robot = (x >= 1194) & (y >= 594)  # within the 4 px invalid band of the dilated robot
    assert true_bottom.sum() > 100 and np.all(weights[true_bottom] > 0)
    assert true_right.sum() > 50 and np.all(weights[true_right] > 0)
    assert beside_robot.sum() > 50 and np.all(weights[beside_robot] == 0)


def test_depth_gates_holes_fold_edges_boundaries_and_uses_metric_units():
    c = camera()
    depth = np.full((120, 160), 1000, np.uint16)
    depth[40, 40] = 0
    depth[:, 80:] = 1200
    sheet = np.full(depth.shape, 255, np.uint8)
    processor = DepthProcessor(stride=1, max_samples=20000)
    metres = depth_metres(depth, '16UC1')
    valid, variance, pixels, points, fraction = processor.process(metres, sheet, c)
    assert not valid[40, 40] and not valid[40, 41]
    assert not valid[50, 79] and not valid[50, 80]
    assert not valid[0, 20] and valid[50, 20] and valid[50, 100]
    assert 0 < fraction < 1 and len(points) == len(pixels)
    assert np.all(variance[valid] > 0)
    projected = cv2.projectPoints(points, np.zeros(3), np.zeros(3), c.k, c.distortion)[0][:, 0]
    np.testing.assert_allclose(projected, pixels, atol=1e-5)
    metres[50, 20] = np.nan; metres[55, 20] = np.inf
    valid2, *_ = processor.process(metres, sheet, c)
    assert not valid2[50, 20] and not valid2[55, 20]
    assert depth_metres(np.ones((2, 2), np.float32), '32FC1')[0, 0] == 1
    with pytest.raises(ValueError):
        depth_metres(depth, 'mono16')


def test_workspace_gate_keeps_rgb_holes_local_and_does_not_create_depth_or_boundary_evidence():
    c = camera()
    bgr = np.full((120, 160, 3), 220, np.uint8)
    bgr[15:105, 15:145] = [20, 30, 55]  # dark but saturated
    depth = np.ones(bgr.shape[:2], np.float32)
    depth[:, :40] = 0
    depth[:, 120:] = 2
    depth[60, 65] = 0
    depth[60, 75] = 2
    gate = DepthWorkspace(depth_range=(.2, 1.25), fill_px=7)
    mask = gate.mask(depth, c)
    assert mask[60, 65] and not mask[60, 75]
    assert not mask[:, :40].any() and not mask[:, 120:].any()
    gloves = np.zeros_like(mask)
    segmenter = Segmenter(cloth_max_saturation=40, cloth_dark_value=60)
    assert not Segmenter(cloth_max_saturation=40).infer(bgr, gloves)[0].any()
    sheet, _, confidence, occlusion, roi = segmenter.infer(bgr, gloves, depth_roi=mask)
    assert sheet[60, 65] and not sheet[60, 75]
    valid, *_ = DepthProcessor().process(depth, sheet, c)
    assert not valid[60, 65] and not valid[60, 75]
    sampled = boundaries(sheet, confidence, occlusion, roi)
    points = np.concatenate([points for points, _, _ in sampled])
    weights = np.concatenate([weights for _, weights, _ in sampled])
    x, y = points.T
    clipped = (x == 40) | (x == 119)
    actual_top = (y == 15) & (45 < x) & (x < 110)
    assert clipped.any() and np.all(weights[clipped] == 0)
    assert actual_top.any() and np.all(weights[actual_top] > 0)
    assert not gate.mask(np.zeros_like(depth), c).any()


def test_workspace_box_uses_calibrated_rays_and_camera_to_box_transform():
    c = camera(distortion=[.1, -.03, .001, .002, 0.])
    depth = np.full((c.height, c.width), 1.0, np.float32)
    depth[15, 20] = np.nan
    rotation = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    translation = np.array([.1, 0., .2])
    box = [.05, -.15, 1., .24, .12, 1.5]
    workspace = DepthWorkspace(box=box)
    y, x = np.indices(depth.shape)
    points = c.unproject(np.column_stack((x.ravel(), y.ravel())), depth.ravel()) @ rotation.T + translation
    expected = ((points >= box[:3]) & (points <= box[3:])).all(axis=1).reshape(depth.shape)
    np.testing.assert_array_equal(workspace.mask(depth, c, (rotation, translation)) > 0, expected)


def test_no_depth_returns_no_fake_points_and_bounded_sampling():
    c = camera()
    sheet = np.full((120, 160), 255, np.uint8)
    valid, _, pixels, points, fraction = DepthProcessor().process(np.zeros(sheet.shape, np.float32), sheet, c)
    assert not valid.any() and not len(points) and not len(pixels) and fraction == 0
    _, _, pixels, _, _ = DepthProcessor(stride=1, max_samples=17).process(np.ones(sheet.shape, np.float32), sheet, c)
    assert len(pixels) == 17


def test_distortion_is_respected_in_unprojection():
    c = camera(distortion=[.1, -.03, .001, .002, 0.])
    pixels = np.array([[20., 20.], [100., 100.], [70., 50.]])
    positions = c.unproject(pixels, [1., .8, 1.2])
    projected = cv2.projectPoints(positions, np.zeros(3), np.zeros(3), c.k, c.distortion)[0][:, 0]
    np.testing.assert_allclose(projected, pixels, atol=1e-4)


def test_klt_preserves_ids_translation_and_resets_on_dropout():
    rng = np.random.default_rng(7)
    image = np.full((180, 240), 15, np.uint8)
    for _ in range(200):
        x, y = int(rng.integers(10, 230)), int(rng.integers(10, 170))
        cv2.circle(image, (x, y), int(rng.integers(2, 6)), int(rng.integers(30, 80)), -1)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    mask = np.full(image.shape[:2], 255, np.uint8)
    tracker = PointTracker(max_tracks=80)
    first = tracker.update(image, mask, 1_000_000_000)
    moved = cv2.warpAffine(image, np.float32([[1, 0, 3], [0, 1, 2]]), (240, 180))
    second = tracker.update(moved, mask, 1_070_000_000)
    matched = [track for track in second if track[1] == 1_000_000_000]
    assert len(matched) > 50
    for identity, _, previous, point, old, error in matched:
        assert identity in {track[0] for track in first} and previous == 1_000_000_000
        np.testing.assert_allclose(point-old, [3., 2.], atol=.15)
        assert error < 1
    third = tracker.update(moved, mask, 2_000_000_000)
    assert not {t[0] for t in third} & {t[0] for t in second}
    assert tracker.update(moved, np.zeros_like(mask), 2_070_000_000) == []


class FakeNet:
    def __init__(self, output):
        self.output = output
    def setInput(self, blob):
        assert blob.shape[0:2] == (1, 3)
    def forward(self):
        return self.output


def test_learned_segmentation_classes_and_malformed_outputs():
    segmenter = Segmenter(min_component_area=10)
    logits = np.zeros((1, 4, 12, 16), np.float32)
    logits[:, 1] = 10
    logits[:, 2, 3:6, 5:8] = 20
    logits[:, 3, :, 12:] = 20
    segmenter.net = FakeNet(logits)
    image = np.zeros((120, 160, 3), np.uint8)
    sheet, labels, *_ = segmenter.infer(image, np.zeros(image.shape[:2], np.uint8))
    assert sheet[20, 20] and labels[40, 60] == 2 and labels[40, 140] == 3
    segmenter.net = FakeNet(np.zeros((1, 2, 12, 16)))
    with pytest.raises(ValueError, match='logits'):
        segmenter.infer(image, np.zeros(image.shape[:2], np.uint8))


def test_uv_is_absent_without_weights_and_is_not_derived_from_pixel_coordinates():
    image = np.zeros((120, 160, 3), np.uint8)
    uv = UVRegressor()
    assert uv.infer(image, [[50, 50]]) == []
    output = np.zeros((1, 3, 12, 16), np.float32)
    output[:, 0] = .2; output[:, 1] = .7; output[:, 2] = .9
    uv.net = FakeNet(output)
    result = uv.infer(image, [[50, 50], [100, 80]])
    assert len(result) == 2
    np.testing.assert_allclose(result[0][1], [.2, .7, .9])
    output[0, 0, 0, 0] = np.nan
    with pytest.raises(ValueError, match='UV ONNX'):
        uv.infer(image, [[50, 50]])


@pytest.mark.parametrize('factory', [lambda: Camera(1, 1, np.zeros((3, 3)), []),
    lambda: Segmenter(roi=[.5, 0, .2, 1]), lambda: DepthProcessor(min_depth=2, max_depth=1),
    lambda: PointTracker(max_tracks=0), lambda: UVRegressor(stddev=0)])
def test_invalid_configuration_fails_early(factory):
    with pytest.raises(ValueError):
        factory()
