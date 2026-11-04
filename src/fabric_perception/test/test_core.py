import cv2
import numpy as np
import pytest
from fabric_perception.core import (Camera, DepthProcessor, PointTracker, Segmenter,
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
