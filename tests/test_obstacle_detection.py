import numpy as np
import pytest

from scripts.hw import obstacle_detect as detector


def test_foreground_requires_stable_depth_and_support_for_background_holes():
    before = np.full((30, 30), 1.2, np.float32)
    observed = before.copy()
    stable = np.ones_like(before)
    observed[5:16, 5:16] = .8
    before[9, 9] = 0
    observed[20:25, 20:25] = .8
    before[20:25, 20:25] = 0
    stable[11, 11] = .55
    observed[2, 2] = .7

    mask = detector.foreground_mask({'depth': before}, {'depth': observed, 'valid': stable}, .03)

    assert mask[9, 9]
    assert not mask[11, 11]
    assert not mask[2, 2]
    assert not mask[20:25, 20:25].any()


def cluster_at(x, size):
    cells = np.indices((size, size, 1)).reshape(3, -1).T
    return np.repeat(.02*cells + [x, .005, .105], 2, axis=0)


def test_fixed_camera_support_beats_a_larger_disconnected_wrist_cluster():
    overhead = cluster_at(.005, 3)
    wrist = cluster_at(.205, 6)
    points = np.vstack((overhead, wrist, [[.5, .5, .5]]))
    fixed = np.arange(len(points)) < len(overhead)

    keep, count = detector.obstacle_cluster(points, fixed, np.zeros(3), .02)

    assert count == 2
    np.testing.assert_array_equal(points[keep], overhead)


def test_two_substantial_overhead_components_require_a_clearer_scan():
    points = np.vstack((cluster_at(.005, 6), cluster_at(.305, 3)))

    with pytest.raises(SystemExit, match='Multiple substantial objects'):
        detector.obstacle_cluster(points, np.ones(len(points), dtype=bool), np.zeros(3), .02)


def test_connected_depth_trail_cannot_stretch_the_footprint():
    surface = cluster_at(.005, 5)
    trail_x = .105 + .02*np.arange(9)
    trail = np.repeat(np.column_stack((trail_x, np.full(9, .045), np.full(9, .105))), 2, axis=0)
    points = np.vstack((surface, trail))

    keep, _ = detector.obstacle_cluster(points, np.ones(len(points), dtype=bool), np.zeros(3), .02)

    assert keep[:len(surface)].all()
    assert not keep[len(surface) + 2:].any()


def test_floor_fit_ignores_foreground_and_reports_mount_error(monkeypatch):
    xy = np.stack(np.meshgrid(np.linspace(-.5, .5, 31), np.linspace(-.5, .5, 31)), axis=-1).reshape(-1, 2)
    expected = np.array([.065, .012, -.06])
    floor = np.column_stack((xy, detector.plane_height(expected, xy)))
    raised = np.column_stack((xy[::4], floor[::4, 2] + .08))
    monkeypatch.setattr(detector, 'back_project', lambda *_: np.vstack((floor, raised)))

    fitted, count = detector.fit_floor({'depth': np.ones((1, 1))}, np.full(3, -.7), np.full(3, .7), {})
    tilt, height = detector.floor_difference(fitted, np.zeros(3), np.zeros(2))

    np.testing.assert_allclose(fitted, expected, atol=1e-8)
    assert count == len(floor)
    assert tilt > 3.
    assert height == pytest.approx(.06)


def test_measured_box_uses_overhead_pose_and_tape_dimensions():
    angle = np.radians(33.)
    rotation = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    center = np.array([.25, -.12])
    xy = np.column_stack([grid.ravel() for grid in np.meshgrid(
        np.linspace(-.14, .14, 20), np.linspace(-.09, .09, 14))])
    surface = np.column_stack((xy @ rotation.T + center, np.full(len(xy), .51)))

    foot, top, pose = detector.measured_prism(surface, [.30, .20, .54], .01)

    np.testing.assert_allclose(foot.mean(axis=0), center, atol=1e-6)
    np.testing.assert_allclose(np.linalg.norm(np.diff(foot, axis=0), axis=1), [.32, .22, .32])
    assert top == pytest.approx(.55)
    assert pose['heading_deg'] == pytest.approx(33.)


def test_measured_box_rejects_an_incompatible_top():
    xy = np.column_stack([grid.ravel() for grid in np.meshgrid(
        np.linspace(0., .04, 8), np.linspace(0., .05, 8))])
    points = np.column_stack((xy, np.full(len(xy), .50)))

    with pytest.raises(SystemExit, match='incompatible with measured'):
        detector.measured_prism(points, [.30, .20, .50], .01)


def test_flat_box_registration_keeps_full_cloud_size_failure_visible():
    xy = np.column_stack([grid.ravel() for grid in np.meshgrid(
        np.linspace(-.11, .11, 17), np.linspace(-.065, .065, 13))])
    top = np.column_stack((xy, np.full(len(xy), .33)))
    base_returns = np.array([[-.17, -.12, .025], [.17, .12, .025]])
    clouds = {'overhead': top, 'blue_wrist': top + [.010, 0, .001],
              'silver_wrist': np.vstack((top + [0, .006, .003], base_returns))}

    result = detector.box_registration(clouds, np.array([.13, .22, .33]))

    assert result['qualified']
    assert not result['full_cloud_size_passed']
    assert result['views']['blue_wrist']['center_error_m'] == pytest.approx(.010, abs=.001)

    clouds['blue_wrist'] += [.025, 0, 0]
    shifted = detector.box_registration(clouds, np.array([.13, .22, .33]))
    assert shifted['views']['blue_wrist']['size_passed']
    assert not shifted['qualified']
