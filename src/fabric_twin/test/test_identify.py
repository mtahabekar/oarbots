"""Identification from recordings (fabric_twin.identify): windows, grip offsets, ES-MDA, batched worlds."""
from dataclasses import replace

import numpy as np
import pytest

from fabric_twin.identify import PARAMETERS, Forward, es_mda, make_window, sensitivity, within_reach


def reference_within_reach(grips, width, height, movable=(2, 3), slack=.002):
    """TwinCloth.within_reach, one world at a time."""
    grips = np.array(grips, float)
    reach = {(0, 1): width, (2, 3): width, (0, 2): height, (1, 3): height,
             (0, 3): np.hypot(width, height), (1, 2): np.hypot(width, height)}
    for _ in range(4):
        for (i, j), rest in reach.items():
            delta = grips[j]-grips[i]
            length = np.linalg.norm(delta)
            excess = max(0., length-rest*(1+slack))
            moving = [c for c in (i, j) if c in movable]
            if excess and moving:
                for c in moving:
                    grips[c] += (1. if c == i else -1.)*excess/len(moving)*delta/length
    return grips


def square(z=1.):
    return np.array([[-.38, -.4, z], [.38, -.4, z], [-.33, .3, z], [.33, .3, z]])


def test_vectorized_within_reach_matches_the_twin():
    random = np.random.default_rng(1)
    grips = square()[None]+random.normal(scale=.15, size=(20, 4, 3))
    batched = within_reach(grips, .84, .84)
    for world in range(len(grips)):
        assert np.allclose(batched[world], reference_within_reach(grips[world], .84, .84), atol=1e-12)


def test_window_assigns_depth_to_the_preceding_grip_frame():
    corners_ns = np.arange(600)*int(1e9/60)
    corners = np.repeat(square()[None], 600, axis=0)
    depth_ns = corners_ns[::4]+int(5e6)
    points = [np.random.default_rng(k).normal(size=(100, 3)) for k in range(len(depth_ns))]
    window = make_window(corners_ns, corners, depth_ns, points, corners_ns[0], corners_ns[-1], depth_every=2,
                         points_per_frame=32, skip_sec=1.)
    assert np.all(np.diff(window.depth_index) > 0)
    assert all(len(p) == 32 for p in window.points)
    for k, index in zip(depth_ns[depth_ns >= corners_ns[0]+1e9][::2], window.depth_index):
        assert corners_ns[index] <= k < corners_ns[index]+int(1e9/60)


class LinearForward:
    """Stand-in forward model: residual = J (theta - truth), so the answer is known."""
    def __init__(self, names, truth, seed=0):
        self.names, self.truth, self.scale = names, np.asarray(truth, float), .1
        self.J = np.random.default_rng(seed).normal(scale=.02, size=(400, len(names)))
        self.runs, self.seconds = 0, 0.

    def raw(self, theta):
        self.runs += 1
        return (np.atleast_2d(theta)-self.truth) @ self.J.T

    def transform(self, raw):
        return self.scale*np.tanh(raw/self.scale)

    def __call__(self, theta):
        return self.transform(self.raw(theta))


def test_es_mda_recovers_a_linear_truth_outside_the_initial_ensemble_mean():
    names = ['stretch', 'hand_down']
    truth = [-.8, .045]                      # x0.16 stretch, 4.5 cm hand offset: far from the prior mean (0, 0)
    result = es_mda(LinearForward(names, truth), members=24, noise_m=.002, seed=3)
    assert np.allclose(result['mean'], truth, atol=[.08, .006])
    assert all(s < p for s, p in zip(result['sd'], result['prior_sd']))


def test_sensitivity_ranks_parameters_by_effect():
    names = ['stretch', 'bend']
    forward = LinearForward(names, [0., 0.])
    forward.J[:, 1] *= .05                   # bending barely moves the prediction
    result = sensitivity(forward)
    assert result['effect_mm_per_step'][0] > 5*result['effect_mm_per_step'][1]
    assert result['cramer_rao_sd'][1] > result['cramer_rao_sd'][0]


def test_parameter_table_is_complete():
    for name, p in PARAMETERS.items():
        assert p.name == name and p.lower <= 0 < p.upper and p.prior_sd > 0


def cuda():
    try:
        import warp as wp
        wp.init()
        return wp.get_cuda_device_count() > 0
    except Exception:
        return False


@pytest.mark.skipif(not cuda(), reason='needs a CUDA device')
def test_batched_worlds_are_independent_and_match_the_twin_without_self_contact():
    from fabric_twin.cloth import Obstacle, TwinCloth, TwinSpec
    from fabric_twin.identify import SheetBatch
    spec = TwinSpec(cells_x=14, cells_y=14, areal_density_kg_m2=.215, bend=(1.4e-5,)*3, self_collision=False,
                    obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1)))
    grips = square(1.05)
    twin = TwinCloth(spec, grips)
    same, mixed = SheetBatch(spec, 2, grips), SheetBatch(spec, 2, grips)
    same.set_materials(stretch=[1., 1.]); mixed.set_materials(stretch=[1., .1])
    for sheet in (twin, same, mixed):
        sheet.place(grips, .03)
    for k in range(90):
        g = grips+np.array([0., .02*np.sin(k/30), 0.])
        for sheet in (twin, same, mixed):
            sheet.step(g)
    q, m = same.positions(), mixed.positions()
    assert np.abs(q[0]-q[1]).max() < 1e-3                 # same parameters: float32 sums differ by < 1 mm
    assert np.abs(q[0]-twin.state()[0]).max() < 1e-3      # the batch reproduces the twin
    assert np.abs(m[1]-m[0]).max() > 1e-3                 # a softer world differs
    # Style3D's PCG is global: a softer neighbour also moves world 0. This is why Forward defaults to
    # independent single-world runs. If this starts failing, the solver became per-world and batching is exact.
    assert np.abs(m[0]-q[0]).max() > 1e-3


@pytest.mark.skipif(not cuda(), reason='needs a CUDA device')
def test_forward_offsets_lower_the_human_edge():
    from fabric_twin.cloth import Obstacle, TwinSpec
    spec = TwinSpec(cells_x=10, cells_y=10, areal_density_kg_m2=.215, bend=(1.4e-5,)*3, self_collision=False,
                    obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1)))
    corners_ns = np.arange(240)*int(1e9/60)
    corners = np.repeat(square(1.05)[None], 240, axis=0)
    flat = np.column_stack((np.random.default_rng(0).uniform(-.3, .3, (300, 2)), np.full(300, .95)))
    window = make_window(corners_ns, corners, corners_ns[::6], [flat]*len(corners_ns[::6]), 0, corners_ns[-1],
                         depth_every=1, points_per_frame=300, skip_sec=1.)
    forward = Forward(replace(spec), window, ['hand_down'], settle_frames=60)
    alone = Forward(replace(spec), window, ['hand_down'], settle_frames=60).raw(np.array([[.05]]))
    grips = forward.grips(np.array([[0.], [.05]]), 0)
    assert np.allclose(grips[1, 2:, 2], grips[0, 2:, 2]-.05) and np.allclose(grips[1, :2], grips[0, :2])
    r = forward.raw(np.array([[0.], [.05]]))
    assert np.nanmedian(r[1]) > np.nanmedian(r[0])        # a lower sheet leaves the points further above it
    assert np.allclose(r[1], alone[0], equal_nan=True, atol=1e-4)   # independent: other rows do not matter


def test_grip_uv_weights_put_each_grip_on_its_point():
    from fabric_twin.cloth import TwinSpec
    from fabric_twin.identify import grip_uv, grip_weights
    spec = TwinSpec()
    uv = grip_uv(spec, (.021, 0., .042, .063))
    assert np.allclose(uv[0], [.025, .025]) and np.allclose(uv[1], [1., 0.]) and np.allclose(uv[3], [.925, .925])
    assert np.allclose(grip_weights(uv, uv), np.eye(4), atol=1e-12)


def test_stretch_guard_limits_follow_inset_grips():
    from fabric_twin.cloth import TwinSpec, inset_grip_spans
    from fabric_twin.stretch import StretchGuard, StretchSpec
    spec = TwinSpec(grip_insets_m=(.02, .02, .05, .05))
    spans = inset_grip_spans(spec)
    assert np.isclose(spans[0, 1], .84-.04) and np.isclose(spans[0, 2], np.hypot(.03, .84-.07))
    guard, plain = StretchGuard(.84, .84, StretchSpec(), spans=spans), StretchGuard(.84, .84, StretchSpec())
    for (i, j, limit), (_, _, old) in zip(guard.pairs, plain.pairs):
        assert limit < old                        # less sheet between inset grips, so tighter limits


@pytest.mark.skipif(not cuda(), reason='needs a CUDA device')
def test_twin_holds_one_particle_per_corner_at_the_calibrated_inset():
    from fabric_twin.cloth import Obstacle, TwinCloth, TwinSpec
    spec = TwinSpec(cells_x=20, cells_y=20, areal_density_kg_m2=.215, bend=(1.4e-5,)*3, self_collision=False,
                    grip_insets_m=(0., .042, .042, .084), obstacle=Obstacle(center=(0., 0., -50.), half_extents=(.1, .1, .1)))
    grips = square(1.05)
    twin = TwinCloth(spec, grips)
    assert len(twin.patch) == 4 and np.allclose(twin.patch_offsets, 0.)
    assert np.allclose(twin.grip_uv, [[0., 0.], [.95, .05], [.05, .95], [.9, .9]], atol=1e-6)
    twin.place(grips, .03)
    assert np.allclose(twin.state()[0][twin.corner_particles], grips, atol=1e-5)
    for _ in range(30):
        twin.step(grips)
    assert np.allclose(twin.corners(), grips, atol=1e-4)
