import numpy as np
import pytest

from fabric_twin.obstacle_shape import (capsule_args, capsule_mesh, capsule_volume, fit_capsule, prism_footprint,
                                        prism_mesh)
from fabric_twin.vendor import use_capsule_cbf

use_capsule_cbf()
from capsule_cbf.capsule import Capsule  # noqa: E402


def blob(seed=0, n=3000):
    """A rotated, irregular footprint: rectangle 0.6 x 0.3 with a bump, turned 25 deg."""
    rng = np.random.default_rng(seed)
    p = rng.uniform([-.3, -.15], [.3, .15], (n, 2))
    p = np.vstack((p, rng.uniform([0., .15], [.15, .25], (n//5, 2))))
    t = np.radians(25)
    return p @ np.array([[np.cos(t), np.sin(t)], [-np.sin(t), np.cos(t)]])+[.1, -.05]


@pytest.mark.parametrize('sides', [8, 18])
def test_prism_contains_every_point_and_has_the_sides(sides):
    pts = blob()
    F = prism_footprint(pts, sides=sides, pad=0.01)
    assert 4 <= len(F) <= sides
    edges = np.roll(F, -1, 0)-F
    cross = edges[:, 0]*np.roll(edges, -1, 0)[:, 1]-edges[:, 1]*np.roll(edges, -1, 0)[:, 0]
    assert np.all(cross > 0)                                            # convex, counter-clockwise
    outward = np.column_stack((edges[:, 1], -edges[:, 0]))/np.linalg.norm(edges, axis=1)[:, None]
    side = np.einsum('kj,nkj->nk', outward, pts[:, None]-F[None])          # signed distance to each side line
    assert side.max() <= -0.01+1e-9                                     # every point at least `pad` inside


def test_more_sides_fit_tighter():
    pts = blob()
    area = lambda F: .5*abs(np.dot(F[:, 0], np.roll(F[:, 1], -1))-np.dot(F[:, 1], np.roll(F[:, 0], -1)))  # noqa: E731
    assert area(prism_footprint(pts, 18, 0.)) < area(prism_footprint(pts, 8, 0.))


def test_prism_mesh_is_closed_and_outward():
    F = prism_footprint(blob(), 18, 0.01)
    v, t = prism_mesh(F, 0., .4)
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    volume = np.einsum('ij,ij->i', a, np.cross(b, c)).sum()/6           # divergence theorem: positive if outward
    area = .5*abs(np.dot(F[:, 0], np.roll(F[:, 1], -1))-np.dot(F[:, 1], np.roll(F[:, 0], -1)))
    assert volume == pytest.approx(area*.4, rel=1e-9)


def test_fitted_capsule_contains_the_prism_as_capsule_cbf_builds_it_and_is_smaller():
    pts = blob()
    F = prism_footprint(pts, 18, 0.01)
    top = .45
    fit = fit_capsule(F, top)
    center, half, scale = capsule_args(fit)
    cap = Capsule(center, half, 'centered_sphere', scale)
    corners = np.vstack((np.column_stack((F, np.zeros(len(F)))), np.column_stack((F, np.full(len(F), top)))))
    assert cap.distances(corners)[0].max() < 1e-4                        # every prism corner inside (to 0.1 mm)
    assert fit['volume_m3'] == pytest.approx(capsule_volume(fit['radius'], fit['cylinder_height'], fit['cap_sphere_radius']), rel=1e-3)
    # the earlier capsule: around the footprint's bounding box (radius = its half diagonal), cap scale 2
    lo, hi = F.min(0), F.max(0)
    old = Capsule([*(lo+hi)/2, top/2], [*(hi-lo)/2, top/2])
    g = np.stack(np.meshgrid(*[np.linspace(-.8, .8, 81)]*2, np.linspace(0, 1.2, 61), indexing='ij'), -1).reshape(-1, 3)
    cell = (1.6/80)**2*(1.2/60)
    old_volume = (old.distances(g)[0] < 0).sum()*cell
    assert fit['volume_m3'] < .8*old_volume


def test_capsule_mesh_stays_on_the_capsule():
    fit = fit_capsule(prism_footprint(blob(), 18, 0.01), .45)
    center, half, scale = capsule_args(fit)
    cap = Capsule(center, half, 'centered_sphere', scale)
    v, _ = capsule_mesh(fit)
    assert np.abs(cap.distances(v)[0]).max() < 1e-6
