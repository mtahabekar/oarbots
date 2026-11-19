"""A detected real obstacle: a rough prism and the CBF capsule fitted to it. NumPy/SciPy.

Prism: an n-sided footprint (default 18, so 20 faces with top and bottom) extruded from the floor to the object's
top: the points' minimum-area rectangle, then the convex-hull edges' support lines that cut away the most area,
one at a time, up to n sides; every side moved out by `pad`, so it contains every point. (The first version of
2026-11-19 had 8 sides: the minimum-area rectangle cut by the 45-degree-turned one; equally spaced directions
miss a box's own sides, which is why sides now come from the outline.)

Capsule: capsule_cbf's own shape (vertical circular cylinder closed above by a spherical cap through its rim,
`centered_sphere` mode), fitted to the prism: center (x, y), radius r, cylinder height h (from the floor) and cap
sphere radius R >= r chosen for the smallest volume that still contains every prism corner. A corner at horizontal
distance rho <= r from the axis is inside when its height is at most h + sqrt(R^2 - rho^2) - sqrt(R^2 - r^2).
`capsule_args` gives capsule_cbf.Capsule's arguments (center, half extents, top_radius_scale) for it; its lower
hemisphere then lies below the floor, where the sheet never is.
"""
import numpy as np


def min_area_yaw(xy):
    """Orientation (rad) of the minimum-area rectangle around 2-D points."""
    from scipy.spatial import ConvexHull
    pts = np.asarray(xy, float)
    try:
        hull = pts[ConvexHull(pts).vertices]
    except Exception:
        return 0.
    best, yaw = np.inf, 0.
    for a, b in zip(hull, np.roll(hull, -1, 0)):
        t = np.arctan2(b[1]-a[1], b[0]-a[0])
        R = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
        local = hull @ R
        area = np.prod(local.max(0)-local.min(0))
        if area < best:
            best, yaw = area, t
    return float((yaw+np.pi/4) % (np.pi/2)-np.pi/4)


def clip(polygon, normal, offset):
    """Part of a convex polygon (CCW) with normal . x <= offset."""
    side = polygon @ normal-offset
    out = []
    for i in range(len(polygon)):
        j = (i+1) % len(polygon)
        if side[i] <= 0:
            out.append(polygon[i])
        if side[i]*side[j] < 0:
            out.append(polygon[i]+(polygon[j]-polygon[i])*side[i]/(side[i]-side[j]))
    return np.array(out)


def area(polygon):
    x, y = polygon[:, 0], polygon[:, 1]
    return .5*abs(np.dot(x, np.roll(y, -1))-np.dot(y, np.roll(x, -1)))


def prism_footprint(xy, sides=18, pad=0.01):
    """Convex polygon (CCW, at most `sides` vertices) around 2-D points, every side `pad` out from them.

    Starts from the minimum-area rectangle, then adds, one at a time, the support line of a convex-hull edge
    that cuts away the most area while the polygon keeps at most `sides` sides. Each side therefore lies on the
    object's outline (moved out by pad), and more sides can only fit tighter.
    """
    from scipy.spatial import ConvexHull
    pts = np.asarray(xy, float)
    hull = pts[ConvexHull(pts).vertices]                       # counter-clockwise
    yaw = min_area_yaw(pts)
    angles = list(yaw+np.pi/2*np.arange(4))
    edges = np.roll(hull, -1, 0)-hull
    angles += list(np.arctan2(-edges[:, 0], edges[:, 1]))     # outward normals of the hull's edges
    normals = np.column_stack((np.cos(angles), np.sin(angles)))
    supports = (hull @ normals.T).max(axis=0)+pad
    c = hull.mean(0)
    big = 10*(np.abs(hull-c).max()+pad+1)
    polygon = c+np.array([[-big, -big], [big, -big], [big, big], [-big, big]])
    for k in range(4):                                         # the minimum-area rectangle
        polygon = clip(polygon, normals[k], supports[k])
    remaining = list(range(4, len(angles)))
    while remaining:
        best = None
        for k in remaining:
            cut = clip(polygon, normals[k], supports[k])
            if len(cut) <= sides and area(polygon)-area(cut) > 1e-9 and (best is None or area(cut) < best[1]):
                best = (k, area(cut), cut)
        if best is None:
            break
        polygon = best[2]
        remaining.remove(best[0])
    return polygon


def capsule_volume(r, h, R):
    t = R-np.sqrt(max(R*R-r*r, 0.))                     # cap height above the rim
    return float(np.pi*r*r*h+np.pi*t*t*(3*R-t)/3)


def cap_height_at(rho, r, h, R):
    """Height of the capsule's surface above the floor at horizontal distance rho <= r from its axis."""
    return h+np.sqrt(np.maximum(R*R-np.asarray(rho)**2, 0.))-np.sqrt(max(R*R-r*r, 0.))


def fit_capsule(footprint, top, min_cylinder=0.01):
    """Smallest capsule (cylinder from the floor + spherical cap) containing the prism footprint x [0, top]."""
    from scipy.optimize import minimize
    F = np.asarray(footprint, float)
    c0 = F.mean(0)
    r0 = float(np.linalg.norm(F-c0, axis=1).max())

    def constraints(x):
        cx, cy, r, h, R = x
        rho = np.hypot(F[:, 0]-cx, F[:, 1]-cy)
        return np.concatenate((r-rho, cap_height_at(np.minimum(rho, r), r, h, R)-top, [R-r]))

    best = None
    for frac in (.6, .85, .97):
        for scale in (1.2, 2., 5.):
            x0 = [c0[0], c0[1], r0*1.01, max(frac*top, min_cylinder), scale*r0*1.01]
            res = minimize(lambda x: capsule_volume(x[2], x[3], x[4]), x0, method='SLSQP',
                           bounds=[(None, None), (None, None), (.01, 3.), (min_cylinder, max(top, min_cylinder)), (.01, 100.)],
                           constraints=[dict(type='ineq', fun=constraints)], options=dict(maxiter=400, ftol=1e-10))
            if np.min(constraints(res.x)) < -1e-6:
                continue
            if best is None or capsule_volume(*res.x[2:]) < capsule_volume(*best[2:]):
                best = res.x
    if best is None:
        raise RuntimeError('capsule fit failed')
    cx, cy, r, h, R = (float(v) for v in best)
    f = lambda v, n=4: round(float(v), n)  # noqa: E731  plain floats (YAML cannot write numpy scalars)
    return dict(center=[f(cx), f(cy)], radius=f(r+5e-5), cylinder_height=f(h), cap_sphere_radius=f(R+5e-5),
                cap_height=f(R-np.sqrt(max(R*R-r*r, 0.))), volume_m3=f(capsule_volume(r, h, R), 5))


def capsule_args(capsule, floor=0.):
    """capsule_cbf.Capsule(center, half_extents, 'centered_sphere', top_radius_scale) for a fitted capsule."""
    r, h, R = capsule['radius'], capsule['cylinder_height'], capsule['cap_sphere_radius']
    half = np.array([r/np.sqrt(2), r/np.sqrt(2), h/2])
    center = np.array([capsule['center'][0], capsule['center'][1], floor+h/2])
    return center, half, max(R/np.linalg.norm(half), r/np.linalg.norm(half))


def prism_mesh(footprint, z0, z1):
    """Closed, outward triangle mesh of the prism: vertices (2n, 3), triangles."""
    F = np.asarray(footprint, float)
    n = len(F)
    vertices = np.vstack((np.column_stack((F, np.full(n, z0))), np.column_stack((F, np.full(n, z1)))))
    tris = []
    for i in range(n):
        j = (i+1) % n
        tris += [(i, j, n+j), (i, n+j, n+i)]
    for i in range(1, n-1):
        tris += [(0, i+1, i), (n, n+i, n+i+1)]
    return vertices, np.array(tris, int)


def capsule_mesh(capsule, floor=0., azimuths=36, rings=6):
    """Triangle mesh of a fitted capsule's surface above the floor (for display)."""
    r, h, R = capsule['radius'], capsule['cylinder_height'], capsule['cap_sphere_radius']
    cx, cy = capsule['center']
    ang = np.linspace(0, 2*np.pi, azimuths, endpoint=False)
    levels = [(r, floor), (r, floor+h)] + [(rho, floor+float(cap_height_at(rho, r, h, R))) for rho in np.linspace(r, 0, rings+1)[1:-1]]
    verts = [np.column_stack((cx+rho*np.cos(ang), cy+rho*np.sin(ang), np.full(azimuths, z))) for rho, z in levels]
    verts = np.vstack(verts+[[[cx, cy, floor+float(cap_height_at(0., r, h, R))]]])
    apex, tris = len(verts)-1, []
    for i in range(len(levels)-1):
        for j in range(azimuths):
            a0, a1 = i*azimuths+j, i*azimuths+(j+1) % azimuths
            tris += [(a0, a1, a1+azimuths), (a0, a1+azimuths, a0+azimuths)]
    last = (len(levels)-1)*azimuths
    tris += [(last+j, last+(j+1) % azimuths, apex) for j in range(azimuths)]
    return verts, np.array(tris, int)
