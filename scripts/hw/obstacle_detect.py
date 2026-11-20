#!/usr/bin/env python3
"""A real obstacle in the cell from depth: background subtraction on every camera, a rough shape, the twin's box.

`fabric_run.sh obstacle background` saves each camera's median depth with the work area empty.
`fabric_run.sh obstacle detect` reuses it with the object placed and runs this. Compare the floor to
check camera registration, then back-project stable foreground depth into the work area. Fit the
connected component supported by the fixed overhead camera, or by the other compatible views.

Outputs in the object folder or --output-dir:
  obstacle.yaml  `obstacle:` shape prism_capsule: `prism` the rough shape, an n-sided footprint (--sides 18: 20 faces)
                 from the floor to the 99th-percentile height + pad (minimum-area rectangle, then the convex-hull
                 edges that cut away the most area; every side --pad out from the points); `capsule` capsule_cbf's
                 cylinder + spherical cap fitted to the prism (center, radius, cylinder height, cap sphere radius:
                 smallest volume containing every prism corner). The twin's sheet collides with the prism; its CBF
                 keeps the sheet clear of the capsule. `source:` points per camera, floor corrections.
  obstacle.obj, obstacle.urdf   the prism as a mesh and a one-link URDF (world-fixed)
  obstacle_points.npz, obstacle.png   the fused points; a top view and a side view with prism and capsule
    python3 scripts/hw/obstacle_detect.py <background_dir> <object_dir> [--roi ...] [--pad 0.01]
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy import ndimage
from scipy.spatial import QhullError

CELL = Path(__file__).resolve().parents[2] / 'experiments' / 'hardware' / 'cell'
BACKGROUND_CAMERAS = {
    'overhead': 'nuc/camera_base',
    'blue_wrist': 'oarbot_blue/base_link',
    'silver_wrist': 'oarbot_silver/base_link',
}
FLOOR_INLIER_M = .012
MIN_FLOOR_POINTS = 200
MIN_CLUSTER_VOXELS = 8
SECONDARY_CLUSTER_FRACTION = .20
PIXEL_NEIGHBORS = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
VOXEL_NEIGHBORS = np.ones((3, 3, 3), dtype=np.uint8)
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'fabric_twin'))
from fabric_twin.obstacle_shape import capsule_volume, cap_height_at, fit_capsule, prism_footprint, prism_mesh  # noqa: E402
from fabric_twin.vendor import use_capsule_cbf  # noqa: E402
use_capsule_cbf()
from capsule_cbf.capsule import Capsule  # noqa: E402


def quat_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def read_camera_calibration(path):
    transforms = {}
    for entry in yaml.safe_load(path.read_text())['transforms']:
        if entry['parent'] == 'world' and entry['child'] in BACKGROUND_CAMERAS.values():
            transform = np.eye(4)
            transform[:3, :3] = quat_matrix(*entry['rotation_xyzw'])
            transform[:3, 3] = entry['translation']
            transforms[entry['child']] = transform
    missing = set(BACKGROUND_CAMERAS.values()) - transforms.keys()
    if missing:
        raise SystemExit(f'{path}: missing world transforms for {", ".join(sorted(missing))}')
    return transforms


def back_project(snap, mask, world_transforms):
    """World points for the masked pixels of a snapshot."""
    v, u = np.nonzero(mask)
    z = snap['depth'][v, u].astype(float)
    if not len(z):
        return np.zeros((0, 3))
    pix = np.stack([u, v], 1).astype(np.float64).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(pix, snap['K'], snap['D']).reshape(-1, 2)
    cam = np.c_[xy * z[:, None], z, np.ones_like(z)]
    T = world_transforms[str(snap['root'])] @ snap['T_root_depth']
    return (cam @ T.T)[:, :3]


def plane_height(plane, xy):
    return xy[..., 0]*plane[0] + xy[..., 1]*plane[1] + plane[2]


def fit_floor(snap, lo, hi, world_transforms):
    points = back_project(snap, snap['depth'] > 0, world_transforms)
    in_xy = np.all((points[:, :2] >= lo[:2]) & (points[:, :2] <= hi[:2]), axis=1)
    floor = points[in_xy & (np.abs(points[:, 2]) < .20)]
    if len(floor) < MIN_FLOOR_POINTS:
        return None, 0
    # Fit the dominant flat surface near the expected floor. A fixed subsample and seed make
    # the result reproducible while keeping the work bounded on full-resolution depth maps.
    sample = floor[::max(1, len(floor)//3000)]
    design = np.c_[sample[:, :2], np.ones(len(sample))]
    rng = np.random.default_rng(17)
    triplets = rng.integers(len(sample), size=(64, 3))
    triplets = triplets[np.abs(np.linalg.det(design[triplets])) > .01]
    if not len(triplets):
        return None, 0
    planes = np.linalg.solve(design[triplets], sample[triplets, 2, None]).squeeze(-1)
    residuals = sample[:, 2, None] - plane_height(planes.T, sample[:, None, :2])
    best = planes[np.argmax(np.count_nonzero(np.abs(residuals) < FLOOR_INLIER_M, axis=0))]
    inliers = floor[np.abs(floor[:, 2] - plane_height(best, floor[:, :2])) < FLOOR_INLIER_M]
    if len(inliers) < max(MIN_FLOOR_POINTS, len(floor)*.5):
        return None, len(inliers)
    design = np.c_[inliers[:, :2], np.ones(len(inliers))]
    return np.linalg.lstsq(design, inliers[:, 2], rcond=None)[0], len(inliers)


def floor_difference(first, second, xy):
    normals = np.c_[-np.array([first[:2], second[:2]]), np.ones(2)]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    tilt = np.degrees(np.arctan2(np.linalg.norm(np.cross(*normals)), normals[0] @ normals[1]))
    height = abs(plane_height(first - second, xy))
    return float(tilt), float(height)


def foreground_mask(bg, ob, min_step):
    before, observed = bg['depth'], ob['depth']
    # Captures carry valid-frame fractions. A missing background pixel needs a nearby,
    # confirmed foreground return of similar depth before it contributes to geometry.
    stable = (observed > 0) & (ob.get('valid', 1.) >= .6)
    confirmed = stable & (before > 0) & (bg.get('valid', 1.) >= .6) & (before - observed > min_step)
    confirmed = cv2.morphologyEx(confirmed.astype(np.uint8), cv2.MORPH_OPEN, PIXEL_NEIGHBORS).astype(bool)
    nearby_depth = cv2.dilate(np.where(confirmed, observed, 0.), PIXEL_NEIGHBORS)
    return confirmed | (stable & (before == 0) & (nearby_depth > 0)
                        & (np.abs(observed - nearby_depth) <= min_step))


def obstacle_cluster(points, overhead, lo, voxel):
    indices = np.floor((points - lo) / voxel).astype(np.intp)
    shape = tuple(indices.max(0) + 1)
    flat = np.ravel_multi_index(indices.T, shape)
    occupied = np.bincount(flat, minlength=int(np.prod(shape))).reshape(shape) >= 2
    # A surface voxel needs three occupied neighbors. This removes single-voxel
    # trails that can connect stray returns to the object and stretch its footprint.
    supported = occupied & (ndimage.convolve(occupied.astype(np.uint8), VOXEL_NEIGHBORS, mode='constant') >= 4)
    labels, count = ndimage.label(supported, structure=VOXEL_NEIGHBORS)
    sizes = np.bincount(labels.ravel(), minlength=count + 1)
    selected = np.flatnonzero(sizes[1:] >= MIN_CLUSTER_VOXELS) + 1
    if not len(selected):
        raise SystemExit('No obstacle with enough connected depth support')
    member = labels.ravel()[flat]
    # Score by occupied voxels so a dense wrist view cannot outvote the fixed overhead view.
    fixed_voxels = np.unique(flat[overhead])
    fixed_support = np.bincount(labels.ravel()[fixed_voxels], minlength=count + 1)
    anchored = selected[fixed_support[selected] >= MIN_CLUSTER_VOXELS]
    if len(anchored):
        ranked = anchored[np.argsort(fixed_support[anchored])[::-1]]
        scores = fixed_support
    else:
        ranked = selected[np.argsort(sizes[selected])[::-1]]
        scores = sizes
    if len(ranked) > 1 and scores[ranked[1]] >= SECONDARY_CLUSTER_FRACTION*scores[ranked[0]]:
        raise SystemExit('Multiple substantial objects in the scan. Clear the sheet or narrow --roi and retry.')
    return member == ranked[0], len(selected)


def check_background(folder, lo, hi, world_transforms):
    """Validate the three two-second captures before replacing the reusable reference."""
    for camera, root in BACKGROUND_CAMERAS.items():
        file = folder / f'{camera}.npz'
        if not file.is_file():
            raise SystemExit(f'{camera}: missing background capture')
        with np.load(file) as snap:
            depth, frames = snap['depth'], int(snap['frames'])
            if (frames < 24 or str(snap['root']) != root or depth.shape != tuple(snap['size'][::-1])
                    or not np.isfinite(depth).all()
                    or not np.isfinite(snap['T_root_depth']).all()):
                raise SystemExit(f'{camera}: incomplete or invalid background capture ({frames} frames)')
            plane, support = fit_floor(snap, lo, hi, world_transforms)
            if plane is None:
                raise SystemExit(f'{camera}: cannot fit the workspace floor ({support} plane inliers)')
            tilt, height = floor_difference(plane, np.zeros(3), (lo + hi)[:2]/2)
            print(f'{camera}: {frames} frames, {support} floor points, floor tilt {tilt:.2f} deg, '
                  f'height offset {height:.3f} m')


def camera_mismatch(background, current, max_translation, max_rotation_deg):
    """Depth subtraction compares corresponding rays, so both pose and intrinsics must match."""
    if (background['depth'].shape != current['depth'].shape
            or any(str(background[key]) != str(current[key]) for key in ('root', 'frame_id', 'model'))
            or any(background[key].shape != current[key].shape or not np.allclose(background[key], current[key])
                   for key in ('K', 'D'))):
        return 'camera frames or intrinsics changed'
    previous = background['T_root_depth']
    now = current['T_root_depth']
    moved = np.linalg.norm(now[:3, 3] - previous[:3, 3])
    relative = previous[:3, :3].T @ now[:3, :3]
    rotation_deg = np.degrees(np.arccos(np.clip((np.trace(relative) - 1) / 2, -1, 1)))
    if moved > max_translation or rotation_deg > max_rotation_deg:
        return f'camera moved {moved*100:.1f} cm or rotated {rotation_deg:.2f} deg since the background'
    return None


def measured_prism(points, size, pad):
    """Fit the world pose of an upright box from its visible top, using tape dimensions."""
    height = float(np.percentile(points[:, 2], 99))
    if abs(height - size[2]) > max(.08, .15 * size[2]):
        raise SystemExit(f'Overhead height {height:.3f} m disagrees with measured height {size[2]:.3f} m')
    top_points = points[np.abs(points[:, 2] - height) <= .015]
    if len(top_points) < 30:
        raise SystemExit('Measured box needs a clear overhead view of its top face')
    center, observed_sides, angle = cv2.minAreaRect(top_points[:, :2].astype(np.float32))
    measured_sides = np.array(sorted(size[:2], reverse=True))
    observed = np.array(sorted(observed_sides, reverse=True))
    if np.any(observed < .5 * measured_sides) or np.any(observed > measured_sides + np.maximum(.04, .3 * measured_sides)):
        raise SystemExit(f'Overhead top spans {observed.round(3).tolist()} m, incompatible with measured '
                         f'box sides {measured_sides.tolist()} m. Clear the full top face and retry.')
    heading = angle + (90 if observed_sides[1] > observed_sides[0] else 0)
    theta = np.radians(heading)
    u = np.array([np.cos(theta), np.sin(theta)])
    v = np.array([-u[1], u[0]])
    half = measured_sides / 2 + pad
    long_side, short_side = half[0] * u, half[1] * v
    footprint = center + np.array([-long_side-short_side, long_side-short_side,
                                   long_side+short_side, -long_side+short_side])
    pose = dict(camera='overhead', center_xy_m=list(center), heading_deg=heading % 180,
                observed_top_m=round(height, 4), top_points=len(top_points))
    return footprint, size[2] + pad, pose


def scan_dimensions(points):
    _, sides, _ = cv2.minAreaRect(points[:, :2].astype(np.float32))
    return np.r_[sorted(sides), np.percentile(points[:, 2], 99)]


def box_registration(clouds, expected):
    """Compare the same flat top across cameras, with the box dimensions held out."""
    overhead = clouds['overhead']
    level = np.percentile(overhead[:, 2], 90)
    tops = {cam: points[np.abs(points[:, 2] - level) <= .015] for cam, points in clouds.items()}
    missing = [cam for cam, top in tops.items() if len(top) < 50]
    if missing:
        raise ValueError(f'Not enough points on the flat box top from {", ".join(missing)}')
    _, _, degrees = cv2.minAreaRect(tops['overhead'][:, :2].astype(np.float32))
    c, s = np.cos(np.radians(degrees)), np.sin(np.radians(degrees))
    axes = np.array([[c, -s], [s, c]])
    tolerance = np.maximum(.03, .10 * expected)
    views = {}
    reference = None
    for cam in BACKGROUND_CAMERAS:
        top = tops[cam]
        lo, hi = np.percentile(top[:, :2] @ axes, [1, 99], axis=0)
        height = float(np.median(top[:, 2]))
        center = np.r_[((lo + hi) / 2) @ axes.T, height]
        if cam == 'overhead':
            reference = center
        error = float(np.linalg.norm(center - reference))
        dims = np.r_[sorted(hi - lo), height]
        fits = bool(np.all(np.abs(dims - expected) <= tolerance))
        views[cam] = dict(points=len(top), size_m=dims.round(4).tolist(), center_m=center.round(4).tolist(),
                          center_error_m=round(error, 4), size_passed=fits, passed=fits and error <= .02)
    full = scan_dimensions(np.concatenate(list(clouds.values())))
    return dict(qualified=all(row['passed'] for row in views.values()), views=views,
                measured_size_m=expected.tolist(), full_cloud_size_m=full.round(4).tolist(),
                full_cloud_size_passed=bool(np.all(np.abs(full - expected) <= tolerance)))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('background')
    ap.add_argument('object', nargs='?')
    ap.add_argument('--check-background', action='store_true', help='validate the reusable empty capture')
    ap.add_argument('--output-dir', type=Path, help='write the model here (default: the object capture directory)')
    ap.add_argument('--wrist-calibration', type=Path, metavar='SESSION',
                    help='replay both qualified wrist calibrations on raw captures taken with nominal mounts in TF')
    ap.add_argument('--check-box-registration', action='store_true',
                    help='validate the same flat box top across three views and save a report without fitting a model')
    ap.add_argument('--roi', type=float, nargs=6, default=(-0.7, 0.7, -0.75, 0.7, 0.02, 1.3),
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'),
                    help='world box searched (default: the area under the sheet between robots and person)')
    ap.add_argument('--min-step', type=float, default=0.03, help='closer than the background by this (m)')
    ap.add_argument('--pad', type=float, default=0.01, help='added around the points (m)')
    ap.add_argument('--voxel', type=float, default=0.02)
    ap.add_argument('--sides', type=int, default=18, help='prism footprint sides (faces = sides + 2)')
    ap.add_argument('--max-pose-change', type=float, default=0.005, help='skip a wrist camera that moved (m)')
    ap.add_argument('--max-rotation-deg', type=float, default=0.5, help='skip a camera that rotated (degrees)')
    ap.add_argument('--max-floor-tilt-deg', type=float, default=2.,
                    help='maximum floor-normal disagreement between camera views')
    ap.add_argument('--max-floor-offset-m', type=float, default=.03,
                    help='maximum floor-height disagreement between camera views at the ROI center')
    sizing = ap.add_mutually_exclusive_group()
    sizing.add_argument('--expected-size-m', type=float, nargs=3, metavar=('WIDTH', 'DEPTH', 'HEIGHT'),
                        help='validate unpadded dimensions using the greater of 3 cm and 10%% per axis')
    sizing.add_argument('--measured-box-m', type=float, nargs=3, metavar=('WIDTH', 'DEPTH', 'HEIGHT'),
                        help='use tape dimensions for an upright rectangular box and overhead depth for its pose')
    a = ap.parse_args()
    bg_dir = Path(a.background)
    lo, hi = np.array(a.roi[0::2]), np.array(a.roi[1::2])
    if not np.isfinite([*lo, *hi, a.voxel]).all() or np.any(hi <= lo) or a.voxel <= 0:
        ap.error('--roi needs finite, increasing bounds and --voxel must be positive')
    limits = (a.min_step, a.pad, a.max_pose_change, a.max_rotation_deg, a.max_floor_tilt_deg, a.max_floor_offset_m)
    if not np.isfinite(limits).all() or a.min_step <= 0 or min(limits) < 0 or a.sides < 4:
        ap.error('use a positive --min-step, at least four --sides, and finite, non-negative padding and tolerances')
    size = a.expected_size_m or a.measured_box_m
    if size and (not np.isfinite(size).all() or min(size) <= 0):
        ap.error('box dimensions must be positive and finite')
    if a.wrist_calibration and a.measured_box_m:
        ap.error('use --expected-size-m to validate a wrist calibration with a measured box')
    if a.check_box_registration and not (a.wrist_calibration and a.expected_size_m):
        ap.error('--check-box-registration requires --wrist-calibration and --expected-size-m')
    if not bg_dir.is_dir():
        ap.error(f'background capture directory does not exist: {bg_dir}')
    calibration = bg_dir / 'calibration.yaml'
    if not calibration.is_file():
        calibration = CELL / 'calibration.yaml'
        print('Background predates saved calibration. Using the current cell calibration.')
    world_transforms = read_camera_calibration(calibration)
    if a.check_background:
        if a.object is not None or a.wrist_calibration:
            ap.error('--check-background takes only an unmodified background capture directory')
        check_background(bg_dir, lo, hi, world_transforms)
        return
    if a.object is None:
        ap.error('an object capture directory is required for detection')
    ob_dir = Path(a.object)
    if not ob_dir.is_dir():
        ap.error(f'object capture directory does not exist: {ob_dir}')
    out = a.output_dir or ob_dir
    saved_calibration = ob_dir / 'calibration.yaml'
    if saved_calibration.is_file():
        object_transforms = read_camera_calibration(saved_calibration)
        if any(not np.array_equal(world_transforms[root], object_transforms[root]) for root in world_transforms):
            raise SystemExit('Camera world calibration changed between captures. Record a matching empty reference.')
    corrections = {}
    if a.wrist_calibration:
        if __package__:
            from .wrist_calibrate import saved_depth_correction
        else:
            from wrist_calibrate import saved_depth_correction
        try:
            surveyed = read_camera_calibration(a.wrist_calibration / 'calibration.yaml')
            if any(not np.array_equal(world_transforms[root], surveyed[root]) for root in world_transforms):
                raise ValueError('Wrist calibration and obstacle captures use different world surveys')
            corrections = {f'{robot}_wrist': saved_depth_correction(a.wrist_calibration, robot)
                           for robot in ('blue', 'silver')}
        except (OSError, KeyError, ValueError) as error:
            raise SystemExit(f'Could not load wrist calibration: {error}') from error
        print(f'Testing wrist calibration from {a.wrist_calibration} on the saved depth poses')
    clouds, source = {}, {}
    reference_floor = np.zeros(3)
    center_xy = (lo + hi)[:2]/2
    # Process the fixed camera first so its measured floor is the reference for the wrists.
    for cam, root in BACKGROUND_CAMERAS.items():
        if a.measured_box_m and cam != 'overhead':
            source[cam] = 'not used for measured box pose'
            continue
        f = ob_dir / f'{cam}.npz'
        if not f.is_file():
            source[cam] = 'no object capture'
            continue
        if not (bg_dir / f.name).exists():
            source[cam] = 'no background'
            continue
        with np.load(f) as current, np.load(bg_dir / f.name) as background:
            ob, bg = dict(current), dict(background)
        if str(ob['root']) != root or int(ob['frames']) < 24:
            source[cam] = 'skipped: object capture has the wrong frame or fewer than 24 frames'
            continue
        mismatch = camera_mismatch(bg, ob, a.max_pose_change, a.max_rotation_deg)
        if mismatch:
            source[cam] = f'skipped: {mismatch}'
            continue
        if cam in corrections:
            frame = f'oarbot_{cam.split("_")[0]}/depth_camera_link'
            if str(ob['frame_id']) != frame:
                source[cam] = 'skipped: optical frame differs from the wrist calibration'
                continue
            for snap in (bg, ob):
                snap['T_root_depth'] = snap['T_root_depth'] @ corrections[cam]
        floor, support = fit_floor(bg, lo, hi, world_transforms)
        if floor is None:
            source[cam] = 'skipped: no reliable floor in the background capture'
            continue
        tilt, height = floor_difference(floor, reference_floor, center_xy)
        if tilt > a.max_floor_tilt_deg or height > a.max_floor_offset_m:
            source[cam] = f'skipped: floor disagrees with reference by {tilt:.2f} deg, {height*100:.1f} cm'
            continue
        current_floor, _ = fit_floor(ob, lo, hi, world_transforms)
        if current_floor is None:
            source[cam] = 'skipped: no reliable exposed floor in the object capture'
            continue
        tilt_change, height_change = floor_difference(current_floor, floor, center_xy)
        if tilt_change > a.max_rotation_deg or height_change > .015:
            source[cam] = (f'skipped: floor changed since background by {tilt_change:.2f} deg, '
                           f'{height_change*100:.1f} cm; verify the mount and refresh the empty reference')
            continue
        if cam == 'overhead':
            reference_floor = floor

        fg = foreground_mask(bg, ob, a.min_step)
        pts = back_project(ob, fg, world_transforms)
        floor_z = plane_height(floor, pts[:, :2])
        pts[:, 2] -= floor_z
        inside = np.all((pts >= lo) & (pts <= hi), axis=1)
        source[cam] = dict(changed_pixels=int(fg.sum()), points_in_roi=int(inside.sum()),
                           floor_z_corrected_m=round(float(plane_height(floor, center_xy)), 4),
                           floor_points=support, floor_tilt_error_deg=round(tilt, 3))
        clouds[cam] = pts[inside]
    pts = np.concatenate(list(clouds.values())) if clouds else np.zeros((0, 3))
    print(json.dumps(source, indent=1))
    if a.measured_box_m and 'overhead' not in clouds:
        raise SystemExit('Measured box needs a usable overhead capture')
    if len(pts) < 50:
        raise SystemExit(f'Only {len(pts)} supported foreground points in the search box')
    fixed = np.repeat([cam == 'overhead' for cam in clouds], [len(p) for p in clouds.values()])
    keep, count = obstacle_cluster(pts, fixed, lo, a.voxel)
    obj = pts[keep]
    start = 0
    for cam, camera_points in clouds.items():
        end = start + len(camera_points)
        members = keep[start:end]
        source[cam]['points_in_obstacle'] = int(members.sum())
        if a.check_box_registration:
            clouds[cam] = camera_points[members]
        start = end
    print(f'{len(pts)} foreground points, {count} supported clusters; '
          f'the selected obstacle has {len(obj)} points ({100*len(obj)/len(pts):.0f} %)')
    for cam in clouds:
        print(f'{cam}: {source[cam]["points_in_obstacle"]} points in the selected obstacle')
    if corrections:
        missing = [cam for cam in BACKGROUND_CAMERAS
                   if not isinstance(source[cam], dict) or source[cam]['points_in_obstacle'] < 50]
        if missing:
            raise SystemExit(f'Wrist calibration check needs the object in all three views. '
                             f'Too few selected points from {", ".join(missing)}.')
    lower, upper = np.percentile(obj, [1, 99], axis=0)
    edge_margin = max(a.pad, a.voxel)
    if np.any(lower[:2] <= lo[:2] + edge_margin) or np.any(upper >= hi - edge_margin):
        raise SystemExit('Foreground reaches the search boundary. Clear the view or expand --roi and retry.')
    if a.expected_size_m:
        expected = np.r_[sorted(a.expected_size_m[:2]), a.expected_size_m[2]]
        if a.check_box_registration:
            try:
                report = box_registration(clouds, expected)
            except ValueError as error:
                raise SystemExit(str(error)) from error
            report.update(background=str(bg_dir), object=str(ob_dir), wrist_calibration=str(a.wrist_calibration))
            out.mkdir(parents=True, exist_ok=True)
            details = json.dumps(report, indent=2)
            (out / 'box_registration.json').write_text(details + '\n')
            print(details)
            if not report['qualified']:
                raise SystemExit('Flat box top check failed. See box_registration.json.')
            return
        measured = scan_dimensions(obj)
        tolerance = np.maximum(.03, .10*expected)
        if np.any(np.abs(measured - expected) > tolerance):
            raise SystemExit(f'Scan size {measured.round(3).tolist()} m disagrees with supplied '
                             f'{expected.round(3).tolist()} m (short side, long side, height)')
        print(f'Scan size {measured.round(3).tolist()} m agrees with supplied '
              f'{expected.round(3).tolist()} m (short side, long side, height)')
    if a.measured_box_m:
        foot, top, measured_pose = measured_prism(obj, a.measured_box_m, a.pad)
        if np.any(foot.min(0) <= lo[:2]) or np.any(foot.max(0) >= hi[:2]) or top >= hi[2]:
            raise SystemExit('Measured box extends outside the search boundary. Check the dimensions or expand --roi.')
    else:
        measured_pose = None
        top = float(upper[2]) + a.pad
        try:
            foot = prism_footprint(obj[:, :2], sides=a.sides, pad=a.pad)
        except QhullError as error:
            raise SystemExit('Obstacle points do not describe a usable footprint') from error
    capsule = fit_capsule(foot, top)
    result = dict(
        obstacle=dict(shape='prism_capsule',
                      prism=dict(sides=int(len(foot)), top=round(top, 4),
                                 footprint=[[round(float(x), 4), round(float(y), 4)] for x, y in foot]),
                      capsule=capsule),
        source=dict(cameras=source, points=int(len(obj)), roi=[float(v) for v in a.roi], min_step_m=a.min_step,
                    pad_m=a.pad, voxel_m=a.voxel, max_floor_tilt_deg=a.max_floor_tilt_deg,
                    max_floor_offset_m=a.max_floor_offset_m, expected_size_m=a.expected_size_m,
                    measured_box_m=a.measured_box_m, measured_box_pose=measured_pose, calibration=str(calibration),
                    wrist_calibration=str(a.wrist_calibration) if a.wrist_calibration else None,
                    background=str(bg_dir), object=str(ob_dir)))
    out.mkdir(parents=True, exist_ok=True)
    vertices, triangles = prism_mesh(foot, 0., top)
    (out / 'obstacle.obj').write_text(''.join(f'v {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in vertices)
                                     + ''.join(f'f {i+1} {j+1} {k+1}\n' for i, j, k in triangles))
    (out / 'obstacle.urdf').write_text(f'''<?xml version="1.0"?>
<robot name="cell_obstacle">
  <link name="world"/>
  <link name="obstacle">
    <visual><geometry><mesh filename="file://{(out / 'obstacle.obj').resolve()}"/></geometry></visual>
    <collision><geometry><mesh filename="file://{(out / 'obstacle.obj').resolve()}"/></geometry></collision>
  </link>
  <joint name="world_to_obstacle" type="fixed"><parent link="world"/><child link="obstacle"/></joint>
</robot>
''')
    np.savez_compressed(out / 'obstacle_points.npz', all=pts, obstacle=obj)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    cx, cy = capsule['center']
    r, h, R = capsule['radius'], capsule['cylinder_height'], capsule['cap_sphere_radius']
    ax[0].scatter(pts[~keep, 0], pts[~keep, 1], s=1, c='0.7', label='other changes')
    ax[0].scatter(obj[:, 0], obj[:, 1], s=1, c=obj[:, 2], cmap='viridis', label='obstacle (colour: height)')
    ax[0].plot(*np.r_[foot, foot[:1]].T, 'b-', label=f'prism footprint ({len(foot)} sides)')
    ax[0].add_patch(plt.Circle((cx, cy), r, fill=False, color='r', label='capsule (CBF)'))
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x (m)'); ax[0].set_ylabel('y (m, +y toward the person)'); ax[0].legend(fontsize=7)
    ax[0].set_title('top view (world)')
    rho = np.hypot(obj[:, 0] - cx, obj[:, 1] - cy)
    ax[1].scatter(rho, obj[:, 2], s=1, c='k', label='points')
    corner = np.hypot(foot[:, 0] - cx, foot[:, 1] - cy)
    ax[1].plot(corner, np.full(len(corner), top), 'bs', ms=4, label='prism top corners')
    u = np.linspace(0, r, 100)
    ax[1].plot(np.r_[u, r, r], np.r_[cap_height_at(u, r, h, R), h, 0], 'r-', label='capsule')
    ax[1].set_aspect('equal'); ax[1].set_xlabel('distance from the capsule axis (m)'); ax[1].set_ylabel('z (m)')
    ax[1].legend(fontsize=7); ax[1].set_title('side view (all azimuths)')
    fig.tight_layout()
    fig.savefig(out / 'obstacle.png', dpi=110)
    lo, hi = foot.min(0), foot.max(0)
    from_box = Capsule([*(lo + hi) / 2, top / 2], [*(hi - lo) / 2, top / 2])   # the earlier capsule: around the box
    old = capsule_volume(from_box.radius, top, from_box.top_sphere_radius) + 2*np.pi*from_box.radius**3/3
    print(f'prism: {len(foot)} sides ({len(foot) + 2} faces), top {top:.3f} m; capsule: center {capsule["center"]}, radius {r:.3f} m, '
          f'cylinder {h:.3f} m, cap {capsule["cap_height"]:.3f} m (sphere {R:.3f} m), {capsule["volume_m3"]*1000:.0f} L '
          f'(the earlier box-based capsule: {old*1000:.0f} L)')
    (out / 'obstacle.yaml').write_text(
        '# Generated by scripts/hw/obstacle_detect.py. The twin sheet collides with the prism.\n'
        '# The CBF keeps the sheet clear of the capsule fitted around every prism corner.\n'
        + yaml.safe_dump(result, sort_keys=False, default_flow_style=None))
    print(f'written: {out}/obstacle.yaml, .urdf, .obj, .png, obstacle_points.npz')

if __name__ == '__main__':
    main()
