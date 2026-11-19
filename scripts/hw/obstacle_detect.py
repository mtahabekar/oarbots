#!/usr/bin/env python3
"""A real obstacle in the cell from depth: background subtraction on every camera, a rough shape, the twin's box.

`fabric_run.sh obstacle background` saves each camera's median depth with the work area empty;
`fabric_run.sh obstacle detect` saves it again with the object placed and runs this. Per camera, pixels that
come closer to the camera than the background by --min-step (or that return depth only now) are the object;
they are back-projected with the camera's intrinsics and distortion, placed in the world with the snapshot's
pose and calibration.yaml, fused, cropped to --roi, and the largest connected cluster is the obstacle.

Outputs in the object folder:
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

CELL = Path(__file__).resolve().parents[2] / 'experiments' / 'hardware' / 'cell'
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'fabric_twin'))
from fabric_twin.obstacle_shape import capsule_args, cap_height_at, fit_capsule, prism_footprint, prism_mesh  # noqa: E402
from fabric_twin.vendor import use_capsule_cbf  # noqa: E402
use_capsule_cbf()
from capsule_cbf.capsule import Capsule  # noqa: E402


def quat_matrix(x, y, z, w):
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def world_from(root):
    for t in yaml.safe_load((CELL / 'calibration.yaml').read_text())['transforms']:
        if t['parent'] == 'world' and t['child'] == root:
            T = np.eye(4)
            T[:3, :3] = quat_matrix(*t['rotation_xyzw'])
            T[:3, 3] = t['translation']
            return T
    raise SystemExit(f'calibration.yaml has no world -> {root}')


def back_project(snap, mask):
    """World points for the masked pixels of a snapshot."""
    v, u = np.nonzero(mask)
    z = snap['depth'][v, u].astype(float)
    if not len(z):
        return np.zeros((0, 3))
    pix = np.stack([u, v], 1).astype(np.float64).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(pix, snap['K'], snap['D']).reshape(-1, 2)
    cam = np.c_[xy * z[:, None], z, np.ones_like(z)]
    T = world_from(str(snap['root'])) @ snap['T_root_depth']
    return (cam @ T.T)[:, :3]


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('background')
    ap.add_argument('object')
    ap.add_argument('--roi', type=float, nargs=6, default=(-0.7, 0.7, -0.75, 0.7, 0.02, 1.3),
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'),
                    help='world box searched (default: the area under the sheet between robots and person)')
    ap.add_argument('--min-step', type=float, default=0.03, help='closer than the background by this (m)')
    ap.add_argument('--pad', type=float, default=0.01, help='added around the points (m)')
    ap.add_argument('--voxel', type=float, default=0.02)
    ap.add_argument('--sides', type=int, default=18, help='prism footprint sides (faces = sides + 2)')
    ap.add_argument('--max-pose-change', type=float, default=0.005, help='skip a wrist camera that moved (m)')
    a = ap.parse_args()
    bg_dir, ob_dir = Path(a.background), Path(a.object)
    lo, hi = np.array(a.roi[0::2]), np.array(a.roi[1::2])
    clouds, source = [], {}
    for f in sorted(ob_dir.glob('*.npz')):
        if f.name.startswith('obstacle'):
            continue
        cam = f.stem
        if not (bg_dir / f.name).exists():
            source[cam] = 'no background'
            continue
        ob, bg = dict(np.load(f)), dict(np.load(bg_dir / f.name))
        moved = np.linalg.norm(ob['T_root_depth'][:3, 3] - bg['T_root_depth'][:3, 3])
        if moved > a.max_pose_change:
            source[cam] = f'skipped: camera moved {moved*100:.1f} cm since the background'
            continue
        zo, zb = ob['depth'], bg['depth']
        fg = (zo > 0) & (((zb > 0) & (zb - zo > a.min_step)) | (zb == 0))
        fg = cv2.morphologyEx(fg.astype(np.uint8), cv2.MORPH_OPEN, np.ones((5, 5), np.uint8)).astype(bool)
        # Height correction per camera: the background's floor under the search area must read z = 0. The
        # overhead reads +0.6 cm; Silver's wrist -5.5 cm (wrist hand-eye not calibrated, 2026-11-19).
        floor = back_project(bg, zb > 0)
        near = np.all((floor[:, :2] >= lo[:2]) & (floor[:, :2] <= hi[:2]), axis=1) & (np.abs(floor[:, 2]) < .15)
        floor_z = float(np.median(floor[near, 2])) if near.sum() > 200 else 0.
        pts = back_project(ob, fg) - [0., 0., floor_z]
        inside = np.all((pts >= lo) & (pts <= hi), axis=1)
        source[cam] = dict(changed_pixels=int(fg.sum()), points_in_roi=int(inside.sum()),
                           floor_z_corrected_m=round(floor_z, 4), floor_points=int(near.sum()))
        clouds.append(pts[inside])
    pts = np.concatenate(clouds) if clouds else np.zeros((0, 3))
    print(json.dumps(source, indent=1))
    if len(pts) < 50:
        raise SystemExit(f'only {len(pts)} changed points in the search box: nothing placed, or it is outside --roi')
    # largest connected cluster on a voxel grid (26-neighbour), ignoring stray voxels
    idx = np.floor((pts - lo) / a.voxel).astype(int)
    grid = np.zeros(idx.max(0) + 1, np.int32)
    np.add.at(grid, tuple(idx.T), 1)
    labels, n = ndimage.label(grid >= 2, structure=np.ones((3, 3, 3)))
    if n == 0:
        raise SystemExit('no cluster')
    sizes = ndimage.sum(grid, labels, range(1, n + 1))
    keep = labels[tuple(idx.T)] == 1 + int(np.argmax(sizes))
    obj = pts[keep]
    print(f'{len(pts)} changed points, {n} clusters; the largest has {len(obj)} points ({100*len(obj)/len(pts):.0f} %)')
    # the rough shape: an n-sided prism (n + 2 faces) on the floor, up to the 99th-percentile height (+ pad)
    top = float(np.percentile(obj[:, 2], 99)) + a.pad
    foot = prism_footprint(obj[:, :2], sides=a.sides, pad=a.pad)
    capsule = fit_capsule(foot, top)
    center, half, scale = capsule_args(capsule)
    result = dict(
        obstacle=dict(shape='prism_capsule',
                      prism=dict(sides=int(len(foot)), top=round(top, 4),
                                 footprint=[[round(float(x), 4), round(float(y), 4)] for x, y in foot]),
                      capsule=capsule),
        source=dict(cameras=source, points=int(len(obj)), roi=[float(v) for v in a.roi], min_step_m=a.min_step, pad_m=a.pad,
                    background=str(bg_dir), object=str(ob_dir)))
    (ob_dir / 'obstacle.yaml').write_text(
        '# Detected by scripts/hw/obstacle_detect.py (fabric_twin/obstacle_shape.py). `prism`: the rough shape, an\n'
        '# n-sided footprint (n + 2 faces) from the floor to `top`, every side pad out from the points; the twin sheet\n'
        '# collides with it. `capsule`: capsule_cbf\'s cylinder + spherical cap, fitted to the prism (smallest volume\n'
        '# containing every prism corner); the CBF keeps the sheet clear of it.\n'
        + yaml.safe_dump(result, sort_keys=False, default_flow_style=None))
    vertices, triangles = prism_mesh(foot, 0., top)
    (ob_dir / 'obstacle.obj').write_text(''.join(f'v {x:.4f} {y:.4f} {z:.4f}\n' for x, y, z in vertices)
                                         + ''.join(f'f {i+1} {j+1} {k+1}\n' for i, j, k in triangles))
    (ob_dir / 'obstacle.urdf').write_text(f'''<?xml version="1.0"?>
<robot name="cell_obstacle">
  <link name="world"/>
  <link name="obstacle">
    <visual><geometry><mesh filename="file://{(ob_dir / 'obstacle.obj').resolve()}"/></geometry></visual>
    <collision><geometry><mesh filename="file://{(ob_dir / 'obstacle.obj').resolve()}"/></geometry></collision>
  </link>
  <joint name="world_to_obstacle" type="fixed"><parent link="world"/><child link="obstacle"/></joint>
</robot>
''')
    np.savez_compressed(ob_dir / 'obstacle_points.npz', all=pts, obstacle=obj)
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
    fig.tight_layout(); fig.savefig(ob_dir / 'obstacle.png', dpi=110)
    lo, hi = foot.min(0), foot.max(0)
    from_box = Capsule([*(lo + hi) / 2, top / 2], [*(hi - lo) / 2, top / 2])   # the earlier capsule: around the box
    g = np.stack(np.meshgrid(*[np.linspace(c - 1.5, c + 1.5, 121) for c in (*(lo + hi) / 2, top / 2)], indexing='ij'), -1).reshape(-1, 3)
    old = float((from_box.distances(g)[0] < 0).mean() * 3. ** 3)
    print(f'prism: {len(foot)} sides ({len(foot) + 2} faces), top {top:.3f} m; capsule: center {capsule["center"]}, radius {r:.3f} m, '
          f'cylinder {h:.3f} m, cap {capsule["cap_height"]:.3f} m (sphere {R:.3f} m), {capsule["volume_m3"]*1000:.0f} L '
          f'(the earlier box-based capsule: {old*1000:.0f} L)')
    print(f'written: {ob_dir}/obstacle.yaml, .urdf, .obj, .png')

if __name__ == '__main__':
    main()
