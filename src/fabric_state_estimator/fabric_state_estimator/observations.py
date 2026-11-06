"""Visibility-aware raw residuals shared by online assimilation and Tier A."""
from dataclasses import dataclass
import cv2
import numpy as np


def camera_points(q, view):
    transform = view.world_from_camera
    return (np.asarray(q)-transform[:3, 3]) @ transform[:3, :3]


def project(q, view):
    points = camera_points(q, view)
    pixels = cv2.projectPoints(points.reshape(-1, 3), np.zeros(3), np.zeros(3), view.camera.k,
                               view.camera.distortion)[0].reshape(points.shape[:-1]+(2,))
    return pixels, points[..., 2]


def ray_stencils(mesh, q, view, pixels):
    """Frontmost positive triangle only: hidden layers are never nearest neighbours."""
    pixels = np.asarray(pixels, float).reshape(-1, 2)
    if len(pixels) == 0:
        return np.empty(0, int), np.empty((0, 3))
    rays2 = cv2.undistortPoints(pixels[:, None], view.camera.k, view.camera.distortion)[:, 0]
    rays = np.column_stack((rays2, np.ones(len(pixels))))
    triangles = camera_points(q, view)[mesh.triangles]
    a = triangles[:, 0]; e1, e2 = triangles[:, 1]-a, triangles[:, 2]-a
    h = np.cross(rays[:, None], e2)
    determinant = np.einsum('fx,mfx->mf', e1, h)
    inverse = np.divide(1., determinant, out=np.zeros_like(determinant), where=np.abs(determinant) > 1e-12)
    u = np.einsum('fx,mfx->mf', -a, h)*inverse
    cross = np.cross(-a, e1)
    v = np.einsum('mx,fx->mf', rays, cross)*inverse
    t = np.einsum('fx,fx->f', e2, cross)[None]*inverse
    valid = (np.abs(determinant) > 1e-12) & (u >= -1e-7) & (v >= -1e-7) & (u+v <= 1+1e-7) & (t > 0)
    distance = np.where(valid, t, np.inf)
    faces = np.argmin(distance, axis=1)
    index = np.arange(len(pixels))
    weights = np.column_stack((1-u[index, faces]-v[index, faces], u[index, faces], v[index, faces]))
    faces[~np.isfinite(distance[index, faces])] = -1
    return faces, weights


@dataclass
class Factor:
    kind: str
    vertices: np.ndarray
    weights: np.ndarray
    value: float
    variance: float
    axis: np.ndarray
    view: object = None
    corner: int = -1

    def predict(self, q, v, loads, physics):
        if self.kind == 'force':
            return physics.reactions(q, loads)[:, self.corner] @ self.axis
        points = np.einsum('nix,i->nx', q[:, self.vertices], self.weights)
        if self.kind in ('normal', 'grip'):
            return points @ self.axis
        return project(points, self.view)[0] @ self.axis


class ObservationBuilder:
    def __init__(self, mesh, max_depth=64, max_boundary=48, max_tracks=48,
                 association_gate_m=.15, pixel_stddev=3., depth_floor_m=.005,
                 use_uv=False, max_track_bindings=2000, max_surface_speed_mps=1., visibility_tolerance_m=.02):
        if (min(max_depth, max_boundary, max_tracks) < 0 or min(association_gate_m, pixel_stddev, depth_floor_m) <= 0
                or max_track_bindings < 1 or min(max_surface_speed_mps, visibility_tolerance_m) <= 0
                or not np.isfinite([association_gate_m, pixel_stddev, depth_floor_m, max_surface_speed_mps, visibility_tolerance_m]).all()):
            raise ValueError('Invalid observation settings')
        self.mesh = mesh
        self.limits = max_depth, max_boundary, max_tracks
        self.gate, self.pixel_noise, self.depth_floor = association_gate_m, pixel_stddev, depth_floor_m
        self.use_uv, self.bindings, self.binding_limit = use_uv, {}, max_track_bindings
        self.max_speed = max_surface_speed_mps
        self.visibility_tolerance = visibility_tolerance_m

    @staticmethod
    def sample(values, limit):
        if limit == 0:
            return values[:0]
        return values if len(values) <= limit else [values[i] for i in np.linspace(0, len(values)-1, limit, dtype=int)]

    def build(self, q, frame):
        factors = []
        for view in frame.views:
            skew = abs(frame.time_ns-view.time_ns)/1e9 if view.time_ns else 0.
            temporal_variance = (self.max_speed*skew)**2
            typical_depth = np.median(view.depth[:, 4]) if len(view.depth) else .5
            temporal_pixel_variance = temporal_variance*(max(view.camera.k[0, 0], view.camera.k[1, 1])/max(.2, typical_depth))**2
            depth = np.asarray(self.sample(view.depth, self.limits[0])).reshape(-1, 6)
            faces, weights = ray_stencils(self.mesh, q, view, depth[:, :2])
            for sample, face, bary in zip(depth, faces, weights):
                if face < 0:
                    continue
                vertices = self.mesh.triangles[face]
                point = bary @ q[vertices]
                observed = view.world_from_camera[:3, :3] @ sample[2:5]+view.world_from_camera[:3, 3]
                normal = np.cross(q[vertices[1]]-q[vertices[0]], q[vertices[2]]-q[vertices[0]])
                norm = np.linalg.norm(normal)
                if norm < 1e-10 or np.linalg.norm(point-observed) > self.gate:
                    continue
                normal /= norm
                factors.append(Factor('normal', vertices, bary, float(observed @ normal),
                    float(sample[5]+self.depth_floor**2+temporal_variance), normal))
            projected, z = project(q, view)
            edges = self.mesh.boundary_edges
            for pixel in self.sample(view.boundary[view.boundary[:, 2] > 0], self.limits[1]):
                starts, delta = projected[edges[:, 0]], projected[edges[:, 1]]-projected[edges[:, 0]]
                t = np.clip(np.sum((pixel[:2]-starts)*delta, axis=1)/np.maximum(np.sum(delta**2, axis=1), 1e-9), 0, 1)
                closest = starts+t[:, None]*delta
                distances = np.linalg.norm(closest-pixel[:2], axis=1)
                distances[(z[edges] <= 0).any(axis=1)] = np.inf
                index = np.argmin(distances)
                if not np.isfinite(distances[index]) or distances[index] > 60:
                    continue
                vertices = edges[index]; bary = np.array([1-t[index], t[index]])
                # The closest outer edge might be behind a folded foreground layer.
                face, hit_weights = ray_stencils(self.mesh, q, view, [closest[index]])
                if face[0] < 0:
                    continue
                hit = hit_weights[0] @ q[self.mesh.triangles[face[0]]]
                if np.linalg.norm(hit-bary @ q[vertices]) > self.visibility_tolerance:
                    continue
                axis = np.array([-delta[index, 1], delta[index, 0]])
                axis /= max(np.linalg.norm(axis), 1e-9)
                factors.append(Factor('boundary', vertices, bary, float(pixel[:2] @ axis),
                    self.pixel_noise**2/max(pixel[2], .05)+temporal_pixel_variance, axis, view))
            for track in self.sample(view.tracks, self.limits[2]):
                key = (view.camera_id, track['id'], track['first_ns'])
                pixel = np.array([track['u'], track['v']])
                if key not in self.bindings:
                    face, bary = ray_stencils(self.mesh, q, view, [pixel])
                    if face[0] < 0:
                        continue
                    self.bindings[key] = (self.mesh.triangles[face[0]], bary[0])
                    if len(self.bindings) > self.binding_limit:
                        self.bindings.pop(next(iter(self.bindings)))
                    continue  # birth correspondence is a prior, not a self-measurement
                vertices, bary = self.bindings[key]
                prediction, depth_z = project(bary @ q[vertices], view)
                face, hit_bary = ray_stencils(self.mesh, q, view, [prediction])
                if face[0] < 0 or depth_z <= 0 or np.linalg.norm(prediction-pixel) > 60:
                    continue
                hit = hit_bary[0] @ q[self.mesh.triangles[face[0]]]
                if np.linalg.norm(hit-bary @ q[vertices]) > self.visibility_tolerance:
                    continue
                for axis in np.eye(2):
                    factors.append(Factor('track', vertices, bary, float(pixel @ axis), self.pixel_noise**2*2+temporal_pixel_variance, axis, view))
            if self.use_uv:
                for sample in self.sample(view.uv, self.limits[2]):
                    if sample[6] < .7:
                        continue
                    vertices, bary = self.mesh.uv_stencil(sample[2:4])
                    projected_pixel, depth_z = project(bary @ q[vertices], view)
                    if depth_z <= 0 or np.linalg.norm(projected_pixel-sample[:2]) > 60:
                        continue
                    face, hit_bary = ray_stencils(self.mesh, q, view, [projected_pixel])
                    if face[0] < 0 or np.linalg.norm(hit_bary[0] @ q[self.mesh.triangles[face[0]]]-bary @ q[vertices]) > self.visibility_tolerance:
                        continue
                    # Conservative material-UV uncertainty -> pixel uncertainty.
                    scale = max(view.camera.k[0, 0], view.camera.k[1, 1])*max(self.mesh.spec.width_m, self.mesh.spec.height_m)/depth_z
                    variance = self.pixel_noise**2+scale**2*max(sample[4:6])+temporal_pixel_variance
                    for axis in np.eye(2):
                        factors.append(Factor('uv_prior', vertices, bary, float(sample[:2] @ axis), float(variance), axis, view))
        for grip in frame.grips:
            # Full covariance is whitened for Tier A; online scalar axes use its
            # eigensystem so correlated glove error is not silently discarded.
            eigenvalues, axes = np.linalg.eigh(grip.covariance)
            for variance, axis in zip(eigenvalues, axes.T):
                factors.append(Factor('grip', self.mesh.corners[grip.corner:grip.corner+1], np.ones(1),
                    float(grip.position @ axis), float(variance), axis))
        for force in frame.forces:
            for axis in np.eye(3):
                factors.append(Factor('force', self.mesh.corners[force.corner:force.corner+1], np.ones(1),
                    float(force.on_sheet @ axis), force.stddev_n**2, axis, corner=force.corner))
        return factors
