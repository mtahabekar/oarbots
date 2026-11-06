"""Versioned, inspectable recordings without ROS or pickle dependencies."""
from dataclasses import asdict, dataclass, field
import json
from pathlib import Path
import numpy as np
from fabric_perception.core import Camera
from .mesh import MaterialMesh, MeshSpec
from .physics import PhysicsSpec


@dataclass
class Grip:
    corner: int
    position: np.ndarray
    covariance: np.ndarray

    def __post_init__(self):
        self.position = np.asarray(self.position, float).reshape(3)
        self.covariance = np.asarray(self.covariance, float).reshape(3, 3)
        if (self.corner not in range(4) or not np.isfinite(self.position).all()
                or not np.isfinite(self.covariance).all() or not np.allclose(self.covariance, self.covariance.T)
                or np.linalg.eigvalsh(self.covariance).min() <= 0):
            raise ValueError('Invalid measured grip or covariance')


@dataclass
class View:
    camera_id: str
    camera: Camera
    world_from_camera: np.ndarray
    # depth columns: pixel_u,pixel_v,optical_x,optical_y,optical_z,axial_variance
    depth: np.ndarray = field(default_factory=lambda: np.empty((0, 6)))
    # boundary columns: pixel_u,pixel_v,confidence; occlusion samples have confidence=0
    boundary: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))
    tracks: list = field(default_factory=list)  # dict id,first_ns,u,v; timestamps remain integers
    # UV columns: pixel_u,pixel_v,material_u,material_v,var_u,var_v,confidence
    uv: np.ndarray = field(default_factory=lambda: np.empty((0, 7)))
    time_ns: int = 0  # original per-camera capture, retained when synchronized views are grouped

    def __post_init__(self):
        self.world_from_camera = np.asarray(self.world_from_camera, float).reshape(4, 4)
        rotation = self.world_from_camera[:3, :3]
        if (not self.camera_id or not np.isfinite(self.world_from_camera).all()
                or not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(rotation), 1., atol=1e-5)
                or not np.allclose(self.world_from_camera[3], [0, 0, 0, 1])):
            raise ValueError('Expected a finite world_from_camera rigid transform')
        for name, width in (('depth', 6), ('boundary', 3), ('uv', 7)):
            value = np.asarray(getattr(self, name), float).reshape(-1, width)
            if not np.isfinite(value).all():
                raise ValueError('Nonfinite '+name+' observations')
            setattr(self, name, value)
        if (np.any(self.depth[:, 5] <= 0) or np.any(self.depth[:, 4] <= 0)
                or np.any(self.boundary[:, 2] < 0) or np.any(self.boundary[:, 2] > 1)
                or np.any(self.uv[:, 4:6] <= 0) or np.any(self.uv[:, 2:4] < 0) or np.any(self.uv[:, 2:4] > 1)):
            raise ValueError('Invalid depth, confidence or UV uncertainty')
        for track in self.tracks:
            if (not isinstance(track['id'], int) or not isinstance(track['first_ns'], int)
                    or not np.isfinite([track['u'], track['v']]).all()):
                raise ValueError('Invalid image track identity or pixel')


@dataclass
class Force:
    corner: int
    on_sheet: np.ndarray
    stddev_n: float

    def __post_init__(self):
        self.on_sheet = np.asarray(self.on_sheet, float).reshape(3)
        if self.corner not in (0, 1) or not np.isfinite(self.on_sheet).all() or not np.isfinite(self.stddev_n) or self.stddev_n <= 0:
            raise ValueError('Invalid robot reaction force')


@dataclass
class Frame:
    time_ns: int
    views: list = field(default_factory=list)
    grips: list = field(default_factory=list)
    forces: list = field(default_factory=list)

    def __post_init__(self):
        if not isinstance(self.time_ns, int) or self.time_ns <= 0:
            raise ValueError('Nonzero integer capture timestamp required')

    def grip_targets(self, fallback=None):
        positions = np.full((4, 3), np.nan) if fallback is None else np.array(fallback, copy=True)
        covariance = np.tile(np.eye(3)*.05**2, (4, 1, 1))
        for corner in range(4):
            measurements = [g for g in self.grips if g.corner == corner]
            if measurements:
                precision = [np.linalg.inv(g.covariance) for g in measurements]
                covariance[corner] = np.linalg.inv(sum(precision))
                positions[corner] = covariance[corner] @ sum(p @ g.position for p, g in zip(precision, measurements))
        if not np.isfinite(positions).all():
            raise ValueError('All four grips are needed for initialization')
        return positions, covariance


def _json(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    raise TypeError(type(value).__name__)


def save_sequence(path, mesh, physics, frames, provenance=None):
    times = [f.time_ns for f in frames]
    if not times or any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('A nonempty, strictly capture-ordered sequence is required')
    payload = {'schema_version': 1, 'mesh': asdict(mesh.spec), 'physics': asdict(physics),
               'provenance': provenance or {}, 'frames': [asdict(frame) for frame in frames]}
    Path(path).write_text(json.dumps(payload, default=_json, allow_nan=False))


def load_sequence(path):
    data = json.loads(Path(path).read_text())
    if data.get('schema_version') != 1:
        raise ValueError('Unsupported sequence schema')
    mesh, physics = MaterialMesh(MeshSpec(**data['mesh'])), PhysicsSpec(**data['physics'])
    frames = []
    for item in data['frames']:
        views = []
        for view in item['views']:
            view['camera'] = Camera(**view['camera'])
            views.append(View(**view))
        frames.append(Frame(item['time_ns'], views, [Grip(**g) for g in item['grips']], [Force(**f) for f in item['forces']]))
    if not frames or any(b.time_ns <= a.time_ns for a, b in zip(frames, frames[1:])):
        raise ValueError('Sequence timestamps must be nonempty and strictly increasing')
    return mesh, physics, frames, data.get('provenance', {})
