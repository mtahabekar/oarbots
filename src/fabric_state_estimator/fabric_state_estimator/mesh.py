"""Persistent rectangular material mesh. Geometry never changes vertex identity."""
from dataclasses import asdict, dataclass
import hashlib
import json
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import shortest_path


@dataclass(frozen=True)
class MeshSpec:
    width_m: float = .84
    height_m: float = .84
    rows: int = 17
    columns: int = 17
    areal_density_kg_m2: float = .3


class MaterialMesh:
    def __init__(self, spec=MeshSpec()):
        if (not isinstance(spec.rows, int) or not isinstance(spec.columns, int)
                or not np.isfinite([spec.width_m, spec.height_m, spec.areal_density_kg_m2]).all()
                or min(spec.width_m, spec.height_m, spec.areal_density_kg_m2) <= 0
                or not 2 <= spec.rows <= 65 or not 2 <= spec.columns <= 65):
            raise ValueError('Positive measured dimensions/density and mesh dimensions 2..65 required')
        self.spec = spec
        u, v = np.meshgrid(np.linspace(0, 1, spec.columns), np.linspace(0, 1, spec.rows))
        self.uv = np.column_stack((u.ravel(), v.ravel()))
        self.rest = np.column_stack((self.uv * [spec.width_m, spec.height_m], np.zeros(u.size)))
        self.count = len(self.rest)
        faces = []
        for row in range(spec.rows-1):
            for column in range(spec.columns-1):
                a = row*spec.columns+column; b = a+1; c = a+spec.columns; d = c+1
                faces.extend(((a, b, d), (a, d, c)))
        self.triangles = np.asarray(faces, int)
        self.corners = np.array([0, spec.columns-1, (spec.rows-1)*spec.columns, self.count-1])
        edges = np.sort(self.triangles[:, [[0, 1], [1, 2], [2, 0]]].reshape(-1, 2), axis=1)
        self.edges, frequency = np.unique(edges, axis=0, return_counts=True)
        self.boundary_edges = self.edges[frequency == 1]
        self.lengths = np.linalg.norm(self.rest[self.edges[:, 1]]-self.rest[self.edges[:, 0]], axis=1)
        areas = np.linalg.norm(np.cross(self.rest[self.triangles[:, 1]]-self.rest[self.triangles[:, 0]],
                                        self.rest[self.triangles[:, 2]]-self.rest[self.triangles[:, 0]]), axis=1)/2
        self.mass = np.zeros(self.count)
        np.add.at(self.mass, self.triangles.ravel(), np.repeat(areas*spec.areal_density_kg_m2/3, 3))
        graph = coo_matrix((np.r_[self.lengths, self.lengths],
                           (np.r_[self.edges[:, 0], self.edges[:, 1]], np.r_[self.edges[:, 1], self.edges[:, 0]])),
                          shape=(self.count, self.count)).tocsr()
        self.geodesic = shortest_path(graph, directed=False)
        self.model_id = hashlib.sha256(json.dumps(asdict(spec), sort_keys=True).encode()).hexdigest()[:20]

    def seed(self, grips, sag_m=0.):
        grips = np.asarray(grips, float)
        if grips.shape != (4, 3) or not np.isfinite(grips).all() or not np.isfinite(sag_m) or sag_m < 0:
            raise ValueError('Four finite corner positions and nonnegative initial sag required')
        u, v = self.uv.T
        weights = np.column_stack(((1-u)*(1-v), u*(1-v), (1-u)*v, u*v))
        q = weights @ grips
        q[:, 2] -= sag_m*16*u*(1-u)*v*(1-v)
        return q

    def uv_stencil(self, uv):
        u, v = np.asarray(uv, float)
        if not np.isfinite([u, v]).all() or not (0 <= u <= 1 and 0 <= v <= 1):
            raise ValueError('Material coordinates outside [0,1]')
        x, y = u*(self.spec.columns-1), v*(self.spec.rows-1)
        column, row = min(int(x), self.spec.columns-2), min(int(y), self.spec.rows-2)
        x, y = x-column, y-row
        a = row*self.spec.columns+column
        if x >= y:
            return np.array([a, a+1, a+self.spec.columns+1]), np.array([1-x, x-y, y])
        return np.array([a, a+self.spec.columns+1, a+self.spec.columns]), np.array([1-y, x, y-x])
