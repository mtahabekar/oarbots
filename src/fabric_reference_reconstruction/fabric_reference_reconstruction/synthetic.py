"""Small deterministic software fixture; these labels are not hardware accuracy."""
import argparse
from pathlib import Path
import numpy as np
from fabric_perception.core import Camera
from fabric_state_estimator.data import Frame, Grip, View, save_sequence
from fabric_state_estimator.mesh import MaterialMesh, MeshSpec
from fabric_state_estimator.physics import PhysicsSpec, make_physics
from fabric_state_estimator.observations import camera_points, project


def generate(rows=3, frames=5, seed=5):
    mesh = MaterialMesh(MeshSpec(rows=rows, columns=rows))
    spec = PhysicsSpec()
    physics = make_physics(mesh, spec)
    corners = mesh.rest[mesh.corners].copy(); corners[:, 2] = 1.
    q, v = mesh.seed(corners, .13)[None], np.zeros((1, mesh.count, 3))
    camera = Camera(640, 480, [[500., 0., 320.], [0., 500., 240.], [0., 0., 1.]], [])
    transform = np.diag([1., -1., -1., 1.]); transform[:3, 3] = [.42, .42, 2.]
    random = np.random.default_rng(seed)
    result, truth = [], []
    for index in range(frames):
        if index:
            q, v = physics.advance(q, v, .04, corners[None])
        view = View('synthetic_overhead', camera, transform)
        positions = q[0, mesh.triangles].mean(axis=1)
        points = camera_points(positions, view)
        pixels = project(positions, view)[0]
        measured = points*(1+random.normal(0, .002, len(points))/points[:, 2])[:, None]
        view.depth = np.column_stack((pixels, measured, np.full(len(points), .002**2)))
        # Simulated image evidence, separate from truth handed to evaluation.
        # Track IDs intentionally encode no material indices for the observer.
        image_points = project(q[0], view)[0]
        view.tracks = [dict(id=int(17*i+3), first_ns=1_000_000_000, u=float(p[0]), v=float(p[1]))
                       for i, p in enumerate(image_points)]
        edge_points = np.concatenate([(1-a)*q[0, edge[0]]+a*q[0, edge[1]]
                                      for edge in mesh.boundary_edges for a in (0., .33, .66)]).reshape(-1, 3)
        boundary_pixels = project(edge_points, view)[0]
        view.boundary = np.column_stack((boundary_pixels, np.full(len(boundary_pixels), .9)))
        grips = [Grip(i, p, np.eye(3)*.002**2) for i, p in enumerate(corners)]
        result.append(Frame(1_000_000_000+index*40_000_000, [view], grips))
        truth.append(q[0].copy())
    return mesh, spec, result, np.stack(truth)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output_directory'); parser.add_argument('--rows', type=int, default=3)
    parser.add_argument('--frames', type=int, default=5)
    options = parser.parse_args(args)
    output = Path(options.output_directory); output.mkdir(parents=True, exist_ok=True)
    mesh, physics, frames, truth = generate(options.rows, options.frames)
    save_sequence(output/'sequence.json', mesh, physics, frames, {'kind': 'synthetic_test_fixture', 'seed': 5})
    np.savez_compressed(output/'truth.npz', positions=truth, times_ns=np.array([f.time_ns for f in frames], np.int64))
    print(str(output/'sequence.json'))


if __name__ == '__main__':
    main()
