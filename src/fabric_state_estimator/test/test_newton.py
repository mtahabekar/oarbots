"""Optional backend contract test. Run explicitly in a Newton/Warp environment."""
import importlib.util
import numpy as np
import pytest
from fabric_state_estimator.mesh import MaterialMesh, MeshSpec
from fabric_state_estimator.physics import PhysicsSpec, make_physics


@pytest.mark.skipif(importlib.util.find_spec('newton') is None, reason='Optional Newton backend is not installed')
def test_newton_world_independence_pin_motion_and_force_buffer():
    mesh = MaterialMesh(MeshSpec(rows=3, columns=3))
    physics = make_physics(mesh, PhysicsSpec(backend='newton_style3d', max_step_sec=.005))
    q = np.repeat(mesh.rest[None], 2, axis=0); q[:, :, 2] = 1.
    v = np.zeros_like(q); load = np.zeros((2, 3)); load[1, 2] = mesh.mass.sum()*9.81
    out, velocity = physics.advance(q, v, .02, q[:, mesh.corners], load)
    np.testing.assert_allclose(out[:, mesh.corners], q[:, mesh.corners], atol=1e-6)
    assert out[0, 4, 2] < out[1, 4, 2]
    np.testing.assert_allclose(out[1], q[1], atol=1e-5)
    single, _ = physics.advance(q[:1], v[:1], .02, q[:1, mesh.corners], load[:1])
    np.testing.assert_allclose(single, out[:1], atol=1e-6)
    assert np.isfinite(physics.reactions(out, load)).all()
    # Independent reference: installed Style3D force kernels, not this adapter's
    # NumPy mechanics, must agree before force factors are usable.
    import warp as wp
    from newton._src.solvers.style3d.kernels import eval_stretch_kernel, eval_bend_kernel
    model = physics.worlds[1]['model']
    perturbed = (q[0]+np.random.default_rng(5).normal(0, .005, q[0].shape)).astype(np.float32)
    positions = wp.array(perturbed, dtype=wp.vec3, device='cpu')
    forces = wp.zeros(mesh.count, dtype=wp.vec3, device='cpu')
    wp.launch(eval_stretch_kernel, model.tri_count, inputs=[positions, model.tri_areas, model.tri_poses,
        model.tri_indices, model.style3d.tri_aniso_ke], outputs=[forces], device='cpu')
    wp.launch(eval_bend_kernel, model.edge_count, inputs=[positions, model.style3d.edge_rest_area,
        model.style3d.edge_bending_cot, model.edge_indices, model.edge_bending_properties], outputs=[forces], device='cpu')
    expected = physics.internal_forces(perturbed[None].astype(float))[0]
    np.testing.assert_allclose(expected, forces.numpy(), atol=max(1e-7, 1e-4*np.abs(expected).max()), rtol=0)


@pytest.mark.skipif(importlib.util.find_spec('newton') is None, reason='Optional Newton backend is not installed')
def test_captured_step_graph_matches_launch_by_launch_stepping():
    import warp as wp
    from dataclasses import replace
    if not wp.get_cuda_device_count():
        pytest.skip('CUDA graphs need a GPU')
    mesh = MaterialMesh(MeshSpec(rows=6, columns=6))
    spec = PhysicsSpec(backend='newton_style3d', device='cuda:0', max_step_sec=1/240)
    rng = np.random.default_rng(1)
    q = np.repeat(mesh.rest[None], 3, axis=0) + rng.normal(0, .002, (3, mesh.count, 3)); q[:, :, 2] += 1.
    v, loads = rng.normal(0, .02, q.shape), rng.normal(0, .05, (3, 3))
    grips = q[:, mesh.corners] + rng.normal(0, .002, (3, 4, 3))
    graph, plain = make_physics(mesh, spec), make_physics(mesh, replace(spec, cuda_graph=False))
    for duration in (1/15, 1/15, .1):  # first call per (steps, dt) captures, later calls replay
        a, b = graph.advance(q, v, duration, grips, loads), plain.advance(q, v, duration, grips, loads)
        np.testing.assert_allclose(a[0], b[0], atol=2e-5)
        np.testing.assert_allclose(a[1], b[1], atol=2e-3)
        np.testing.assert_allclose(a[0][:, mesh.corners], grips, atol=1e-6)
    assert len(graph.worlds[3]['graphs']) == 1  # one captured 1/240 s step serves 1/15 s and 0.1 s


@pytest.mark.skipif(importlib.util.find_spec('newton') is None, reason='Optional Newton backend is not installed')
def test_bending_force_isolated_from_dominant_membrane_forces():
    import warp as wp
    from newton._src.solvers.style3d.kernels import eval_bend_kernel
    mesh = MaterialMesh(MeshSpec(rows=8, columns=8))
    physics = make_physics(mesh, PhysicsSpec(backend='newton_style3d', device='cpu'))
    world = physics._world(1)
    model = world['model']
    q = mesh.rest.astype(np.float32).copy()
    q[:, 2] += .02*np.sin(np.pi*mesh.uv[:, 0])*np.sin(np.pi*mesh.uv[:, 1])
    native = wp.zeros(mesh.count, dtype=wp.vec3, device='cpu')
    wp.launch(eval_bend_kernel, model.edge_count, inputs=[wp.array(q, dtype=wp.vec3, device='cpu'),
        model.style3d.edge_rest_area, model.style3d.edge_bending_cot,
        model.edge_indices, model.edge_bending_properties], outputs=[native], device='cpu')
    expected = native.numpy()
    assert np.max(np.abs(expected)) > 1e-6
    physics.stretch[:] = 0.  # isolate bending: large membrane force must not hide coefficient errors
    actual = physics.internal_forces(q[None].astype(float))[0]
    np.testing.assert_allclose(actual, expected, atol=1e-6, rtol=2e-5)
