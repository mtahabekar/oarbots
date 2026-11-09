import numpy as np
import pytest
from fabric_perception.core import Camera
from fabric_state_estimator.mesh import MaterialMesh, MeshSpec
from fabric_state_estimator.physics import PhysicsSpec, make_physics
from fabric_state_estimator.data import Frame, View, Grip, save_sequence, load_sequence
from fabric_state_estimator.observations import Factor, ObservationBuilder, ray_stencils
from fabric_state_estimator.ensemble import ObserverSpec, EnsembleObserver, localized_update, gaspari_cohn
from fabric_state_estimator.transforms import TransformHistory
from fabric_state_estimator.metrics import evaluate


def fixture():
    mesh = MaterialMesh(MeshSpec(rows=3, columns=3))
    grips = mesh.rest[mesh.corners].copy(); grips[:, 2] = 1.
    frame = Frame(1_000_000_000, grips=[Grip(i, p, np.eye(3)*.002**2) for i, p in enumerate(grips)])
    return mesh, grips, frame


def test_material_mass_topology_identity_and_uv_stencils():
    mesh, grips, _ = fixture()
    assert mesh.mass.sum() == pytest.approx(.84*.84*.3)
    assert len(mesh.triangles) == 8 and len(mesh.boundary_edges) == 8
    assert mesh.model_id == MaterialMesh(mesh.spec).model_id
    for uv in ([0, 0], [1, 1], [.23, .8], [.8, .23]):
        vertices, weights = mesh.uv_stencil(uv)
        np.testing.assert_allclose(weights @ mesh.uv[vertices], uv)
    seeded = mesh.seed(grips, .1)
    np.testing.assert_allclose(seeded[mesh.corners], grips)
    assert seeded[4, 2] == pytest.approx(.9)
    assert mesh.geodesic[0, -1] > 0
    with pytest.raises(ValueError):
        mesh.uv_stencil([2, 0])


def test_spring_gravity_prescribed_grips_mass_and_force_conservation():
    mesh, grips, _ = fixture(); physics = make_physics(mesh, PhysicsSpec())
    q = mesh.seed(grips)[None]; v = np.zeros_like(q)
    assert np.max(np.abs(physics.internal_forces(q))) < 1e-10
    moved = q.copy(); moved[:, 4, 2] -= .1
    np.testing.assert_allclose(physics.internal_forces(moved).sum(axis=1), 0, atol=1e-10)
    out, velocity = physics.advance(q, v, .02, grips[None])
    np.testing.assert_allclose(out[0, mesh.corners], grips)
    assert out[0, 4, 2] < 1 and velocity[0, 4, 2] < 0
    support = np.array([[0., 0., mesh.mass.sum()*9.81]])
    neutral, _ = physics.advance(q, v, .02, grips[None], support)
    np.testing.assert_allclose(neutral, q, atol=1e-8)
    assert np.isfinite(physics.reactions(q, np.zeros((1, 3)))).all()


def test_local_transform_matches_linear_kalman_mean_and_joint_covariance():
    mesh, _, _ = fixture(); physics = make_physics(mesh, PhysicsSpec())
    random = np.random.default_rng(1)
    q = np.zeros((20, mesh.count, 3)); q[:, :, 0] = random.normal(size=(20, 1))
    v = q*.3; loads = np.zeros((20, 3))
    factor = Factor('normal', np.array([4]), np.ones(1), 1., .04, np.array([1., 0., 0.]))
    prior_mean = q[:, 4, 0].mean(); prior_var = q[:, 4, 0].var(ddof=1)
    out, velocity, _, _ = localized_update(mesh, q, v, loads, [factor], physics, 10., inflation=1.)
    expected_mean = prior_mean+prior_var/(prior_var+.04)*(1-prior_mean)
    assert out[:, 4, 0].mean() == pytest.approx(expected_mean)
    assert out[:, 4, 0].var(ddof=1) == pytest.approx(prior_var*.04/(prior_var+.04))
    np.testing.assert_allclose(velocity[:, 4, 0], out[:, 4, 0]*.3)
    np.testing.assert_allclose(gaspari_cohn(np.array([0, 2, 3]), 1), [1, 0, 0])


def test_depth_rays_use_frontmost_layer_and_only_normal_residual():
    mesh = MaterialMesh(MeshSpec(rows=2, columns=4))
    q = np.array([[x, y, z] for y in (-.2, .2) for x, z in [(-.2, 1), (.2, 1), (-.2, 2), (.2, 2)]])
    view = View('camera', Camera(160, 120, [[200., 0, 80], [0, 200., 60], [0, 0, 1]], []), np.eye(4),
                depth=[[80., 60., 0., 0., 1., 1e-4]])
    faces, _ = ray_stencils(mesh, q, view, [[80, 60]])
    assert faces[0] in (0, 1)
    factors = ObservationBuilder(mesh).build(q, Frame(1_000_000_000, [view]))
    assert len(factors) == 1 and np.all(q[factors[0].vertices, 2] == 1)
    shifted = q[None].copy(); shifted[:, :, 0] += .1
    original = factors[0].predict(q[None], q[None]*0, np.zeros((1, 3)), None)
    np.testing.assert_allclose(factors[0].predict(shifted, shifted*0, np.zeros((1, 3)), None), original)


def test_track_birth_is_not_an_independent_measurement_and_occlusion_edges_are_ignored():
    mesh, grips, _ = fixture(); q = mesh.seed(grips)
    transform = np.diag([1., -1., -1., 1.]); transform[:3, 3] = [.42, .42, 2.]
    view = View('cam', Camera(160, 120, [[100., 0, 80], [0, 100., 60], [0, 0, 1]], []), transform,
        boundary=[[80, 60, 0]], tracks=[dict(id=1, first_ns=1_000_000_000, u=80., v=60.)])
    builder = ObservationBuilder(mesh)
    assert builder.build(q, Frame(1_000_000_000, [view])) == []
    assert len(builder.build(q, Frame(1_040_000_000, [view]))) == 2


def test_occluded_tracks_and_uv_priors_do_not_attract_the_hidden_layer():
    mesh = MaterialMesh(MeshSpec(rows=2, columns=4))
    q = np.array([[x, y, z] for y in (-.2, .2) for x, z in [(-.2, 1), (.2, 1), (-.2, 1.05), (.2, 1.05)]])
    view = View('cam', Camera(160, 120, [[200., 0, 80], [0, 200., 60], [0, 0, 1]], []), np.eye(4),
        tracks=[dict(id=1, first_ns=7, u=80., v=60.)], uv=[[80., 60., 5/6, .5, .0001, .0001, .9]])
    builder = ObservationBuilder(mesh, use_uv=True)
    builder.bindings[('cam', 1, 7)] = (mesh.triangles[4], np.array([.5, 0, .5]))
    # Layers are only 5 cm apart, inside the 15 cm association gate. Visibility
    # still excludes the rear material point rather than attracting it forward.
    assert builder.build(q, Frame(1_000_000_000, [view])) == []


def test_forecast_is_nonmutating_causal_times_and_gap_reinitialization():
    mesh, _, frame = fixture()
    observer = EnsembleObserver(mesh, make_physics(mesh, PhysicsSpec()), ObserverSpec(ensemble_size=8))
    state = observer.assimilate(frame)
    assert state['degraded'] and state['visual_factors'] == 0
    old_q, old_v, stamp = observer.q.copy(), observer.v.copy(), observer.time_ns
    predicted = observer.forecast([.02])[0]
    np.testing.assert_array_equal(observer.q, old_q); np.testing.assert_array_equal(observer.v, old_v)
    assert observer.time_ns == stamp and predicted['measurement_ns'] == stamp and predicted['time_ns'] > stamp
    assert np.linalg.eigvalsh(predicted['covariance']).min() >= -1e-10
    with pytest.raises(ValueError, match='backward'):
        observer.assimilate(frame)
    frame.time_ns += 1_000_000_000
    assert observer.assimilate(frame)['reinitialized']


def test_sequence_round_trip_preserves_integer_track_stamps_and_rejects_backwards(tmp_path):
    mesh, _, frame = fixture()
    view = View('a', Camera(10, 10, [[10., 0, 5], [0, 10., 5], [0, 0, 1]], []), np.eye(4),
                tracks=[dict(id=3, first_ns=1_793_822_360_000_000_777, u=5., v=5.)])
    frame.views.append(view)
    path = tmp_path/'sequence.json'
    save_sequence(path, mesh, PhysicsSpec(), [frame])
    _, _, frames, _ = load_sequence(path)
    assert frames[0].views[0].tracks[0]['first_ns'] == view.tracks[0]['first_ns']
    with pytest.raises(ValueError):
        save_sequence(path, mesh, PhysicsSpec(), [frame, frame])


def test_recorded_tf_interpolates_capture_time_and_never_uses_latest():
    history = TransformHistory()
    history.add('world', 'base', 1_000_000_000, [1, 0, 0], [0, 0, 0, 1])
    history.add('world', 'base', 2_000_000_000, [3, 0, 0], [0, 0, 0, 1])
    history.add('base', 'camera', 0, [0, .5, 0], [0, 0, 0, 1], True)
    np.testing.assert_allclose(history.lookup('world', 'camera', 1_500_000_000)[:3, 3], [2., .5, 0])
    history.add('map', 'world', 3_000_000_000, [10, 0, 0], [0, 0, 0, 1])
    history.add('map', 'world', 4_000_000_000, [20, 0, 0], [0, 0, 0, 1])
    np.testing.assert_allclose(history.lookup('world', 'camera', 1_500_000_000)[:3, 3], [2., .5, 0])
    with pytest.raises(ValueError, match='bracket'):
        history.lookup('world', 'camera', 2_100_000_000)


def test_metrics_use_independent_indexed_truth_and_report_partial_coverage():
    truth = {'times_ns': np.array([1]), 'positions': np.zeros((1, 3, 3))}
    truth['positions'][0, 2] = np.nan
    estimate = {'times_ns': np.array([1]), 'positions': np.zeros((1, 3, 3)), 'visibility': np.array([[1, 0, 2]]),
                'covariance': np.tile(np.eye(6)*.01**2, (1, 3, 1, 1))}
    estimate['positions'][0, 1, 0] = .1
    report = evaluate(estimate, truth)
    assert report['all']['count'] == 2 and report['truth_coverage_fraction'] == pytest.approx(2/3)
    assert report['observed']['max_m'] == 0 and report['inferred']['max_m'] == .1
    assert report['position_95_percent_coverage'] == .5
    truth['times_ns'] = np.array([2])
    with pytest.raises(ValueError, match='timestamps'):
        evaluate(estimate, truth)


def test_batched_local_update_matches_per_vertex_reference():
    from fabric_state_estimator.ensemble import transform_weights
    mesh = MaterialMesh(MeshSpec(rows=5, columns=5)); physics = make_physics(mesh, PhysicsSpec())
    random = np.random.default_rng(3)
    q = mesh.seed(np.array([[0, 0, 1], [.84, 0, 1], [0, .84, 1], [.84, .84, 1.]]), .1)[None]+random.normal(0, .02, (16, mesh.count, 3))
    v = random.normal(0, .05, q.shape); loads = random.normal(0, .05, (16, 3))
    factors = [Factor('normal', np.array([i]), np.ones(1), float(q[:, i, 2].mean()+random.normal(0, .02)), .01**2, np.array([0, 0, 1.]))
               for i in (6, 12, 18, 3)] + [Factor('normal', np.array([7]), np.ones(1), 9., .01**2, np.array([0, 0, 1.]))]  # outlier: Huber
    out_q, out_v, _, _ = localized_update(mesh, q, v, loads, factors, physics, .25)
    predictions = np.column_stack([f.predict(q, v, loads, physics) for f in factors])
    values, variance = np.array([f.value for f in factors]), np.array([f.variance for f in factors])
    anchors = np.array([f.vertices[0] for f in factors])
    for vertex in range(mesh.count):
        taper = gaspari_cohn(mesh.geodesic[vertex, anchors], .25)
        state = np.column_stack((q[:, vertex], v[:, vertex]))
        if taper.any():
            w, t = transform_weights(predictions, values, variance, taper, 1.02)
            state = state.mean(axis=0)+w @ (state-state.mean(axis=0))+t @ (state-state.mean(axis=0))
        np.testing.assert_allclose(np.column_stack((out_q[:, vertex], out_v[:, vertex])), state, atol=1e-10)


def test_chained_rollout_matches_independent_rollouts_and_resumes():
    mesh, grips, frame = fixture()
    observer = EnsembleObserver(mesh, make_physics(mesh, PhysicsSpec()), ObserverSpec(ensemble_size=6))
    observer.assimilate(frame)
    velocity = np.tile([.05, 0., .02], (4, 1))
    chained, end = observer.rollout([.06, .02, .04], velocity)
    for horizon, snapshot in zip([.06, .02, .04], chained):
        targets = observer.q[:, mesh.corners]+velocity[None]*horizon
        q, _ = observer.physics.advance(observer.q, observer.v, horizon, targets, observer.loads)
        np.testing.assert_allclose(snapshot['q'], q.mean(axis=0), atol=2e-3)
    resumed, _ = observer.rollout([.08], velocity, end)
    fresh, _ = observer.rollout([.08], velocity)
    np.testing.assert_allclose(resumed[0]['q'], fresh[0]['q'], atol=2e-3)


def test_twisted_or_overstretched_corner_assignment_is_rejected():
    mesh, grips, frame = fixture()
    observer = EnsembleObserver(mesh, make_physics(mesh, PhysicsSpec()), ObserverSpec(ensemble_size=6))
    # Slack sheet (corners 30% closer than rest) so distances alone pass and
    # only the bow-tie orientation check can catch the swapped human corners.
    slack = grips.mean(axis=0)+.7*(grips-grips.mean(axis=0))
    observer.assimilate(Frame(frame.time_ns-1, grips=[Grip(i, p, g.covariance) for (i, p), g in zip(enumerate(slack), frame.grips)]))
    observer.q = observer.time_ns = None
    swapped = [Grip(g.corner, slack[{2: 3, 3: 2}.get(g.corner, g.corner)], g.covariance) for g in frame.grips]
    with pytest.raises(ValueError, match='twisted'):
        observer.assimilate(Frame(frame.time_ns, grips=swapped))
    stretched = [Grip(g.corner, grips[g.corner]*[2., 1., 1.], g.covariance) for g in frame.grips]
    with pytest.raises(ValueError, match='farther apart'):
        observer.assimilate(Frame(frame.time_ns, grips=stretched))


def test_projection_bounds_overextension_without_moving_attachments():
    mesh, grips, frame = fixture()
    observer = EnsembleObserver(mesh, make_physics(mesh, PhysicsSpec()), ObserverSpec(ensemble_size=4, max_edge_strain=.1))
    observer.assimilate(frame)
    corners = observer.q[:, mesh.corners].copy()
    observer.q[:, 4, 2] -= .5  # centre pulled far below its neighbours
    observer._project(.04)
    lengths = np.linalg.norm(observer.q[:, mesh.edges[:, 1]]-observer.q[:, mesh.edges[:, 0]], axis=2)
    assert np.max(lengths/mesh.lengths) <= 1.1+1e-4
    np.testing.assert_array_equal(observer.q[:, mesh.corners], corners)


def test_monitor_depth_residual_is_measured_before_assimilation():
    mesh, _, _ = fixture(); physics = make_physics(mesh, PhysicsSpec())
    q = np.zeros((8, mesh.count, 3)); q[:, :, 2] = np.linspace(-.01, .01, 8)[:, None]
    v, loads = np.zeros_like(q), np.zeros((8, 3))
    factor = Factor('normal', np.array([4]), np.ones(1), .04, .000001, np.array([0., 0., 1.]))
    diagnostics = {}
    updated, _, _, _ = localized_update(mesh, q, v, loads, [factor], physics, 10., diagnostics=diagnostics)
    assert diagnostics['depth_factors'] == 1
    assert diagnostics['prior_depth_p95_m'] == pytest.approx(.04)
    assert abs(.04-updated[:, 4, 2].mean()) < diagnostics['prior_depth_p95_m']
