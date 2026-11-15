"""Headless checks of the twin: frames, slaved grips, feedback, calibration and the CBF loop."""
import numpy as np
import pytest

pytest.importorskip('newton')
from fabric_twin.bank import Bank, Case, surface_cost  # noqa: E402
from fabric_twin.cloth import Obstacle, TwinCloth, TwinSpec, material_lift, twin_frame  # noqa: E402
from fabric_twin.feedback import Nudger  # noqa: E402
from fabric_twin.scenario import LoopSpec, run  # noqa: E402
from fabric_twin.warmup import WarmupScript  # noqa: E402

W = .84
SPEC = TwinSpec(cells_x=16, cells_y=16, device='cuda:0')


def grips(inset=0., lateral=0.):
    return np.array([[-W/2+lateral, -W/2+inset, 1.05], [W/2-lateral, -W/2+inset, 1.05],
                     [-W/2+lateral, W/2, 1.05], [W/2-lateral, W/2, 1.05]])


def test_twin_frame_handles_rotation_and_mirrored_corner_order():
    turned = grips() @ np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])+[1.4, .1, 0.]
    a, origin = twin_frame(turned)
    assert np.linalg.det(a) == pytest.approx(1.) and origin[2] == 0.
    np.testing.assert_allclose((turned-origin) @ a.T, grips(), atol=1e-9)
    mirrored = grips()[[1, 0, 3, 2]]
    a, origin = twin_frame(mirrored)
    assert np.linalg.det(a) == pytest.approx(-1.)
    np.testing.assert_allclose((mirrored-origin) @ a.T, grips(), atol=1e-9)
    lift = material_lift(np.array([[0., 0.], [1., 1.], [.5, .5]]), 8, 8)
    np.testing.assert_allclose(lift.sum(axis=1), 1.)
    assert lift[0, 0] == 1. and lift[1, 63] == 1.


def test_grip_patches_follow_measured_grips_and_reach_is_enforced():
    turned = grips() @ np.array([[0., 1., 0.], [-1., 0., 0.], [0., 0., 1.]])+[1.4, .1, 0.]
    cloth = TwinCloth(SPEC, turned)
    target = turned.copy()
    for k in range(60):
        target[:, 2] -= .1/60  # all four grips descend at 0.1 m/s
        cloth.step(target)
    np.testing.assert_allclose(cloth.corners(), target, atol=1e-5)
    q, v = cloth.state()
    assert q[:, 2].max() < 1.0 and np.isfinite(v).all() and cloth.mass.sum() == pytest.approx(W*W*.2, rel=1e-3)
    far = turned.copy(); far[2] += 5.  # a glove outlier cannot yank the sheet
    moved = cloth.vector_to_world(cloth.to_twin(target))  # noqa: F841
    cloth.step(far)
    assert np.linalg.norm(cloth.corners()[2]-target[2]) <= SPEC.max_grip_speed_mps*SPEC.frame_dt+1e-6
    stretched = turned.copy(); stretched[3] += (stretched[3]-stretched[2])*.05  # hands 4 cm too far apart
    fixed = cloth.within_reach(stretched)
    assert np.linalg.norm(fixed[3]-fixed[2]) <= W*1.002+1e-9
    np.testing.assert_allclose(fixed[:2], stretched[:2])  # robot grips are never moved


def test_obstacle_clearance_and_material_change():
    cloth = TwinCloth(TwinSpec(cells_x=16, cells_y=16, obstacle=Obstacle((0., 0., .16), (.22, .22, .16), .3)), grips(.03))
    for _ in range(120):
        cloth.step(grips(.03))
    assert .5 < cloth.obstacle_clearance() < .75
    before = cloth.state()[0]
    cloth.set_material(bend=4., shear=.25)
    for _ in range(180):
        cloth.step(grips(.03))
    assert np.percentile(np.linalg.norm(cloth.state()[0]-before, axis=1), 95) > .003  # the slack sheet's shape depends on it
    with pytest.raises(ValueError):
        cloth.set_material(bend=-1.)


def test_feedback_pulls_a_wrong_twin_toward_the_estimate_and_is_bounded():
    # 3 cm of slack in both directions: there the sheet's shape depends strongly on its
    # coefficients (a free-running twin with the nominal ones ends up centimetres away).
    spec, held = TwinSpec(), grips(.03, .03)
    real, twin, free = TwinCloth(spec, held), TwinCloth(spec, held), TwinCloth(spec, held)
    real.set_material(3., .4)
    nudger = Nudger(twin, 8, 8)
    u, v = np.meshgrid(np.linspace(0, 1, 8), np.linspace(0, 1, 8))
    sample = material_lift(np.column_stack((u.ravel(), v.ravel())), 29, 29)
    for k in range(600):
        real.step(held); twin.step(held); free.step(held)
        if k % 4 == 0:
            q, vel = twin.state()
            force, info = nudger.force(q, vel, sample @ real.state()[0], sample @ real.state()[1])
            assert np.abs(force[~twin.free]).max() == 0.
            assert np.linalg.norm(force, axis=1).max() <= twin.mass.max()*nudger.spec.accel_limit_mps2+1e-9
            twin.set_external(force)
    error = lambda cloth: np.percentile(np.linalg.norm(cloth.state()[0]-real.state()[0], axis=1), 95)  # noqa: E731
    # The bounded correction narrows the gap (77 -> 59 mm here); calibration is what closes it.
    assert error(free) > .03 and error(twin) < .85*error(free)
    assert info['residual_total_n'] > 0


def test_calibration_recovers_a_shear_scale_and_refuses_uninformative_data():
    random = np.random.default_rng(0)
    real = TwinCloth(SPEC, grips(0., .02)); real.set_material(1., .4)
    cases, g = [], grips(0., .02)
    for inset in (.02, .04):
        target = grips(inset, .02)
        for k in range(300):
            real.step(g+(target-g)*min(1., k/60))
        g = target
        start = real.state()[0].copy()
        for _ in range(90):
            real.step(g)
        end = real.state()[0]
        cases.append(Case(start, np.repeat(g[None], 90, 0), end[random.choice(len(end), 800)]+random.normal(0, .002, (800, 3))))
    bank = Bank(SPEC, grips(0., .02))
    assert surface_cost(end, bank.grid, end)[0] < 1e-10
    result = bank.calibrate(cases, rounds=2)
    assert .25 <= result['shear'] <= .7 and result['cost_m'] <= result['start_cost_m']
    with pytest.raises(ValueError):
        Case(start, g[None], end[:5])
    # A taut sheet: candidates end in the same shape, so nothing is changed.
    taut = TwinCloth(SPEC, grips())
    for _ in range(120):
        taut.step(grips())
    q = taut.state()[0]
    scales, report = Bank(SPEC, grips()).adapt([Case(q, np.repeat(grips()[None], 60, 0), q[::2])], (1., 1.))
    assert scales == (1., 1.) and not report['informative']


def test_warmup_script_steps_in_and_returns():
    script = WarmupScript(insets_m=(.02, .04), speed_mps=.03, hold_sec=2., settle_sec=1.)
    script.start(1_000_000_000, grips())
    assert [round(h[2], 2) for h in script.holds()] == [0., .02, .04, 0.]
    held = grips()[:2].copy()
    for k in range(int(script.duration*60)):
        t = 1_000_000_000+int(k/60*1e9)
        command = script.command(t, held)
        assert np.linalg.norm(command, axis=1).max() <= .06+1e-9 and np.abs(command[:, 2]).max() < 1e-9
        held += command/60
    np.testing.assert_allclose(held, grips()[:2], atol=3e-3)
    assert not script.active(1_000_000_000+int((script.duration+1)*1e9))
    with pytest.raises(ValueError):
        WarmupScript(insets_m=(.5,))


def test_closed_loop_follows_the_real_sheet_and_the_cbf_runs():
    log = run(LoopSpec(seconds=6., move_start_s=2., move_seconds=3., real_scales=(1., 1.)), TwinSpec(cells_x=28, cells_y=28))
    moving = log['time'] > 2.
    assert np.median(log['error_p95'][moving]) < .005
    assert len(log['status']) > 50 and 'solver_failure' not in set(log['status'].tolist())
    assert log['clearance_real'].min() > .05


def test_carry_keeps_the_real_sheet_slack_and_clear_of_the_obstacle():
    """Headless rehearsal of the carry with the default settings: the stretch guard keeps every
    robot-to-grip distance under the sheet's size minus the slack, the CBF needs no hard rows,
    and the pull at the robot grips stays at the level of the sheet's own weight and motion."""
    from fabric_twin.scenario import CarrySpec, carry
    log = carry(CarrySpec(segments=(('move', 0., 0., -.5, .08), ('hold', 3.)), seconds=16.))
    after = log['time'] > 3.
    assert log['distance'][after][:, :3].max() <= .84-.03+.005  # edges through a robot, 5 mm for arm lag
    assert log['distance'][after][:, 3:5].max() <= np.hypot(.84, .84)-.03+.005
    assert 'relaxed_hard' not in set(log['status'].tolist())
    assert log['clearance'][after].min() > .05
    assert np.percentile(log['tension'][after][:, :2], 95) < 10.  # model pull, N; 46 N before the guard
