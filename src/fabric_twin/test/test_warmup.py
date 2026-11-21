"""CPU-only trajectory checks; these do not establish material recovery."""
import numpy as np
import pytest
from fabric_twin.warmup import WarmupScript


def run(script):
    corners = np.array([[0., 0., 1.], [.84, 0., 1.], [0., .84, 1.], [.84, .84, 1.]])
    script.start(0, corners)
    held = corners[:2].copy()
    path = []
    for t in np.arange(0, script.duration, 1/60):
        velocity = script.command(int(t*1e9), held)
        assert np.isfinite(velocity).all()
        assert np.linalg.norm(velocity, axis=1).max() <= 2*script.speed+1e-9
        held += velocity/60
        path.append(held.copy())
    np.testing.assert_allclose(held, corners[:2], atol=.003)
    return np.array(path)


def test_extended_motion_exercises_more_directions_than_baseline():
    baseline = run(WarmupScript())
    script = WarmupScript(transverse_m=.03, lift_m=.05, lateral_m=.03)
    richer = run(script)
    assert np.ptp(baseline[:, 0, 0]) == 0
    assert np.ptp(baseline[:, 0, 2]) == 0
    assert np.ptp(richer[:, 0, 0]) > .05
    assert np.ptp(richer[:, 0, 2]) > .045
    assert np.ptp(richer[:, 1, 2]) > .045
    assert len(script.holds()) > 4
    assert all(end-start >= 2.9 for start, end, _ in script.holds())


@pytest.mark.parametrize('value', [-.01, .2, float('nan')])
def test_invalid_excursions(value):
    with pytest.raises(ValueError):
        WarmupScript(lift_m=value)


def test_plan_labels_every_hold_and_matches_holds():
    script = WarmupScript(insets_m=(.04, .08), transverse_m=.06, lift_m=.06, lateral_m=.08)
    corners = np.array([[-.39, -.41, 1.05], [.38, -.42, 1.06], [-.33, .25, 1.08], [.41, .21, 1.03]])
    before = script.duration
    script.start(0, corners)
    assert script.duration == pytest.approx(before)          # durations do not depend on where the grips are
    plan = script.plan()
    assert [h['label'] for h in plan] == ['start', 'slack 4 cm', 'slack 8 cm', 'narrow 6 cm', 'back', 'lift Blue 6 cm', 'back',
                                         'lift Silver 6 cm', 'back', 'shift toward Silver 8 cm', 'shift toward Blue 8 cm',
                                         'back', 'return']
    assert [(h['start'], h['end']) for h in plan] == [(a, b) for a, b, _ in script.holds()]
    insets = [round(h[2], 3) for h in script.holds()]
    assert insets[:3] == [0., .04, .08] and insets[-1] == 0.
    assert all(insets[k] == .08 for k, h in enumerate(plan) if h['label'] == 'back')
    assert all(abs(x-.08) < .01 for x in insets[3:-1])   # excursions sit at the deepest inset (sideways parts aside)
    across = (corners[1]-corners[0])*[1, 1, 0]
    across /= np.linalg.norm(across)
    shift = np.array(plan[9]['target'])-np.array(plan[8]['target'])
    np.testing.assert_allclose(shift, [.08*across, .08*across], atol=1e-4)
    assert np.array(plan[5]['target'])[0, 2] == pytest.approx(.06, abs=1e-4) and np.array(plan[5]['target'])[1, 2] == 0.
    np.testing.assert_allclose(script.hold_targets(corners), [h['target'] for h in plan], atol=1e-4)
    for h in plan:   # the command reaches each target by the end of its hold
        np.testing.assert_allclose(script.offset(h['end']), h['target'], atol=1e-4)


def test_inset_only_profile_is_unchanged():
    script = WarmupScript(insets_m=(.02, .04), speed_mps=.03, hold_sec=5., settle_sec=3.)
    assert script.duration == pytest.approx(3+.02/.03+5+.02/.03+5+.04/.03+5)
    assert [round(h[2], 3) for h in script.holds()] == [0., .02, .04, 0.]
    assert script.summary() == 'insets [0.02, 0.04] m at 0.03 m/s, 21 s'


def test_person_cues_follow_the_robot_part_with_the_robots_still():
    corners = np.array([[-.39, -.41, 1.05], [.38, -.42, 1.06], [-.33, .25, 1.08], [.41, .21, 1.03]])
    script = WarmupScript(insets_m=(.02, .04), person_moves_m=dict(up=.15, down=.15, toward=.15, together=.12, side=.1))
    script.start(0, corners)
    plan = script.plan()
    labels = [h['label'] for h in plan]
    assert labels[:4] == ['start', 'slack 2 cm', 'slack 4 cm', 'return']
    assert labels[4:] == ['hands up 15 cm', 'back', 'hands down 15 cm', 'back', 'hands toward the robots 15 cm',
                          'hands together 12 cm each', 'apart', 'hands to your left 10 cm', 'hands to your right 10 cm',
                          'back', 'return']
    for h in plan[3:]:
        assert np.allclose(h['target'], 0.)                     # robots hold at the start during the person's moves
    hands = {h['label']: np.array(h['hands']) for h in plan}
    assert np.allclose(hands['hands up 15 cm'][:, 2], .15)
    toward = hands['hands toward the robots 15 cm']
    assert np.allclose(np.linalg.norm(toward, axis=1), .15, atol=1e-3) and np.all(toward[:, 1] < -.14)
    together = hands['hands together 12 cm each']-toward
    assert together[0, 0] > .11 and together[1, 0] < -.11      # the right hand moves toward the left and vice versa
    left = hands['hands to your left 10 cm']-toward
    assert left[0, 0] > .09 and left[1, 0] > .09               # toward Silver's side
    for h in plan:                                              # the hands are asked to be there by the end of each hold
        np.testing.assert_allclose(script.hand_offset(h['end']), h['hands'], atol=1e-4)
    # each person move leaves the reaction time plus the move at 10 cm/s before its hold starts
    assert plan[4]['start']-plan[3]['end'] == pytest.approx(2.+.15/.1)


@pytest.mark.parametrize('moves', [dict(up=.3), dict(wave=.1), dict(side=-.1)])
def test_invalid_person_moves(moves):
    with pytest.raises(ValueError):
        WarmupScript(person_moves_m=moves)
