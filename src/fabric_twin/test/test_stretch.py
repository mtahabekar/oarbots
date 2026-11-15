"""The stretch guard: geometry only, no simulation needed."""
import numpy as np
import pytest

from fabric_twin.stretch import StretchGuard, StretchSpec

W = H = .84
ROBOTS = np.array([[-.40, -.40, 1.0], [.40, -.40, 1.0]])
HANDS = np.array([[-.40, .40, 1.0], [.40, .40, 1.0]])


def distance(a, b):
    return float(np.linalg.norm(a-b))


def test_a_slack_sheet_passes_the_command_unchanged():
    guard = StretchGuard(W, H)
    command = np.array([[.02, .03, -.05], [-.01, .02, -.05]])
    out, report = guard.filter(ROBOTS, HANDS, np.zeros((2, 3)), command)
    np.testing.assert_allclose(out, command, atol=1e-9)
    assert report['status'] == 'feasible' and report['min_slack_m'] > .0


def test_spreading_or_pulling_away_is_stopped_at_the_limit():
    guard = StretchGuard(W, H, StretchSpec(alpha_per_s=2.))
    robots = np.array([[-.405, -.40, 1.0], [.405, -.40, 1.0]])  # robot-to-robot 0.81 = W - 3 cm
    spread = np.array([[-.08, 0., 0.], [.08, 0., 0.]])
    out, _ = guard.filter(robots, HANDS, np.zeros((2, 3)), spread)
    assert out[1, 0]-out[0, 0] <= 1e-9  # the pair may not lengthen any further
    # Hands moving away (the human walks off) faster than the command follows: the robots must follow.
    hands_away = np.array([[0., .08, 0.], [0., .08, 0.]])
    robots = np.array([[-.40, -.41, 1.0], [.40, -.41, 1.0]])  # robot-to-hand 0.81
    out, report = guard.filter(robots, HANDS, hands_away, np.zeros((2, 3)))
    assert np.all(out[:, 1] >= .08-1e-6) and report['status'] == 'feasible'


def test_an_overstretched_sheet_is_actively_relieved_and_rising_is_limited():
    guard = StretchGuard(W, H, StretchSpec(alpha_per_s=2.))
    robots = np.array([[-.40, -.43, 1.0], [.40, -.43, 1.0]])  # 0.83 > 0.81: 2 cm over the limit
    out, _ = guard.filter(robots, HANDS, np.zeros((2, 3)), np.zeros((2, 3)))
    assert np.all(out[:, 1] >= 2.*.02-1e-6)  # toward the hands at least alpha * excess
    # Raising the robots far above the hands lengthens the edge: allowed only while slack remains.
    up = np.array([[0., 0., .1], [0., 0., .1]])
    robots = np.array([[-.40, -.40, 1.0], [.40, -.40, 1.0]])
    out, report = guard.filter(robots, HANDS-[0, 0, .3], np.zeros((2, 3)), up)  # hands 0.3 m lower: d = 0.854
    for k in range(2):
        n = (robots[k]-(HANDS[k]-[0, 0, .3]))/distance(robots[k], HANDS[k]-[0, 0, .3])
        assert n @ out[k] <= 2.*(.81-distance(robots[k], HANDS[k]-[0, 0, .3]))+report['violation_mps']+1e-4  # reported to 0.1 mm/s
    out, report = guard.filter(robots, HANDS-[0, 0, .3], np.zeros((2, 3)), up)
    assert report['status'] == 'speed_limited' and report['violation_mps'] < .002


def test_force_feedback_yields_along_the_pull_only_above_the_soft_limit():
    guard = StretchGuard(W, H, StretchSpec(force_feedback=True, force_soft_n=6., force_gain_mps_per_n=.004))
    pull = [np.array([0., 16., 0.]), np.array([0., 4., 0.])]  # 10 N over the soft limit on Blue only
    out, report = guard.filter(ROBOTS, HANDS, np.zeros((2, 3)), np.zeros((2, 3)), pull)
    np.testing.assert_allclose(out[0], [0., .04, 0.], atol=1e-9)
    np.testing.assert_allclose(out[1], 0., atol=1e-9)
    assert report['force_yield_mps'] == pytest.approx([.04, 0.])
    off = StretchGuard(W, H, StretchSpec(force_feedback=False))
    np.testing.assert_allclose(off.filter(ROBOTS, HANDS, np.zeros((2, 3)), np.zeros((2, 3)), pull)[0], 0.)


def test_speed_limit_and_invalid_input():
    guard = StretchGuard(W, H, StretchSpec(max_speed_mps=.1))
    out, _ = guard.filter(ROBOTS, HANDS, np.zeros((2, 3)), np.full((2, 3), .5))
    assert np.linalg.norm(out, axis=1).max() <= .1+1e-9
    out, report = guard.filter(ROBOTS*np.nan, HANDS, np.zeros((2, 3)), np.ones((2, 3)))
    assert not out.any() and report['status'] == 'invalid input'
    with pytest.raises(ValueError):
        StretchGuard(W, H, StretchSpec(alpha_per_s=0.))
