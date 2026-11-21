"""The dedicated pose and the actual stretch guard must permit the complete return."""
from pathlib import Path

import numpy as np
import pytest
import yaml

from fabric_twin.stretch import StretchGuard, StretchSpec
from fabric_twin.warmup import WarmupScript

REPO = Path(__file__).resolve().parents[3]
CELL = REPO / 'experiments/hardware/cell'


def test_prepared_pose_has_clearance_reserve_and_preserves_height_and_orientation():
    pose = yaml.safe_load((CELL / 'warmup_grasp.yaml').read_text())
    report = __import__('json').loads((REPO / pose['validation_report']).read_text())
    assert report['qualified'] and not report['commanded_motion']
    assert pose['robot_span_m'] == pytest.approx(.77, abs=1e-5)
    assert pose['reserve_m'] >= .025
    assert report['checked_link_clearance_m'] >= .15
    for robot in ('blue', 'silver'):
        arm = report['validation'][robot]
        assert abs(arm['grip_world_m'][2] - arm['original_grip_world_m'][2]) < 1e-5
        assert arm['orientation_error_deg'] < .01
        assert arm['minimum_directional_speed_mps'] > .03


@pytest.mark.parametrize('hands', [
    [[-.30, .305, 1.064], [.43, .292, 1.037]],  # measured in the failed wide-pose run
    [[-.42, .30, 1.05], [.42, .30, 1.05]],
    [[-.35, .25, .95], [.35, .25, 1.15]],
])
@pytest.mark.parametrize('filtered', [True, False])
def test_narrow_pose_returns_in_3d_with_arm_lag(hands, filtered):
    import json
    report = json.loads((REPO / 'experiments/hardware/sessions/2026-11-20_warmup_pose/validation.json').read_text())
    robots = np.array([report['validation'][r]['grip_world_m'] for r in ('blue', 'silver')])
    hands = np.array(hands)
    corners = np.vstack((robots, hands))
    script = WarmupScript()
    script.start(0, corners)
    guard = StretchGuard(.84, .84, StretchSpec(edge_slack_m=.035, diagonal_slack_m=.035, max_speed_mps=.15))
    initial, velocity, previous = robots.copy(), np.zeros((2, 3)), np.zeros((2, 3))
    dt, delay = 1/60, [robots.copy() for _ in range(9)]
    for frame, t in enumerate(np.arange(0, script.duration, dt)):
        if frame % 4 == 0:
            desired = script.command(int(t*1e9), delay[0])
            if filtered:
                desired = (desired + .25*previous) / 1.25  # prior CBF profile, for comparison
                command, info = guard.filter(robots, hands, np.zeros((2, 3)), desired)
                assert info['change_mps'] < .001
            else:
                command = desired  # requested warm-up: no CBF or stretch filtering
            previous = command
        velocity += dt / .5 * (command - velocity)  # measured arm-lag allowance
        robots += velocity * dt
        delay.pop(0)
        delay.append(robots.copy())
    assert np.max(np.linalg.norm(robots-initial, axis=1)) < .003


def configured_profile():
    from fabric_twin.node import load_twin_config
    return WarmupScript(**load_twin_config(CELL / 'twin.yaml')[4])


def test_configured_warmup_stays_inside_the_validated_workspace_boxes():
    """bridge.yaml's boxes are the region swept on 2026-11-19 (collision-free, joint limits, >= 4 cm/s everywhere);
    every hold of the configured warm-up, started at the prepared pose, must stay 2 cm inside them."""
    import json
    report = json.loads((REPO / 'experiments/hardware/sessions/2026-11-20_warmup_pose/validation.json').read_text())
    robots = np.array([report['validation'][r]['grip_world_m'] for r in ('blue', 'silver')])
    hands = np.array([[-.33, .25, 1.06], [.41, .21, 1.04]])
    script = configured_profile()
    bridge = yaml.safe_load((CELL / 'bridge.yaml').read_text())
    for offset in script.hold_targets(np.vstack((robots, hands))):
        for robot, position in zip(('blue', 'silver'), robots+offset):
            box = {**bridge['common'], **bridge[robot]}
            assert np.all(position >= np.array(box['workspace_min'])+.02), (robot, position)
            assert np.all(position <= np.array(box['workspace_max'])-.02), (robot, position)


@pytest.mark.parametrize('hands', [
    [[-.30, .305, 1.064], [.43, .292, 1.037]],
    [[-.35, .25, .95], [.35, .25, 1.15]],
])
def test_configured_warmup_only_slackens_the_sheet_and_returns(hands):
    import json
    report = json.loads((REPO / 'experiments/hardware/sessions/2026-11-20_warmup_pose/validation.json').read_text())
    robots = np.array([report['validation'][r]['grip_world_m'] for r in ('blue', 'silver')])
    hands = np.array(hands)
    script = configured_profile()
    script.start(0, np.vstack((robots, hands)))
    guard = StretchGuard(.84, .84, StretchSpec(edge_slack_m=.035, diagonal_slack_m=.035, max_speed_mps=.15))
    initial, velocity = robots.copy(), np.zeros((2, 3))
    dt, delay = 1/60, [robots.copy() for _ in range(9)]
    for frame, t in enumerate(np.arange(0, script.duration, dt)):
        if frame % 4 == 0:
            command = script.command(int(t*1e9), delay[0])
            moved = hands+script.hand_offset(t)        # the person follows the spoken cues
            _, info = guard.filter(robots, moved, np.zeros((2, 3)), command)
            assert info['change_mps'] < .001          # never near a stretch limit, so the guard would not act
        velocity += dt / .5 * (command - velocity)
        robots += velocity * dt
        delay.pop(0)
        delay.append(robots.copy())
    assert np.max(np.linalg.norm(robots-initial, axis=1)) < .003
