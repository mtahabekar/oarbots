import pytest
from oarbot_runtime.cbf_bridge import CbfBridge
from oarbot_runtime.guard import Guard

IDENTITY = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
HALF_TURN_Z = [[-1, 0, 0], [0, -1, 0], [0, 0, 1]]  # Silver's arm base relative to the world axes


def armed(rotation=IDENTITY, **limits):
    bridge = CbfBridge(**limits)
    bridge.force_update((0., 0., -3.), 1.)
    bridge.pose_update((0., 0., 1.), rotation, 1.)
    assert bridge.rearm(1.)
    return bridge


def test_starts_stopped_and_refuses_to_arm_without_fresh_inputs():
    bridge = CbfBridge()
    bridge.command_update((.05, 0., 0.), 0., 1.)
    assert not any(bridge.output(1.))
    assert not bridge.rearm(1.) and 'wrist force' in bridge.tripped
    bridge.force_update((0., 0., 0.), 1.)
    assert not bridge.rearm(1.) and 'tool pose' in bridge.tripped
    bridge.pose_update((0., 0., 1.), IDENTITY, 1.)
    assert not bridge.rearm(2.)  # both stale by now


def test_world_command_is_rotated_into_the_arm_frame_and_speed_limited():
    bridge = armed(HALF_TURN_Z)
    bridge.command_update((.05, .02, -.01), 0., 1.)
    assert bridge.output(1.01) == pytest.approx((-.05, -.02, -.01, 0., 0., 0.))
    bridge.command_update((.3, 0., .4), 0., 1.02)  # 0.5 m/s requested
    out = bridge.output(1.03)
    assert sum(v*v for v in out[:3])**.5 == pytest.approx(.1) and out[3:] == (0., 0., 0.)


def test_late_or_missing_command_gives_zero_but_stays_armed():
    bridge = armed()
    bridge.command_update((.05, 0., 0.), .3, 1.)  # stamped 0.3 s ago: not a command
    assert not any(bridge.output(1.01)) and not bridge.tripped
    bridge.command_update((.05, 0., 0.), 0., 1.02)
    assert bridge.output(1.05)[0] == pytest.approx(.05)
    assert not any(bridge.output(1.2)) and not bridge.tripped  # command timed out


def test_force_change_workspace_and_stale_inputs_latch_until_rearmed():
    bridge = armed(force_limit_n=15.)
    bridge.command_update((.05, 0., 0.), 0., 1.)
    bridge.force_update((0., 0., -17.), 1.01)  # 14 N change: allowed
    assert bridge.output(1.02)[0] > 0
    bridge.force_update((10., 0., -17.), 1.03)
    assert 'wrist force' in bridge.tripped and not any(bridge.output(1.04))
    bridge.force_update((0., 0., -3.), 1.05)  # the load going away does not re-arm
    bridge.command_update((.05, 0., 0.), 0., 1.05)
    assert not any(bridge.output(1.06))
    bridge.pose_update((0., 0., 1.), IDENTITY, 1.06)
    assert bridge.rearm(1.06) and bridge.output(1.07)[0] > 0

    bridge = armed(workspace_min=(-.5, -.5, .4), workspace_max=(.5, .5, 1.5))
    bridge.pose_update((0., 0., .3), IDENTITY, 1.01)
    assert 'workspace' in bridge.tripped
    assert not bridge.rearm(1.02) and 'outside' in bridge.tripped

    bridge = armed()
    bridge.command_update((.05, 0., 0.), 0., 1.5)
    assert not any(bridge.output(1.5)) and 'stale' in bridge.tripped  # force and pose are 0.5 s old

    bridge = armed()
    bridge.command_update((float('nan'), 0., 0.), 0., 1.)
    assert 'invalid' in bridge.tripped


def test_guard_forwards_the_fabric_cbf_source_only_in_its_mode():
    guard = Guard()
    guard.estop_update(False, 1.); guard.deadman_update(False, 1.); guard.deadman_update(True, 1.01)
    guard.mode_update('fabric_cbf', 1.01)
    guard.command('fabric_cbf', (.05, 0., 0., 0., 0., 0.), 1.02, .1, .3)
    guard.command('teleop', (0., .08, 0., 0., 0., 0.), 1.02, .1, .3)
    assert guard.output(1.03)[0] == pytest.approx((.05, 0., 0., 0., 0., 0.))
    guard.mode_update('teleop', 1.04)  # a mode change clears every stored command
    assert not any(guard.output(1.05)[0])
    guard.mode_update('nonsense', 1.06)
    assert guard.mode == 'disabled'


def test_arm_without_wrist_sensor_runs_on_pose_alone_when_force_is_not_required():
    bridge = CbfBridge(require_force=False, workspace_min=(-1, -1, -1), workspace_max=(1, 1, 1))
    identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
    bridge.pose_update((0, 0, 0), identity, 0.0)
    assert bridge.rearm(0.0)
    bridge.command_update((0.05, 0, 0), 0.0, 0.0)
    assert bridge.output(0.05)[:3] == (0.05, 0, 0)
    bridge.pose_update((2, 0, 0), identity, 0.06)
    assert bridge.tripped == 'tool left the workspace box'
    assert bridge.output(0.07) == (0.,)*6
    # The default still refuses to arm without a wrist force.
    strict = CbfBridge()
    strict.pose_update((0, 0, 0), identity, 0.0)
    assert not strict.rearm(0.0) and strict.tripped == 'cannot arm: no fresh wrist force'
