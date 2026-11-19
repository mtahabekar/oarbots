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
    for i in range(30):  # a sustained 14 N change: allowed
        bridge.force_update((0., 0., -17.), 1.01 + i*.01)
        bridge.pose_update((0., 0., 1.), IDENTITY, 1.01 + i*.01)
    bridge.command_update((.05, 0., 0.), 0., 1.3)
    assert bridge.output(1.3)[0] > 0
    for i in range(30):  # the 0.25 s mean moves past 15 N
        bridge.force_update((10., 0., -17.), 1.31 + i*.01)
        bridge.pose_update((0., 0., 1.), IDENTITY, 1.31 + i*.01)
    assert 'wrist force' in bridge.tripped and not any(bridge.output(1.61))
    for i in range(30):  # the load going away does not re-arm
        bridge.force_update((0., 0., -3.), 1.62 + i*.01)
        bridge.pose_update((0., 0., 1.), IDENTITY, 1.62 + i*.01)
    bridge.command_update((.05, 0., 0.), 0., 1.91)
    assert not any(bridge.output(1.92))
    bridge.pose_update((0., 0., 1.), IDENTITY, 1.92)
    assert bridge.rearm(1.92) and bridge.output(1.93)[0] > 0

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


def test_force_spike_shorter_than_the_window_does_not_stop_the_arm():
    bridge = armed(force_limit_n=15.)
    for i in range(1, 25):
        bridge.force_update((0., 0., -3.), 1. + i*.01)
        bridge.pose_update((0., 0., 1.), IDENTITY, 1. + i*.01)
    bridge.force_update((0., 0., -23.), 1.25)  # one 20 N sample as the arm jerks into motion
    assert not bridge.tripped


def test_soft_workspace_edge_holds_outward_motion_and_stops_beyond_the_margin():
    bridge = armed(workspace_min=(-.5, -.5, .5), workspace_max=(.5, .5, 1.), workspace_margin=.02)
    bridge.command_update((.05, 0., .05), 0., 1.)
    assert bridge.output(1.01)[:3] == pytest.approx((.05, 0., 0.))   # on the top face: up is held
    bridge.command_update((0., 0., -.05), 0., 1.02)
    assert bridge.output(1.02)[2] == pytest.approx(-.05)            # back in is allowed
    bridge.pose_update((0., 0., 1.015), IDENTITY, 1.03)               # past the face, inside the margin
    assert not bridge.tripped
    bridge.pose_update((0., 0., 1.03), IDENTITY, 1.04)                # beyond the margin
    assert 'workspace' in bridge.tripped and not any(bridge.output(1.05))


def test_wall_only_box_never_stops_and_arms_outside():
    bridge = armed(workspace_min=(-.5, -.5, .5), workspace_max=(.5, .5, 1.5), workspace_stop=False)
    bridge.pose_update((.9, 0., 1.), IDENTITY, 1.01)          # 40 cm beyond the face: no stop
    bridge.force_update((0., 0., -3.), 1.01)
    assert not bridge.tripped
    bridge.command_update((.05, .03, 0.), 0., 1.01)
    out = bridge.output(1.01)
    assert out[0] == 0. and out[1] == pytest.approx(.03)       # outward held, the rest passes
    bridge.command_update((-.05, 0., 0.), 0., 1.02)
    assert bridge.output(1.02)[0] == pytest.approx(-.05)       # back toward the box passes
    bridge.disarm()
    assert bridge.rearm(1.02)                                  # arming outside the box is allowed


def test_predictive_wall_slows_outward_motion_near_the_face():
    bridge = armed(workspace_min=(-.5, -.5, .5), workspace_max=(.5, .5, 1.5), workspace_stop=False, workspace_gain=2.)
    bridge.pose_update((.45, 0., 1.), IDENTITY, 1.01)
    bridge.force_update((0., 0., -3.), 1.01)
    bridge.command_update((.2, 0., 0.), 0., 1.01)
    assert bridge.output(1.01)[0] == pytest.approx(.1)         # 2/s x 5 cm left
    bridge.command_update((-.2, 0., 0.), 0., 1.01)
    assert bridge.output(1.01)[0] == pytest.approx(-.1)        # inward is limited only by max_speed (0.1)


def test_force_limit_zero_reads_force_but_never_stops():
    bridge = armed(force_limit_n=0.)
    bridge.force_update((0., 0., 60.), 1.01)
    assert not bridge.tripped and bridge.force_change() == pytest.approx(31.5)
