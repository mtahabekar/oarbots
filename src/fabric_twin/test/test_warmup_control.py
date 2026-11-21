"""Warm-up command boundaries and operator sequencing, with no ROS graph or arm motion."""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('newton')
from builtin_interfaces.msg import Time  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from fabric_twin.node import FabricTwinNode  # noqa: E402
from fabric_twin.warmup import WarmupScript  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'scripts/hw'))
import warmup as operator  # noqa: E402
import warmup_auto as automatic  # noqa: E402


class Publisher:
    def __init__(self, receive):
        self.publish = receive


def controller(only=True):
    script = WarmupScript()
    sent, windows = [], []
    node = SimpleNamespace(
        warmup=script, warmup_only=only, direct_warmup=False, warmup_stop_after=only, warmup_announced=True,
        commands_enabled=True, command=np.full((2, 3), .08), fault='', cbf=object(),
        runtime={'world_frame': 'world'}, shadow_pubs=[Publisher(lambda m: None) for _ in range(2)],
        command_pubs=[Publisher(lambda m: sent.append([m.twist.linear.x, m.twist.linear.y, m.twist.linear.z]))
                      for _ in range(2)],
        window_pub=Publisher(lambda m: windows.append((node.commands_enabled, np.array(sent[-2:]), m))),
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=Time)),
        get_logger=lambda: SimpleNamespace(info=lambda m: None, warn=lambda m: None),
        warmup_geometry_problem=lambda: '',
    )
    node.publish_commands = lambda time_ns: FabricTwinNode.publish_commands(node, time_ns)
    corners = np.array([[0., 0., 1.], [.84, 0., 1.], [0., .84, 1.], [.84, .84, 1.]])
    node.cloth = SimpleNamespace(corners=lambda: corners)
    node.twin_time = 1_000_000_000
    return node, sent, windows


def test_warmup_only_never_sends_cached_follow_commands_outside_the_trajectory():
    node, sent, _ = controller()
    node.publish_commands(node.twin_time)  # enabled, waiting for the deadman/start
    np.testing.assert_array_equal(sent, np.zeros((2, 3)))
    FabricTwinNode.start_warmup(node, String(data='warmup_auto'))
    node.command[:] = .03
    node.publish_commands(node.twin_time + 1_000_000_000)
    np.testing.assert_array_equal(sent[-2:], np.full((2, 3), .03))
    node.publish_commands(node.twin_time + int((node.warmup.duration + .01) * 1e9))
    np.testing.assert_array_equal(sent[-2:], np.zeros((2, 3)))


@pytest.mark.parametrize('only', [False, True])
def test_manual_completion_stops_before_announcing_the_bank_window(only):
    node, sent, windows = controller(only)
    FabricTwinNode.start_warmup(node, String(data='operator'))
    node.command[:] = .15
    end = node.twin_time + int((node.warmup.duration + .01) * 1e9)
    FabricTwinNode.finish_warmup(node, end)
    assert not node.commands_enabled
    np.testing.assert_array_equal(node.command, np.zeros((2, 3)))
    assert len(windows) == 1 and windows[0][0] is False
    np.testing.assert_array_equal(windows[0][1], np.zeros((2, 3)))
    FabricTwinNode.finish_warmup(node, end + 1)
    assert len(windows) == 1
    # Even a subsequent CBF update cannot re-enable physical output.
    node.command[:] = .15
    node.publish_commands(end)
    np.testing.assert_array_equal(sent[-2:], np.zeros((2, 3)))


def test_scheduled_carry_keeps_its_explicit_following_behavior():
    node, sent, windows = controller(False)
    FabricTwinNode.start_warmup(node, String(data='scheduled'))
    node.command[:] = .03
    end = node.twin_time + int((node.warmup.duration + .01) * 1e9)
    FabricTwinNode.finish_warmup(node, end)
    node.publish_commands(end)
    assert node.commands_enabled and len(windows) == 1
    np.testing.assert_array_equal(sent[-2:], np.full((2, 3), .03))


def test_operator_disable_clears_and_publishes_zero_immediately():
    from std_msgs.msg import Bool
    node, sent, _ = controller(False)
    FabricTwinNode.enable(node, Bool(data=False))
    assert not node.commands_enabled
    np.testing.assert_array_equal(sent, np.zeros((2, 3)))


@pytest.mark.parametrize(('direct', 'phase'), [(False, 'ended'), (True, 'ended'), (True, 'settle'), (True, 'ramp')])
def test_frame_loop_uses_only_the_requested_controller_and_stops_at_completion(direct, phase):
    node, sent, windows = controller()
    FabricTwinNode.start_warmup(node, String(data='warmup_auto'))
    corners = node.cloth.corners()
    node.cloth.targets = corners
    node.cloth.to_world = lambda p: p
    node.cloth.within_reach = lambda p: p
    node.cloth.step = lambda p: None
    node.cloth.state = lambda: (corners, np.zeros_like(corners))
    node.cloth.obstacle_clearance = lambda: 1.
    node.grips_at = lambda t: (corners.copy(), np.zeros((2, 3)))
    node.runtime.update(leader_smoothing_sec=.1, leader_speed_limit_mps=.5, collision_clearance_m=0.,
                        estimate_stale_sec=1., warmup_at_sec=-1., state_every_frames=1)
    node.spec = SimpleNamespace(frame_dt=1/60)
    end = node.twin_time + int((node.warmup.duration + .01) * 1e9)
    when = end if phase == 'ended' else node.twin_time + int((.5 if phase == 'settle' else 3.2)*1e9)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=when, to_msg=Time))
    node.estimate_received = when
    node.frame, node.stale_grips = 0, 0
    node.history, node.cbf_ms, node.frame_ms = [], [], []
    node.leader_velocity = np.zeros((2, 3))
    node.apply_estimates = lambda: None
    node.finish_warmup = lambda t: FabricTwinNode.finish_warmup(node, t)
    node.follow_command = lambda: pytest.fail('warm-up-only mode must not call hand following')
    node.cbf_options = {'period': 1}
    node.cbf = SimpleNamespace(update=lambda *args: (np.full((2, 3), .15), {}), applied=lambda *args: None)
    if direct:
        node.cbf, node.direct_warmup = None, True
    node.guarded = lambda command: (command, {})
    node.publish_debug = lambda *args: None
    node.publish_corners = node.publish_state = node.publish_markers = lambda *args: None
    FabricTwinNode.step(node, when)
    if phase == 'ended':
        assert not node.commands_enabled and len(windows) == 1
        np.testing.assert_array_equal(sent, np.zeros((4, 3)))
    else:
        assert node.commands_enabled and not windows
        expected = node.warmup.command(when, corners[:2])
        np.testing.assert_array_equal(sent, expected)
        assert np.max(np.linalg.norm(expected, axis=1)) == pytest.approx(0. if phase == 'settle' else .039)


def test_automatic_runner_disarms_before_end_speech(monkeypatch):
    events = []
    node = SimpleNamespace(deadman=False, window=None, bank=[], cmd={'blue': 2., 'silver': 2.}, started=False, twin={})
    node.start_pub = Publisher(lambda m: setattr(node, 'started', True))

    def spin(seconds, until=lambda: False):
        node.deadman = True  # deliberate press after the start prompt (toggle mode)
        if node.started:
            node.window = {'holds': [(0, 1, 0)]}
        return until()

    node.spin = spin
    node.commands = lambda on: events.append(('commands', on))
    monkeypatch.setattr(automatic, 'say', lambda text, **kwargs: events.append(('speech', text)))
    monkeypatch.setattr(automatic, 'log', lambda text: None)
    automatic.excite(node)
    stopped = events.index(('commands', False))
    released = next(i for i, (kind, text) in enumerate(events) if kind == 'speech' and text.startswith('Warm-up motion done.'))
    assert stopped < released
    assert sum(kind == 'commands' for kind, _ in events) == 1


@pytest.mark.parametrize(('index', 'attempt'), [(1, 1), (2, 1), (1, 2)])
def test_every_run_and_retry_rejects_an_incorrect_start_before_launch(monkeypatch, index, attempt):
    events = []
    node = SimpleNamespace(commands=lambda on: events.append(('commands', on)))

    def wrong_start(restore=False):
        events.append(('check', False))
        raise automatic.Abort('off start pose')

    monkeypatch.setattr(automatic, 'check_start_pose', wrong_start)
    monkeypatch.setattr(automatic, 'reusable', lambda *args: pytest.fail('must check before reusing'))
    monkeypatch.setattr(automatic.subprocess, 'Popen', lambda *args, **kwargs: pytest.fail('must check before launch'))
    with pytest.raises(automatic.Abort, match='off start pose'):
        automatic.one_run(node, '10 0.1', index, 2, attempt)
    assert events == [('commands', False), ('check', False)]


def test_automatic_enable_rejects_an_old_or_following_twin():
    node = SimpleNamespace(twin={'commands_enabled': 'False'})
    with pytest.raises(automatic.Abort, match='warm-up-only mode'):
        automatic.enable(node)


def test_manual_operator_disarms_even_on_interruption(monkeypatch):
    stopped = []
    node = SimpleNamespace(stop_commands=lambda: stopped.append(True))

    def interrupted(*args):
        raise KeyboardInterrupt

    monkeypatch.setattr(operator, 'run_warmup', interrupted)
    with pytest.raises(KeyboardInterrupt):
        operator.cmd_start(node, False, 1)
    assert stopped == [True]


def test_automatic_run_checks_pose_before_arming_and_before_fitting(monkeypatch):
    events = []
    node = SimpleNamespace(commands=lambda on: None, twin={}, spin=lambda seconds: None)
    monkeypatch.setattr(automatic, 'check_start_pose', lambda restore=False: events.append('restore' if restore else 'pose'))
    monkeypatch.setattr(automatic, 'reusable', lambda *args: Path('/tmp/fake_warmup_test'))
    monkeypatch.setattr(automatic, 'REACH_SEC', 0.)
    monkeypatch.setattr(automatic, 'say', lambda *args, **kwargs: None)
    monkeypatch.setattr(automatic, 'log', lambda *args: None)
    monkeypatch.setattr(automatic, 'wait_ready', lambda *args: events.append('ready'))
    monkeypatch.setattr(automatic, 'enable', lambda *args: events.append('enable'))
    monkeypatch.setattr(automatic, 'excite', lambda *args: events.append('excite'))
    monkeypatch.setattr(automatic, 'fit', lambda *args: events.append('fit'))
    monkeypatch.setattr(automatic, 'finish_run', lambda *args: events.append('saved'))
    automatic.one_run(node, '10 0.1', 2, 2)
    assert events == ['restore', 'ready', 'pose', 'enable', 'excite', 'pose', 'fit', 'saved', 'pose']


def test_warmup_rejects_the_original_wide_origin_and_accepts_the_narrow_pose():
    from fabric_twin.stretch import StretchGuard, StretchSpec
    node, _, _ = controller()
    node.guard = StretchGuard(.84, .84, StretchSpec(edge_slack_m=.035, diagonal_slack_m=.035))
    robots = np.array([[-.4212, 0, 1.05], [.4212, 0, 1.05]])
    hands = np.array([[-.35, .7, 1.05], [.35, .7, 1.05]])
    node.measured_now = lambda: (robots, hands, np.zeros((2, 3)))
    problem = FabricTwinNode.warmup_geometry_problem(node)
    assert 'robot-to-robot' in problem and '0.842' in problem and '0.805' in problem
    robots[:, 0] = [-.385, .385]
    assert FabricTwinNode.warmup_geometry_problem(node) == ''


def test_incompatible_geometry_is_rejected_by_both_enable_and_start():
    from std_msgs.msg import Bool
    node, sent, windows = controller()
    node.warmup_geometry_problem = lambda: 'robot-to-robot grip span exceeds stretch-guard limit'
    node.get_logger = lambda: SimpleNamespace(error=lambda m: None)
    FabricTwinNode.enable(node, Bool(data=True))
    FabricTwinNode.start_warmup(node, String(data='warmup_auto'))
    assert not node.commands_enabled and node.warmup.start_ns is None and not windows
    np.testing.assert_array_equal(sent, np.zeros((4, 3)))


def test_automatic_runner_checks_the_dedicated_pose_and_only_restores_when_requested(monkeypatch):
    calls = []
    monkeypatch.setattr(automatic.subprocess, 'run', lambda command, **kwargs: (
        calls.append(command) or SimpleNamespace(returncode=0)))
    automatic.check_start_pose(restore=True)
    automatic.check_start_pose()
    assert all(str(automatic.WARMUP_POSE) in command for command in calls)
    assert '--check' not in calls[0] and '--check' in calls[1]


def test_automatic_enable_rejects_guard_conflicts_before_publishing():
    node = SimpleNamespace(twin={'warmup_only': 'True', 'warmup_start_problem': 'robot span exceeds limit'})
    with pytest.raises(automatic.Abort, match='robot span exceeds limit'):
        automatic.enable(node)


def test_automatic_runner_restarts_a_twin_without_the_geometry_check(monkeypatch, tmp_path):
    import json
    run = tmp_path / 'warmup_example'
    run.mkdir()
    (run / 'material_start.txt').write_text('bend 10 shear 0.1')
    state = tmp_path / 'fabric_lab'
    state.mkdir()
    (state / 'last_run').write_text(str(run))
    monkeypatch.setenv('XDG_RUNTIME_DIR', str(tmp_path))
    monkeypatch.setattr(automatic, 'twin_running', lambda: True)
    status = dict(warmup_only='True', warmup='idle', material=json.dumps(dict(version=0, bend=10., shear=.1)),
                  cbf_enabled='False', stretch_guard_enabled='False', command_mode='warmup_direct')
    node = SimpleNamespace(twin={}, get_node_names=lambda: ['fabric_parameter_bank'])
    node.spin = lambda *args: setattr(node, 'twin', dict(status))
    assert automatic.reusable(node, (10., .1)) is None
    status['warmup_start_problem'] = ''
    assert automatic.reusable(node, (10., .1)) is None
    status['warmup_protocol_version'] = '3'
    assert automatic.reusable(node, (10., .1)) == run


def test_direct_warmup_moves_without_a_cbf_and_stops_on_fault_or_outside_trajectory():
    node, sent, _ = controller()
    node.cbf, node.direct_warmup = None, True
    node.publish_commands(node.twin_time)
    np.testing.assert_array_equal(sent[-2:], np.zeros((2, 3)))
    FabricTwinNode.start_warmup(node, String(data='warmup_auto'))
    node.command[:] = .03
    node.publish_commands(node.twin_time + 1_000_000_000)
    np.testing.assert_array_equal(sent[-2:], np.full((2, 3), .03))
    node.fault = 'a grip measurement is missing'
    node.publish_commands(node.twin_time + 2_000_000_000)
    np.testing.assert_array_equal(sent[-2:], np.zeros((2, 3)))
    node.fault = ''
    node.publish_commands(node.twin_time + int((node.warmup.duration + .01)*1e9))
    np.testing.assert_array_equal(sent[-2:], np.zeros((2, 3)))


def test_disabled_stretch_guard_does_not_modify_zero_or_nonzero_commands():
    node, _, _ = controller()
    node.guard = SimpleNamespace(spec=SimpleNamespace(enabled=False))
    node.measured_now = lambda: pytest.fail('disabled guard must not consume grip data or run its filter')
    for command in (np.zeros((2, 3)), np.full((2, 3), .03)):
        actual, info = FabricTwinNode.guarded(node, command)
        np.testing.assert_array_equal(actual, command)
        assert info == {'enabled': False}


def test_disabling_cbf_does_not_enable_unfiltered_normal_following():
    node, sent, _ = controller(False)
    node.cbf, node.direct_warmup = None, False
    node.publish_commands(node.twin_time)
    np.testing.assert_array_equal(sent, np.zeros((2, 3)))


@pytest.mark.parametrize(('cbf', 'stretch', 'mode'), [
    ('True', 'False', 'warmup_direct'), ('False', 'True', 'warmup_direct'), ('False', 'False', 'cbf'),
])
def test_operator_rejects_a_warmup_with_either_filter_still_active(cbf, stretch, mode):
    node = SimpleNamespace(twin=dict(warmup_only='True', warmup_start_problem='', cbf_enabled=cbf,
                                     stretch_guard_enabled=stretch, command_mode=mode))
    with pytest.raises(automatic.Abort, match='CBF off and stretch guard off'):
        automatic.enable(node)


@pytest.mark.parametrize('converged', [True, False])
def test_operator_finishes_immediately_when_the_search_result_is_already_available(monkeypatch, converged):
    result = dict(bend=1., shear=1., start_cost_m=.02, cost_m=.01, sensitivity_m=.01,
                  cases=4, seconds=12., informative=True, converged=converged,
                  stop_reason='converged' if converged else 'search_edge', rounds_completed=2,
                  evaluations=30, release_expected=converged)
    node = SimpleNamespace(bank=[dict(message='calibrated' if converged else 'calibration stopped', result=result)],
                           release={'source': 'warmup', 'version': 1} if converged else None)
    # Use the real wait implementation: with the completion predicate already true,
    # rclpy.spin_once must never be reached and no 5-second/3-second delay is imposed.
    node.spin = lambda seconds, until=lambda: False: operator.Warmup.spin(node, seconds, until)
    monkeypatch.setattr(operator.rclpy, 'spin_once', lambda *args, **kwargs: pytest.fail('completed fit must not wait'))
    monkeypatch.setattr(automatic, 'say', lambda *args, **kwargs: None)
    monkeypatch.setattr(automatic, 'log', lambda *args: None)
    actual = automatic.fit(node, (1., 1.))
    assert actual['converged'] is converged
    assert actual['released'] is converged


def test_every_hold_target_is_checked_against_stretch_even_with_the_guard_off():
    node, _, _ = controller()
    robots = np.array([[0., 0., 1.], [.84, 0., 1.]])
    hands = np.array([[0., .84, 1.], [.84, .84, 1.]])
    node.measured_now = lambda: (robots, hands, np.zeros((2, 3)))
    start = float(np.linalg.norm(robots[1]-hands[0]))
    node.guard = SimpleNamespace(spec=SimpleNamespace(enabled=False), pairs=[(1, 2, start)])   # diagonal at its limit
    assert FabricTwinNode.warmup_geometry_problem(node) == ''        # insets only ever shorten the pairs
    node.warmup = WarmupScript(insets_m=(.01,), lateral_m=.08)
    problem = FabricTwinNode.warmup_geometry_problem(node)
    assert 'shift toward Silver' in problem and 'Silver to right hand' in problem
