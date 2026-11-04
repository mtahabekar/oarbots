import math
import pytest
from oarbot_runtime.guard import Guard, limited


def enable(g, time=1.0):
    g.estop_update(False, time)
    g.deadman_update(False, time)
    g.deadman_update(True, time + .01)
    g.mode_update('teleop', time + .01)
    g.base_update(True, time + .01)


def test_startup_and_held_deadman_cannot_arm():
    g = Guard()
    g.estop_update(False, 1.)
    g.deadman_update(True, 1.)
    assert not g.permitted(1.)
    assert not any(g.output(1.)[0])


def test_release_press_then_command_and_timeout():
    g = Guard()
    enable(g)
    g.command('teleop', (1.,0.,0.,0.,0.,0.), 1.02, .1, .3)
    assert g.output(1.03)[0][0] == pytest.approx(.1)
    assert not any(g.output(1.2)[0])
    assert not g.permitted(1.4)
    g.estop_update(False, 1.41)
    g.deadman_update(True, 1.41)
    assert not g.permitted(1.41)  # restoring connectivity cannot restart a held command
    enable(g, 1.5)
    assert g.permitted(1.52)
    assert not any(g.output(1.52)[0])


def test_stop_and_mode_switch_clear_commands():
    g = Guard()
    enable(g)
    g.command('teleop', (.1,0.,0.,0.,0.,0.), 1.02, .1, .3)
    g.mode_update('admittance', 1.03)
    assert not any(g.output(1.04)[0])
    g.mode_update('teleop', 1.05)
    assert not any(g.output(1.06)[0])
    g.estop_update(True, 1.07)
    assert not g.permitted(1.08)
    g.estop_update(False, 1.09)
    assert not g.permitted(1.10)


def test_stale_mode_and_base_lease_stop_independently():
    g = Guard()
    enable(g)
    g.deadman_update(True, 1.3)
    g.estop_update(False, 1.3)
    g.command('teleop', (.1,0.,0.,0.,0.,0.), 1.3, .1, .3)
    g.command('base', (.1,0.,0.,0.,0.,0.), 1.3, .15, .3)
    assert not any(g.output(1.31)[0])
    assert not any(g.output(1.31)[1])


def test_nonfinite_command_latches_stop():
    g = Guard()
    enable(g)
    g.command('teleop', (math.nan,0.,0.,0.,0.,0.), 1.02, .1, .3)
    assert not g.permitted(1.03)


def test_vector_norm_limits():
    values = limited((3.,4.,0.,0.,0.,2.), .1, .3)
    assert math.hypot(*values[:2]) == pytest.approx(.1)
    assert values[5] == pytest.approx(.3)


@pytest.mark.parametrize('value', [0, -1, math.nan, math.inf])
def test_invalid_timeout(value):
    with pytest.raises(ValueError):
        Guard(timeout=value)
