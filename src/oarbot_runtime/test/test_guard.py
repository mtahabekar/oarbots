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



def test_fingers_held_closed_only_while_permitted():
    assert Guard().finger_output(1.) is None            # off by default: the fingers are never commanded
    g = Guard(hold_fingers=100.)
    g.estop_update(False, 1.)
    g.deadman_update(True, 1.)
    assert g.finger_output(1.) is None                  # held deadman at start-up cannot arm
    enable(g)
    assert g.finger_output(1.02) is None                # deadman held for the other robot (not in fabric mode)
    g.mode_update('fabric_cbf', 1.02)
    assert g.finger_output(1.02) == pytest.approx(100.)
    assert not any(g.output(1.02)[0])                   # holding the fingers never moves the arm
    g.deadman_update(False, 1.03)
    assert g.finger_output(1.03) is None                # released: the fingers stay where they are
    g.estop_update(True, 1.04)
    assert g.finger_output(1.04) is None


def test_hold_fingers_must_be_a_percentage():
    for bad in (-1., 101., math.nan):
        with pytest.raises(ValueError):
            Guard(hold_fingers=bad)


def enable_cbf(g, time=1.0):
    enable(g, time)
    g.mode_update('fabric_cbf', time + .01)


def test_joint_moves_need_fabric_mode_deadman_and_fresh_commands():
    g = Guard()
    enable(g)                                            # teleop mode
    g.joint_command((10.,0.,0.,0.,0.,0.), 1.02, 15.)
    assert g.joint_output(1.02) is None
    enable_cbf(g, 1.1)
    g.joint_command((30.,-15.,0.,0.,0.,0.), 1.12, 15.)
    assert g.joint_output(1.12) == pytest.approx((15., -7.5, 0., 0., 0., 0.))   # scaled, direction kept
    assert g.joint_output(1.3) is None                  # lapses
    g.joint_command((5.,0.,0.,0.,0.,0.), 1.3, 15.)
    g.deadman_update(False, 1.31)
    assert g.joint_output(1.31) is None                 # released: stops, does not resume by itself
    g.deadman_update(True, 1.32)
    assert g.joint_output(1.32) is None


def test_joint_and_cartesian_commands_together_trip():
    g = Guard()
    enable_cbf(g)
    g.command('fabric_cbf', (.02,0.,0.,0.,0.,0.), 1.02, .1, .3)
    g.joint_command((5.,0.,0.,0.,0.,0.), 1.02, 15.)
    assert g.joint_output(1.03) is None
    assert not g.permitted(1.03)


def test_invalid_joint_command_trips():
    g = Guard()
    enable_cbf(g)
    g.joint_command((math.nan,0.,0.,0.,0.,0.), 1.02, 15.)
    assert not g.permitted(1.03)
