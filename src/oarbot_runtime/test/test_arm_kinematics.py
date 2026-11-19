import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from oarbot_runtime.arm_kinematics import Chain, joint_rates, rotation_vector, scale_to_limit

ARM = """<robot name="arm">
  <link name="base"/><link name="a"/><link name="b"/><link name="c"/><link name="tip"/>
  <joint name="j1" type="continuous"><parent link="base"/><child link="a"/><axis xyz="0 0 1"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/><origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="j3" type="revolute"><parent link="b"/><child link="c"/><origin xyz="0.4 0 0" rpy="0.3 0 0"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="1" velocity="1"/></joint>
  <joint name="fixed_tip" type="fixed"><parent link="c"/><child link="tip"/><origin xyz="0.3 0.05 0"/></joint>
</robot>"""


def numeric_jacobian(chain, q, eps=1e-6):
    T0 = chain.fk(q)
    cols = []
    for i in range(len(q)):
        dq = np.array(q, float); dq[i] += eps
        T1 = chain.fk(dq)
        cols.append(np.r_[(T1[:3, 3] - T0[:3, 3]) / eps, rotation_vector(T1[:3, :3] @ T0[:3, :3].T) / eps])
    return np.array(cols).T


def test_jacobian_matches_finite_differences():
    chain = Chain(ARM, 'base', 'tip')
    assert chain.names == ['j1', 'j2', 'j3']
    q = [0.4, -0.7, 1.1]
    J, _ = chain.jacobian(q)
    assert J == pytest.approx(numeric_jacobian(chain, q), abs=1e-5)


def test_tip_offset_moves_the_controlled_point():
    chain = Chain(ARM, 'base', 'tip')
    offset = np.eye(4); offset[:3, 3] = [0.05, 0., -0.02]
    q = [0.2, 0.3, -0.4]
    T, Tc = chain.fk(q), chain.fk(q, offset)
    assert Tc[:3, 3] == pytest.approx(T[:3, 3] + T[:3, :3] @ offset[:3, 3])


def test_rates_reproduce_a_feasible_twist_and_scale_together():
    chain = Chain(ARM, 'base', 'tip')
    J, _ = chain.jacobian([0.4, -0.7, 1.1])
    rates = np.array([0.1, -0.2, 0.05])
    twist = J @ rates                       # reachable by construction
    assert joint_rates(J, twist, damping=1e-6) == pytest.approx(rates, abs=1e-5)
    scaled = scale_to_limit(np.array([0.3, -0.6, 0.1]), 0.2)
    assert np.max(np.abs(scaled)) == pytest.approx(0.2) and scaled / scaled[0] == pytest.approx([1., -2., 1. / 3])


def jaco_urdf():
    if shutil.which('xacro') is None:
        pytest.skip('xacro not available')
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('oarbot_description'))
    except Exception:
        pytest.skip('oarbot_description not installed')
    xacro_file = share / 'urdf' / 'kinova' / 'j2n6s300_with_force_torque_standalone.xacro'
    return subprocess.run(['xacro', str(xacro_file)], check=True, capture_output=True, text=True).stdout


@pytest.mark.parametrize('q', [
    [-0.5157, 2.9269, 4.7380, 1.5188, 4.8584, 1.8635],    # Blue's grasp joints (grasp.yaml)
    [0.5157, 3.3563, 1.5452, 4.7644, 1.4248, -1.8635]])   # Silver's
def test_jaco_resolved_rate_moves_straight_and_holds_orientation(q):
    """2 cm along x at the grasp joints, 100 Hz, with orientation hold: straight line, tool turns < 0.1 deg."""
    chain = Chain(jaco_urdf(), 'j2n6s300_link_base', 'j2n6s300_end_effector')
    q = np.array(q, float)
    T0 = chain.fk(q)
    v = np.array([0.02, 0., 0.])           # 2 cm/s for 1 s
    for _ in range(100):
        J, T = chain.jacobian(q)
        hold = 1.0 * rotation_vector(T0[:3, :3] @ T[:3, :3].T)
        q = q + 0.01 * joint_rates(J, np.r_[v, hold])
    T1 = chain.fk(q)
    moved = T1[:3, 3] - T0[:3, 3]
    assert moved == pytest.approx([0.02, 0., 0.], abs=5e-4)
    turn = np.degrees(np.linalg.norm(rotation_vector(T1[:3, :3] @ T0[:3, :3].T)))
    assert turn < 0.1


def test_grip_rates_hold_orientation_and_respect_the_rate_limit():
    from oarbot_runtime.arm_kinematics import GripRates
    chain = Chain(ARM, 'base', 'tip')
    grip = GripRates(chain, max_rate=0.5, gain=1.0)
    q = np.array([0.4, -0.7, 1.1])
    r = grip.rates(q, [0., 0., 0.])
    assert r == pytest.approx(np.zeros(3), abs=1e-9)          # no command, no error: no motion
    fast = grip.rates(q, [2., 0., 0.])                         # far above what 0.5 rad/s allows
    assert np.max(np.abs(fast)) == pytest.approx(0.5)
    grip.reset()
    assert grip.orientation_error(q) == 0.
