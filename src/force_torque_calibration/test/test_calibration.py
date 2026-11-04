import numpy as np
import pytest
from geometry_msgs.msg import WrenchStamped
from force_torque_calibration.main import get_avg_force_torque
from force_torque_calibration.calibration_math import design_matrix, solve_calibration


@pytest.mark.parametrize('count', [1, 2, 3, 100])
def test_blue_mean_has_one_axis_rotation(count):
    message = WrenchStamped()
    message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = 1., 2., 3.
    message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = 4., 5., 6.
    mean = get_avg_force_torque([message]*count, 'oarbot_blue/')
    assert np.allclose([mean.wrench.force.x,mean.wrench.force.y,mean.wrench.force.z], [1,-2,-3])
    assert np.allclose([mean.wrench.torque.x,mean.wrench.torque.y,mean.wrench.torque.z], [4,-5,-6])


def test_recovers_known_payload_and_bias():
    mass = 1.2
    com = np.array([-.04, .01, -.05])
    force_bias, torque_bias = np.array([1.,2.,3.]), np.array([.1,.2,.3])
    gravity = np.array([[9.8,0,0],[-9.8,0,0],[0,9.8,0],[0,-9.8,0],[0,0,9.8],[0,0,-9.8]])
    matrix = np.vstack([design_matrix(g) for g in gravity])
    observations = np.concatenate([np.r_[mass*g+force_bias, np.cross(com,mass*g)+torque_bias] for g in gravity])
    result, rms = solve_calibration(matrix, observations)
    assert result['end_effector_mass'] == pytest.approx(mass)
    assert result['center_of_mass_vector'] == pytest.approx(com)
    assert result['force_bias_vector'] == pytest.approx(force_bias)
    assert result['torque_bias_vector'] == pytest.approx(torque_bias)
    assert rms < 1e-12


def test_rejects_unobservable_calibration():
    matrix = np.vstack([design_matrix([0,0,-9.8])]*10)
    with pytest.raises(ValueError, match='excite'):
        solve_calibration(matrix, np.zeros(60))
