import rclpy
import time
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
import numpy as np
import json
from pathlib import Path
from force_torque_calibration.calibration_math import solve_calibration

from force_torque_calibration.force_torque_calibration_node import ForceTorqueCalibration

def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ForceTorqueCalibration()
        run_calibration(node)
    except Exception:
        if node is not None:
            node.stop_arm()
        raise
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


def run_calibration(node) -> None:
    matrix_stack = np.zeros((0, 10))
    vector_stack = np.zeros((0, 1))

    node.get_logger().warn("-----------WARNING-----------")
    node.get_logger().warn("Unplug the Azure Kinect camera (this requires a restart of the node after running this calibration) and remove the both the Azure Kinect and force torque cables from the Zip-Tie")
    node.get_logger().warn("-----------------------------")

    node.get_logger().info("Continuing in 5 seconds...")
    node.settle(5.0)

    for i, joint_angles in enumerate(node.joint_angles_list):
        node.get_logger().info(f"Moving arm to position {i + 1} of {len(node.joint_angles_list)}")
        node.move_arm_to_state(joint_angles)

        node.get_logger().info("Waiting for arm to finish shaking")
        node.settle(5.0)

        node.get_logger().info("Collecting force torque data")
        node.collect_force_torque_data()
        avg_force_torque = get_avg_force_torque(node.force_torque_data, node.tf_prefix)
        
        node.get_logger().debug(str(avg_force_torque))

        node.get_logger().info("Getting gravity in force torque frame")
        gravity_vector = Vector3Stamped()
        for i in range(10):
            try:
                gravity_vector = node.get_gravity_vector()
            except LookupError as ex:
                node.get_logger().info("Trying to get transform again")
                node.settle(0.5)
                continue

            break

        if gravity_vector == Vector3Stamped():
            raise LookupError("Failed to get transform 10 times; exiting")
        
        node.get_logger().debug(str(gravity_vector))

        node.get_logger().info("Adding data to matrix")

        # Fill in the matrix with values 
        matrix = np.zeros(shape=(6, 10))
        matrix[0, 0] = gravity_vector.vector.x
        matrix[1, 0] = gravity_vector.vector.y
        matrix[2, 0] = gravity_vector.vector.z

        matrix[4, 3] = gravity_vector.vector.x
        matrix[5, 2] = -gravity_vector.vector.x
        matrix[5, 1] = gravity_vector.vector.y
        matrix[3, 3] = -gravity_vector.vector.y
        matrix[3, 2] = gravity_vector.vector.z
        matrix[4, 1] = -gravity_vector.vector.z

        matrix[0, 4] = 1
        matrix[1, 5] = 1
        matrix[2, 6] = 1
        matrix[3, 7] = 1
        matrix[4, 8] = 1
        matrix[5, 9] = 1

        vector = np.zeros((6, 1))
        vector[0, 0] = avg_force_torque.wrench.force.x
        vector[1, 0] = avg_force_torque.wrench.force.y
        vector[2, 0] = avg_force_torque.wrench.force.z
        vector[3, 0] = avg_force_torque.wrench.torque.x
        vector[4, 0] = avg_force_torque.wrench.torque.y
        vector[5, 0] = avg_force_torque.wrench.torque.z

        matrix_stack = np.vstack((matrix_stack, matrix))
        vector_stack = np.vstack((vector_stack, vector))

        node.get_logger().debug(str(matrix))
        node.get_logger().debug(str(vector))

    node.get_logger().debug(str(matrix_stack))
    node.get_logger().debug(str(vector_stack))

    node.get_logger().info("Solving the system")
    parameters, rms = solve_calibration(matrix_stack, vector_stack)
    node.get_logger().info(f"Calibration parameters: {parameters}; mixed-unit fit RMS: {rms}")
    output = node.get_parameter("output_file").value
    if output:
        # JSON is valid YAML. Refuse accidental replacement of an existing calibration.
        with Path(output).open('x') as file:
            json.dump({"/**": {"ros__parameters": parameters}}, file, indent=2)


def get_avg_force_torque(force_torque_data: list[WrenchStamped], tf_prefix: str) -> WrenchStamped:
    if not force_torque_data:
        raise ValueError("No force/torque samples collected")
    avg_force_torque = WrenchStamped()
    avg_force_torque.header = force_torque_data[0].header

    for data_point in force_torque_data:
        # The force torque axis is rotated about the y-axis, so the x and z axes are flipped
        avg_force_torque.wrench.force.x += - data_point.wrench.force.x / len(force_torque_data)
        avg_force_torque.wrench.force.y += data_point.wrench.force.y / len(force_torque_data)
        avg_force_torque.wrench.force.z += - data_point.wrench.force.z / len(force_torque_data)

        avg_force_torque.wrench.torque.x += - data_point.wrench.torque.x / len(force_torque_data)
        avg_force_torque.wrench.torque.y += data_point.wrench.torque.y / len(force_torque_data)
        avg_force_torque.wrench.torque.z += - data_point.wrench.torque.z / len(force_torque_data)

    # Rotate the completed mean once, not the running sum on every iteration.
    if tf_prefix == "oarbot_blue/":
        avg_force_torque.wrench.force.x *= -1
        avg_force_torque.wrench.force.y *= -1
        avg_force_torque.wrench.torque.x *= -1
        avg_force_torque.wrench.torque.y *= -1

    return avg_force_torque