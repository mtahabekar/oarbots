"""Another robot's Rokubi wrist sensor plugged into this helper laptop.

Started by fabric_session.sh when this helper's hosts/<robot>.args says `hosted_ft:=<other robot>`.
The node has the same name, namespace, calibration file and frames as in that robot's own helper
launch, so everything that uses the force (guard, admittance, bridge, first_motion.py) works
unchanged: it subscribes by topic, and the calibrated output needs only that robot's TF on /tf.

Why: with Blue's sensor on Blue's own laptop, the sensor body (bolted to the wrist) and the arm's
USB tie the wrist and the arm base to one ground, and Blue's arm jitters and reports stale joint
angles while it moves. On Silver's laptop there is no second connection to Blue's arm.
Lab log 2026-11-18, "Blue's arm jitters with its wrist sensor connected".

    ros2 launch scripts/hw/hosted_ft.launch.py robot:=blue serial_port:=/dev/serial/by-id/...
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def sensor_node(context):
    robot = LaunchConfiguration('robot').perform(context)
    if robot not in ('blue', 'silver'):
        raise RuntimeError(f'robot must be blue or silver, not {robot!r}')
    share = get_package_share_directory('oarbot_launch')
    return [Node(
        executable='rokubi_force_torque_publisher',
        package='rokubi_force_torque_publisher',
        name='rokubi_force_torque_publisher',
        namespace=f'oarbot_{robot}',
        parameters=[
            f'{share}/config/oarbot_{robot}_ft.yaml',
            {'tf_prefix': f'oarbot_{robot}/', 'serial_port': LaunchConfiguration('serial_port').perform(context)},
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot', description='whose sensor this is: blue or silver'),
        DeclareLaunchArgument('serial_port', description="that sensor's /dev/serial/by-id path on this laptop"),
        OpaqueFunction(function=sensor_node),
    ])
