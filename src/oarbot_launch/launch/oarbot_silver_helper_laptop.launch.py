from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("setup_mode", default_value="false", description="Supervised position-action setup; disables velocity guard"),
        DeclareLaunchArgument("camera_fps", default_value="15"),
        DeclareLaunchArgument("sensor_sn", default_value=""),
        DeclareLaunchArgument("wired_sync_mode", default_value="0"),
        DeclareLaunchArgument("subordinate_delay_off_master_usec", default_value="0"),
        DeclareLaunchArgument("depth_delay_off_color_usec", default_value="0"),
        DeclareLaunchArgument("color_exposure_usec", default_value="0"),
        DeclareLaunchArgument("body_id", default_value="-1"),
        DeclareLaunchArgument("ft_serial_port", default_value="/dev/sensor_bota_rokubi_ft_0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "dingo_bridge.launch.py"
                ])
            ),
            launch_arguments={
                "oarbot_name": "oarbot_silver"
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "kinova_bringup.launch.py"
                ])
            ),
            launch_arguments={
                "kinova_namespace": "oarbot_silver/kinova"
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "azure_kinect_bringup.launch.py"
                ])
            ),
            launch_arguments={
                "azure_kinect_namespace": "oarbot_silver/azure_kinect",
                "frame_prefix": "oarbot_silver/",
                "fps": LaunchConfiguration("camera_fps"),
                "color_resolution": "1080P",
                "body_tracking_enabled": "true",
                "sensor_sn": LaunchConfiguration("sensor_sn"),
                "wired_sync_mode": LaunchConfiguration("wired_sync_mode"),
                "subordinate_delay_off_master_usec": LaunchConfiguration("subordinate_delay_off_master_usec"),
                "depth_delay_off_color_usec": LaunchConfiguration("depth_delay_off_color_usec"),
                "color_exposure_usec": LaunchConfiguration("color_exposure_usec"),
                "depth_mode": "NFOV_UNBINNED",
                "point_cloud": "false",
                "rgb_point_cloud": "false"
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "oarbot_state_publisher.launch.py"
                ])
            ),
            launch_arguments={
                "oarbot_namespace": "oarbot_silver/",
                "robot_description": "oarbot_silver.urdf.xacro"
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "oarbot_silver_moveit.launch.py"
                ])
            ),
            launch_arguments={"allow_trajectory_execution": LaunchConfiguration("setup_mode")}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "kinova_action_servers.launch.py"
                ])
            ),
            condition=IfCondition(LaunchConfiguration("setup_mode")),
            launch_arguments={
                "kinova_namespace": "oarbot_silver/kinova/",
                "joint_prefix": "oarbot_silver"
            }.items()
        ),
        Node(
            executable="rokubi_force_torque_publisher",
            package="rokubi_force_torque_publisher",
            name="rokubi_force_torque_publisher",
            namespace="oarbot_silver",
            parameters=[
                PathJoinSubstitution([FindPackageShare("oarbot_launch"), "config", "oarbot_silver_ft.yaml"]),
                {"tf_prefix": "oarbot_silver/", "serial_port": ParameterValue(LaunchConfiguration("ft_serial_port"), value_type=str)}
            ]
        ),
        Node(package="admittance_control", executable="admittance_control", namespace="oarbot_silver",
             parameters=[{"tf_prefix": "oarbot_silver/"}],
             condition=UnlessCondition(LaunchConfiguration("setup_mode"))),
        Node(package="oarbot_runtime", executable="command_guard", namespace="oarbot_silver",
             condition=UnlessCondition(LaunchConfiguration("setup_mode"))),
        Node(package="oarbot_runtime", executable="human_hand_measurements", namespace="oarbot_silver",
             parameters=[{"body_id": ParameterValue(LaunchConfiguration("body_id"), value_type=int)}],
             remappings=[("body_tracking_data", "azure_kinect/body_tracking_data")]),
    ])
