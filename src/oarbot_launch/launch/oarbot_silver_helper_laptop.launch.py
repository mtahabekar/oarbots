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
        # Hardware switches. Defaults keep the full stack; turn a part off when its device is unplugged.
        DeclareLaunchArgument("camera", default_value="true", description="Start the wrist Azure Kinect driver"),
        DeclareLaunchArgument("body_tracking", default_value="true", description="Azure body tracking (uses the GPU)"),
        DeclareLaunchArgument("color_resolution", default_value="1080P"),
        DeclareLaunchArgument("ft", default_value="true", description="Start the Rokubi force-torque driver"),
        DeclareLaunchArgument("max_arm_joint_dps", default_value="15.0",
                              description="Command guard: no arm joint faster than this in joint mode (deg/s). The Jaco2 "
                                          "model's own limits are 36 deg/s for joints 1-3 and 48 for joints 4-6"),
        DeclareLaunchArgument("hold_fingers", default_value="-1",
                              description="Finger closure % the guard holds while the deadman is held; -1: never commands the fingers"),

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
            condition=IfCondition(LaunchConfiguration("camera")),
            launch_arguments={
                "azure_kinect_namespace": "oarbot_silver/azure_kinect",
                "frame_prefix": "oarbot_silver/",
                "fps": LaunchConfiguration("camera_fps"),
                "color_resolution": LaunchConfiguration("color_resolution"),
                "body_tracking_enabled": LaunchConfiguration("body_tracking"),
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
            condition=IfCondition(LaunchConfiguration("ft")),
            parameters=[
                PathJoinSubstitution([FindPackageShare("oarbot_launch"), "config", "oarbot_silver_ft.yaml"]),
                {"tf_prefix": "oarbot_silver/", "serial_port": ParameterValue(LaunchConfiguration("ft_serial_port"), value_type=str)}
            ]
        ),
        Node(package="admittance_control", executable="admittance_control", namespace="oarbot_silver",
             parameters=[{"tf_prefix": "oarbot_silver/"}],
             condition=UnlessCondition(LaunchConfiguration("setup_mode"))),
        Node(package="oarbot_runtime", executable="command_guard", namespace="oarbot_silver",
             parameters=[{"hold_fingers_percent": ParameterValue(LaunchConfiguration("hold_fingers"), value_type=float),
                          "max_arm_joint_dps": ParameterValue(LaunchConfiguration("max_arm_joint_dps"), value_type=float)}],
             condition=UnlessCondition(LaunchConfiguration("setup_mode"))),
        Node(package="oarbot_runtime", executable="human_hand_measurements", namespace="oarbot_silver",
             parameters=[{"body_id": ParameterValue(LaunchConfiguration("body_id"), value_type=int)}],
             remappings=[("body_tracking_data", "azure_kinect/body_tracking_data")]),
    ])
