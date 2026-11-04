from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("extrinsics_file", default_value=PathJoinSubstitution([FindPackageShare("oarbot_launch"), "config", "nuc_extrinsics.yaml"])),
        DeclareLaunchArgument("camera_fps", default_value="15"),
        DeclareLaunchArgument("sensor_sn", default_value=""),
        DeclareLaunchArgument("wired_sync_mode", default_value="0"),
        DeclareLaunchArgument("depth_delay_off_color_usec", default_value="0"),
        DeclareLaunchArgument("color_exposure_usec", default_value="0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("oarbot_launch"),
                    "launch",
                    "azure_kinect_bringup.launch.py"
                ])
            ),
            launch_arguments={
                "azure_kinect_namespace": "nuc",
                "frame_prefix": "nuc/",
                "fps": LaunchConfiguration("camera_fps"),
                "color_resolution": "1080P",
                "depth_mode": "NFOV_UNBINNED",
                "point_cloud": "false",
                "rgb_point_cloud": "false",
                "sensor_sn": LaunchConfiguration("sensor_sn"),
                "wired_sync_mode": LaunchConfiguration("wired_sync_mode"),
                "depth_delay_off_color_usec": LaunchConfiguration("depth_delay_off_color_usec"),
                "color_exposure_usec": LaunchConfiguration("color_exposure_usec")
            }.items()
        ),
        Node(
            package="aruco_pose_estimation",
            executable="aruco_node.py",
            name="aruco_pose_estimation",
            namespace="nuc",
            parameters=[{
                "marker_size": 0.36,
                "aruco_dictionary_id": "DICT_4X4_50",
                "image_topic": "/nuc/rgb/image_raw",
                "use_depth_input": False,
                "depth_image_topic": "/nuc/depth_to_rgb/image_raw",
                "camera_info_topic": "/nuc/rgb/camera_info",
                "camera_frame": "nuc/rgb_camera_link",
                "detected_markers_topic": "/aruco/markers",
                "markers_visualization_topic": "/aruco/poses",
                "output_image_topic": "/aruco/image"
            }]
        ),
        Node(
            package="oarbot_pose_publisher",
            executable="oarbot_pose_publisher",
            name="oarbot_pose_publisher",
            namespace="nuc",
            parameters=[LaunchConfiguration("extrinsics_file"), {
                "aruco_markers_topic": "/aruco/markers",
                "kinect_imu_topic": "/nuc/imu",
                "aruco_tag_ids": [0, 4],
                "oarbot_namespaces": ["oarbot_blue/", "oarbot_silver/"],
            }]
        )
    ])
