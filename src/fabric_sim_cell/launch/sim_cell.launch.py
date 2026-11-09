"""The fabric estimation stack against the simulated OARBot cell (oarbot_simulation).

Start the plant first, with the same `FASTRTPS_DEFAULT_PROFILES_FILE` and ROS domain:
    ./run_oarbot_fabric.sh --attach --sensors --gloves [--mocap-hands] [--truth-output ...]
then:
    ros2 launch fabric_sim_cell sim_cell.launch.py urdf_dir:=/path/to/oarbot_simulation/assets/oarbots_jazzy

Everything here is what the real cell runs, on simulation time: driver joint-state renamers
and robot_state_publisher per robot, one perception node per camera, hand fusion (or the
labelled oracle), the estimator, optional F/T compensation and a recorder. The deployment
calibration replaces measured hardware calibration.
"""
from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

COLORS = ('blue', 'silver')
CAMERAS = {'blue_wrist': '/oarbot_blue/azure_kinect', 'silver_wrist': '/oarbot_silver/azure_kinect', 'overhead': '/nuc'}


def nodes(context):
    get = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = Path(FindPackageShare('fabric_sim_cell').perform(context))
    config = lambda value, default: Path(value) if value else share / 'config' / default  # noqa: E731
    urdf_dir = get('urdf_dir')
    if not urdf_dir:
        raise RuntimeError('urdf_dir is required: oarbot_simulation/assets/oarbots_jazzy (the plant robots)')
    sim = {'use_sim_time': True}
    # profile:=fast selects the real-time configs (perception_*_fast.yaml, estimator_fast.yaml)
    # made for the plant's real-time settings (oarbot_simulation run_demo.sh fast/teleop).
    if get('profile') not in ('', 'fast', 'monitor'):
        raise RuntimeError("profile must be '', 'fast' or 'monitor'")
    suffix = '_fast' if get('profile') in ('fast', 'monitor') else ''
    estimator_suffix = '_monitor' if get('profile') == 'monitor' else suffix
    sampling = ({'adaptive_sampling': True, 'fabric_rate_hz': 5.0, 'monitor_rate_hz': 1.0,
                 'monitor_timeout_sec': 2.5} if get('profile') == 'monitor' else {})
    out = []
    for color in COLORS:
        ns = f'oarbot_{color}'
        description = (Path(urdf_dir) / f'oarbot_{color}_ros.urdf').read_text()
        out.append(Node(package='robot_state_publisher', executable='robot_state_publisher', name='robot_state_publisher',
                        namespace=ns, parameters=[{'robot_description': description, **sim}]))
        for name, source in (('kinova', 'kinova/j2n6s300_driver/out/joint_state'), ('dingo', 'dingo/platform/joint_states')):
            out.append(Node(package='joint_state_renamer', executable='joint_state_renamer', name=f'{name}_joint_state_renamer',
                            namespace=ns, parameters=[{'joint_state_prefix': f'{ns}/', 'input_topic': source,
                                                       'output_topic': 'joint_states', **sim}]))
    calibration_file = str(config(get('calibration_file'), 'calibration_exact.yaml'))
    out.append(Node(package='fabric_sim_cell', executable='calibration_publisher', name='sim_cell_calibration',
                    parameters=[{'calibration_file': calibration_file, 'errors_file': get('errors_file'), **sim}]))
    for camera_id, namespace in CAMERAS.items():
        if camera_id == 'overhead' and get('overhead') != 'true':
            continue
        out.append(Node(package='fabric_perception', executable='camera', namespace=namespace, name='fabric_perception',
                        output='screen', parameters=[str(config('', f'perception_{camera_id}{suffix}.yaml')),
                                                     {'camera_id': camera_id, **sampling, **sim}]))
    # With the grasp gate, the hand source publishes to /fabric/hand_states_ungated and the gate
    # forwards it only while both grippers are closed: estimation starts once the corners are held.
    gated = get('grasp_gate') == 'true'
    hands_remap = [('/fabric/hand_states', '/fabric/hand_states_ungated')] if gated else []
    if get('hands') == 'oracle':
        out.append(Node(package='fabric_sim_cell', executable='oracle_hands', name='oracle_hands', output='screen',
                        remappings=hands_remap,
                        parameters=[{'position_std_m': float(get('oracle_position_std_m')),
                                     'noise_position_std_m': float(get('oracle_noise_m')), **sim}]))
    elif get('hands') == 'gloves':
        out.append(Node(package='human_hand_tracking', executable='fusion', name='hand_fusion', output='screen',
                        remappings=hands_remap, parameters=[str(config('', 'fusion.yaml')), sim]))
    else:
        raise RuntimeError("hands must be 'oracle' or 'gloves'")
    if gated:
        out.append(Node(package='fabric_sim_cell', executable='grasp_gate', name='grasp_gate', output='screen',
                        parameters=[sim]))
    out.append(Node(package='fabric_state_estimator', executable='observer', name='fabric_state_estimator', output='screen',
                    parameters=[{'config_file': str(config(get('estimator_config'), f'estimator{estimator_suffix}.yaml')), **sim}]))
    if get('force_torque') == 'true':
        calibration = yaml.safe_load(Path(calibration_file).read_text())['force_torque']
        for color in COLORS:
            ns = f'oarbot_{color}'
            out.append(Node(package='rokubi_force_torque_publisher', executable='rokubi_force_torque_compensator',
                            namespace=ns, name='rokubi_force_torque_compensator', output='screen',
                            parameters=[{'tf_prefix': f'{ns}/', **calibration[ns], **sim}]))
    if get('rviz') == 'true':
        # Demo view: the estimated mesh (sensors only) in 3D and drawn over the camera images.
        out.append(Node(package='fabric_sim_cell', executable='demo_markers', name='sim_cell_demo_markers',
                        parameters=[sim]))
        out.append(Node(package='rviz2', executable='rviz2', name='sim_cell_rviz', output='log',
                        arguments=['-d', str(share / 'config' / 'sim_cell_demo.rviz')], parameters=[sim]))
    if get('record'):
        out.append(Node(package='fabric_sim_cell', executable='recorder', name='sim_cell_recorder', output='screen',
                        parameters=[{'output': get('record'), 'record_forces': get('force_torque') == 'true', **sim}]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('urdf_dir', default_value='', description='oarbot_simulation/assets/oarbots_jazzy'),
        DeclareLaunchArgument('calibration_file', default_value='', description='Default: config/calibration_exact.yaml'),
        DeclareLaunchArgument('errors_file', default_value='', description='Deliberate calibration errors (YAML)'),
        DeclareLaunchArgument('estimator_config', default_value='', description='Default: config/estimator.yaml'),
        DeclareLaunchArgument('profile', default_value='', description="'fast': live observer; 'monitor': adaptive 5-to-1 Hz fabric validation"),
        DeclareLaunchArgument('hands', default_value='oracle', description="'oracle' (labelled ideal input) or 'gloves'"),
        DeclareLaunchArgument('oracle_position_std_m', default_value='0.005', description='Covariance the oracle states'),
        DeclareLaunchArgument('oracle_noise_m', default_value='0.0', description='Noise the oracle actually adds'),
        DeclareLaunchArgument('overhead', default_value='true'),
        DeclareLaunchArgument('grasp_gate', default_value='true',
                              description='Start estimation only once both grippers are closed (false: from launch)'),
        DeclareLaunchArgument('force_torque', default_value='false', description='Run the F/T compensators'),
        DeclareLaunchArgument('record', default_value='', description='Recorder output NPZ; empty = no recorder'),
        DeclareLaunchArgument('rviz', default_value='false', description='Demo view: estimated mesh over the camera images'),
        OpaqueFunction(function=nodes),
    ])
