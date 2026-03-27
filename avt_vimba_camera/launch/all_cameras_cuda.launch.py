"""
Launch all 6 IAC (Indy Autonomous Challenge) cameras with CUDA-accelerated nodes.

Camera positions and default IPs follow the standard IAC Dallara AV-21 layout:
  - front_left_center  (FLC)
  - front_right_center (FRC)
  - front_left_far     (FLF)
  - front_right_far    (FRF)
  - rear_left          (RL)
  - rear_right         (RR)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Default IAC camera configuration
CAMERAS = [
    {'name': 'front_left_center',  'ip': '169.254.100.1',  'frame_id': 'front_left_center_camera'},
    {'name': 'front_right_center', 'ip': '169.254.100.2',  'frame_id': 'front_right_center_camera'},
    {'name': 'front_left_far',     'ip': '169.254.100.3',  'frame_id': 'front_left_far_camera'},
    {'name': 'front_right_far',    'ip': '169.254.100.4',  'frame_id': 'front_right_far_camera'},
    {'name': 'rear_left',          'ip': '169.254.100.5',  'frame_id': 'rear_left_camera'},
    {'name': 'rear_right',         'ip': '169.254.100.6',  'frame_id': 'rear_right_camera'},
]


def generate_launch_description():
    ld = LaunchDescription()

    # Global arguments
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value='',
        description='Path to shared params YAML file for all cameras'))
    ld.add_action(DeclareLaunchArgument(
        'use_ptp', default_value='true',
        description='Use PTP timestamps (recommended for multi-camera sync)'))
    ld.add_action(DeclareLaunchArgument(
        'ptp_offset', default_value='-37',
        description='PTP offset in nanoseconds (TAI to UTC)'))
    ld.add_action(DeclareLaunchArgument(
        'num_buffers', default_value='6',
        description='Number of pinned DMA buffers per camera'))
    ld.add_action(DeclareLaunchArgument(
        'publish_compressed', default_value='false',
        description='Publish JPEG compressed images'))
    ld.add_action(DeclareLaunchArgument(
        'enable_pcap', default_value='false',
        description='Enable PCAP replay mode'))
    ld.add_action(DeclareLaunchArgument(
        'pcap_file', default_value='',
        description='Path to PCAP file for replay'))

    cuda_camera_launch = PathJoinSubstitution([
        FindPackageShare('avt_vimba_camera'), 'launch', 'cuda_camera.launch.py'
    ])

    for cam in CAMERAS:
        # Allow per-camera IP override via launch arguments
        ip_arg_name = f"{cam['name']}_ip"
        ld.add_action(DeclareLaunchArgument(
            ip_arg_name, default_value=cam['ip'],
            description=f"IP address for {cam['name']} camera"))

        calibration_url_arg_name = f"{cam['name']}_camera_info_url"
        default_cal_url = PathJoinSubstitution([
            FindPackageShare('avt_vimba_camera'),
            'calibrations', f"{cam['name']}.yaml"
        ])
        ld.add_action(DeclareLaunchArgument(
            calibration_url_arg_name, default_value='',
            description=f"Calibration URL for {cam['name']} camera"))

        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cuda_camera_launch),
            launch_arguments={
                'name': cam['name'],
                'frame_id': cam['frame_id'],
                'ip': LaunchConfiguration(ip_arg_name),
                'camera_info_url': LaunchConfiguration(calibration_url_arg_name),
                'params_file': LaunchConfiguration('params_file'),
                'use_ptp': LaunchConfiguration('use_ptp'),
                'ptp_offset': LaunchConfiguration('ptp_offset'),
                'num_buffers': LaunchConfiguration('num_buffers'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'enable_pcap': LaunchConfiguration('enable_pcap'),
                'pcap_file': LaunchConfiguration('pcap_file'),
            }.items(),
        ))

    return ld
