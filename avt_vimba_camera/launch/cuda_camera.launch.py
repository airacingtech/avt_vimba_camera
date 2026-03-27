"""Launch a single CUDA-accelerated camera node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='camera',
                              description='Camera node name'),
        DeclareLaunchArgument('frame_id', default_value=LaunchConfiguration('name'),
                              description='TF frame id'),
        DeclareLaunchArgument('ip', default_value='',
                              description='Camera IP address'),
        DeclareLaunchArgument('guid', default_value='',
                              description='Camera GUID'),
        DeclareLaunchArgument('camera_info_url', default_value='',
                              description='Camera calibration URL'),
        DeclareLaunchArgument('params_file', default_value='',
                              description='Path to params YAML file'),
        DeclareLaunchArgument('use_ptp', default_value='false',
                              description='Use PTP timestamps'),
        DeclareLaunchArgument('ptp_offset', default_value='-37',
                              description='PTP offset (TAI to UTC)'),
        DeclareLaunchArgument('num_buffers', default_value='6',
                              description='Number of pinned DMA buffers'),
        DeclareLaunchArgument('publish_compressed', default_value='false',
                              description='Publish JPEG compressed images'),
        DeclareLaunchArgument('enable_pcap', default_value='false',
                              description='Enable PCAP replay mode'),
        DeclareLaunchArgument('pcap_file', default_value='',
                              description='Path to PCAP file for replay'),

        Node(
            package='avt_vimba_camera',
            executable='cuda_camera_exec',
            name=LaunchConfiguration('name'),
            output='screen',
            parameters=[
                {
                    'ip': LaunchConfiguration('ip'),
                    'guid': LaunchConfiguration('guid'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'camera_info_url': LaunchConfiguration('camera_info_url'),
                    'use_ptp': LaunchConfiguration('use_ptp'),
                    'ptp_offset': LaunchConfiguration('ptp_offset'),
                    'num_buffers': LaunchConfiguration('num_buffers'),
                    'publish_compressed': LaunchConfiguration('publish_compressed'),
                    'enable_pcap': LaunchConfiguration('enable_pcap'),
                    'pcap_file': LaunchConfiguration('pcap_file'),
                },
                LaunchConfiguration('params_file'),
            ],
        ),
    ])
