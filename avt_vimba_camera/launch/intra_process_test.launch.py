from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_dir = get_package_share_directory('avt_vimba_camera')
    params_file = os.path.join(package_share_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='avt_vimba_camera',
            executable='intra_process_exec',
            name='camera_with_subscriber',
            parameters=[params_file],
            output='screen',
            emulate_tty=True
        )
    ])