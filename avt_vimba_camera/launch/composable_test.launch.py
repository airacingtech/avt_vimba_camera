from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_share_dir = get_package_share_directory('avt_vimba_camera')
    params_file = os.path.join(package_share_dir, 'config', 'params.yaml')

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     package='avt_vimba_camera',
            #     plugin='avt_vimba_camera::MonoCameraNode',
            #     name='camera',
            #     parameters=[params_file],
            #     extra_arguments=[{'use_intra_process_comms': True}]
            # ),
            ComposableNode(
                package='avt_vimba_camera',
                plugin='avt_vimba_camera::ImageSubscriberNode',
                name='image_subscriber',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen',
    )

    # # Add a delay before starting the camera
    # start_camera_cmd = ExecuteProcess(
    #     cmd=[FindExecutable(name='ros2'), 'service', 'call', 
    #          '/camera/start_stream', 'std_srvs/srv/Trigger', '{}'],
    #     output='screen'
    # )

    # # Wait for container to be ready before starting camera
    # delay_camera_start = TimerAction(
    #     period=2.0,  # 2 second delay
    #     actions=[start_camera_cmd]
    # )

    return LaunchDescription([
        container,
        # delay_camera_start
    ])