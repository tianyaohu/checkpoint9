from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_shelf_service_server',
            output='screen',
            name='approach_shelf_service_server',
        )
    ])