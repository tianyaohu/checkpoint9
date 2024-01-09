from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='pre_approach_node',
            name='pre_approach_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'velocity': 0.2}
            ]
        )
    ])