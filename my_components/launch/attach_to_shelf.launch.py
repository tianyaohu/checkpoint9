import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from launch_ros.actions import Node

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='my_components',
                    plugin='my_components::PreApproach',
                    name='pre_approach'),
                ComposableNode(
                    package='my_components',
                    plugin='my_components::AttachServer',
                    name='attach_server')
            ],
            output='screen',
    )

    manual_composition = Node(
        package='my_components',
        executable='manual_composition',
        output='screen',
        name='manual_composition',
    )

    return launch.LaunchDescription([
        container,
        # manual_composition,
    ])