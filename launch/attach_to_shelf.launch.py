import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # args that can be set from the command line or a default will be used
    # TextSubstitution(text="0.0") W ill only evaluate that in execution time.

    obstacle_arg = DeclareLaunchArgument(
        "obstacle", default_value=TextSubstitution(text="0.3")
    )

    degrees_arg = DeclareLaunchArgument(
        "degrees", default_value="-90"
    )

    final_approach_arg = DeclareLaunchArgument(
        "final_approach", default_value="True"
    )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    approach_load_service_server = Node(
        package='attach_shelf',
        executable='approach_shelf_service_server',
        output='screen',
        name='approach_shelf_service_server',
        emulate_tty=True,
    )

    pre_approach_v2_node = Node(
        package='attach_shelf',
        executable='pre_approach_v2_node',
        output='screen',
        name='pre_approach_v2_node',
        emulate_tty=True,
        arguments=["-obstacle", obstacle_f,
                   "-degrees", degrees_f,
                   "-final_approach", final_approach_f,
                   ]
    )
    ############### RVIZ ################
    # RVIZ Configuration
    package_description = "attach_shelf"
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'launch_part.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        approach_load_service_server,
        pre_approach_v2_node,
        rviz_node
    ])