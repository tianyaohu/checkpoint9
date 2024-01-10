from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    package_description = "launch_tests_pkg"

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
    # rviz_config_file_name_arg = DeclareLaunchArgument(
    #     "rviz_config_file_name", default_value=TextSubstitution(text="launch_part.rviz")
    # )

    obstacle_f = LaunchConfiguration('obstacle')
    degrees_f = LaunchConfiguration('degrees')
    final_approach_f = LaunchConfiguration('final_approach')

    # rviz_config_file_name_f = LaunchConfiguration('rviz_config_file_name')

    # include another launch file
    # use items because you need to pass a list with a key-value structure
    # [(key1,value_x),(key2,value_y),...]

    # start_rviz_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare(package_description),
    #             'launch',
    #             'start_rviz_with_arguments.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'rviz_config_file_name': rviz_config_file_name_f}.items()
    # )

    # approach_load_service_server = Node(
    #     package='attach_shelf',
    #     executable='approach_shelf_service_server',
    #     output='screen',
    #     name='approach_shelf_service_server',
    #     emulate_tty=True,
    #     # arguments=["-obstacle", obstacle_f,
    #     #            "-degrees", degrees_f,
    #     #            "-final_approach", final_approach_arg,
    #     #            ]
    # )

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

    return LaunchDescription([
        obstacle_arg,
        degrees_arg,
        final_approach_arg,
        # approach_load_service_server,
        pre_approach_v2_node,
        # rviz_config_file_name_arg,
    ])