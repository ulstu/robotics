import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    package_path = get_package_share_directory('autorace')

    # clean_world = os.path.join(
    #     package_path,
    #     'worlds',
    #     'autorace-clean.world.xml'
    # )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package_path, 'launch/world.launch.py')),
            # launch_arguments={'world': clean_world}.items()
        ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory('turtlebot3_bringup'),
        #             'launch/rviz2.launch.py'))
        # ),
        Node(
            package='mapping',
            executable='mapping_controller',
            name='mapping_controller',
            prefix='xterm -e',
        ),
        Node(
            package='autorace',
            executable='autorace_node',
            name='autorace_node',
            prefix='xterm -e',
        ),
    ])


