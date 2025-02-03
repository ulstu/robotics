#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

PACKAGE_NAME = 'ulstu_turtlebot_gazebo'
USE_SIM_TIME = False

def generate_launch_description():
    # Узел sim_node
    sim_node = Node(
        package=PACKAGE_NAME,
        executable='ulstu_turtlebot_gazebo_node',
        name='ulstu_turtlebot_gazebo_node',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    turtlebot3_dir = get_package_share_directory('turtlebot3_gazebo')
    gazebo_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    turtlebot3_dir + '/launch/turtlebot3_house.launch.py'))

    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}],
        prefix='xterm -e',
    )

    # Описание launch-файла
    return LaunchDescription([
        sim_node,
        gazebo_launch,
        teleop_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=sim_node,  # Указываем, для какого узла отслеживать завершение
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])