#!/usr/bin/env python3
import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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

    teleop_node = Node(
        package='turtlebot3_teleop',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}],
        prefix='xterm -e',
    )

    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    nav_ros = get_package_share_directory('turtlebot3_navigation2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='1.5')

    world = os.path.join(
        get_package_share_directory(PACKAGE_NAME),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    package_dir = get_package_share_directory(PACKAGE_NAME)
    map_yaml_file = os.path.join(package_dir, 'resource', 'my_map.yaml')
    params_file = os.path.join(package_dir, 'resource', 'params.yaml')
    nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_ros, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'map': map_yaml_file, 'params_file': params_file}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    # Описание launch-файла
    ld = LaunchDescription([
        sim_node,
        teleop_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=sim_node,  # Указываем, для какого узла отслеживать завершение
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(nav_cmd)

    return ld