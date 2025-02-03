#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE_NAME = 'ulstu_turtlesim'
USE_SIM_TIME = False

def generate_launch_description():
    # Узел sim_node
    sim_node = Node(
        package=PACKAGE_NAME,
        executable='sim_node',
        name='sim_node_l',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    # Узел turtlesim_node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen',
        parameters=[{'use_sim_time': USE_SIM_TIME}]
    )

    # Описание launch-файла
    return LaunchDescription([
        sim_node,
        turtlesim_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=turtlesim_node,  # Указываем, для какого узла отслеживать завершение
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])