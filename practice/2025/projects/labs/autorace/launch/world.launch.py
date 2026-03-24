import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.245')
    y_pose = LaunchConfiguration('y_pose', default='-1.787')
    turtlebot3_model = LaunchConfiguration('turtlebot3_model', default='waffle')

    default_world = os.path.join(
        get_package_share_directory('autorace'),
        'worlds',
        'autorace.world.xml'
    )
    world = LaunchConfiguration('world', default=default_world)

    declare_world = DeclareLaunchArgument(
        'world', default_value=default_world,
        description='Specify the world file to use')
    declare_turtlebot3_model = DeclareLaunchArgument(
        'turtlebot3_model', default_value='waffle',
        description='TurtleBot3 model to spawn (burger, waffle, waffle_pi)')
    set_turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', turtlebot3_model)

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

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(declare_world)
    ld.add_action(declare_turtlebot3_model)
    ld.add_action(set_turtlebot3_model)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
