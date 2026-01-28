from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = FindPackageShare('farm_robot_description')
    world = PathJoinSubstitution([pkg_desc, 'worlds', 'farm.world'])
    robot_xacro = PathJoinSubstitution([pkg_desc, 'urdf', 'farm_bot.xacro'])

    robot_description = {'robot_description': Command(['xacro ', robot_xacro])}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch/gazebo.launch.py']
        ),
        launch_arguments={'world': world}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # If Gazebo is already running (or you relaunch), delete any existing model
    # named 'farm_bot' before spawning again.
    delete_existing = Node(
        package='farm_robot_description',
        executable='delete_entity.py',
        name='delete_entity_once',
        output='screen',
        parameters=[{
            'entity_name': 'farm_bot',
            'service_name': '/delete_entity',
            'timeout_sec': 5.0,
        }]
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'farm_bot', '-topic', 'robot_description'],
        output='screen'
    )

    spawn_after_delete = RegisterEventHandler(
        OnProcessExit(
            target_action=delete_existing,
            on_exit=[spawn],
        )
    )

    return LaunchDescription([
        gazebo,
        rsp,
        delete_existing,
        spawn_after_delete,
    ])
