from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    nav2_bringup = FindPackageShare('nav2_bringup')
    nav2_params = PathJoinSubstitution([FindPackageShare('farm_robot_navigation'), 'config', 'nav2_params.yaml'])

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup, '/launch/bringup_launch.py']),
        launch_arguments={
            'use_sim_time': 'True',
            'slam': 'True',
            'map': '',  # Not used when slam=True but required by launch file arg declaration
            'params_file': nav2_params,
        }.items()
    )

    return LaunchDescription([
        bringup
    ])
