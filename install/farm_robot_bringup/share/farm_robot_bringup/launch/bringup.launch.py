from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    desc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('farm_robot_description'), '/launch/bringup_gazebo.launch.py']
        )
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('farm_robot_navigation'), '/launch/navigation.launch.py']
        )
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('farm_robot_perception'), '/launch/perception.launch.py']
        )
    )

    return LaunchDescription([
        desc_launch,
        nav_launch,
        perception_launch,
    ])
