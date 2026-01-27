from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farm_robot_perception',
            executable='crop_stress_node',
            name='crop_stress_node',
            parameters=[{'image_topic': '/camera/image_raw'}],
            output='screen'
        )
    ])
