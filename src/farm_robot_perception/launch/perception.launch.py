from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='farm_robot_perception',
            executable='crop_stress_node',
            name='crop_stress_node',
            # Gazebo camera plugin publishes on /farm_camera/image_raw
            parameters=[{'image_topic': '/farm_camera/image_raw'}],
            output='screen'
        )
    ])
