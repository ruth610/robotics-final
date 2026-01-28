#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class SimpleNavigator(Node):
    def __init__(self):
        super().__init__('simple_navigator')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, w=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = float(w)

        self.get_logger().info(f'Navigating to x={x}, y={y}...')
        self._action_client.wait_for_server()
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()

    # Example: Move 3 meters forward
    future = navigator.send_goal(3.0, 0.0)

    rclpy.spin_until_future_complete(navigator, future)
    navigator.get_logger().info('Goal sent!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
