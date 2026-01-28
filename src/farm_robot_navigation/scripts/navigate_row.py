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

    def split_goal(self, x, y, w=1.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = float(w)
        return goal_msg

    def navigate_to(self, x, y, w=1.0):
        self.get_logger().info(f'Navigating to x={x}, y={y}...')

        goal_msg = self.split_goal(x, y, w)
        self._action_client.wait_for_server()

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        # Status 4 is SUCCEEDED, others are ABORTED/CANCELED etc.
        if status == 4:
            self.get_logger().info(f'Goal succeeded!')
            return True
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    navigator = SimpleNavigator()

    # Define a series of waypoints to cover different parts of the farm
    # The plants are located roughly around X = -5.0 with Y varying
    waypoints = [
        (3.0, 0.0),   # Forward to clear any obstacles/start
        (-5.0, -2.0), # Near start of plant row
        (-5.0, 2.0),  # Drive along the row
        (0.0, 0.0)    # Return to base
    ]

    for x, y in waypoints:
        success = navigator.navigate_to(x, y)
        if not success:
            break

    navigator.get_logger().info('Completed full farm inspection!')
    rclpy.shutdown()

if __name__ == '__main__':
    main()
