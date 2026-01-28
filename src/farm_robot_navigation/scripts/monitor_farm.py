#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class FarmMonitor(Node):
    def __init__(self):
        super().__init__('farm_monitor')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Subscribe to crop status to log what we find
        self.create_subscription(String, '/crop_monitor/status', self.status_callback, 10)

        # Define patrol route (waypoints)
        # Strategy: Drive OUT of the row first to turn safely in open space
        # Lanes are at y=0, y=1, y=-1. Plants exist from x=-5 to x=5.

        self.waypoints = [
            # 1. Drive East down Row 1 (y=0.5) - Straddling the row
            {'x': 5.5, 'y': 0.5, 'w': 1.0, 'z': 0.0},

            # 2. Move to start of Row 2 (y=-0.5) in the open area
            {'x': 5.5, 'y': -0.5, 'w': 0.707, 'z': -0.707},

            # 3. Drive West down Row 2 (y=-0.5)
            {'x': -5.5, 'y': -0.5, 'w': 0.0, 'z': 1.0},

            # 4. Return Home
            {'x': 0.0, 'y': 0.0, 'w': 1.0, 'z': 0.0}
        ]
        self.current_wp_index = 0

    def status_callback(self, msg):
        if "stress" in msg.data:
            self.get_logger().warn(f"⚠️  ALERT: {msg.data}")
        # Else silent for healthy to avoid spam

    def start_patrol(self):
        if self.current_wp_index < len(self.waypoints):
            wp = self.waypoints[self.current_wp_index]
            self.get_logger().info(f"🚜 Navigating to Waypoint {self.current_wp_index + 1}/{len(self.waypoints)}: x={wp['x']}, y={wp['y']}")
            self.send_goal(wp['x'], wp['y'], wp['w'], wp.get('z', 0.0))
        else:
            self.get_logger().info("✅ Farm Patrol Completed!")
            rclpy.shutdown()

    def send_goal(self, x, y, w=1.0, z=0.0):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.orientation.w = float(w)
        goal_msg.pose.pose.orientation.z = float(z)

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f"Reached Waypoint {self.current_wp_index + 1}")
        self.current_wp_index += 1
        self.start_patrol() # Trigger next Goal

def main(args=None):
    rclpy.init(args=args)
    monitor = FarmMonitor()

    # Give some time for connections
    monitor.start_patrol()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
