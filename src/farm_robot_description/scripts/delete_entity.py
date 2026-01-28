#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity


class DeleteEntityOnce(Node):
    def __init__(self):
        super().__init__('delete_entity_once')
        self.declare_parameter('entity_name', 'farm_bot')
        self.declare_parameter('service_name', '/delete_entity')
        self.declare_parameter('timeout_sec', 5.0)

        self._entity_name = self.get_parameter('entity_name').get_parameter_value().string_value
        self._service_name = self.get_parameter('service_name').get_parameter_value().string_value
        self._timeout_sec = self.get_parameter('timeout_sec').get_parameter_value().double_value

        self._client = self.create_client(DeleteEntity, self._service_name)

    def run(self) -> int:
        if not self._client.wait_for_service(timeout_sec=float(self._timeout_sec)):
            self.get_logger().warn(
                f"Service {self._service_name} not available; skipping delete of {self._entity_name}"
            )
            return 0

        req = DeleteEntity.Request()
        req.name = self._entity_name
        future = self._client.call_async(req)

        rclpy.spin_until_future_complete(self, future, timeout_sec=float(self._timeout_sec))
        if future.done() and future.result() is not None:
            self.get_logger().info(f"DeleteEntity requested for {self._entity_name}")
            return 0

        self.get_logger().warn(f"Timed out deleting {self._entity_name}; continuing anyway")
        return 0


def main(argv=None):
    rclpy.init(args=argv)
    node = DeleteEntityOnce()
    rc = 0
    try:
        rc = node.run()
    except Exception as e:
        node.get_logger().warn(f"delete_entity_once failed: {e}")
        rc = 0
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)

