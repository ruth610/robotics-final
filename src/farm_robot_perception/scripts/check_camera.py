#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/farm_camera/image_raw',
            self.listener_callback,
            10)
        self.count = 0
        self.save_path = os.path.join(os.getcwd(), 'camera_view.jpg')
        self.get_logger().info('Waiting for images...')

    def listener_callback(self, msg):
        if self.count == 0:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                cv2.imwrite(self.save_path, cv_image)
                self.get_logger().info(f'Saved image to {self.save_path}')
                self.count += 1
            except Exception as e:
                self.get_logger().error(f'Failed to save image: {e}')

def main(args=None):
    rclpy.init(args=args)
    image_saver = ImageSaver()
    # Spin once to process one message then exit
    import time
    time.sleep(1) # wait for connection
    rclpy.spin_once(image_saver, timeout_sec=5.0)
    image_saver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
