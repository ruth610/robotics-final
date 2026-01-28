import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
import cv2
import numpy as np


class CropStressNode(Node):
    def __init__(self):
        super().__init__('crop_stress_node')
        self.bridge = CvBridge()

        image_topic = self.declare_parameter('image_topic', '/camera/image_raw').get_parameter_value().string_value
        self.subscription = self.create_subscription(Image, image_topic, self.image_callback, 10)

        self.pub_chl = self.create_publisher(Float32, '/crop_monitor/chlorophyll_index', 10)
        self.pub_dry = self.create_publisher(Float32, '/crop_monitor/dryness_index', 10)
        self.pub_tex = self.create_publisher(Float32, '/crop_monitor/texture_index', 10)
        self.pub_status = self.create_publisher(String, '/crop_monitor/status', 10)

    def image_callback(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion error: {e}')
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Green mask (approximate chlorophyll-rich range)
        green_mask = cv2.inRange(hsv, (35, 60, 30), (85, 255, 255))
        green_ratio = float(np.count_nonzero(green_mask)) / float(frame.shape[0] * frame.shape[1])

        # Dryness mask: yellow/brown hues + high brightness (avoid soil)
        # Hue: 10-35 (Orange/Yellow)
        # Sat: > 60
        # Val: > 120 (Soil is approx 100, this filters it out)
        dryness_mask = cv2.inRange(hsv, (10, 60, 120), (35, 255, 255))
        dryness_ratio = float(np.count_nonzero(dryness_mask)) / float(frame.shape[0] * frame.shape[1])

        # Texture index: variance of Laplacian
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        lap_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        texture_index = float(lap_var) / 1000.0  # simple normalization

        # Simple status logic
        status = 'healthy'
        if green_ratio < 0.15 and dryness_ratio > 0.10:
            status = 'stress: dryness/discoloration'
        elif green_ratio < 0.10:
            status = 'stress: low chlorophyll'
        elif dryness_ratio > 0.20:
            status = 'stress: high dryness'

        # Publish
        self.pub_chl.publish(Float32(data=float(green_ratio)))
        self.pub_dry.publish(Float32(data=float(dryness_ratio)))
        self.pub_tex.publish(Float32(data=float(texture_index)))
        self.pub_status.publish(String(data=status))

        self.get_logger().debug(f'Indices - green: {green_ratio:.3f}, dry: {dryness_ratio:.3f}, tex: {texture_index:.3f}, status: {status}')


def main():
    rclpy.init()
    node = CropStressNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

