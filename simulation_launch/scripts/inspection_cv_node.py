#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import cv2
import numpy as np
from cv_bridge import CvBridge


class InspectionCVNode(Node):

    def __init__(self):
        super().__init__('inspection_cv_node')

        # --- Parameters ---
        self.declare_parameter('red_area_threshold', 1000)
        self.red_area_threshold = self.get_parameter(
            'red_area_threshold').value

        # --- State ---
        self.inspection_active = False
        self.max_red_area = 0.0

        # --- CV Bridge ---
        self.bridge = CvBridge()

        # --- Subscribers ---
        self.create_subscription(
            Image,
            '/iiwa/camera',
            self.image_cb,
            10
        )

        self.create_subscription(
            Bool,
            '/inspection_active',
            self.active_cb,
            10
        )

        # --- Publisher ---
        self.result_pub = self.create_publisher(
            String,
            '/inspection_result',
            10
        )

        self.get_logger().info("Inspection CV node ready")

    # =====================================================
    # INSPECTION STATE CALLBACK
    # =====================================================
    def active_cb(self, msg: Bool):
        if msg.data and not self.inspection_active:
            # Inspection just started
            self.get_logger().info("Inspection started")
            self.max_red_area = 0.0

        if not msg.data and self.inspection_active:
            # Inspection just ended → publish result
            self.publish_result()

        self.inspection_active = msg.data

    # =====================================================
    # IMAGE CALLBACK
    # =====================================================
    def image_cb(self, msg: Image):
        if not self.inspection_active:
            return

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg)

        red_area = self.detect_red_area(frame)
        self.max_red_area = max(self.max_red_area, red_area)

    # =====================================================
    # RED DETECTION
    # =====================================================
    def detect_red_area(self, frame):
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        # Red has two HSV ranges
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        mask = mask1 | mask2

        # Morphological cleanup
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        # Compute red area
        red_area = cv2.countNonZero(mask)

        return red_area

    # =====================================================
    # RESULT PUBLISHING
    # =====================================================
    def publish_result(self):
        result = String()

        if self.max_red_area > self.red_area_threshold:
            result.data = "REJECT"
            self.get_logger().info(
                f"Inspection result: REJECT (red area = {self.max_red_area})"
            )
        else:
            result.data = "ADMIT"
            self.get_logger().info(
                f"Inspection result: ADMIT (red area = {self.max_red_area})"
            )

        self.result_pub.publish(result)


# =====================================================
# MAIN
# =====================================================
def main():
    rclpy.init()
    node = InspectionCVNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
