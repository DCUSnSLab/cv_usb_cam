#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class BrightnessAdjuster(Node):
    def __init__(self):
        super().__init__('brightness_adjuster')

        # 밝기 증감값 (0~255), 파라미터로 조정 가능
        self.declare_parameter('brightness_beta', 80)
        self.beta = self.get_parameter('brightness_beta').get_parameter_value().integer_value

        self.bridge = CvBridge()

        # 구독할 원본 이미지 토픽
        self.sub = self.create_subscription(
            Image,
            '/cam/image_raw',
            self.callback,
            10)

        # 발행할 밝기 조정된 이미지 토픽
        self.pub = self.create_publisher(Image, '/cam/image_bright', 10)

        self.get_logger().info(f'BrightnessAdjuster started: beta = {self.beta}')

    def callback(self, msg):
        try:
            # ROS Image → OpenCV BGR 이미지
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')
            return

        # 밝기 조절: convertScaleAbs 사용 (alpha=1.0 유지, beta만 증가)
        bright = cv2.convertScaleAbs(cv_image, alpha=1.0, beta=self.beta)

        try:
            # OpenCV 이미지 → ROS Image
            out_msg = self.bridge.cv2_to_imgmsg(bright, encoding='bgr8')
            out_msg.header = msg.header  # 타임스탬프, 프레임 유지
            self.pub.publish(out_msg)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = BrightnessAdjuster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
