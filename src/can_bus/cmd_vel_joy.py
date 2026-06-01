#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class JoyToCmdVel(Node):

    def __init__(self):
        super().__init__('joy_to_cmdvel')

        # ===== Subscriber =====
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # ===== Publisher =====
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        # ===== Steering limit =====
        self.max_steer = 0.873   # rad (≈ 50 deg)

        self.get_logger().info("Joy → cmd_vel started")

    # =========================
    # Quantize linear.x
    # =========================
    def quantize_linear(self, val):
        if val > 0.5:
            return 1.0
        elif val < -0.5:
            return -1.0
        else:
            return 0.0

    # =========================
    # Callback
    # =========================
    def joy_callback(self, msg):

        twist = Twist()

        # -------------------------
        # รวม linear จาก axes[1] + axes[0]
        # -------------------------
        linear_input = 0.0
        if len(msg.axes) > 1:
            linear_input += msg.axes[1]
        if len(msg.axes) > 0:
            linear_input += msg.axes[0]

        twist.linear.x = self.quantize_linear(linear_input)

        # -------------------------
        # รวม angular จาก axes[2] + axes[3]
        # -------------------------
        angular_input = 0.0
        if len(msg.axes) > 2:
            angular_input += msg.axes[2]
        if len(msg.axes) > 3:
            angular_input += msg.axes[3]

        # ----- map + clamp -----
        steer = angular_input * self.max_steer

        # จำกัดช่วง -0.873 ถึง 0.873
        steer = max(-self.max_steer, min(self.max_steer, steer))

        # เอาแค่ทศนิยม 3 ตำแหน่ง
        twist.angular.z = round(steer, 3)

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = JoyToCmdVel()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()