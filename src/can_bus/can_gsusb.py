#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import time
import math
import os

CAN_IFACE = "can0"   # ✅ USB-CAN (gs_usb) ของคุณขึ้นเป็น can2


class SteeringNode(Node):
    def __init__(self):
        super().__init__('steering_node')

        # 🔧 ตั้งค่า CAN bitrate = 250000
        self.setup_can_interface()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.last_sent_time = 0.0
        self.message_interval = 0.5
        self.max_angle_deg = 50.0

        # scale ซ้ายขวา
        self.scale_pos = 180
        self.scale_neg = 320

        # ================= PID =================
        self.kp = 0.8
        self.ki = 0.0
        self.kd = 0.1

        self.integral = 0.0
        self.prev_error = 0.0
        self.current_angle = 0.0   # angle ที่ระบบใช้จริง (smoothed)

        # เปิด motor ตอน start
        self.send_motor_on_message()

    # =====================================================
    # Setup CAN
    # =====================================================
    def setup_can_interface(self):
        self.get_logger().info(f"🔧 Setting up {CAN_IFACE} bitrate = 250000")

        os.system(f"sudo ip link set {CAN_IFACE} down")
        os.system(f"sudo ip link set {CAN_IFACE} type can bitrate 250000 restart-ms 100")
        os.system(f"sudo ip link set {CAN_IFACE} txqueuelen 100000")
        os.system(f"sudo ip link set {CAN_IFACE} up")

        time.sleep(1.0)

    # =====================================================
    # Motor ON
    # =====================================================
    def send_motor_on_message(self):
        toggle_message = "230D200100000000"
        can_id = "06000001"
        formatted_frame = f"{can_id}#{toggle_message}"
        try:
            subprocess.run(['cansend', CAN_IFACE, formatted_frame], check=True)
            self.get_logger().info("✅ Motor turned ON")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to turn on motor: {e}")

    # =====================================================
    # Callback
    # =====================================================
    def cmd_vel_callback(self, msg):
        current_time = time.time()

        if current_time - self.last_sent_time < self.message_interval:
            return

        # ==========================================
        # Target angle from cmd_vel
        # ==========================================
        target_angle = math.degrees(msg.angular.z)
        target_angle = max(-self.max_angle_deg, min(self.max_angle_deg, target_angle))

        # ==========================================
        # PID smoothing
        # ==========================================
        dt = current_time - self.last_sent_time
        if dt <= 0.0:
            dt = 0.01

        error = target_angle - self.current_angle

        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        pid_output = (
            self.kp * error +
            self.ki * self.integral +
            self.kd * derivative
        )

        # update smoothed angle
        self.current_angle += pid_output

        # clamp กันหลุด
        self.current_angle = max(-self.max_angle_deg, min(self.max_angle_deg, self.current_angle))

        self.prev_error = error
        angle_deg = self.current_angle

        # ==========================================
        # scale
        # ==========================================
        if angle_deg >= 0:
            scaled_value = int(angle_deg * self.scale_pos)
        else:
            scaled_value = int(angle_deg * self.scale_neg)

        scaled_value = max(-15000, min(15000, scaled_value))

        # ==========================================
        # hex build
        # ==========================================
        if scaled_value < 0:
            hex_value = hex((1 << 16) + scaled_value)[2:].upper().zfill(4)  # ✅ fix
            direction = 'FFFF'
        else:
            hex_value = f"{scaled_value:04X}"
            direction = '0000'

        final_output = f"23022001{hex_value}{direction}"

        # ✅ sanity: ต้อง 8 bytes = 16 hex chars
        if len(final_output) != 16:
            self.get_logger().error(f"❌ BAD payload len={len(final_output)} hex: {final_output}")
            return

        can_id = "06000001"
        formatted_frame = f"{can_id}#{final_output}"

        # ==========================================
        # send CAN
        # ==========================================
        try:
            subprocess.run(['cansend', CAN_IFACE, formatted_frame], check=True)
            self.get_logger().info(
                f"📤 Sent CAN({CAN_IFACE}): {formatted_frame} | "
                f"target: {target_angle:.2f}° | smooth: {angle_deg:.2f}°"
            )
        except Exception as e:
            self.get_logger().error(f"❌ Failed to send CAN message: {e}")

        self.last_sent_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = SteeringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
