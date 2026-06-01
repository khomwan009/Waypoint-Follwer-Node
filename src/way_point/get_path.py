#!/usr/bin/env python3

from collections import deque
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TwistStamped
from sensor_msgs.msg import Imu
import csv
import os


class PathToCSV(Node):

    def __init__(self):
        super().__init__('path_to_csv')

        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('speed_topic', '/filter/twist')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('yaw_offset', 0.0)
        self.declare_parameter('file_path', os.path.expanduser('~/path_data.csv'))
        self.declare_parameter('min_spacing_m', 0.3)
        self.declare_parameter('append_to_file', False)
        self.declare_parameter('max_speed_age_sec', 0.2)
        self.declare_parameter('max_imu_age_sec', 0.2)
        self.declare_parameter('drop_unsynced_samples', False)

        self.file_path = os.path.expanduser(self.get_parameter('file_path').value)
        self.pose_topic = self.get_parameter('pose_topic').value
        self.speed_topic = self.get_parameter('speed_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.min_spacing_m = float(self.get_parameter('min_spacing_m').value)
        self.append_to_file = bool(self.get_parameter('append_to_file').value)
        self.max_speed_age_sec = float(self.get_parameter('max_speed_age_sec').value)
        self.max_imu_age_sec = float(self.get_parameter('max_imu_age_sec').value)
        self.drop_unsynced_samples = bool(
            self.get_parameter('drop_unsynced_samples').value
        )
        self.saved_waypoint_count = 0
        self.last_written_position = None
        self.speed_samples = deque(maxlen=200)
        self.imu_samples = deque(maxlen=200)
        self.last_unsynced_warn_ns = 0

        file_dir = os.path.dirname(self.file_path)
        if file_dir:
            os.makedirs(file_dir, exist_ok=True)

        if (not self.append_to_file) or (not os.path.exists(self.file_path)):
            with open(self.file_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(
                    [
                        'index',
                        'pose_stamp',
                        'frame_id',
                        'x',
                        'y',
                        'z',
                        'qx',
                        'qy',
                        'qz',
                        'qw',
                        'speed_mps',
                        'steering_angle_rad',
                        'imu_yaw_rad',
                        'speed_age_sec',
                        'steering_age_sec',
                        'imu_age_sec',
                    ]
                )

        self.pose_cov_subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            self.pose_topic,
            self.pose_cov_callback,
            10
        )
        self.pose_stamped_subscription = self.create_subscription(
            PoseStamped,
            self.pose_topic,
            self.pose_stamped_callback,
            10
        )
        self.speed_subscription = self.create_subscription(
            TwistStamped,
            self.speed_topic,
            self.speed_callback,
            10
        )
        self.imu_subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f'Pose logger started on {self.pose_topic}, writing to {self.file_path}, '
            f'min_spacing={self.min_spacing_m:.2f} m, '
            f'speed_topic={self.speed_topic}, imu_topic={self.imu_topic}, '
            f'max_speed_age={self.max_speed_age_sec:.3f}s, '
            f'max_imu_age={self.max_imu_age_sec:.3f}s'
        )

    def speed_callback(self, msg: TwistStamped):
        speed_stamp = self.stamp_to_sec(msg.header.stamp)
        self.speed_samples.append((speed_stamp, float(msg.twist.linear.x)))

    def imu_callback(self, msg: Imu):
        imu_stamp = self.stamp_to_sec(msg.header.stamp)
        yaw = self.quaternion_to_yaw(msg.orientation)
        imu_yaw = self.normalize_angle(yaw + self.yaw_offset)
        self.imu_samples.append((imu_stamp, imu_yaw))

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        self.save_pose(msg.header, msg.pose.pose)

    def pose_stamped_callback(self, msg: PoseStamped):
        self.save_pose(msg.header, msg.pose)

    def save_pose(self, header, pose):
        pose_stamp = self.stamp_to_sec(header.stamp)
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)

        if self.last_written_position is not None:
            dx = x - self.last_written_position[0]
            dy = y - self.last_written_position[1]
            dz = z - self.last_written_position[2]
            if (dx * dx + dy * dy + dz * dz) ** 0.5 < self.min_spacing_m:
                return

        speed_mps, speed_age_sec = self.find_closest_sample(
            self.speed_samples, pose_stamp, self.max_speed_age_sec
        )
        imu_yaw_rad, imu_age_sec = self.find_closest_sample(
            self.imu_samples, pose_stamp, self.max_imu_age_sec
        )
        steering_angle_rad = imu_yaw_rad
        steering_age_sec = imu_age_sec

        if self.drop_unsynced_samples and (speed_mps is None or imu_yaw_rad is None):
            self.log_unsynced_pose(speed_age_sec, imu_age_sec)
            return

        frame_id = header.frame_id
        qx = float(pose.orientation.x)
        qy = float(pose.orientation.y)
        qz = float(pose.orientation.z)
        qw = float(pose.orientation.w)

        with open(self.file_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    self.saved_waypoint_count,
                    pose_stamp,
                    frame_id,
                    x,
                    y,
                    z,
                    qx,
                    qy,
                    qz,
                    qw,
                    speed_mps if speed_mps is not None else '',
                    steering_angle_rad if steering_angle_rad is not None else '',
                    imu_yaw_rad if imu_yaw_rad is not None else '',
                    speed_age_sec if speed_age_sec is not None else '',
                    steering_age_sec if steering_age_sec is not None else '',
                    imu_age_sec if imu_age_sec is not None else '',
                ]
            )

        self.saved_waypoint_count += 1
        self.last_written_position = (x, y, z)
        self.get_logger().info(
            f'Saved waypoint {self.saved_waypoint_count} from {self.pose_topic} '
            f'at x={x:.3f}, y={y:.3f}'
        )

    def stamp_to_sec(self, stamp):
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds * 1e-9

    def find_closest_sample(self, samples, target_stamp_sec, max_age_sec):
        best_value = None
        best_age_sec = None

        for sample_stamp_sec, sample_value in reversed(samples):
            age_sec = abs(sample_stamp_sec - target_stamp_sec)
            if age_sec <= max_age_sec and (
                best_age_sec is None or age_sec < best_age_sec
            ):
                best_value = sample_value
                best_age_sec = age_sec

        return best_value, best_age_sec

    def log_unsynced_pose(self, speed_age_sec, imu_age_sec):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_unsynced_warn_ns < 1_000_000_000:
            return

        self.last_unsynced_warn_ns = now_ns
        self.get_logger().warn(
            'Skipped pose because synced speed/imu data was not available within '
            f'age limits: speed_age={speed_age_sec}, imu_age={imu_age_sec}'
        )

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)

    node = PathToCSV()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
