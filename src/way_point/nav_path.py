#!/usr/bin/env python3

from collections import deque
import csv
import math
import os
import statistics

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class CsvPathPublisher(Node):
    def __init__(self):
        super().__init__('csv_path_publisher')

        self.declare_parameter('csv_path', os.path.expanduser('~/path_data.csv'))
        self.declare_parameter('path_topic', '/nav_path')
        self.declare_parameter('display_path_topic', '/nav_path_display')
        self.declare_parameter('driven_path_topic', '/driven_path')
        self.declare_parameter('target_pose_topic', '/target_goal')
        self.declare_parameter('target_marker_topic', '/target_goal_marker')
        self.declare_parameter('enable_pose_tracking', False)
        self.declare_parameter('pose_topic', '/pcl_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('target_name_topic', '/target_name')
        self.declare_parameter('target_name_map', 'kmutt canteen:550')
        self.declare_parameter('target_timeout_sec', 0.0)
        self.declare_parameter('use_target_name', False)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('wheelbase', 1.8)
        self.declare_parameter('path_smoothing_enabled', True)
        self.declare_parameter('path_smoothing_median_window', 5)
        self.declare_parameter('path_smoothing_data_weight', 0.35)
        self.declare_parameter('path_smoothing_smooth_weight', 0.2)
        self.declare_parameter('path_smoothing_iterations', 120)
        self.declare_parameter('path_gap_fill_enabled', True)
        self.declare_parameter('path_gap_fill_max_spacing_m', 0.5)
        self.declare_parameter('turn_highlight_steer_deg', 8.0)
        self.declare_parameter('publish_period', 1.0)
        self.declare_parameter('graph_update_period', 0.2)
        self.declare_parameter('graph_pose_epsilon_m', 0.05)
        self.declare_parameter('graph_history_sec', 0.0)
        self.declare_parameter('measured_speed_min_mps', 0.05)
        self.declare_parameter('driven_path_min_spacing_m', 0.05)
        self.declare_parameter('speed_command_scale', 1.0)
        self.declare_parameter('speed_command_offset', 0.0)
        self.declare_parameter(
            'graph_path',
            os.path.expanduser('~/path_speed_steering.png')
        )

        self.csv_path = os.path.expanduser(self.get_parameter('csv_path').value)
        self.path_topic = self.get_parameter('path_topic').value
        self.display_path_topic = self.get_parameter('display_path_topic').value
        self.driven_path_topic = self.get_parameter('driven_path_topic').value
        self.target_pose_topic = self.get_parameter('target_pose_topic').value
        self.target_marker_topic = self.get_parameter('target_marker_topic').value
        self.enable_pose_tracking = bool(
            self.get_parameter('enable_pose_tracking').value
        )
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.target_name_topic = self.get_parameter('target_name_topic').value
        self.target_name_map_raw = self.get_parameter('target_name_map').value
        self.target_timeout_sec = max(
            float(self.get_parameter('target_timeout_sec').value),
            0.0
        )
        self.use_target_name = bool(self.get_parameter('use_target_name').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.wheelbase = float(self.get_parameter('wheelbase').value)
        self.path_smoothing_enabled = bool(
            self.get_parameter('path_smoothing_enabled').value
        )
        self.path_smoothing_median_window = max(
            int(self.get_parameter('path_smoothing_median_window').value),
            1
        )
        self.path_smoothing_data_weight = max(
            float(self.get_parameter('path_smoothing_data_weight').value),
            0.0
        )
        self.path_smoothing_smooth_weight = max(
            float(self.get_parameter('path_smoothing_smooth_weight').value),
            0.0
        )
        self.path_smoothing_iterations = max(
            int(self.get_parameter('path_smoothing_iterations').value),
            0
        )
        self.path_gap_fill_enabled = bool(
            self.get_parameter('path_gap_fill_enabled').value
        )
        self.path_gap_fill_max_spacing_m = max(
            float(self.get_parameter('path_gap_fill_max_spacing_m').value),
            1e-3
        )
        self.turn_highlight_steer_rad = math.radians(
            float(self.get_parameter('turn_highlight_steer_deg').value)
        )
        self.publish_period = float(self.get_parameter('publish_period').value)
        self.graph_update_period = float(
            self.get_parameter('graph_update_period').value
        )
        self.graph_pose_epsilon_m = float(
            self.get_parameter('graph_pose_epsilon_m').value
        )
        self.graph_history_sec = float(
            self.get_parameter('graph_history_sec').value
        )
        self.measured_speed_min_mps = max(
            float(self.get_parameter('measured_speed_min_mps').value),
            1e-3
        )
        self.driven_path_min_spacing_m = max(
            float(self.get_parameter('driven_path_min_spacing_m').value),
            0.0
        )
        self.speed_command_scale = max(
            float(self.get_parameter('speed_command_scale').value),
            1e-6
        )
        self.speed_command_offset = max(
            float(self.get_parameter('speed_command_offset').value),
            0.0
        )
        self.graph_path = os.path.expanduser(self.get_parameter('graph_path').value)

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.display_path_pub = self.create_publisher(Path, self.display_path_topic, 10)
        self.driven_path_pub = self.create_publisher(Path, self.driven_path_topic, 10)
        self.target_pose_pub = self.create_publisher(PoseStamped, self.target_pose_topic, 10)
        self.target_marker_pub = self.create_publisher(Marker, self.target_marker_topic, 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.display_path_msg = Path()
        self.display_path_msg.header.frame_id = self.frame_id
        self.driven_path_msg = Path()
        self.driven_path_msg.header.frame_id = self.frame_id
        self.last_loaded_signature = None
        self.full_path_poses = []
        self.full_path_x = []
        self.full_path_y = []
        self.full_path_z = []
        self.full_path_indices = []
        self.full_path_pose_stamp_sec = []
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.path_indices = []
        self.path_pose_stamp_sec = []
        self.path_heading_rad = []
        self.path_steering_rad = []
        self.driven_path_x = []
        self.driven_path_y = []
        self.driven_path_z = []
        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.pose_received = False
        self.last_graph_signature = None
        self.graph_start_sec = self.get_clock().now().nanoseconds * 1e-9
        self.prev_pose_x = None
        self.prev_pose_y = None
        self.prev_pose_z = None
        self.last_pose_stamp_sec = None
        self.command_history = deque()
        self.measured_history = deque()
        self.cte_history = deque()
        self.last_command_speed = None
        self.last_command_steer = None
        self.last_measured_speed = None
        self.last_measured_steer = None
        self.last_measured_turn_angle = None
        self.cumulative_turn_angle_rad = 0.0
        self.current_cte = None
        self.current_cte_state = None
        self.current_target_name = '-'
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None
        self.last_target_msg_sec = None
        self.target_name_map = self.parse_target_name_map(self.target_name_map_raw)

        self.pose_cov_subscription = None
        self.pose_stamped_subscription = None
        if self.enable_pose_tracking:
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
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        self.target_name_subscription = self.create_subscription(
            String,
            self.target_name_topic,
            self.target_name_callback,
            10
        )

        self.load_csv()
        self.timer = self.create_timer(self.publish_period, self.publish_path)
        self.graph_timer = self.create_timer(
            self.graph_update_period,
            self.update_graph
        )
        self.update_graph(force=True)

        self.get_logger().info(
            f'Publishing {self.path_topic} from {self.csv_path} in frame {self.frame_id}'
        )
        self.get_logger().info(f'Display path topic: {self.display_path_topic}')
        self.get_logger().info(f'Driven path topic: {self.driven_path_topic}')
        self.get_logger().info(f'Target pose topic: {self.target_pose_topic}')
        self.get_logger().info(f'Target marker topic: {self.target_marker_topic}')
        if self.enable_pose_tracking:
            self.get_logger().info(f'Pose topic: {self.pose_topic}')
        else:
            self.get_logger().info(
                f'Pose tracking disabled; not subscribing to {self.pose_topic}'
            )
        self.get_logger().info(f'CMD topic: {self.cmd_vel_topic}')
        self.get_logger().info(f'Target topic: {self.target_name_topic}')
        self.get_logger().info(f'Target map: {self.target_name_map_raw}')
        self.get_logger().info(
            f'Use target_name filtering: {self.use_target_name}'
        )
        if self.target_timeout_sec > 0.0:
            self.get_logger().info(f'Target timeout: {self.target_timeout_sec:.1f}s')
        else:
            self.get_logger().info(
                'Target timeout: disabled (keep latest target until a new one arrives or it is cleared)'
            )
        self.get_logger().info(f'Wheelbase: {self.wheelbase:.3f} m')
        self.get_logger().info(
            'Path smoothing: '
            f'enabled={self.path_smoothing_enabled}, '
            f'median_window={self.path_smoothing_median_window}, '
            f'data_weight={self.path_smoothing_data_weight:.2f}, '
            f'smooth_weight={self.path_smoothing_smooth_weight:.2f}, '
            f'iterations={self.path_smoothing_iterations}'
        )
        self.get_logger().info(
            'Path gap fill: '
            f'enabled={self.path_gap_fill_enabled}, '
            f'max_spacing={self.path_gap_fill_max_spacing_m:.2f} m'
        )
        self.get_logger().info(
            f'Speed graph mapping: target_mps = (cmd - {self.speed_command_offset:.3f}) / '
            f'{self.speed_command_scale:.3f}'
        )
        if self.graph_history_sec > 0.0:
            self.get_logger().info(
                f'Graph history window: last {self.graph_history_sec:.1f}s'
            )
        else:
            self.get_logger().info(
                'Graph history window: full run (no trimming)'
            )
        self.get_logger().info(
            f'Turn highlight threshold: {math.degrees(self.turn_highlight_steer_rad):.1f} deg'
        )
        self.get_logger().info(
            f'Driven path min spacing: {self.driven_path_min_spacing_m:.2f} m'
        )
        self.get_logger().info(f'Graph output: {self.graph_path}')

    def get_file_signature(self):
        try:
            stat = os.stat(self.csv_path)
        except FileNotFoundError:
            return None
        return (stat.st_mtime_ns, stat.st_size)

    def load_csv(self, force=False):
        if not os.path.exists(self.csv_path):
            self.get_logger().error(f'CSV not found: {self.csv_path}')
            return

        file_signature = self.get_file_signature()
        if not force and file_signature == self.last_loaded_signature:
            return

        poses = []
        path_x = []
        path_y = []
        path_z = []
        path_indices = []
        path_pose_stamp_sec = []
        with open(self.csv_path, 'r', newline='') as f:
            reader = csv.DictReader(f)
            for row_number, row in enumerate(reader):
                if not row.get('x') or not row.get('y'):
                    continue

                pose = PoseStamped()
                pose.header.frame_id = row.get('frame_id', self.frame_id) or self.frame_id

                try:
                    pose.pose.position.x = float(row['x'])
                    pose.pose.position.y = float(row['y'])
                    pose.pose.position.z = float(row.get('z', 0.0) or 0.0)
                    pose.pose.orientation.x = float(row.get('qx', 0.0) or 0.0)
                    pose.pose.orientation.y = float(row.get('qy', 0.0) or 0.0)
                    pose.pose.orientation.z = float(row.get('qz', 0.0) or 0.0)
                    pose.pose.orientation.w = float(row.get('qw', 1.0) or 1.0)
                except (TypeError, ValueError):
                    continue

                poses.append(pose)
                path_x.append(pose.pose.position.x)
                path_y.append(pose.pose.position.y)
                path_z.append(pose.pose.position.z)
                path_indices.append(self.parse_row_index(row.get('index'), row_number))
                pose_stamp_sec = self.parse_float(row.get('pose_stamp'))
                path_pose_stamp_sec.append(pose_stamp_sec)
                if pose_stamp_sec is not None and pose_stamp_sec > 0.0:
                    pose.header.stamp.sec = int(pose_stamp_sec)
                    pose.header.stamp.nanosec = int(
                        round((pose_stamp_sec - int(pose_stamp_sec)) * 1e9)
                    )

        inserted_pose_count = 0

        if self.path_smoothing_enabled and len(poses) >= 3:
            path_x, path_y = self.smooth_path_positions(path_x, path_y)

        if self.path_gap_fill_enabled and len(poses) >= 2:
            (
                poses,
                path_x,
                path_y,
                path_z,
                path_indices,
                path_pose_stamp_sec,
                inserted_pose_count,
            ) = self.fill_path_gaps(
                poses,
                path_x,
                path_y,
                path_z,
                path_indices,
                path_pose_stamp_sec,
            )

        if poses:
            self.update_pose_positions_and_headings(
                poses,
                path_x,
                path_y,
                path_z,
                path_pose_stamp_sec,
            )

        self.last_loaded_signature = file_signature
        self.full_path_poses = poses
        self.full_path_x = path_x
        self.full_path_y = path_y
        self.full_path_z = path_z
        self.full_path_indices = path_indices
        self.full_path_pose_stamp_sec = path_pose_stamp_sec
        self.rebuild_active_path()
        self.rebuild_display_path()
        self.refresh_target_waypoint()
        self.refresh_current_cte(record_history=False)
        self.update_graph()

        if inserted_pose_count > 0:
            self.get_logger().info(
                f'Loaded {len(poses)} poses from CSV ({inserted_pose_count} interpolated)'
            )
        else:
            self.get_logger().info(f'Loaded {len(poses)} poses from CSV')

    def publish_path(self):
        self.load_csv()
        self.check_target_timeout()
        self.rebuild_active_path()
        self.rebuild_display_path()

        now = self.get_clock().now().to_msg()
        self.path_msg.header.stamp = now
        self.path_msg.header.frame_id = self.frame_id
        self.display_path_msg.header.stamp = now
        self.display_path_msg.header.frame_id = self.frame_id
        self.driven_path_msg.header.stamp = now
        self.driven_path_msg.header.frame_id = self.frame_id

        for pose in self.path_msg.poses:
            pose.header.frame_id = self.frame_id

        for pose in self.display_path_msg.poses:
            pose.header.frame_id = self.frame_id

        for pose in self.driven_path_msg.poses:
            pose.header.frame_id = self.frame_id

        self.path_pub.publish(self.path_msg)
        self.display_path_pub.publish(self.display_path_msg)
        self.driven_path_pub.publish(self.driven_path_msg)
        self.publish_target_visualization(now)

    def pose_cov_callback(self, msg: PoseWithCovarianceStamped):
        self.update_current_pose(msg.header.stamp, msg.pose.pose)

    def pose_stamped_callback(self, msg: PoseStamped):
        self.update_current_pose(msg.header.stamp, msg.pose)

    def cmd_vel_callback(self, msg: Twist):
        sample_time_sec = self.get_clock().now().nanoseconds * 1e-9 - self.graph_start_sec
        speed_cmd = self.map_speed_cmd_to_target(float(msg.linear.x))
        steer_cmd = float(msg.angular.z)
        self.command_history.append(
            {
                't': sample_time_sec,
                'speed': speed_cmd,
                'steer': steer_cmd,
            }
        )
        self.last_command_speed = speed_cmd
        self.last_command_steer = steer_cmd
        self.trim_history(self.command_history)

    def target_name_callback(self, msg: String):
        if not self.use_target_name:
            if self.current_target_name != '-':
                self.clear_current_target()
            return

        self.last_target_msg_sec = self.get_clock().now().nanoseconds * 1e-9
        target_name = msg.data.strip() or '-'
        if target_name == self.current_target_name:
            self.rebuild_active_path()
            self.rebuild_display_path()
            self.refresh_current_cte(record_history=False)
            self.update_graph()
            return

        self.current_target_name = target_name
        self.refresh_target_waypoint(log_when_missing=True)
        self.refresh_current_cte(record_history=False)
        self.update_graph()

    def clear_current_target(self):
        self.current_target_name = '-'
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None
        self.last_target_msg_sec = None

    def check_target_timeout(self):
        if not self.use_target_name:
            return

        if self.target_timeout_sec <= 0.0:
            return

        if self.last_target_msg_sec is None or self.current_target_name in ('', '-'):
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_target_msg_sec <= self.target_timeout_sec:
            return

        timed_out_target = self.current_target_name
        self.clear_current_target()
        self.refresh_current_cte(record_history=False)
        self.get_logger().warn(
            f'Target "{timed_out_target}" timed out after {self.target_timeout_sec:.1f}s'
        )

    def publish_target_visualization(self, stamp):
        if not self.has_active_target():
            self.delete_target_marker(stamp)
            return

        target_pose = PoseStamped()
        target_pose.header.stamp = stamp
        target_pose.header.frame_id = self.frame_id
        target_pose.pose.position.x = float(self.target_waypoint_x)
        target_pose.pose.position.y = float(self.target_waypoint_y)
        target_pose.pose.position.z = 0.0
        target_pose.pose.orientation.w = 1.0
        self.target_pose_pub.publish(target_pose)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'target_goal'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = target_pose.pose
        marker.scale.x = 1.4
        marker.scale.y = 1.4
        marker.scale.z = 1.4
        marker.color.a = 0.9
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        self.target_marker_pub.publish(marker)

    def delete_target_marker(self, stamp):
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.ns = 'target_goal'
        marker.id = 0
        marker.action = Marker.DELETE
        self.target_marker_pub.publish(marker)

    def refresh_target_waypoint(self, log_when_missing=False):
        self.target_waypoint_x = None
        self.target_waypoint_y = None
        self.target_waypoint_index = None

        if not self.use_target_name:
            return

        normalized_target_name = self.normalize_target_name(self.current_target_name)
        if not normalized_target_name or normalized_target_name == '-':
            return

        mapped_index = self.target_name_map.get(normalized_target_name)
        if mapped_index is None:
            if log_when_missing:
                self.get_logger().warn(
                    f'Target "{self.current_target_name}" is not in target_name_map'
                )
            return

        for path_pos, path_index in enumerate(self.full_path_indices):
            if path_index != mapped_index:
                continue

            self.target_waypoint_index = mapped_index
            self.target_waypoint_x = self.full_path_x[path_pos]
            self.target_waypoint_y = self.full_path_y[path_pos]
            self.get_logger().info(
                f'Target "{self.current_target_name}" -> waypoint index {mapped_index} '
                f'at x={self.target_waypoint_x:.3f}, y={self.target_waypoint_y:.3f}'
            )
            return

        if log_when_missing:
            self.get_logger().warn(
                f'Target "{self.current_target_name}" maps to index {mapped_index}, '
                'but that index was not found in the loaded path'
            )

    def rebuild_active_path(self):
        self.path_msg = Path()
        self.path_msg.header.frame_id = self.frame_id
        self.path_msg.poses = [
            self.copy_pose_stamped(pose)
            for pose in self.full_path_poses
        ]
        self.path_x = self.full_path_x[:]
        self.path_y = self.full_path_y[:]
        self.path_z = self.full_path_z[:]
        self.path_indices = self.full_path_indices[:]
        self.path_pose_stamp_sec = self.full_path_pose_stamp_sec[:]
        self.path_heading_rad = self.compute_path_headings(self.path_x, self.path_y)
        self.path_steering_rad = self.compute_path_steering(self.path_x, self.path_y)

    def rebuild_display_path(self):
        self.display_path_msg = Path()
        self.display_path_msg.header.frame_id = self.frame_id

        if len(self.full_path_poses) < 2:
            return

        self.display_path_msg.poses = [
            self.copy_pose_stamped(pose)
            for pose in self.full_path_poses
        ]

    def update_current_pose(self, stamp, pose):
        first_pose = not self.pose_received
        pose_stamp_sec = self.stamp_to_sec(stamp)
        self.current_x = float(pose.position.x)
        self.current_y = float(pose.position.y)
        self.current_z = float(pose.position.z)
        current_yaw = self.quaternion_to_yaw(pose.orientation)
        self.pose_received = True

        if (
            self.prev_pose_x is not None
            and self.prev_pose_y is not None
            and self.prev_pose_z is not None
            and self.last_pose_stamp_sec is not None
            and self.current_yaw is not None
            and pose_stamp_sec > self.last_pose_stamp_sec + 1e-6
        ):
            dt = pose_stamp_sec - self.last_pose_stamp_sec
            dx = self.current_x - self.prev_pose_x
            dy = self.current_y - self.prev_pose_y
            dz = self.current_z - self.prev_pose_z
            measured_speed = math.sqrt(dx * dx + dy * dy + dz * dz) / dt
            delta_yaw = self.normalize_angle(current_yaw - self.current_yaw)
            self.cumulative_turn_angle_rad += delta_yaw
            yaw_rate = delta_yaw / dt
            if measured_speed >= self.measured_speed_min_mps:
                measured_steer = math.atan(
                    self.wheelbase * yaw_rate / max(measured_speed, 1e-6)
                )
            else:
                measured_steer = 0.0

            sample_time_sec = self.get_clock().now().nanoseconds * 1e-9 - self.graph_start_sec
            self.measured_history.append(
                {
                    't': sample_time_sec,
                    'speed': measured_speed,
                    'steer': measured_steer,
                    'turn_angle': self.cumulative_turn_angle_rad,
                }
            )
            self.trim_history(self.measured_history)
            self.last_measured_speed = measured_speed
            self.last_measured_steer = measured_steer
            self.last_measured_turn_angle = self.cumulative_turn_angle_rad

        self.prev_pose_x = self.current_x
        self.prev_pose_y = self.current_y
        self.prev_pose_z = self.current_z
        self.last_pose_stamp_sec = pose_stamp_sec
        self.current_yaw = current_yaw
        self.append_driven_path_pose(stamp, current_yaw)
        self.rebuild_active_path()
        self.refresh_current_cte()

        if first_pose:
            self.get_logger().info(
                f'Pose received on {self.pose_topic}: '
                f'x={self.current_x:.3f}, y={self.current_y:.3f}, z={self.current_z:.3f}'
            )

    def parse_float(self, value):
        if value in (None, ''):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    def parse_row_index(self, value, fallback_index):
        if value in (None, ''):
            return int(fallback_index)
        try:
            return int(float(value))
        except (TypeError, ValueError):
            return int(fallback_index)

    def normalize_target_name(self, target_name):
        return ' '.join((target_name or '').strip().lower().split())

    def parse_target_name_map(self, raw_value):
        target_map = {}
        for item in str(raw_value).split(';'):
            item = item.strip()
            if not item or ':' not in item:
                continue

            name, index_value = item.rsplit(':', 1)
            normalized_name = self.normalize_target_name(name)
            if not normalized_name:
                continue

            try:
                target_map[normalized_name] = int(index_value.strip())
            except ValueError:
                continue

        return target_map

    def has_active_target(self):
        return (
            self.current_target_name not in ('', '-')
            and self.target_waypoint_index is not None
        )

    def clear_active_path(self):
        self.path_msg.poses = []
        self.path_x = []
        self.path_y = []
        self.path_z = []
        self.path_indices = []
        self.path_pose_stamp_sec = []
        self.path_heading_rad = []
        self.path_steering_rad = []

    def copy_pose_stamped(self, source_pose):
        copied_pose = PoseStamped()
        copied_pose.header.frame_id = source_pose.header.frame_id
        copied_pose.header.stamp = source_pose.header.stamp
        copied_pose.pose.position.x = source_pose.pose.position.x
        copied_pose.pose.position.y = source_pose.pose.position.y
        copied_pose.pose.position.z = source_pose.pose.position.z
        copied_pose.pose.orientation.x = source_pose.pose.orientation.x
        copied_pose.pose.orientation.y = source_pose.pose.orientation.y
        copied_pose.pose.orientation.z = source_pose.pose.orientation.z
        copied_pose.pose.orientation.w = source_pose.pose.orientation.w
        return copied_pose

    def make_pose_stamped(self, x, y, z, yaw):
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        self.set_pose_yaw(pose, yaw)
        return pose

    def append_driven_path_pose(self, stamp, yaw):
        if self.current_x is None or self.current_y is None or self.current_z is None:
            return

        new_pose = self.make_pose_stamped(
            self.current_x,
            self.current_y,
            self.current_z,
            yaw,
        )
        new_pose.header.stamp = stamp

        if self.driven_path_x:
            dx = self.current_x - self.driven_path_x[-1]
            dy = self.current_y - self.driven_path_y[-1]
            dz = self.current_z - self.driven_path_z[-1]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
            if distance < self.driven_path_min_spacing_m:
                self.driven_path_msg.poses[-1] = new_pose
                self.driven_path_x[-1] = self.current_x
                self.driven_path_y[-1] = self.current_y
                self.driven_path_z[-1] = self.current_z
                return

        self.driven_path_msg.poses.append(new_pose)
        self.driven_path_x.append(self.current_x)
        self.driven_path_y.append(self.current_y)
        self.driven_path_z.append(self.current_z)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, q):
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq < 1e-12:
            return 0.0
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stamp_to_sec(self, stamp):
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds * 1e-9

    def trim_history(self, history):
        if not history or self.graph_history_sec <= 0.0:
            return

        newest_time = history[-1]['t']
        cutoff_time = newest_time - self.graph_history_sec
        while history and history[0]['t'] < cutoff_time:
            history.popleft()

    def fill_path_gaps(
        self,
        poses,
        path_x,
        path_y,
        path_z,
        path_indices,
        path_pose_stamp_sec,
    ):
        if len(poses) < 2 or self.path_gap_fill_max_spacing_m <= 1e-6:
            return (
                poses[:],
                path_x[:],
                path_y[:],
                path_z[:],
                path_indices[:],
                path_pose_stamp_sec[:],
                0,
            )

        filled_poses = [self.copy_pose_stamped(poses[0])]
        filled_x = [path_x[0]]
        filled_y = [path_y[0]]
        filled_z = [path_z[0]]
        filled_indices = [path_indices[0]]
        filled_pose_stamp_sec = [path_pose_stamp_sec[0]]
        inserted_pose_count = 0

        for index in range(len(poses) - 1):
            start_x = path_x[index]
            start_y = path_y[index]
            start_z = path_z[index]
            end_x = path_x[index + 1]
            end_y = path_y[index + 1]
            end_z = path_z[index + 1]
            segment_length = math.hypot(end_x - start_x, end_y - start_y)
            insert_count = max(
                int(math.ceil(segment_length / self.path_gap_fill_max_spacing_m)) - 1,
                0,
            )

            for step in range(1, insert_count + 1):
                ratio = step / float(insert_count + 1)
                filled_poses.append(
                    self.make_pose_stamped(
                        self.interpolate_value(start_x, end_x, ratio),
                        self.interpolate_value(start_y, end_y, ratio),
                        self.interpolate_value(start_z, end_z, ratio),
                        0.0,
                    )
                )
                filled_x.append(self.interpolate_value(start_x, end_x, ratio))
                filled_y.append(self.interpolate_value(start_y, end_y, ratio))
                filled_z.append(self.interpolate_value(start_z, end_z, ratio))
                filled_indices.append(path_indices[index])
                filled_pose_stamp_sec.append(
                    self.interpolate_optional_value(
                        path_pose_stamp_sec[index],
                        path_pose_stamp_sec[index + 1],
                        ratio,
                    )
                )
                inserted_pose_count += 1

            filled_poses.append(self.copy_pose_stamped(poses[index + 1]))
            filled_x.append(end_x)
            filled_y.append(end_y)
            filled_z.append(end_z)
            filled_indices.append(path_indices[index + 1])
            filled_pose_stamp_sec.append(path_pose_stamp_sec[index + 1])

        return (
            filled_poses,
            filled_x,
            filled_y,
            filled_z,
            filled_indices,
            filled_pose_stamp_sec,
            inserted_pose_count,
        )

    def smooth_path_positions(self, path_x, path_y):
        filtered_x = self.apply_median_filter(
            path_x,
            self.path_smoothing_median_window
        )
        filtered_y = self.apply_median_filter(
            path_y,
            self.path_smoothing_median_window
        )

        if self.path_smoothing_iterations <= 0 or (
            self.path_smoothing_data_weight <= 0.0
            and self.path_smoothing_smooth_weight <= 0.0
        ):
            return filtered_x, filtered_y

        smoothed_x = filtered_x[:]
        smoothed_y = filtered_y[:]
        reference_x = filtered_x[:]
        reference_y = filtered_y[:]

        for _ in range(self.path_smoothing_iterations):
            for index in range(1, len(smoothed_x) - 1):
                smoothed_x[index] += (
                    self.path_smoothing_data_weight
                    * (reference_x[index] - smoothed_x[index])
                    + self.path_smoothing_smooth_weight
                    * (
                        smoothed_x[index - 1]
                        + smoothed_x[index + 1]
                        - 2.0 * smoothed_x[index]
                    )
                )
                smoothed_y[index] += (
                    self.path_smoothing_data_weight
                    * (reference_y[index] - smoothed_y[index])
                    + self.path_smoothing_smooth_weight
                    * (
                        smoothed_y[index - 1]
                        + smoothed_y[index + 1]
                        - 2.0 * smoothed_y[index]
                    )
                )

        smoothed_x[0] = path_x[0]
        smoothed_y[0] = path_y[0]
        smoothed_x[-1] = path_x[-1]
        smoothed_y[-1] = path_y[-1]
        return smoothed_x, smoothed_y

    def apply_median_filter(self, values, window_size):
        if len(values) < 3 or window_size <= 1:
            return values[:]

        if window_size % 2 == 0:
            window_size += 1
        half_window = window_size // 2
        filtered = values[:]

        for index in range(1, len(values) - 1):
            start_index = max(0, index - half_window)
            end_index = min(len(values), index + half_window + 1)
            filtered[index] = statistics.median(values[start_index:end_index])

        return filtered

    def update_pose_positions_and_headings(
        self,
        poses,
        path_x,
        path_y,
        path_z,
        path_pose_stamp_sec,
    ):
        headings = self.compute_path_headings(path_x, path_y)
        for index, pose in enumerate(poses):
            pose.pose.position.x = path_x[index]
            pose.pose.position.y = path_y[index]
            pose.pose.position.z = path_z[index]
            pose.header.frame_id = self.frame_id
            pose_stamp_sec = self.parse_float(
                None if index >= len(path_pose_stamp_sec) else path_pose_stamp_sec[index]
            )
            if pose_stamp_sec is not None and pose_stamp_sec > 0.0:
                pose.header.stamp.sec = int(pose_stamp_sec)
                pose.header.stamp.nanosec = int(
                    round((pose_stamp_sec - int(pose_stamp_sec)) * 1e9)
                )
            self.set_pose_yaw(pose, headings[index])

    def set_pose_yaw(self, pose_stamped, yaw):
        half_yaw = 0.5 * yaw
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = math.sin(half_yaw)
        pose_stamped.pose.orientation.w = math.cos(half_yaw)

    def map_speed_cmd_to_target(self, speed_cmd):
        speed_cmd = max(float(speed_cmd), 0.0)
        if speed_cmd <= self.speed_command_offset + 1e-6:
            return 0.0
        return max(
            0.0,
            (speed_cmd - self.speed_command_offset) / self.speed_command_scale
        )

    def interpolate_value(self, start_value, end_value, ratio):
        return start_value + (end_value - start_value) * ratio

    def interpolate_optional_value(self, start_value, end_value, ratio):
        if start_value is None and end_value is None:
            return None
        if start_value is None:
            return end_value
        if end_value is None:
            return start_value
        return self.interpolate_value(start_value, end_value, ratio)

    def find_closest_path_state_in_series(self, path_x, path_y, x, y):
        if len(path_x) < 2:
            return None

        best_state = None
        best_dist = float('inf')
        for index in range(len(path_x) - 1):
            x1 = path_x[index]
            y1 = path_y[index]
            x2 = path_x[index + 1]
            y2 = path_y[index + 1]
            seg_x = x2 - x1
            seg_y = y2 - y1
            seg_len_sq = seg_x * seg_x + seg_y * seg_y

            if seg_len_sq < 1e-9:
                proj_ratio = 0.0
                proj_x = x1
                proj_y = y1
            else:
                proj_ratio = ((x - x1) * seg_x + (y - y1) * seg_y) / seg_len_sq
                proj_ratio = max(0.0, min(1.0, proj_ratio))
                proj_x = x1 + proj_ratio * seg_x
                proj_y = y1 + proj_ratio * seg_y

            dist = math.hypot(proj_x - x, proj_y - y)
            if dist < best_dist:
                best_dist = dist
                best_state = {
                    'index': index,
                    'proj_ratio': proj_ratio,
                    'proj_x': proj_x,
                    'proj_y': proj_y,
                    'distance': dist,
                }

        return best_state

    def find_closest_path_state(self, x, y):
        return self.find_closest_path_state_in_series(self.path_x, self.path_y, x, y)

    def compute_signed_cte(self, closest_state):
        if closest_state is None or len(self.path_x) < 2:
            return None

        start_index = closest_state['index']
        x1 = self.path_x[start_index]
        y1 = self.path_y[start_index]
        x2 = self.path_x[start_index + 1]
        y2 = self.path_y[start_index + 1]
        seg_x = x2 - x1
        seg_y = y2 - y1
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return None

        cross = seg_x * (self.current_y - y1) - seg_y * (self.current_x - x1)
        return -cross / seg_len

    def refresh_current_cte(self, record_history=True):
        if not self.pose_received or len(self.path_x) < 2:
            self.current_cte = None
            self.current_cte_state = None
            return

        closest_state = self.find_closest_path_state(self.current_x, self.current_y)
        signed_cte = self.compute_signed_cte(closest_state)
        self.current_cte = signed_cte
        self.current_cte_state = closest_state
        if signed_cte is None or not record_history:
            return

        sample_time_sec = self.get_clock().now().nanoseconds * 1e-9 - self.graph_start_sec
        self.cte_history.append(
            {
                't': sample_time_sec,
                'cte': signed_cte,
            }
        )
        self.trim_history(self.cte_history)

    def compute_path_headings(self, path_x, path_y):
        point_count = len(path_x)
        if point_count == 0:
            return []
        if point_count == 1:
            return [0.0]

        headings = []
        for index in range(point_count):
            if index == 0:
                dx = path_x[1] - path_x[0]
                dy = path_y[1] - path_y[0]
            elif index == point_count - 1:
                dx = path_x[-1] - path_x[-2]
                dy = path_y[-1] - path_y[-2]
            else:
                dx = path_x[index + 1] - path_x[index - 1]
                dy = path_y[index + 1] - path_y[index - 1]

            if abs(dx) < 1e-9 and abs(dy) < 1e-9:
                heading = headings[-1] if headings else 0.0
            else:
                heading = math.atan2(dy, dx)
            headings.append(heading)

        return headings

    def compute_path_steering(self, path_x, path_y):
        point_count = len(path_x)
        if point_count == 0:
            return []
        if point_count < 3:
            return [0.0] * point_count

        headings = self.compute_path_headings(path_x, path_y)
        steering_rad = [0.0] * point_count

        for index in range(1, point_count - 1):
            dx_prev = path_x[index] - path_x[index - 1]
            dy_prev = path_y[index] - path_y[index - 1]
            dx_next = path_x[index + 1] - path_x[index]
            dy_next = path_y[index + 1] - path_y[index]
            ds_prev = math.sqrt(dx_prev * dx_prev + dy_prev * dy_prev)
            ds_next = math.sqrt(dx_next * dx_next + dy_next * dy_next)
            ds = 0.5 * (ds_prev + ds_next)

            if ds <= 1e-6:
                steering_rad[index] = steering_rad[index - 1]
                continue

            d_heading = self.normalize_angle(headings[index + 1] - headings[index - 1])
            curvature = d_heading / max(ds_prev + ds_next, 1e-6)
            steering_rad[index] = math.atan(self.wheelbase * curvature)

        steering_rad[0] = steering_rad[1]
        steering_rad[-1] = steering_rad[-2]
        return steering_rad

    def find_turn_highlight_points(self, path_x, path_y, path_steering_rad):
        if len(path_x) < 3 or len(path_steering_rad) != len(path_x):
            return []

        highlight_points = []
        run_start = None

        for index, steer_rad in enumerate(path_steering_rad):
            is_turn = abs(steer_rad) >= self.turn_highlight_steer_rad
            if is_turn and run_start is None:
                run_start = index
            elif not is_turn and run_start is not None:
                highlight_points.append(
                    self.pick_turn_peak_index(
                        path_x,
                        path_y,
                        path_steering_rad,
                        run_start,
                        index - 1,
                    )
                )
                run_start = None

        if run_start is not None:
            highlight_points.append(
                self.pick_turn_peak_index(
                    path_x,
                    path_y,
                    path_steering_rad,
                    run_start,
                    len(path_steering_rad) - 1,
                )
            )

        return highlight_points

    def pick_turn_peak_index(
        self,
        path_x,
        path_y,
        path_steering_rad,
        start_index,
        end_index,
    ):
        peak_index = start_index
        peak_abs_steer = abs(path_steering_rad[start_index])

        for index in range(start_index + 1, end_index + 1):
            steer_abs = abs(path_steering_rad[index])
            if steer_abs > peak_abs_steer:
                peak_abs_steer = steer_abs
                peak_index = index

        return {
            'index': peak_index,
            'x': path_x[peak_index],
            'y': path_y[peak_index],
            'steer_rad': path_steering_rad[peak_index],
        }

    def build_speed_colors(self, path_x, path_y, path_z, path_pose_stamp_sec):
        segment_speeds = []
        for index in range(max(len(path_x) - 1, 0)):
            start_stamp = path_pose_stamp_sec[index]
            end_stamp = path_pose_stamp_sec[index + 1]
            if start_stamp is None or end_stamp is None:
                segment_speeds.append(None)
                continue

            dt = end_stamp - start_stamp
            if dt <= 1e-6:
                segment_speeds.append(None)
                continue

            dx = path_x[index + 1] - path_x[index]
            dy = path_y[index + 1] - path_y[index]
            dz = path_z[index + 1] - path_z[index]
            segment_speeds.append(math.sqrt(dx * dx + dy * dy + dz * dz) / dt)

        valid_speeds = [speed for speed in segment_speeds if speed is not None]
        if not valid_speeds:
            return None, None, None

        min_speed = min(valid_speeds)
        max_speed = max(valid_speeds)
        if max_speed - min_speed < 1e-6:
            return ['tab:green'] * max(len(path_x) - 1, 0), min_speed, max_speed

        low_threshold = min_speed + (max_speed - min_speed) / 3.0
        high_threshold = min_speed + 2.0 * (max_speed - min_speed) / 3.0

        segment_colors = []
        last_valid_color = 'tab:green'
        for segment_speed in segment_speeds:
            if segment_speed is None:
                segment_colors.append(last_valid_color)
                continue

            if segment_speed < low_threshold:
                color = 'tab:green'
            elif segment_speed < high_threshold:
                color = 'gold'
            else:
                color = 'tab:red'

            segment_colors.append(color)
            last_valid_color = color

        return segment_colors, low_threshold, high_threshold

    def update_graph(self, force=False):
        self.load_csv()
        self.check_target_timeout()

        graph_signature = self.build_graph_signature()
        graph_exists = os.path.exists(self.graph_path)
        if not force and graph_exists and graph_signature == self.last_graph_signature:
            return

        graph_dir = os.path.dirname(self.graph_path)
        if graph_dir:
            os.makedirs(graph_dir, exist_ok=True)

        fig, axes = plt.subplots(
            4,
            1,
            figsize=(12, 15),
            gridspec_kw={'height_ratios': [2.1, 1.0, 1.0, 0.9]},
        )
        ax = axes[0]
        speed_ax = axes[1]
        steer_ax = axes[2]
        cte_ax = axes[3]
        graph_path_x = self.full_path_x
        graph_path_y = self.full_path_y
        graph_path_z = self.full_path_z
        graph_path_pose_stamp_sec = self.full_path_pose_stamp_sec
        driven_path_x = self.driven_path_x
        driven_path_y = self.driven_path_y
        graph_path_steering_rad = (
            self.compute_path_steering(graph_path_x, graph_path_y)
            if graph_path_x else []
        )
        turn_highlight_points = self.find_turn_highlight_points(
            graph_path_x,
            graph_path_y,
            graph_path_steering_rad,
        )

        segment_colors, low_threshold, high_threshold = self.build_speed_colors(
            graph_path_x,
            graph_path_y,
            graph_path_z,
            graph_path_pose_stamp_sec,
        )
        if len(graph_path_x) >= 2 and segment_colors is not None:
            segments = [
                [
                    (graph_path_x[index], graph_path_y[index]),
                    (graph_path_x[index + 1], graph_path_y[index + 1]),
                ]
                for index in range(len(graph_path_x) - 1)
            ]
            path_collection = LineCollection(
                segments,
                colors=segment_colors,
                linewidths=4.0,
                capstyle='round',
                joinstyle='round',
                zorder=2,
            )
            ax.add_collection(path_collection)
        else:
            if graph_path_x:
                ax.plot(
                    graph_path_x,
                    graph_path_y,
                    color='tab:green',
                    linewidth=4.0,
                    solid_capstyle='round',
                    solid_joinstyle='round',
                    zorder=2,
                )
        if len(driven_path_x) >= 2:
            ax.plot(
                driven_path_x,
                driven_path_y,
                color='magenta',
                linewidth=2.4,
                linestyle='--',
                solid_capstyle='round',
                solid_joinstyle='round',
                zorder=3,
            )
        target_title = 'Path XY Plot'
        if self.current_target_name not in ('', '-'):
            target_title += f' | target: {self.current_target_name}'
        if self.target_waypoint_index is not None:
            target_title += f' | target idx: {self.target_waypoint_index}'
        ax.set_title(target_title)
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')

        if graph_path_x and graph_path_y:
            ax.scatter(
                graph_path_x[0],
                graph_path_y[0],
                color='tab:blue',
                s=40,
                label='start',
                zorder=4,
            )
            ax.scatter(
                graph_path_x[-1],
                graph_path_y[-1],
                color='tab:red',
                s=40,
                label='end',
                zorder=4,
            )

        if turn_highlight_points:
            ax.scatter(
                [point['x'] for point in turn_highlight_points],
                [point['y'] for point in turn_highlight_points],
                color='deepskyblue',
                s=110,
                marker='o',
                edgecolors='black',
                linewidths=0.8,
                label='turn point',
                zorder=5,
            )
        elif not graph_path_x:
            ax.text(
                0.5,
                0.5,
                'Waiting for path...',
                transform=ax.transAxes,
                ha='center',
                va='center',
                alpha=0.7,
            )

        if self.target_waypoint_x is not None and self.target_waypoint_y is not None:
            ax.scatter(
                self.target_waypoint_x,
                self.target_waypoint_y,
                color='lime',
                s=120,
                marker='*',
                edgecolors='black',
                linewidths=0.8,
                label='target',
                zorder=6,
            )

        if self.pose_received:
            ax.scatter(
                self.current_x,
                self.current_y,
                color='magenta',
                s=60,
                marker='x',
                linewidths=2.0,
                label='vehicle',
                zorder=5,
            )
            if self.current_cte_state is not None and self.current_cte is not None:
                ax.plot(
                    [self.current_x, self.current_cte_state['proj_x']],
                    [self.current_y, self.current_cte_state['proj_y']],
                    color='cyan',
                    linewidth=2.0,
                    linestyle='--',
                    zorder=4,
                )
                text_x = 0.5 * (self.current_x + self.current_cte_state['proj_x'])
                text_y = 0.5 * (self.current_y + self.current_cte_state['proj_y'])
                ax.text(
                    text_x,
                    text_y,
                    f'CTE {self.current_cte:+.2f} m',
                    color='cyan',
                    fontsize=10,
                    bbox={
                        'facecolor': 'black',
                        'alpha': 0.35,
                        'edgecolor': 'none',
                        'pad': 2.5,
                    },
                    zorder=6,
                )

        legend_handles = [
            Line2D([0], [0], color='tab:green', lw=2.5, label='slow'),
            Line2D([0], [0], color='gold', lw=2.5, label='medium'),
            Line2D([0], [0], color='tab:red', lw=2.5, label='fast'),
            Line2D([0], [0], marker='o', color='tab:blue', linestyle='None', label='start'),
            Line2D([0], [0], marker='o', color='tab:red', linestyle='None', label='end'),
        ]
        if turn_highlight_points:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    marker='o',
                    color='deepskyblue',
                    markeredgecolor='black',
                    linestyle='None',
                    markersize=8,
                    label='turn point',
                )
            )
        if self.target_waypoint_x is not None and self.target_waypoint_y is not None:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    marker='*',
                    color='lime',
                    markeredgecolor='black',
                    linestyle='None',
                    markersize=10,
                    label='target',
                )
            )
        if self.pose_received:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    marker='x',
                    color='magenta',
                    linestyle='None',
                    markersize=8,
                    markeredgewidth=2.0,
                    label='vehicle',
                )
            )
        if len(driven_path_x) >= 2:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color='magenta',
                    lw=2.4,
                    linestyle='--',
                    label='driven path',
                )
            )
        if self.current_cte_state is not None and self.current_cte is not None:
            legend_handles.append(
                Line2D(
                    [0],
                    [0],
                    color='cyan',
                    lw=2.0,
                    linestyle='--',
                    label='cte',
                )
            )

        legend_title = 'Speed classes'
        if low_threshold is not None and high_threshold is not None:
            legend_title = (
                'Speed classes\n'
                f'slow < {low_threshold:.2f} m/s\n'
                f'medium < {high_threshold:.2f} m/s\n'
                f'fast >= {high_threshold:.2f} m/s'
            )
        ax.legend(handles=legend_handles, loc='best', title=legend_title)

        self.plot_measured_graph(
            speed_ax,
            title='Measured Speed',
            ylabel='Speed (m/s)',
            key='speed',
            line_color='tab:orange',
        )
        self.plot_measured_graph(
            steer_ax,
            title='Vehicle Turn Angle',
            ylabel='Turn angle (deg)',
            key='turn_angle',
            line_color='tab:red',
            value_scale=180.0 / math.pi,
        )
        self.plot_cte_graph(cte_ax)

        fig.tight_layout()
        graph_root, graph_ext = os.path.splitext(self.graph_path)
        temp_graph_path = f'{graph_root}.tmp{graph_ext or ".png"}'
        fig.savefig(temp_graph_path, dpi=150)
        plt.close(fig)
        os.replace(temp_graph_path, self.graph_path)
        self.last_graph_signature = graph_signature

    def build_graph_signature(self):
        pose_signature = None
        if self.pose_received:
            pose_signature = (
                round(self.current_x / self.graph_pose_epsilon_m)
                if self.graph_pose_epsilon_m > 0.0 else self.current_x,
                round(self.current_y / self.graph_pose_epsilon_m)
                if self.graph_pose_epsilon_m > 0.0 else self.current_y,
            )

        return (
            self.last_loaded_signature,
            len(self.full_path_x),
            len(self.driven_path_x),
            pose_signature,
            self.current_target_name,
            None if self.target_waypoint_x is None else round(self.target_waypoint_x, 3),
            None if self.target_waypoint_y is None else round(self.target_waypoint_y, 3),
            self.target_waypoint_index,
            None if not self.driven_path_x else round(self.driven_path_x[-1], 3),
            None if not self.driven_path_y else round(self.driven_path_y[-1], 3),
            len(self.command_history),
            len(self.measured_history),
            len(self.cte_history),
            self.round_history_sample(self.command_history, 'speed'),
            self.round_history_sample(self.command_history, 'steer'),
            self.round_history_sample(self.measured_history, 'speed'),
            self.round_history_sample(self.measured_history, 'turn_angle'),
            self.round_history_sample(self.cte_history, 'cte'),
            None if self.current_cte is None else round(self.current_cte, 3),
        )

    def round_history_sample(self, history, key):
        if not history:
            return None
        sample = history[-1]
        return (
            round(sample['t'], 2),
            round(sample[key], 3),
        )

    def plot_measured_graph(
        self,
        ax,
        title,
        ylabel,
        key,
        line_color,
        value_scale=1.0,
    ):
        meas_t = [sample['t'] for sample in self.measured_history]
        meas_v = [sample[key] * value_scale for sample in self.measured_history]

        if meas_t:
            ax.plot(
                meas_t,
                meas_v,
                color=line_color,
                linewidth=1.8,
            )

        if meas_t:
            start_time = meas_t[0]
            latest_time = meas_t[-1]
            ax.set_xlim(start_time, max(latest_time, start_time + 1.0))
        else:
            ax.text(
                0.5,
                0.5,
                'Waiting for data...',
                transform=ax.transAxes,
                ha='center',
                va='center',
                alpha=0.7,
            )

        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
        if key in ('steer', 'turn_angle'):
            ax.axhline(0.0, color='black', linewidth=0.8, alpha=0.3)
        ax.set_xlabel('Time (s)')

    def plot_cte_graph(self, ax):
        cte_t = [sample['t'] for sample in self.cte_history]
        cte_v = [sample['cte'] for sample in self.cte_history]

        if cte_t:
            ax.plot(
                cte_t,
                cte_v,
                color='cyan',
                linewidth=1.8,
                label='cte',
            )
            ax.legend(loc='upper left')
            start_time = cte_t[0]
            latest_time = cte_t[-1]
            ax.set_xlim(start_time, max(latest_time, start_time + 1.0))
        else:
            ax.text(
                0.5,
                0.5,
                'Waiting for CTE...',
                transform=ax.transAxes,
                ha='center',
                va='center',
                alpha=0.7,
            )

        ax.set_title('Cross-Track Error')
        ax.set_ylabel('CTE (m)')
        ax.set_xlabel('Time (s)')
        ax.grid(True, alpha=0.3)
        ax.axhline(0.0, color='black', linewidth=0.8, alpha=0.3)


def main(args=None):
    rclpy.init(args=args)
    node = CsvPathPublisher()

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
