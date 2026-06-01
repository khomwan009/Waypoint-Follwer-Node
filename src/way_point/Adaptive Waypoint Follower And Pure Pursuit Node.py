#!/usr/bin/env python3
import csv
import math
import os

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import (
    Point,
    PointStamped,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Path
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


class WaypointFollowerCmdVelIMU(Node):
    def __init__(self):
        super().__init__('waypoint_follower_cmdvel_imu')

        # เส้นทางหลักที่รถจะใช้ในการวิ่งตาม
        self.declare_parameter('path_topic', '/nav_path')
        # เส้นทางสำหรับใช้เป็นเงื่อนไขอนุญาตหรือบล็อกการวิ่งเท่านั้น
        self.declare_parameter('desired_path_topic', '/desired_path')
        # ถ้าเป็น True รถจะวิ่งเฉพาะตอนที่ desired_path ไม่ว่าง
        self.declare_parameter('use_desired_path_gate', True)
        # ถ้าพิกัด x/y ของ desired_path เกินค่านี้ ให้ถือว่าผิดปกติและสั่งหยุด
        self.declare_parameter('desired_path_coordinate_limit', 500.0)
        # topic ของตำแหน่งและทิศทางปัจจุบันของรถ
        self.declare_parameter('pose_topic', '/pcl_pose')
        # topic คำสั่งควบคุมที่ส่งออกสำหรับ linear.x และ angular.z
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        # global frame ที่ใช้สำหรับ static TF
        self.declare_parameter('world_frame', 'map')
        # frame ที่ใช้แสดง path, pose และ marker ใน RViz
        self.declare_parameter('output_frame', 'map')
        # base frame ของรถที่ใช้ตอน publish TF
        self.declare_parameter('base_frame', 'base_link')
        # เปิดการบันทึกข้อมูล controller ลง CSV
        self.declare_parameter('enable_csv_log', True)
        # โฟลเดอร์สำหรับเก็บไฟล์ CSV
        self.declare_parameter('csv_log_dir', 'csv_logs')

        # ระยะฐานล้อของรถที่ใช้ในสมการเลี้ยวแบบ pure pursuit
        self.declare_parameter('wheelbase', 1.8)
        # ค่าชดเชยมุม yaw แบบคงที่ที่บวกกับ pose ขาเข้า
        self.declare_parameter('yaw_offset', -0.045)
        # หยุดรถเมื่อระยะทางที่เหลือน้อยกว่าค่านี้
        self.declare_parameter('arrival_tolerance', 1.0)
        # คาบเวลาการอัปเดตของตัวควบคุม หน่วยเป็นวินาที
        self.declare_parameter('control_period', 0.12)
        # ถ้าไม่ได้รับ pose ใหม่ภายในเวลานี้ ให้ถือว่า pose เก่าเกินไปและหยุดคุมชั่วคราว
        self.declare_parameter('pose_stale_timeout', 0.40)
        # ถ้า pose เพิ่งค้างไม่นาน ให้คงคำสั่งล่าสุดไว้ชั่วคราวแทนการสั่งหยุดทันที
        self.declare_parameter('stale_pose_hold_timeout', 0.70)
        # ระยะทางต่ำสุดที่รถต้องเคลื่อนที่ก่อนเพิ่มจุดใหม่ใน vehicle_trace
        self.declare_parameter('trace_min_dist', 0.2)

        # ค่า threshold ของความโค้งสำหรับสลับจากโหมดตรงไปโหมดโค้ง
        self.declare_parameter('curve_curvature_threshold', 0.05)
        # ค่า threshold ของความโค้งสำหรับสลับจากโหมดโค้งไปโหมดโค้งมาก
        self.declare_parameter('hard_curve_curvature_threshold', 0.1)
        # จำนวนช่วงของ path ที่มองล่วงหน้าเพื่อประเมินความโค้งข้างหน้า
        self.declare_parameter('curvature_preview_points', 8)

        # ความเร็วเชิงเส้นที่ใช้ในโหมดตรง
        self.declare_parameter('straight_speed', 1.0)
        # ความเร็วเชิงเส้นที่ใช้ในโหมดโค้ง
        self.declare_parameter('curve_speed', 0.95)
        # ความเร็วเชิงเส้นที่ใช้ในโหมดโค้งมาก
        self.declare_parameter('hard_curve_speed', 0.9)

        # ระยะ lookahead ที่ใช้ในโหมดตรง
        self.declare_parameter('straight_lookahead', 4.0)
        # ระยะ lookahead ที่เพิ่มให้เฉพาะตอนวิ่งทางตรงและรถอยู่ใกล้ path
        self.declare_parameter('straight_lookahead_bonus', 0.0)
        # ระยะ lookahead ที่ใช้ในโหมดโค้ง
        self.declare_parameter('curve_lookahead', 3.5)
        # ระยะ lookahead ที่ใช้ในโหมดโค้งมาก
        self.declare_parameter('hard_curve_lookahead', 3.0)

        # ค่า gain ของ CTE ที่นำไปบวกกับ angular.z โดยตรงในโหมดตรง
        self.declare_parameter('straight_cte_gain', 0.20)
        # ตัวคูณลดแรง CTE ในโหมดตรงเพื่อกันการแก้ซ้ายขวาแรงเกินไป
        self.declare_parameter('straight_cte_gain_scale', 0.60)
        # ถ้า CTE ยังเล็กกว่าค่านี้ ให้ลดแรงแก้ CTE ลงอย่างมากเพื่อให้รถนิ่งบนทางตรง
        self.declare_parameter('straight_path_lock_cte_m', 0.12)
        # ค่า gain ของ CTE ที่นำไปบวกกับ angular.z โดยตรงในโหมดโค้ง
        self.declare_parameter('curve_cte_gain', 0.40)
        # ค่า gain ของ CTE ที่นำไปบวกกับ angular.z โดยตรงในโหมดโค้งมาก
        self.declare_parameter('hard_curve_cte_gain', 0.60)
        # CTE ต้องมากกว่าค่านี้ก่อนจึงจะเริ่มใช้เทอมแก้ไข
        self.declare_parameter('cte_activate_threshold', 0.1)

        # คำสั่งเลี้ยวสูงสุดในโหมดตรง
        self.declare_parameter('straight_max_steer_rad', 0.26)
        # ค่ากรองคำสั่งเลี้ยวในโหมดตรงเพื่อลดการแก้กลับไปมารวดเร็วเกินไป
        self.declare_parameter('straight_steer_filter_alpha', 0.60)
        # จำกัดอัตราการเปลี่ยนความเร็วคำสั่งเพื่อให้รถไม่เร่ง/เบรกกระชากเกินไป
        self.declare_parameter('linear_cmd_rate_limit', 0.80)
        # จำกัดอัตราการเปลี่ยนคำสั่งเลี้ยวเพื่อให้พวงมาลัยไม่หมุนเร็วเกินไป
        self.declare_parameter('angular_cmd_rate_limit', 0.90)
        # คำสั่งเลี้ยวสูงสุดในโหมดโค้ง
        self.declare_parameter('curve_max_steer_rad', 0.72)
        # คำสั่งเลี้ยวสูงสุดในโหมดโค้งมาก
        self.declare_parameter('hard_curve_max_steer_rad', 0.95)

        self.path_topic = self.get_parameter('path_topic').value
        self.desired_path_topic = self.get_parameter('desired_path_topic').value
        self.use_desired_path_gate = bool(
            self.get_parameter('use_desired_path_gate').value
        )
        self.desired_path_coordinate_limit = max(
            float(self.get_parameter('desired_path_coordinate_limit').value),
            0.0,
        )
        self.pose_topic = self.get_parameter('pose_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.world_frame = self.get_parameter('world_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.enable_csv_log = bool(self.get_parameter('enable_csv_log').value)
        self.csv_log_dir = str(self.get_parameter('csv_log_dir').value)

        self.wheelbase = max(float(self.get_parameter('wheelbase').value), 1e-3)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)
        self.arrival_tolerance = max(
            float(self.get_parameter('arrival_tolerance').value),
            0.05,
        )
        self.control_period = max(
            float(self.get_parameter('control_period').value),
            0.01,
        )
        self.pose_stale_timeout = max(
            float(self.get_parameter('pose_stale_timeout').value),
            self.control_period,
        )
        self.stale_pose_hold_timeout = max(
            float(self.get_parameter('stale_pose_hold_timeout').value),
            self.pose_stale_timeout,
        )
        self.trace_min_dist = max(
            float(self.get_parameter('trace_min_dist').value),
            0.0,
        )

        self.curve_curvature_threshold = max(
            float(self.get_parameter('curve_curvature_threshold').value),
            0.0,
        )
        self.hard_curve_curvature_threshold = max(
            float(self.get_parameter('hard_curve_curvature_threshold').value),
            self.curve_curvature_threshold,
        )
        self.curvature_preview_points = max(
            int(self.get_parameter('curvature_preview_points').value),
            1,
        )

        self.mode_configs = {
            'straight': {
                'speed': float(self.get_parameter('straight_speed').value),
                'lookahead': max(
                    float(self.get_parameter('straight_lookahead').value),
                    0.1,
                ),
                'cte_gain': float(self.get_parameter('straight_cte_gain').value),
                'max_steer': max(
                    float(self.get_parameter('straight_max_steer_rad').value),
                    0.01,
                ),
            },
            'curve': {
                'speed': float(self.get_parameter('curve_speed').value),
                'lookahead': max(
                    float(self.get_parameter('curve_lookahead').value),
                    0.1,
                ),
                'cte_gain': float(self.get_parameter('curve_cte_gain').value),
                'max_steer': max(
                    float(self.get_parameter('curve_max_steer_rad').value),
                    0.01,
                ),
            },
            'hard_curve': {
                'speed': float(self.get_parameter('hard_curve_speed').value),
                'lookahead': max(
                    float(self.get_parameter('hard_curve_lookahead').value),
                    0.1,
                ),
                'cte_gain': float(self.get_parameter('hard_curve_cte_gain').value),
                'max_steer': max(
                    float(self.get_parameter('hard_curve_max_steer_rad').value),
                    0.01,
                ),
            },
        }
        self.cte_activate_threshold = max(
            float(self.get_parameter('cte_activate_threshold').value),
            0.0,
        )
        self.straight_lookahead_bonus = max(
            float(self.get_parameter('straight_lookahead_bonus').value),
            0.0,
        )
        self.straight_cte_gain_scale = max(
            float(self.get_parameter('straight_cte_gain_scale').value),
            0.0,
        )
        self.straight_path_lock_cte_m = max(
            float(self.get_parameter('straight_path_lock_cte_m').value),
            1e-3,
        )
        self.straight_steer_filter_alpha = self.clamp(
            float(self.get_parameter('straight_steer_filter_alpha').value),
            0.0,
            1.0,
        )
        self.linear_cmd_rate_limit = max(
            float(self.get_parameter('linear_cmd_rate_limit').value),
            0.0,
        )
        self.angular_cmd_rate_limit = max(
            float(self.get_parameter('angular_cmd_rate_limit').value),
            0.0,
        )

        self.path_points_local = []
        self.path_ready = False
        self.path_follow_enabled = not self.use_desired_path_gate

        self.current_x = None
        self.current_y = None
        self.current_z = None
        self.current_yaw = None
        self.raw_pose_yaw = None
        self.pose_speed_mps = 0.0
        self.pose_received = False
        self.last_pose_update_time = None
        self.prev_pose_sample = None
        self.current_mode = None
        self.last_stop_reason = None
        self.prev_angular_cmd = 0.0
        self.prev_linear_cmd = 0.0
        self.last_control_debug = {}

        self.csv_file_handle = None
        self.csv_writer = None
        self.csv_log_path = None
        self.csv_start_time = None
        self.csv_row_index = 0

        self.vehicle_trace = Path()
        self.vehicle_trace.header.frame_id = self.output_frame
        self.last_trace_x = None
        self.last_trace_y = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10
        )
        self.desired_path_sub = self.create_subscription(
            Path, self.desired_path_topic, self.desired_path_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self.pose_callback, 10
        )
        self.pose_stamped_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.pose_stamped_callback, 10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.path_local_pub = self.create_publisher(Path, '/nav_path_local', 10)
        self.target_pub = self.create_publisher(PoseStamped, '/target_waypoint', 10)
        self.lookahead_point_pub = self.create_publisher(
            PointStamped, '/lookahead_point', 10
        )
        self.current_pose_pub = self.create_publisher(PoseStamped, '/current_pose', 10)
        self.vehicle_trace_pub = self.create_publisher(Path, '/vehicle_trace', 10)
        self.cte_marker_pub = self.create_publisher(Marker, '/cte_line', 10)
        self.lookahead_marker_pub = self.create_publisher(Marker, '/lookahead_marker', 10)

        self.timer = self.create_timer(self.control_period, self.timer_callback)

        self.publish_static_world_to_output_tf()
        self.setup_csv_logger()

        self.get_logger().info(f'Path topic: {self.path_topic}')
        self.get_logger().info(f'Desired path topic: {self.desired_path_topic}')
        self.get_logger().info(f'Pose topic: {self.pose_topic}')
        self.get_logger().info(f'CMD topic: {self.cmd_vel_topic}')
        self.get_logger().info(
            f'Desired path gate enabled: {self.use_desired_path_gate}'
        )
        if self.csv_log_path is not None:
            self.get_logger().info(f'CSV log path: {self.csv_log_path}')

    def desired_path_callback(self, msg: Path):
        if not self.use_desired_path_gate:
            return

        follow_enabled = len(msg.poses) > 0
        stop_reason = None

        for index, pose_stamped in enumerate(msg.poses):
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            z = float(pose_stamped.pose.position.z)

            if not self.is_finite_point(x, y, z):
                follow_enabled = False
                stop_reason = (
                    f'desired_path_invalid_point index={index} x={x} y={y} z={z}'
                )
                break

            if (
                abs(x) > self.desired_path_coordinate_limit
                or abs(y) > self.desired_path_coordinate_limit
            ):
                follow_enabled = False
                stop_reason = (
                    f'desired_path_limit_exceeded index={index} '
                    f'x={x:.3f} y={y:.3f} '
                    f'limit={self.desired_path_coordinate_limit:.1f}'
                )
                break

        if follow_enabled == self.path_follow_enabled:
            return

        self.path_follow_enabled = follow_enabled
        if self.path_follow_enabled:
            self.get_logger().info(
                f'Follow enabled by {self.desired_path_topic}'
            )
            return

        if stop_reason is not None:
            self.get_logger().warn(
                f'Follow disabled by {self.desired_path_topic}: {stop_reason}'
            )
            self.publish_inactive_state(stop_reason)
            return

        self.get_logger().info(
            f'Follow disabled by empty {self.desired_path_topic}'
        )
        self.publish_inactive_state('desired_path_disabled')

    def path_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self.path_points_local = []
            self.path_ready = False
            self.publish_empty_local_path()
            self.clear_cte_marker()
            self.clear_lookahead_marker()
            return

        points = []
        out = Path()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.output_frame

        for pose_stamped in msg.poses:
            x = float(pose_stamped.pose.position.x)
            y = float(pose_stamped.pose.position.y)
            z = float(pose_stamped.pose.position.z)

            if not self.is_finite_point(x, y, z):
                self.get_logger().warn('Ignore path update with invalid point')
                return

            if points and math.hypot(points[-1][0] - x, points[-1][1] - y) < 1e-6:
                continue

            points.append((x, y))

            copied = PoseStamped()
            copied.header = out.header
            copied.pose = pose_stamped.pose
            out.poses.append(copied)

        if len(points) < 2:
            self.path_points_local = []
            self.path_ready = False
            self.publish_empty_local_path()
            return

        self.path_points_local = points
        self.path_ready = True
        self.path_local_pub.publish(out)

    def pose_callback(self, msg: PoseWithCovarianceStamped):
        self.update_current_pose(msg.pose.pose.position, msg.pose.pose.orientation)

    def pose_stamped_callback(self, msg: PoseStamped):
        self.update_current_pose(msg.pose.position, msg.pose.orientation)

    def update_current_pose(self, position, orientation):
        now = self.get_clock().now()
        new_x = float(position.x)
        new_y = float(position.y)
        new_z = float(position.z)
        raw_yaw = self.quaternion_to_yaw(orientation)

        if self.prev_pose_sample is not None:
            dt = (now - self.prev_pose_sample['time']).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = new_x - self.prev_pose_sample['x']
                dy = new_y - self.prev_pose_sample['y']
                self.pose_speed_mps = math.hypot(dx, dy) / dt

        self.pose_received = True
        self.current_x = new_x
        self.current_y = new_y
        self.current_z = new_z
        self.raw_pose_yaw = raw_yaw
        self.current_yaw = self.normalize_angle(raw_yaw + self.yaw_offset)
        self.last_pose_update_time = now
        self.prev_pose_sample = {
            'x': new_x,
            'y': new_y,
            'time': now,
        }

    def timer_callback(self):
        if self.pose_received:
            self.publish_current_pose()
            self.publish_vehicle_tf()
            self.update_vehicle_trace_if_ready()

        if not self.pose_received:
            self.publish_inactive_state('waiting_for_pose')
            self.write_csv_row(status='waiting_for_pose')
            return

        if self.is_pose_stale():
            if self.should_hold_last_command():
                self.republish_last_command('stale_pose_hold')
                self.write_csv_row(status='stale_pose_hold')
            else:
                self.publish_inactive_state('stale_pose')
                self.write_csv_row(status='stale_pose')
            return

        if not self.path_follow_enabled:
            self.publish_inactive_state('desired_path_disabled')
            self.write_csv_row(status='desired_path_disabled')
            return

        if not self.path_ready:
            self.publish_inactive_state('path_not_ready')
            self.write_csv_row(status='path_not_ready')
            return

        closest_state = self.find_closest_path_state(self.current_x, self.current_y)
        if closest_state is None:
            self.publish_inactive_state('closest_state_not_found')
            self.write_csv_row(status='closest_state_not_found')
            return

        remaining_distance = self.compute_remaining_distance(closest_state)
        if remaining_distance <= self.arrival_tolerance:
            self.clear_cte_marker()
            self.clear_lookahead_marker()
            self.publish_stop('goal_reached')
            self.write_csv_row(
                status='goal_reached',
                remaining_distance=remaining_distance,
            )
            return

        curvature = self.compute_preview_curvature(closest_state['index'])
        mode = self.classify_mode(curvature)
        base_config = self.mode_configs[mode]

        if mode != self.current_mode:
            self.current_mode = mode
            self.get_logger().info(
                f'Mode: {mode} curvature={curvature:.4f}'
            )

        signed_cte = self.compute_signed_cte(closest_state)
        config = self.get_effective_mode_config(base_config)
        lookahead_target = self.find_lookahead_target(
            closest_state,
            config['lookahead'],
        )
        if lookahead_target is None:
            self.publish_stop('lookahead_not_found')
            self.write_csv_row(
                status='lookahead_not_found',
                mode=mode,
                signed_cte=signed_cte,
                curvature=curvature,
                remaining_distance=remaining_distance,
            )
            return

        angular_cmd = self.compute_angular_command(
            mode,
            lookahead_target,
            config,
        )

        self.publish_target(lookahead_target['x'], lookahead_target['y'])
        self.publish_lookahead_target(lookahead_target['x'], lookahead_target['y'])
        self.publish_cte_line(closest_state)

        cmd = Twist()
        cmd.linear.x = float(config['speed'])
        cmd.angular.z = float(angular_cmd)
        cmd = self.apply_command_limits(cmd)
        self.cmd_vel_pub.publish(cmd)
        self.prev_linear_cmd = cmd.linear.x
        self.prev_angular_cmd = cmd.angular.z
        self.last_stop_reason = None
        self.write_csv_row(
            status='control',
            mode=mode,
            signed_cte=signed_cte,
            cmd=cmd,
            curvature=curvature,
            remaining_distance=remaining_distance,
            lookahead_target=lookahead_target,
        )

    def get_effective_mode_config(self, base_config):
        return dict(base_config)

    def compute_angular_command(self, mode, lookahead_target, config):
        dx = lookahead_target['x'] - self.current_x
        dy = lookahead_target['y'] - self.current_y
        lookahead_distance = max(math.hypot(dx, dy), 1e-6)

        target_heading = math.atan2(dy, dx)
        alpha = self.normalize_angle(target_heading - self.current_yaw)
        pure_pursuit_term = math.atan2(
            2.0 * self.wheelbase * math.sin(alpha),
            lookahead_distance,
        )

        cte_term = 0.0
        raw_angular = pure_pursuit_term
        clamped_angular = self.clamp(
            raw_angular,
            -config['max_steer'],
            config['max_steer'],
        )
        filtered_angular = clamped_angular
        if mode == 'straight':
            filtered_angular = self.prev_angular_cmd + self.straight_steer_filter_alpha * (
                clamped_angular - self.prev_angular_cmd
            )
        self.last_control_debug = {
            'lookahead_distance': lookahead_distance,
            'target_heading_rad': target_heading,
            'alpha_rad': alpha,
            'pure_pursuit_term': pure_pursuit_term,
            'cte_term': cte_term,
            'raw_angular': raw_angular,
            'clamped_angular': clamped_angular,
            'filtered_angular': filtered_angular,
        }
        return filtered_angular

    def classify_mode(self, curvature):
        if curvature >= self.hard_curve_curvature_threshold:
            return 'hard_curve'
        if curvature >= self.curve_curvature_threshold:
            return 'curve'
        return 'straight'

    def find_closest_path_state(self, x, y):
        if len(self.path_points_local) < 2:
            return None

        best_state = None
        best_dist = float('inf')

        for index in range(len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[index]
            x2, y2 = self.path_points_local[index + 1]
            seg_x = x2 - x1
            seg_y = y2 - y1
            seg_len_sq = seg_x * seg_x + seg_y * seg_y

            if seg_len_sq < 1e-9:
                proj_ratio = 0.0
                proj_x = x1
                proj_y = y1
            else:
                proj_ratio = (
                    ((x - x1) * seg_x + (y - y1) * seg_y) / seg_len_sq
                )
                proj_ratio = self.clamp(proj_ratio, 0.0, 1.0)
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
                }

        return best_state

    def find_lookahead_target(self, closest_state, lookahead_distance):
        if len(self.path_points_local) < 2:
            return None

        start_index = closest_state['index']
        remaining = max(0.0, lookahead_distance)

        for index in range(start_index, len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[index]
            x2, y2 = self.path_points_local[index + 1]
            seg_len = math.hypot(x2 - x1, y2 - y1)
            if seg_len < 1e-9:
                continue

            start_ratio = closest_state['proj_ratio'] if index == start_index else 0.0
            usable_len = max(0.0, (1.0 - start_ratio) * seg_len)

            if remaining <= usable_len:
                ratio = start_ratio + remaining / seg_len
                return {
                    'x': x1 + ratio * (x2 - x1),
                    'y': y1 + ratio * (y2 - y1),
                    'index': index + 1,
                }

            remaining -= usable_len

        last_x, last_y = self.path_points_local[-1]
        return {
            'x': last_x,
            'y': last_y,
            'index': len(self.path_points_local) - 1,
        }

    def compute_remaining_distance(self, closest_state):
        total = math.hypot(
            closest_state['proj_x'] - self.current_x,
            closest_state['proj_y'] - self.current_y,
        )

        start_index = closest_state['index']
        x1, y1 = self.path_points_local[start_index]
        x2, y2 = self.path_points_local[start_index + 1]
        seg_len = math.hypot(x2 - x1, y2 - y1)
        total += max(0.0, (1.0 - closest_state['proj_ratio']) * seg_len)

        for index in range(start_index + 1, len(self.path_points_local) - 1):
            x1, y1 = self.path_points_local[index]
            x2, y2 = self.path_points_local[index + 1]
            total += math.hypot(x2 - x1, y2 - y1)

        return total

    def compute_signed_cte(self, closest_state):
        start_index = closest_state['index']
        x1, y1 = self.path_points_local[start_index]
        x2, y2 = self.path_points_local[start_index + 1]

        seg_x = x2 - x1
        seg_y = y2 - y1
        seg_len = math.hypot(seg_x, seg_y)
        if seg_len < 1e-6:
            return 0.0

        cross = seg_x * (self.current_y - y1) - seg_y * (self.current_x - x1)
        return -cross / seg_len

    def compute_preview_curvature(self, start_index):
        if len(self.path_points_local) < 3:
            return 0.0

        max_curvature = 0.0
        end_index = min(
            len(self.path_points_local) - 2,
            start_index + self.curvature_preview_points,
        )
        for index in range(start_index, end_index + 1):
            curvature = self.compute_curvature_at_index(index)
            if curvature > max_curvature:
                max_curvature = curvature
        return max_curvature

    def compute_curvature_at_index(self, index):
        if len(self.path_points_local) < 3:
            return 0.0

        i0 = max(0, index - 1)
        i1 = index
        i2 = min(len(self.path_points_local) - 1, index + 1)
        if i0 == i1 or i1 == i2:
            return 0.0

        x0, y0 = self.path_points_local[i0]
        x1, y1 = self.path_points_local[i1]
        x2, y2 = self.path_points_local[i2]

        h1 = math.atan2(y1 - y0, x1 - x0)
        h2 = math.atan2(y2 - y1, x2 - x1)
        ds = 0.5 * (
            math.hypot(x1 - x0, y1 - y0) +
            math.hypot(x2 - x1, y2 - y1)
        )
        if ds < 1e-6:
            return 0.0

        return abs(self.normalize_angle(h2 - h1)) / ds

    def publish_empty_local_path(self):
        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = self.output_frame
        self.path_local_pub.publish(empty_path)

    def publish_inactive_state(self, reason):
        self.clear_cte_marker()
        self.clear_lookahead_marker()
        self.publish_stop(reason)

    def publish_stop(self, reason=None):
        if reason is not None and reason != self.last_stop_reason:
            self.get_logger().warn(f'Publishing stop: {reason}')
        self.last_stop_reason = reason
        self.last_control_debug = {}

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.prev_linear_cmd = 0.0
        self.prev_angular_cmd = 0.0

    def is_pose_stale(self):
        if self.last_pose_update_time is None:
            return True

        pose_age = (
            self.get_clock().now() - self.last_pose_update_time
        ).nanoseconds * 1e-9
        return pose_age > self.pose_stale_timeout

    def get_pose_age_sec(self):
        if self.last_pose_update_time is None:
            return float('inf')
        return (
            self.get_clock().now() - self.last_pose_update_time
        ).nanoseconds * 1e-9

    def should_hold_last_command(self):
        if self.prev_linear_cmd <= 1e-3 and abs(self.prev_angular_cmd) <= 1e-3:
            return False
        return self.get_pose_age_sec() <= self.stale_pose_hold_timeout

    def republish_last_command(self, reason):
        if reason != self.last_stop_reason:
            self.get_logger().warn(f'Holding last command: {reason}')
        self.last_stop_reason = reason
        cmd = Twist()
        cmd.linear.x = float(self.prev_linear_cmd)
        cmd.angular.z = float(self.prev_angular_cmd)
        self.cmd_vel_pub.publish(cmd)

    def apply_command_limits(self, cmd):
        limited = Twist()
        linear_step = self.linear_cmd_rate_limit * self.control_period
        angular_step = self.angular_cmd_rate_limit * self.control_period
        limited.linear.x = self.limit_step(
            cmd.linear.x,
            self.prev_linear_cmd,
            linear_step,
        )
        limited.angular.z = self.limit_step(
            cmd.angular.z,
            self.prev_angular_cmd,
            angular_step,
        )
        return limited

    def limit_step(self, target, current, max_step):
        if max_step <= 0.0:
            return float(target)
        delta = target - current
        delta = self.clamp(delta, -max_step, max_step)
        return float(current + delta)

    def setup_csv_logger(self):
        if not self.enable_csv_log:
            return

        os.makedirs(self.csv_log_dir, exist_ok=True)
        index = 1
        while True:
            filename = f'{index:03d}.csv'
            candidate = os.path.join(self.csv_log_dir, filename)
            if not os.path.exists(candidate):
                self.csv_log_path = candidate
                break
            index += 1

        self.csv_file_handle = open(self.csv_log_path, 'w', newline='', encoding='utf-8')
        fieldnames = [
            'row_index',
            'status',
            'mode',
            'time_sec',
            'pose_age_sec',
            'x_m',
            'y_m',
            'z_m',
            'heading_rad',
            'heading_deg',
            'raw_pose_yaw_rad',
            'raw_pose_yaw_deg',
            'speed_mps',
            'cmd_linear_x',
            'cmd_angular_z',
            'cte_m',
            'curvature',
            'remaining_distance_m',
            'lookahead_x',
            'lookahead_y',
            'lookahead_distance_m',
            'target_heading_rad',
            'alpha_rad',
            'pure_pursuit_term',
            'cte_term',
            'raw_angular',
            'clamped_angular',
            'filtered_angular',
            'path_ready',
            'path_follow_enabled',
            'pose_received',
        ]
        self.csv_writer = csv.DictWriter(self.csv_file_handle, fieldnames=fieldnames)
        self.csv_writer.writeheader()
        self.csv_file_handle.flush()
        self.csv_start_time = self.get_clock().now()

    def write_csv_row(
        self,
        status,
        mode='',
        signed_cte=None,
        cmd=None,
        curvature=None,
        remaining_distance=None,
        lookahead_target=None,
    ):
        if self.csv_writer is None or self.csv_start_time is None:
            return

        now = self.get_clock().now()
        pose_age_sec = ''
        if self.last_pose_update_time is not None:
            pose_age_sec = (now - self.last_pose_update_time).nanoseconds * 1e-9

        heading_rad = self.current_yaw if self.current_yaw is not None else ''
        raw_pose_yaw_rad = self.raw_pose_yaw if self.raw_pose_yaw is not None else ''
        heading_deg = (
            math.degrees(self.current_yaw)
            if self.current_yaw is not None
            else ''
        )
        raw_pose_yaw_deg = (
            math.degrees(self.raw_pose_yaw)
            if self.raw_pose_yaw is not None
            else ''
        )

        debug = self.last_control_debug if self.last_control_debug else {}
        self.csv_row_index += 1
        row = {
            'row_index': self.csv_row_index,
            'status': status,
            'mode': mode,
            'time_sec': (now - self.csv_start_time).nanoseconds * 1e-9,
            'pose_age_sec': pose_age_sec,
            'x_m': self.current_x if self.current_x is not None else '',
            'y_m': self.current_y if self.current_y is not None else '',
            'z_m': self.current_z if self.current_z is not None else '',
            'heading_rad': heading_rad,
            'heading_deg': heading_deg,
            'raw_pose_yaw_rad': raw_pose_yaw_rad,
            'raw_pose_yaw_deg': raw_pose_yaw_deg,
            'speed_mps': self.pose_speed_mps,
            'cmd_linear_x': cmd.linear.x if cmd is not None else 0.0,
            'cmd_angular_z': cmd.angular.z if cmd is not None else 0.0,
            'cte_m': signed_cte if signed_cte is not None else '',
            'curvature': curvature if curvature is not None else '',
            'remaining_distance_m': (
                remaining_distance if remaining_distance is not None else ''
            ),
            'lookahead_x': (
                lookahead_target['x'] if lookahead_target is not None else ''
            ),
            'lookahead_y': (
                lookahead_target['y'] if lookahead_target is not None else ''
            ),
            'lookahead_distance_m': debug.get('lookahead_distance', ''),
            'target_heading_rad': debug.get('target_heading_rad', ''),
            'alpha_rad': debug.get('alpha_rad', ''),
            'pure_pursuit_term': debug.get('pure_pursuit_term', ''),
            'cte_term': debug.get('cte_term', ''),
            'raw_angular': debug.get('raw_angular', ''),
            'clamped_angular': debug.get('clamped_angular', ''),
            'filtered_angular': debug.get('filtered_angular', ''),
            'path_ready': self.path_ready,
            'path_follow_enabled': self.path_follow_enabled,
            'pose_received': self.pose_received,
        }
        self.csv_writer.writerow(row)
        self.csv_file_handle.flush()

    def close_csv_logger(self):
        if self.csv_file_handle is None:
            return
        self.csv_file_handle.flush()
        self.csv_file_handle.close()
        self.csv_file_handle = None
        self.csv_writer = None

    def destroy_node(self):
        self.close_csv_logger()
        return super().destroy_node()

    def clear_cte_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = 'cte_line'
        marker.id = 0
        marker.action = Marker.DELETE
        self.cte_marker_pub.publish(marker)

    def clear_lookahead_marker(self):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = 'lookahead_target'
        marker.id = 0
        marker.action = Marker.DELETE
        self.lookahead_marker_pub.publish(marker)

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_static_world_to_output_tf(self):
        if self.world_frame == self.output_frame:
            return

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.output_frame
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)

    def publish_current_pose(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(self.current_x)
        msg.pose.position.y = float(self.current_y)
        msg.pose.position.z = float(self.current_z if self.current_z is not None else 0.0)
        msg.pose.orientation = self.yaw_to_quaternion(self.current_yaw)
        self.current_pose_pub.publish(msg)

    def publish_vehicle_tf(self):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.output_frame
        tf_msg.child_frame_id = self.base_frame
        tf_msg.transform.translation.x = float(self.current_x)
        tf_msg.transform.translation.y = float(self.current_y)
        tf_msg.transform.translation.z = float(self.current_z if self.current_z is not None else 0.0)
        tf_msg.transform.rotation = self.yaw_to_quaternion(self.current_yaw)
        self.tf_broadcaster.sendTransform(tf_msg)

    def update_vehicle_trace_if_ready(self):
        if self.current_x is None or self.current_y is None or self.current_yaw is None:
            return

        if self.last_trace_x is not None and self.last_trace_y is not None:
            dist = math.hypot(
                self.current_x - self.last_trace_x,
                self.current_y - self.last_trace_y,
            )
            if dist < self.trace_min_dist:
                return

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.output_frame
        pose.pose.position.x = float(self.current_x)
        pose.pose.position.y = float(self.current_y)
        pose.pose.position.z = float(self.current_z if self.current_z is not None else 0.0)
        pose.pose.orientation = self.yaw_to_quaternion(self.current_yaw)

        self.vehicle_trace.header = pose.header
        self.vehicle_trace.poses.append(pose)
        self.vehicle_trace_pub.publish(self.vehicle_trace)

        self.last_trace_x = self.current_x
        self.last_trace_y = self.current_y

    def publish_cte_line(self, closest_state):
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.output_frame
        marker.ns = 'cte_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.08
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1

        current_point = Point()
        current_point.x = float(self.current_x)
        current_point.y = float(self.current_y)
        current_point.z = 0.1

        projected_point = Point()
        projected_point.x = float(closest_state['proj_x'])
        projected_point.y = float(closest_state['proj_y'])
        projected_point.z = 0.1

        marker.points = [current_point, projected_point]
        self.cte_marker_pub.publish(marker)

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.target_pub.publish(msg)

    def publish_lookahead_target(self, x, y):
        point_msg = PointStamped()
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.header.frame_id = self.output_frame
        point_msg.point.x = float(x)
        point_msg.point.y = float(y)
        self.lookahead_point_pub.publish(point_msg)

        marker = Marker()
        marker.header = point_msg.header
        marker.ns = 'lookahead_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.45
        marker.scale.y = 0.45
        marker.scale.z = 0.45
        marker.color.a = 0.95
        marker.color.r = 0.1
        marker.color.g = 0.9
        marker.color.b = 1.0
        self.lookahead_marker_pub.publish(marker)

    def quaternion_to_yaw(self, q):
        norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm_sq < 1e-12:
            return 0.0
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def is_finite_point(self, x, y, z):
        return math.isfinite(x) and math.isfinite(y) and math.isfinite(z)

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollowerCmdVelIMU()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
