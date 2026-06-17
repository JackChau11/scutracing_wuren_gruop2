#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
import threading
import sys
import select
import time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from fsd_common_msgs.msg import ConeDetections
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time

class PerceptionPathPlanner(Node):
    def __init__(self):
        super().__init__('perception_path_planner')

        # ---------- 参数 ----------
        self.declare_parameter('lane_width', 3.0)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_range_lidar', 10.0)
        self.declare_parameter('max_range_cones', 15.0)
        self.declare_parameter('z_min', -2.5)
        self.declare_parameter('z_max', -0.5)
        self.declare_parameter('debug_interval', 10)
        self.declare_parameter('x_diff_threshold', 0.5)
        self.declare_parameter('confidence_threshold', 0.6)

        self.lane_width = self.get_parameter('lane_width').value
        self.min_pts = self.get_parameter('min_points').value
        self.max_range_lidar = self.get_parameter('max_range_lidar').value
        self.max_range_cones = self.get_parameter('max_range_cones').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.debug_interval = self.get_parameter('debug_interval').value
        self.x_diff_threshold = self.get_parameter('x_diff_threshold').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # ---------- 订阅者 ----------
        self.lidar_sub = self.create_subscription(PointCloud2, '/points_raw', self.lidar_callback, 10)
        self.cones_sub = self.create_subscription(ConeDetections, '/perception/cones', self.cones_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # ---------- 发布者 ----------
        self.path_pub = self.create_publisher(Path, '/path', 10)

        # ---------- TF ----------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 状态变量 ----------
        self.current_pose = None                 # (x, y, yaw) in odom
        self.lidar_points = []                  # (横向, 纵向)
        self.cone_data = []                     # (x_odom, y_odom, color, color_confidence)
        self.cone_confidence_ok = False
        self.mode = 'CONES'                     # 初始模式为感知节点
        self.last_k = 0.0
        self.last_b = 0.0
        self.frame_count = 0
        self.switch_requested = False
        self.switch_target = None
        self.running = True

        # 启动输入监听线程
        self.start_input_listener()

        self.get_logger().info('Perception Path Planner started in CONES mode (感知节点)')

    # ================== 回调函数 ==================
    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_pose = (msg.pose.pose.position.x,
                             msg.pose.pose.position.y,
                             yaw)
        self.x = self.current_pose[0]
        self.y = self.current_pose[1]
        self.yaw = self.current_pose[2]

    def lidar_callback(self, msg):
        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            orig_x, orig_y, z = p
            y_forward = orig_x       # 纵向
            x_lateral = orig_y       # 横向
            if 0.1 < y_forward < self.max_range_lidar and abs(x_lateral) < 6.0 and self.z_min <= z <= self.z_max:
                points.append((x_lateral, y_forward))
        self.lidar_points = points
        self.publish_path()

    def cones_callback(self, msg):
        if self.current_pose is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                'odom', 'base_link', Time(), timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return

        cone_odom = []
        all_conf_ok = True
        for cone in msg.cone_detections:
            ps = PointStamped()
            ps.header.stamp = msg.header.stamp
            ps.header.frame_id = 'base_link'
            ps.point = cone.position
            transformed = do_transform_point(ps, transform)
            x_odom = transformed.point.x
            y_odom = transformed.point.y
            color_conf = cone.color_confidence
            if color_conf < self.conf_thresh:
                all_conf_ok = False
            cone_odom.append((x_odom, y_odom, cone.color, color_conf))

        self.cone_data = cone_odom
        self.cone_confidence_ok = all_conf_ok and len(cone_odom) >= 4

        # 模式切换请求逻辑（仅在非等待状态时发起新请求）
        if not self.switch_requested:
            if self.mode == 'CONES' and not self.cone_confidence_ok:
                self.request_switch('LIDAR')
            elif self.mode == 'LIDAR' and self.cone_confidence_ok:
                self.request_switch('CONES')

        self.publish_path()

    # ================== 路径生成 ==================
    def publish_path(self):
        # 如果正在等待用户确认，立即停车，不生成任何路径
        if self.switch_requested:
            self.publish_empty_path()
            return

        if self.current_pose is None:
            return

        path = None
        if self.mode == 'CONES':
            path = self.generate_path_from_cones()
            if path is None:
                self.get_logger().warn('CONES path generation failed, falling back to LIDAR')
                path = self.generate_path_from_lidar()
        else:  # LIDAR
            path = self.generate_path_from_lidar()

        if path is not None:
            self.path_pub.publish(path)
        else:
            # 如果两种都失败，发布直线（避免停车）
            self.get_logger().warn('Both modes failed, publishing straight path')
            path = self.generate_straight_path()
            if path is not None:
                self.path_pub.publish(path)

    # ---------- 锥桶路径 ----------
    def generate_path_from_cones(self):
        if len(self.cone_data) < 4:
            return None

        vx, vy, vyaw = self.current_pose
        forward = (math.cos(vyaw), math.sin(vyaw))
        right = (-math.sin(vyaw), math.cos(vyaw))

        front_cones = []
        for (cx, cy, color, _) in self.cone_data:
            dx = cx - vx
            dy = cy - vy
            proj = dx * forward[0] + dy * forward[1]
            if proj < 0.5:
                continue
            lat = dx * right[0] + dy * right[1]
            front_cones.append((proj, lat, color))

        if len(front_cones) < 4:
            return None

        left_pts = [(p, l) for p, l, c in front_cones if c == 'red']
        right_pts = [(p, l) for p, l, c in front_cones if c == 'blue']

        if len(left_pts) < 2 or len(right_pts) < 2:
            return None

        left_pts = sorted(left_pts, key=lambda x: x[0])
        right_pts = sorted(right_pts, key=lambda x: x[0])

        left_s = [p[0] for p in left_pts]
        left_l = [p[1] for p in left_pts]
        right_s = [p[0] for p in right_pts]
        right_l = [p[1] for p in right_pts]

        deg = 1 if min(len(left_pts), len(right_pts)) == 2 else 2
        left_coeff = np.polyfit(left_s, left_l, deg)
        right_coeff = np.polyfit(right_s, right_l, deg)

        max_s = min(max(left_s), max(right_s))
        if max_s < 1.0:
            return None

        num_points = 50
        s_vals = np.linspace(0.0, max_s, num_points)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'

        for s in s_vals:
            l_left = np.polyval(left_coeff, s)
            l_right = np.polyval(right_coeff, s)
            l_mid = (l_left + l_right) / 2.0
            x = vx + s * forward[0] + l_mid * right[0]
            y = vy + s * forward[1] + l_mid * right[1]
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.get_logger().info(f'[CONES] Published path with {len(path_msg.poses)} points')
        return path_msg

    # ---------- 激光雷达路径 ----------
    def generate_path_from_lidar(self):
        points = self.lidar_points
        if len(points) < self.min_pts * 2:
            return self.generate_straight_path()

        left_pts = [(y, x) for x, y in points if x > 0]
        right_pts = [(y, x) for x, y in points if x < 0]

        if len(left_pts) < self.min_pts and len(right_pts) < self.min_pts:
            return self.generate_straight_path()

        mean_y_left = np.mean([p[0] for p in left_pts]) if left_pts else float('inf')
        mean_y_right = np.mean([p[0] for p in right_pts]) if right_pts else float('inf')

        if len(left_pts) < self.min_pts:
            k_r, b_r = self.fit_line(right_pts)
            if k_r is None:
                return self.generate_straight_path()
            k_c, b_c = k_r, b_r + self.lane_width / 2.0
        elif len(right_pts) < self.min_pts:
            k_l, b_l = self.fit_line(left_pts)
            if k_l is None:
                return self.generate_straight_path()
            k_c, b_c = k_l, b_l - self.lane_width / 2.0
        else:
            if abs(mean_y_left - mean_y_right) > self.x_diff_threshold:
                if mean_y_left < mean_y_right:
                    k_l, b_l = self.fit_line(left_pts)
                    if k_l is None:
                        return self.generate_straight_path()
                    k_c, b_c = k_l, b_l - self.lane_width / 2.0
                else:
                    k_r, b_r = self.fit_line(right_pts)
                    if k_r is None:
                        return self.generate_straight_path()
                    k_c, b_c = k_r, b_r + self.lane_width / 2.0
            else:
                k_l, b_l = self.fit_line(left_pts)
                k_r, b_r = self.fit_line(right_pts)
                if k_l is None or k_r is None:
                    return self.generate_straight_path()
                k_c = (k_l + k_r) / 2.0
                b_c = (b_l + b_r) / 2.0

        alpha = 0.6
        k_c = alpha * k_c + (1 - alpha) * self.last_k
        b_c = alpha * b_c + (1 - alpha) * self.last_b
        self.last_k, self.last_b = k_c, b_c

        step = 0.2
        path_pts_local = []
        for i in range(int(self.max_range_lidar / step) + 1):
            yf = i * step
            xl = k_c * yf + b_c
            xl = np.clip(xl, -5.0, 5.0)
            path_pts_local.append((yf, xl))

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        world_pts = []
        for yf, xl in path_pts_local:
            wx = self.x + yf * cos_y - xl * sin_y
            wy = self.y + yf * sin_y + xl * cos_y
            world_pts.append((wx, wy))

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for wx, wy in world_pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)

        self.get_logger().info(f'[LIDAR] Published path with {len(path_msg.poses)} points')
        return path_msg

    def fit_line(self, pts):
        if len(pts) < self.min_pts:
            return None, None
        ys = np.array([p[0] for p in pts])
        xs = np.array([p[1] for p in pts])
        A = np.vstack([ys, np.ones(len(ys))]).T
        k, b = np.linalg.lstsq(A, xs, rcond=None)[0]
        return k, b

    def generate_straight_path(self):
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        step = 0.2
        pts = []
        for i in range(int(self.max_range_lidar / step) + 1):
            yf = i * step
            xl = 0.0
            wx = self.x + yf * cos_y - xl * sin_y
            wy = self.y + yf * sin_y + xl * cos_y
            pts.append((wx, wy))
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        for wx, wy in pts:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        return path_msg

    # ================== 模式切换与用户输入 ==================
    def request_switch(self, target_mode):
        if self.switch_requested:
            return
        if target_mode == self.mode:
            return
        self.switch_requested = True
        self.switch_target = target_mode
        # 立即停车
        self.publish_empty_path()
        self.get_logger().warn(
            f"***** 系统建议切换到 {target_mode} 模式，是否确认？ (Y/n) *****"
        )

    def start_input_listener(self):
        def listener():
            while self.running:
                if self.switch_requested:
                    if sys.stdin.isatty():
                        if select.select([sys.stdin], [], [], 0.1)[0]:
                            ch = sys.stdin.readline().strip().lower()
                            if ch == 'y':
                                self.mode = self.switch_target
                                self.get_logger().info(f"已切换到 {self.mode} 模式")
                                self.switch_requested = False
                                # 切换后立即尝试发布路径（恢复行驶）
                                self.publish_path()
                            elif ch == 'n':
                                self.get_logger().info("切换取消，车辆停车")
                                self.publish_empty_path()
                                self.switch_requested = False
                                # 保持停车，不清除任何标志，车辆继续停止
                            else:
                                # 无效输入，重新提示（但避免刷屏，可忽略）
                                pass
                    else:
                        time.sleep(0.1)
                time.sleep(0.1)
        self.input_thread = threading.Thread(target=listener, daemon=True)
        self.input_thread.start()

    def publish_empty_path(self):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'odom'
        self.path_pub.publish(path_msg)
        # 仅当第一次停车或状态变化时打印，避免刷屏，这里不重复打印

    def destroy_node(self):
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
