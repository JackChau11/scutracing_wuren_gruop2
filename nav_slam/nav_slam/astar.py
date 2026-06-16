#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import math
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')
        self.declare_parameter('lane_width', 3.0)
        self.declare_parameter('min_points', 3)
        self.declare_parameter('max_range', 1.8)          # 缩小到1.8米，只取近处
        self.declare_parameter('z_min', -2.5)
        self.declare_parameter('z_max', -0.5)
        self.declare_parameter('debug_interval', 10)
        self.declare_parameter('x_diff_threshold', 0.5)   # 左右平均x差阈值
        self.lane_width = self.get_parameter('lane_width').value
        self.min_pts = self.get_parameter('min_points').value
        self.max_range = self.get_parameter('max_range').value
        self.z_min = self.get_parameter('z_min').value
        self.z_max = self.get_parameter('z_max').value
        self.debug_interval = self.get_parameter('debug_interval').value
        self.x_diff_threshold = self.get_parameter('x_diff_threshold').value

        self.lidar_sub = self.create_subscription(PointCloud2, '/points_raw', self.lidar_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/path', 10)

        self.x = self.y = self.yaw = 0.0
        self.last_k = 0.0
        self.last_b = 0.0
        self.frame_count = 0

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def lidar_callback(self, msg):
        self.frame_count += 1
        points = []
        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            x, y, z = p
            if 0.1 < x < self.max_range and abs(y) < 6.0 and self.z_min <= z <= self.z_max:
                points.append((x, y))

        if self.frame_count % self.debug_interval == 0:
            self.get_logger().info(f"Frame {self.frame_count}: Raw points in ROI = {len(points)}")

        if len(points) < self.min_pts * 2:
            if self.frame_count % self.debug_interval == 0:
                self.get_logger().warn(f"Too few points ({len(points)}), using straight path")
            path = self.generate_straight_path()
            self.publish_path(path)
            return

        left_pts = [(x, y) for x, y in points if y > 0]
        right_pts = [(x, y) for x, y in points if y < 0]

        if self.frame_count % self.debug_interval == 0:
            self.get_logger().info(f"Left: {len(left_pts)}, Right: {len(right_pts)}")

        # 计算左右点云的平均 x
        mean_x_left = np.mean([p[0] for p in left_pts]) if left_pts else float('inf')
        mean_x_right = np.mean([p[0] for p in right_pts]) if right_pts else float('inf')

        # 如果某一侧点太少，直接使用另一侧
        if len(left_pts) < self.min_pts and len(right_pts) < self.min_pts:
            path = self.generate_straight_path()
            self.publish_path(path)
            return
        elif len(left_pts) < self.min_pts:
            # 只用右侧，向左偏移
            k_r, b_r = self.fit_line(right_pts)
            if k_r is None:
                path = self.generate_straight_path()
                self.publish_path(path)
                return
            k_c, b_c = k_r, b_r + self.lane_width / 2.0
        elif len(right_pts) < self.min_pts:
            # 只用左侧，向右偏移
            k_l, b_l = self.fit_line(left_pts)
            if k_l is None:
                path = self.generate_straight_path()
                self.publish_path(path)
                return
            k_c, b_c = k_l, b_l - self.lane_width / 2.0
        else:
            # 两侧都有足够点，判断纵向距离差
            if abs(mean_x_left - mean_x_right) > self.x_diff_threshold:
                # 选择较近的一侧（平均 x 较小）
                if mean_x_left < mean_x_right:
                    # 使用左侧，向右偏移
                    k_l, b_l = self.fit_line(left_pts)
                    if k_l is None:
                        path = self.generate_straight_path()
                        self.publish_path(path)
                        return
                    k_c, b_c = k_l, b_l - self.lane_width / 2.0
                    if self.frame_count % self.debug_interval == 0:
                        self.get_logger().info("Using left side (closer)")
                else:
                    # 使用右侧，向左偏移
                    k_r, b_r = self.fit_line(right_pts)
                    if k_r is None:
                        path = self.generate_straight_path()
                        self.publish_path(path)
                        return
                    k_c, b_c = k_r, b_r + self.lane_width / 2.0
                    if self.frame_count % self.debug_interval == 0:
                        self.get_logger().info("Using right side (closer)")
            else:
                # 纵向距离接近，可以两侧拟合取平均
                k_l, b_l = self.fit_line(left_pts)
                k_r, b_r = self.fit_line(right_pts)
                if k_l is None or k_r is None:
                    # 如果某侧拟合失败，回退
                    path = self.generate_straight_path()
                    self.publish_path(path)
                    return
                k_c = (k_l + k_r) / 2.0
                b_c = (b_l + b_r) / 2.0

        # 低通滤波
        alpha = 0.6
        k_c = alpha * k_c + (1 - alpha) * self.last_k
        b_c = alpha * b_c + (1 - alpha) * self.last_b
        self.last_k, self.last_b = k_c, b_c

        if self.frame_count % self.debug_interval == 0:
            self.get_logger().info(f"Center fit: k={k_c:.3f}, b={b_c:.3f}")

        # 生成路径
        step = 0.2
        path_pts = []
        for i in range(int(self.max_range / step) + 1):
            x_local = i * step
            y_local = k_c * x_local + b_c
            y_local = np.clip(y_local, -5.0, 5.0)
            path_pts.append((x_local, y_local))

        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        world_pts = []
        for px, py in path_pts:
            wx = self.x + px * cos_y - py * sin_y
            wy = self.y + px * sin_y + py * cos_y
            world_pts.append((wx, wy))

        self.publish_path(world_pts)

    def fit_line(self, pts):
        if len(pts) < self.min_pts:
            return None, None
        xs = np.array([p[0] for p in pts])
        ys = np.array([p[1] for p in pts])
        A = np.vstack([xs, np.ones(len(xs))]).T
        k, b = np.linalg.lstsq(A, ys, rcond=None)[0]
        return k, b

    def generate_straight_path(self):
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        step = 0.2
        pts = []
        for i in range(int(self.max_range / step) + 1):
            px = i * step
            py = 0.0
            wx = self.x + px * cos_y - py * sin_y
            wy = self.y + px * sin_y + py * cos_y
            pts.append((wx, wy))
        return pts

    def publish_path(self, world_pts):
        if len(world_pts) < 2:
            return
        msg = Path()
        msg.header.frame_id = 'odom'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in world_pts:
            pose = PoseStamped()
            pose.header.frame_id = 'odom'
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        self.path_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaneFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
