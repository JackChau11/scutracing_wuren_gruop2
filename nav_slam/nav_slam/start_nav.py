import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import numpy as np
from scipy.spatial import KDTree
from nav_msgs.msg import Path
import yaml
import os

# 纯追踪控制器
class PurePursuitController:
    def __init__(self, lookahead_distance):
        self.lookahead_distance = lookahead_distance

    def calculate_steering_angle(self, vehicle_pose, path_points):
        # 找到离车辆最近的路径点
        closest_point_idx = KDTree(path_points[:, :2]).query(vehicle_pose[:2])[1]
        
        # 动态选择最合适的路径点作为目标点
        target_point = path_points[-1]  # 默认用最后一个点
        for i in range(len(path_points)):
            lookahead_point_idx = (closest_point_idx + i) % len(path_points)
            target_point = path_points[lookahead_point_idx]
            dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
            distance_to_target = math.sqrt(dx**2 + dy**2)
            if distance_to_target >= self.lookahead_distance:
                break
        
        # 计算车辆到目标点的向量
        dx, dy = target_point[0] - vehicle_pose[0], target_point[1] - vehicle_pose[1]
        # 计算目标角度
        target_angle = math.atan2(dy, dx)
        # 计算转向角
        steering_angle = target_angle - vehicle_pose[2]
        # 确保转向角在-pi到pi之间
        while steering_angle > math.pi:
            steering_angle -= 2 * math.pi
        while steering_angle < -math.pi:
            steering_angle += 2 * math.pi
        return steering_angle, target_point

# ROS 2节点
class PathFollowingNode(Node):
    def __init__(self):
        super().__init__('path_following_node')
        # 创建纯追踪控制器
        self.pure_pursuit = PurePursuitController(lookahead_distance=2.0)  # 增大前瞻
        # 创建路径点
        self.path_points = None
        # 创建订阅者
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        # 创建发布者
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # 创建路径点订阅者
        self.path_subscriber = self.create_subscription(Path, '/path', self.path_callback, 10)
        # 变量初始化
        self.current_odom = None
        self.stop_flag = False
        self.path_received = False
        self.cmd_count = 0  # 用于控制打印频率

    def path_callback(self, msg):
        self.path_points_list = [[point.pose.position.x, point.pose.position.y] for point in msg.poses]
        self.path_points = np.array(self.path_points_list)
        assert self.path_points.ndim == 2, "path_points must be a 2D array"
        self.path_points = self.interpolate_path(self.path_points, segment_length=0.05)
        self.path_received = True

    def interpolate_path(self, points, segment_length=0.1):
        interpolated_points = []
        for i in range(len(points) - 1):
            start_point = points[i]
            end_point = points[i+1]
            distance = np.linalg.norm(end_point - start_point)
            num_points = int(distance / segment_length) + 1
            t_values = np.linspace(0, 1, num_points)
            interpolated_segment = start_point + (end_point - start_point)[np.newaxis, :] * t_values[:, np.newaxis]
            interpolated_points.append(interpolated_segment)
        return np.vstack(interpolated_points)

    def quaternion_to_yaw(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def odometry_callback(self, msg):
        self.cmd_count += 1
        # 先提取车辆位姿
        pose = [msg.pose.pose.position.x, msg.pose.pose.position.y, self.quaternion_to_yaw(msg.pose.pose.orientation)]
        self.current_xy = pose[:2]

        if not self.path_received or self.path_points is None:
            return

        # 计算控制量
        steering_angle, target_point = self.pure_pursuit.calculate_steering_angle(pose, self.path_points)
        distance_to_end = np.linalg.norm(np.array(pose[:2]) - self.path_points[-1])

        # 停止条件
        if distance_to_end < 0.3:  # 增大停止阈值
            speed = 0.0
            steering_angle = 0.0
            self.path_received = False
            if self.cmd_count % 10 == 0:
                self.get_logger().info("Goal reached, stopping.")
        else:
            # 速度根据转向角调整
            speed = max(0.8, 1.5 - abs(steering_angle) / (math.pi/2))
            

        # 发布指令
        cmd_vel_msg = Twist()
        cmd_vel_msg.linear.x = float(speed)
        cmd_vel_msg.angular.z = float(steering_angle)
        self.cmd_vel_publisher.publish(cmd_vel_msg)

        # 每5帧打印一次控制信息（避免刷屏）
        if self.cmd_count % 5 == 0:
            self.get_logger().info(
                f"Cmd #{self.cmd_count}: v={speed:.2f}, ang={steering_angle:.3f}, "
                f"dist_to_end={distance_to_end:.2f}, target=({target_point[0]:.2f},{target_point[1]:.2f})"
            )

def main(args=None):
    rclpy.init(args=args)
    node = PathFollowingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
