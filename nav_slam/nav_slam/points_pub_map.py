#!/usr/bin/env python3
"""
节点: pointcloud_transform_node
功能: 将激光雷达点云从 base_link 坐标系转换到 world 坐标系，供建图节点使用

输入:
订阅 /points_raw (sensor_msgs/PointCloud2) : 原始激光雷达点云（base_link 坐标系）
订阅 /odom (nav_msgs/Odometry) : 里程计，用于获取车辆位姿

输出:
发布 /mapokk (sensor_msgs/PointCloud2) : 转换后的点云，frame_id = 'world'

逻辑:
  1. 缓存最新里程计，提取位置 (x,y,z) 和四元数。
  2. 根据四元数计算旋转矩阵，构建 4x4 变换矩阵（包括平移）。
  3. 收到点云后，将每个点 (x,y,z) 转为齐次坐标，乘以变换矩阵得到 world 坐标。
  4. 发布转换后的点云，frame_id 设置为 'world'（也可以通过参数 'frame_id' 修改）。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudTransformNode(Node):
    def __init__(self):
        super().__init__('pointcloud_transform_node')
        self.declare_parameter('frame_id', 'world')   
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/points_raw', self.pointcloud_callback, 10)
        self.transformed_pointcloud_pub = self.create_publisher(PointCloud2, '/mapokk', 10)

        self.rotation_matrix = None
        self.old_quat = None
        self.old_trans = None

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        new_quat = (q.x, q.y, q.z, q.w)
        new_trans = (t.x, t.y, t.z)

        if (self.old_quat is None or self.old_trans is None or
            new_quat != self.old_quat or new_trans != self.old_trans):
            self.old_quat = new_quat
            self.old_trans = new_trans
            qx, qy, qz, qw = new_quat
            sqx, sqy, sqz = qx*qx, qy*qy, qz*qz
            m00 = 1 - 2*(sqy + sqz)
            m01 = 2*(qx*qy - qw*qz)
            m02 = 2*(qx*qz + qw*qy)
            m10 = 2*(qx*qy + qw*qz)
            m11 = 1 - 2*(sqx + sqz)
            m12 = 2*(qy*qz - qw*qx)
            m20 = 2*(qx*qz - qw*qy)
            m21 = 2*(qy*qz + qw*qx)
            m22 = 1 - 2*(sqx + sqy)
            self.rotation_matrix = np.array([
                [m00, m01, m02, new_trans[0]],
                [m10, m11, m12, new_trans[1]],
                [m20, m21, m22, new_trans[2]],
                [0, 0, 0, 1]
            ], dtype=np.float32)

    def pointcloud_callback(self, msg):
        if self.rotation_matrix is None:
            return

        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append((p[0], p[1], p[2]))
        if not points_list:
            return

        N = len(points_list)
        points = np.empty((N, 3), dtype=np.float32)
        for i, (x, y, z) in enumerate(points_list):
            points[i, 0] = x
            points[i, 1] = y
            points[i, 2] = z

        ones = np.ones((N, 1), dtype=np.float32)
        points_hom = np.hstack([points, ones])
        transformed = (self.rotation_matrix @ points_hom.T).T[:, :3]

        header = msg.header
        header.frame_id = self.get_parameter('frame_id').value  
        cloud_msg = pc2.create_cloud_xyz32(header, transformed)
        self.transformed_pointcloud_pub.publish(cloud_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
