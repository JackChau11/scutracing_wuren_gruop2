#!/usr/bin/env python3
"""
节点: obstacle_grid_node
功能: 利用转换后的点云 (mapokk) 构建占据栅格地图 (OccupancyGrid)，包含障碍物及多层膨胀区域。

输入:
订阅 /mapokk (sensor_msgs/PointCloud2) : 已转换到 world 坐标系的点云
订阅 /odom (nav_msgs/Odometry) : 里程计（用于确定地图原点，但实际代码中未使用，仅作依赖）

输出:
发布 /combined_grid (nav_msgs/OccupancyGrid) : 占据栅格地图，frame_id = 'world'

逻辑:
1. 初始化固定大小的栅格地图（默认 60m×60m，分辨率 0.1m），所有栅格初始化为 -1（未知）。
2. 收到点云后，对每个点检查高度是否在 min_height ~ max_height 之间，若满足则视为障碍物。
3. 计算障碍物在栅格中的索引，存入 obstacles 集合。
4. 对障碍物进行三层膨胀（半径分别为 obstacle_radius, 2*obstacle_radius, 3*obstacle_radius），分别存入三个膨胀层集合。
5. 在 update_combined_grid 中，将地图数据置为 1（空闲），然后按层级赋值：
 障碍物本身: 100
 第一层膨胀: 5
 第二层膨胀: -8
 第三层膨胀: -120
6. 发布更新后的地图。

关键参数（可通过 launch 或 yaml 设置）:
grid_width: 60.0 (米)
grid_height: 60.0 (米)
resolution: 0.1 (米/格)
min_height: 0.1 (米) 障碍物最低高度
max_height: 1.0 (米) 障碍物最高高度
obstacle_radius: 0.2 (米) 膨胀半径基础值
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        self.declare_parameter('grid_width', 60.0)
        self.declare_parameter('grid_height', 60.0)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('min_height', 0.1)
        self.declare_parameter('max_height', 1.0)
        self.declare_parameter('obstacle_radius', 0.2)
        self.grid_width = self.get_parameter('grid_width').get_parameter_value().double_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value

        self.obstacles = set()
        self.dilated_obstacles_layer1 = set()
        self.dilated_obstacles_layer2 = set()
        self.dilated_obstacles_layer3 = set()

        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'world'   # 改为 world
        self.grid_combined.info.width = int(self.grid_width / self.resolution)
        self.grid_combined.info.height = int(self.grid_height / self.resolution)
        self.grid_combined.info.resolution = self.resolution
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)

        self.pointcloud_sub = self.create_subscription(PointCloud2, '/mapokk', self.pointcloud_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.grid_combined_pub = self.create_publisher(OccupancyGrid, 'combined_grid', 10)
        self.odom_data = None

    def odom_callback(self, msg):
        self.odom_data = msg

    def pointcloud_callback(self, msg):
        if self.odom_data is None:
            return
        origin_x = self.odom_data.pose.pose.position.x
        origin_y = self.odom_data.pose.pose.position.y
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        radius_cells = int(self.obstacle_radius / self.resolution)

        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()

        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)

                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    index = center_y * self.grid_combined.info.width + center_x
                    new_obstacles.add(index)

                for layer, dilated_set in enumerate([new_dilated_obstacles_layer1, new_dilated_obstacles_layer2, new_dilated_obstacles_layer3]):
                    for dx in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                        for dy in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                            if dx**2 + dy**2 <= ((layer + 1) * radius_cells)**2:
                                grid_x = center_x + dx
                                grid_y = center_y + dy
                                if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                    index = grid_y * self.grid_combined.info.width + grid_x
                                    dilated_set.add(index)

        self.obstacles.update(new_obstacles)
        self.dilated_obstacles_layer1.update(new_dilated_obstacles_layer1)
        self.dilated_obstacles_layer2.update(new_dilated_obstacles_layer2)
        self.dilated_obstacles_layer3.update(new_dilated_obstacles_layer3)

        self.update_combined_grid()

    def update_combined_grid(self):
        self.grid_combined.data = [1] * (self.grid_combined.info.width * self.grid_combined.info.height)
        for index in self.obstacles:
            if self.grid_combined.data[index] != 100:
                self.grid_combined.data[index] = 100
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = 5
        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -8
        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -120

        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'world'
        self.grid_combined.info.origin.position.x = -self.grid_width / 2
        self.grid_combined.info.origin.position.y = -self.grid_height / 2
        self.grid_combined.info.origin.position.z = 0.0
        self.grid_combined_pub.publish(self.grid_combined)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
