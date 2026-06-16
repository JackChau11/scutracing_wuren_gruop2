import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
import re
import os
import math

# 定义一个ROS节点类，用于生成障碍物网格地图
class ObstacleGridNode(Node):
    def __init__(self):
        super().__init__('obstacle_grid_node')

        # 声明并获取参数
        self.declare_parameter('grid_width', 60.0)  # 地图宽度
        self.declare_parameter('grid_height', 60.0)  # 地图高度
        self.declare_parameter('resolution', 0.1)  # 地图分辨率
        self.declare_parameter('min_height', 0.1)  # 点云最小高度
        self.declare_parameter('max_height', 1.0)  # 点云最大高度
        self.declare_parameter('obstacle_radius', 0.2)  # 障碍物半径
        # 获取参数值
        self.grid_width = self.get_parameter('grid_width').get_parameter_value().double_value
        self.grid_height = self.get_parameter('grid_height').get_parameter_value().double_value
        self.resolution = self.get_parameter('resolution').get_parameter_value().double_value
        self.min_height = self.get_parameter('min_height').get_parameter_value().double_value
        self.max_height = self.get_parameter('max_height').get_parameter_value().double_value
        self.obstacle_radius = self.get_parameter('obstacle_radius').get_parameter_value().double_value
        # 初始化障碍物和膨胀层集合
        self.obstacles = set()
        self.dilated_obstacles_layer1 = set()
        self.dilated_obstacles_layer2 = set()
        self.dilated_obstacles_layer3 = set()
        # 初始化OccupancyGrid消息
        self.grid_combined = OccupancyGrid()
        self.grid_combined.header.frame_id = 'map'  # 设置参考系
        self.grid_combined.info.width = int(self.grid_width / self.resolution)  # 计算地图宽度（单元格数）
        self.grid_combined.info.height = int(self.grid_height / self.resolution)  # 计算地图高度（单元格数）
        self.grid_combined.info.resolution = self.resolution  # 设置分辨率
        self.grid_combined.data = [-1] * (self.grid_combined.info.width * self.grid_combined.info.height)  # 初始化地图数据为未知（-1）
        # 创建订阅者和发布者
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/mapokk', self.pointcloud_callback, 10)  # 订阅点云话题
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # 订阅里程计话题
        self.grid_combined_pub = self.create_publisher(OccupancyGrid, 'combined_grid', 10)  # 发布综合网格地图
        self.odom_data = None  # 初始化里程计数据为None

        # 预加载 track.txt 中的锥桶作为先验地图
        self.load_track_cones()

    def odom_callback(self, msg):
        # 处理里程计数据回调
        self.odom_data = msg
    def pointcloud_callback(self, msg):
        # 处理点云数据回调
        if self.odom_data is None:
            return  # 如果没有里程计数据，返回
        # 获取地图原点位置
        origin_x = self.odom_data.pose.pose.position.x
        origin_y = self.odom_data.pose.pose.position.y
        # 读取点云数据
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        radius_cells = int(self.obstacle_radius / self.resolution)  # 计算障碍物半径对应的单元格数
        new_obstacles = set()
        new_dilated_obstacles_layer1 = set()
        new_dilated_obstacles_layer2 = set()
        new_dilated_obstacles_layer3 = set()
        for x, y, z in points:
            if self.min_height <= z <= self.max_height:
                # 计算点在地图中的坐标
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)

                # 标记障碍物单元格
                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    index = center_y * self.grid_combined.info.width + center_x
                    new_obstacles.add(index)  # 添加到新障碍物集合

                # 膨胀障碍物单元格
                for layer, dilated_set in enumerate([new_dilated_obstacles_layer1, new_dilated_obstacles_layer2, new_dilated_obstacles_layer3]):
                    for dx in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                        for dy in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                            if dx**2 + dy**2 <= ((layer + 1) * radius_cells)**2:
                                grid_x = center_x + dx
                                grid_y = center_y + dy
                                if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                    index = grid_y * self.grid_combined.info.width + grid_x
                                    dilated_set.add(index)  # 添加到新膨胀层集合
        # 更新障碍物和膨胀层集合
        self.obstacles.update(new_obstacles)
        self.dilated_obstacles_layer1.update(new_dilated_obstacles_layer1)
        self.dilated_obstacles_layer2.update(new_dilated_obstacles_layer2)
        self.dilated_obstacles_layer3.update(new_dilated_obstacles_layer3)

        # 更新综合网格地图
        self.update_combined_grid()

    def update_combined_grid(self):
        # 初始化综合网格数据
        self.grid_combined.data = [1] * (self.grid_combined.info.width * self.grid_combined.info.height)  # 初始化地图数据为未知（-1）
        # 标记障碍物为黑色（100）
        for index in self.obstacles:
            if self.grid_combined.data[index] != 100:  # 避免重复标记
                self.grid_combined.data[index] = 100  # 标记为障碍物
        # 标记三层膨胀层为不同颜色
        for index in self.dilated_obstacles_layer1 - self.obstacles:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = 5  # 第一层膨胀层
        for index in self.dilated_obstacles_layer2 - self.dilated_obstacles_layer1:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -8  # 第二层膨胀层
        for index in self.dilated_obstacles_layer3 - self.dilated_obstacles_layer2:
            if self.grid_combined.data[index] == 1:
                self.grid_combined.data[index] = -120 # 第三层膨胀层
        # 更新综合网格地图消息头
        self.grid_combined.header.stamp = self.get_clock().now().to_msg()
        self.grid_combined.header.frame_id = 'map'
        self.grid_combined.info.origin.position.x = -self.grid_width / 2
        self.grid_combined.info.origin.position.y = -self.grid_height / 2
        self.grid_combined.info.origin.position.z = 0.0  # 假设没有垂直偏移
        # 发布综合网格地图
        self.grid_combined_pub.publish(self.grid_combined)

    def load_track_cones(self):
        """预加载 track.txt 中的锥桶坐标作为先验障碍物地图，并在相邻锥桶之间插值连成连续墙"""
        track_path = os.path.expanduser(
            '~/scutracing_ws/src/percep_node_track/tracks/models/shixi/track.txt'
        )
        if not os.path.exists(track_path):
            self.get_logger().warn(f'未找到赛道文件: {track_path}，跳过先验地图加载')
            return

        try:
            with open(track_path, 'r') as f:
                content = f.read()

            # 提取所有 <pose>x y z roll pitch yaw</pose> 中的 x, y, 以及锥桶名称
            # 格式: <include> ... <pose>x y z r p y</pose> ... <name>blue_cone_X</name> ...
            includes = re.findall(
                r'<include>\s*<pose>([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)</pose>\s*<uri>[^<]+</uri>\s*<name>([^<]+)</name>',
                content
            )

            blue_cones = []
            yellow_cones = []
            for inc in includes:
                x, y = float(inc[0]), float(inc[1])
                name = inc[6].strip()
                if 'blue' in name.lower():
                    blue_cones.append((x, y))
                elif 'yellow' in name.lower():
                    yellow_cones.append((x, y))

            # 按 y 坐标排序，让同侧锥桶按赛道顺序排列
            blue_cones.sort(key=lambda p: p[1])
            yellow_cones.sort(key=lambda p: p[1])

            # 把所有原始锥桶和插值点统一放入一个列表
            all_points = []
            all_points.extend(blue_cones)
            all_points.extend(yellow_cones)

            # 在相邻的同色锥桶之间插值，生成连续的墙
            step = self.resolution  # 每 0.1m 插入一个点
            for cone_list in [blue_cones, yellow_cones]:
                for i in range(len(cone_list) - 1):
                    x1, y1 = cone_list[i]
                    x2, y2 = cone_list[i + 1]
                    dist = math.hypot(x2 - x1, y2 - y1)
                    if dist > step:
                        num_steps = int(dist / step)
                        for k in range(1, num_steps):
                            t = k / num_steps
                            xi = x1 + t * (x2 - x1)
                            yi = y1 + t * (y2 - y1)
                            all_points.append((xi, yi))

            # 先把起点处的锥桶向 y 轴负方向延长 3m，然后在 y=-18 处设置横向虚拟墙
            if len(blue_cones) > 0 and len(yellow_cones) > 0:
                x_blue_start, y_blue_start = blue_cones[0]
                x_yellow_start, y_yellow_start = yellow_cones[0]
                wall_y = min(y_blue_start, y_yellow_start) - 3.0  # 向后 3m

                # 1) 把起点处两个锥桶向后延长到 wall_y
                for x_start, y_start in [(x_blue_start, y_blue_start), (x_yellow_start, y_yellow_start)]:
                    dist_back = y_start - wall_y
                    if dist_back > 0:
                        num_steps = int(dist_back / step)
                        for k in range(1, num_steps + 1):
                            yi = y_start - k * step
                            all_points.append((x_start, yi))

                # 2) 在 wall_y 处设置横向虚拟墙，连接两侧延长线
                x_min = min(x_blue_start, x_yellow_start)
                x_max = max(x_blue_start, x_yellow_start)
                num_steps = int((x_max - x_min) / step)
                for k in range(num_steps + 1):
                    xi = x_min + k * step
                    all_points.append((xi, wall_y))

            radius_cells = int(self.obstacle_radius / self.resolution)
            count = 0
            for x, y in all_points:
                center_x = int((x + self.grid_width / 2) / self.resolution)
                center_y = int((y + self.grid_height / 2) / self.resolution)
                if 0 <= center_x < self.grid_combined.info.width and 0 <= center_y < self.grid_combined.info.height:
                    index = center_y * self.grid_combined.info.width + center_x
                    if index not in self.obstacles:
                        self.obstacles.add(index)
                        count += 1
                        # 同时膨胀三层，逻辑与 pointcloud_callback 保持一致
                        for layer, dilated_set in enumerate([self.dilated_obstacles_layer1, self.dilated_obstacles_layer2, self.dilated_obstacles_layer3]):
                            for dx in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                                for dy in range(-(layer + 1) * radius_cells, (layer + 1) * radius_cells + 1):
                                    if dx**2 + dy**2 <= ((layer + 1) * radius_cells)**2:
                                        grid_x = center_x + dx
                                        grid_y = center_y + dy
                                        if 0 <= grid_x < self.grid_combined.info.width and 0 <= grid_y < self.grid_combined.info.height:
                                            dilated_set.add(grid_y * self.grid_combined.info.width + grid_x)

            self.update_combined_grid()
            self.get_logger().info(f'先验地图加载完成: 共 {count} 个障碍物点 (含插值墙)')
        except Exception as e:
            self.get_logger().error(f'加载赛道文件失败: {e}')

def main(args=None):
    # ROS2初始化
    rclpy.init(args=args)
    node = ObstacleGridNode()
    rclpy.spin(node)  # 进入主循环
    rclpy.shutdown()  # 关闭ROS2节点

if __name__ == '__main__':
    main()
