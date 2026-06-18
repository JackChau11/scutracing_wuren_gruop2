#!/usr/bin/env python3
"""
此文件只是伪代码，没有真正实现，主要是展示建图后对已有地图的一些处理想法，以方便后续使用各种路径规划方法
节点: cone_based_planner
功能: 利用锥桶的颜色信息构建赛道的可通行区域（封闭多边形），并将该区域标记在栅格地图上，
      然后在该区域内使用各种路径规划算法进行全局路径规划。此文件为设计伪代码，需根据实际系统
      补充坐标转换、地图初始化、起点终点设置等细节。

输入:
/perception/cones (fsd_common_msgs/ConeDetections) : 锥桶检测结果（位置、颜色、置信度）
/odom (nav_msgs/Odometry) : 里程计，用于获取车辆位姿（可选，用于坐标转换）

输出:
/global_plan (nav_msgs/Path) : 全局规划路径

逻辑步骤:
  1. 缓存锥桶数据，按颜色分组（红色/蓝色），并将坐标转换到全局坐标系（world/odom）。
  2. 检查左右两侧锥桶数量是否足够（≥2），不足则放弃规划。
  3. 分别对红、蓝两组点进行边界拟合（样条或多项式），得到左右边界点序列。
  4. 将左右边界首尾相连，形成封闭多边形（赛道可通行区域的轮廓）。
  5. 根据多边形更新栅格地图：多边形内部标记为自由空间（0），外部标记为障碍（100）。
  6. 若已设置起点和终点，在可通行区域内使用 A* 、RRT 等算法搜索路径，并发布为 /global_plan。


待实现/待完善部分:
transform_to_world() : 需根据 TF 或里程计将锥桶从 base_link 转换到 world/odom。
grid 初始化 : 需创建与 map_pub.py 一致尺寸的栅格数据。
start_pose, goal_pose : 需由外部设置（如 RViz 2D Nav Goal 或固定值）。
与现有地图叠加 : 可结合 /combined_grid 已有障碍物信息进行融合。

使用说明:
本文件为设计草案，不可直接运行。需实现上述缺失函数，并与现有建图节点(map_pub.py) 配合使用，或独立维护栅格数据。
规划结果发布到 /global_plan，可被下游控制器（如 start_nav.py）订阅。
"""

import rclpy, numpy as np, math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid
from fsd_common_msgs.msg import ConeDetections
from scipy.interpolate import splprep, splev        
from scipy.spatial import ConvexHull               
import networkx as nx                              

class ConeBasedPlanner(Node):
    def __init__(self):
        super().__init__('cone_based_planner')
        self.sub_cones = self.create_subscription(ConeDetections, '/perception/cones', self.cones_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pub_plan = self.create_publisher(Path, '/global_plan', 10)
        
        self.current_pose = None   # (x, y, yaw) in world frame
        self.grid = None      # 当前地图栅格数据（若需要）
        self.resolution = 0.1     # 地图分辨率
        self.grid_width = 600  # 栅格数量（60m/0.1）
        self.grid_height = 600
        
        # 用于存储锥桶数据（坐标已转换到world或odom）
        self.red_cones = []   # (x, y)
        self.blue_cones = []  # (x, y)
        
        # 规划参数
        self.start_pose = None # 起点 (x, y)
        self.goal_pose = None # 终点 (x, y)

    #回调 
    def cones_callback(self, msg):
        #1.清除旧数据，按颜色分组，并转换到 odom/world 坐标系
        self.red_cones.clear()
        self.blue_cones.clear()
        for cone in msg.cone_detections:
            #假设坐标已在 base_link 下，需转换到 odom（或world）
            x_world, y_world = self.transform_to_world(cone.position.x, cone.position.y)
            if cone.color == 'red':
                self.red_cones.append((x_world, y_world))
            elif cone.color == 'blue':
                self.blue_cones.append((x_world, y_world))
        
        #2.检查是否有足够的点（至少两侧各2个）
        if len(self.red_cones) < 2 or len(self.blue_cones) < 2:
            self.get_logger().warn("Not enough cones to build region")
            return
        
        #3.拟合左右边界曲线（样条或多项式）
        left_boundary = self.fit_boundary(self.red_cones)    
        right_boundary = self.fit_boundary(self.blue_cones)
        
        #4.封闭边界：将左边界首尾与右边界首尾相连，形成封闭多边形
        #注意：保证点序为顺时针或逆时针（可能要针对8字形赛道做进一步优化）
        boundary_polygon = self.close_boundary(left_boundary, right_boundary)
        
        #5.生成占据栅格地图，标记可通行区域
        self.update_costmap(boundary_polygon)  #标记多边形内为自由空间，外为障碍
        
        #6.在可通行区域内进行全局路径规划
        if self.start_pose and self.goal_pose:
            plan = self.plan_astar(self.start_pose, self.goal_pose)
            self.publish_plan(plan)

    #辅助函数
    def fit_boundary(self, points):
        """对一组点进行样条拟合，返回均匀采样的边界点序列"""
        if len(points) < 4:
            #点数少时用直线或简单多项式拟合
            return self.fit_line_or_poly(points)
        
        #按x或弧长排序
        pts_sorted = sorted(points, key=lambda p: p[0])  
        #使用scipy样条插值
        tck, u = splprep([ [p[0] for p in pts_sorted], [p[1] for p in pts_sorted] ], s=0.5)
        #均匀采样 50 个点
        u_new = np.linspace(0, 1, 50)
        x_new, y_new = splev(u_new, tck)
        return list(zip(x_new, y_new))

    def fit_line_or_poly(self, points):
        """简单直线或二次多项式拟合"""
        #使用numpy polyfit拟合 y=ax^2+bx+c
        xs = [p[0] for p in points]; ys = [p[1] for p in points]
        coeffs = np.polyfit(xs, ys, 1)   # 一次
        #生成均匀采样
        x_min, x_max = min(xs), max(xs)
        x_vals = np.linspace(x_min, x_max, 20)
        y_vals = np.polyval(coeffs, x_vals)
        return list(zip(x_vals, y_vals))

    def close_boundary(self, left_pts, right_pts):
        """将左右边界首尾相连，形成封闭多边形（这里先假设用逆时针方向）"""
        # 假设 left_pts 顺序为车辆前进方向，right_pts 同向
        # 封闭顺序：左起点 -> 左终点 -> 右终点 -> 右起点 -> 左起点
        poly = []
        poly.extend(left_pts)                     
        poly.extend(reversed(right_pts))        
        poly.append(left_pts[0])     
        return poly  

    def update_costmap(self, polygon):
        """将多边形内部标记为可通行（0），外部标记为障碍（100）"""
        #此处应获取当前地图或创建新地图，使用点与多边形关系
        #假定已有 grid 数据 (list of ints)，宽度和高度已知
        #使用 matplotlib.path 的 contains_points 或自定义射线法
        from matplotlib.path import Path
        poly_path = Path(polygon)
        #遍历所有栅格中心点
        for iy in range(self.grid_height):
            for ix in range(self.grid_width):
                x_world = ix * self.resolution - self.grid_width/2  
                y_world = iy * self.resolution - self.grid_height/2
                if poly_path.contains_point((x_world, y_world)):
                    self.grid[iy][ix] = 0     
                else:
                    self.grid[iy][ix] = 100
        # 发布更新后的地图（可选）

    def plan_astar(self, start, goal):
        """这里先使用A*算法为例，后面可以改为其他更好用的算法，起点终点为世界坐标"""
        # 将起点终点转换为栅格索引
        start_idx = self.world_to_grid(start)
        goal_idx = self.world_to_grid(goal)
        # 使用networkx或自定义A*实现
        # 这里以networkx举例
        G = nx.grid_2d_graph(self.grid_width, self.grid_height)
        # 移除障碍物节点（值为100）
        for iy in range(self.grid_height):
            for ix in range(self.grid_width):
                if self.grid[iy][ix] == 100:
                    G.remove_node((ix, iy))
        #计算路径
        path_indices = nx.astar_path(G, start_idx, goal_idx, 
                                     heuristic=lambda a,b: abs(a[0]-b[0])+abs(a[1]-b[1]))
        #转换回世界坐标
        world_path = [self.grid_to_world(idx) for idx in path_indices]
        return world_path

    def world_to_grid(self, point):
        ix = int((point[0] + self.grid_width/2) / self.resolution)
        iy = int((point[1] + self.grid_height/2) / self.resolution)
        return (ix, iy)
    def grid_to_world(self, idx):
        x = idx[0]*self.resolution - self.grid_width/2
        y = idx[1]*self.resolution - self.grid_height/2
        return (x, y)

    def publish_plan(self, world_path):
        path_msg = Path()
        path_msg.header.frame_id = 'world'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in world_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            path_msg.poses.append(pose)
        self.pub_plan.publish(path_msg)
