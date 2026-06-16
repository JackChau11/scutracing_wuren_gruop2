
import rclpy
from rclpy.node import Node
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
from rclpy.qos import QoSProfile
import scipy.interpolate as si
import numpy as np
from nav_msgs.msg import Odometry
from scipy.interpolate import BSpline
from scipy import ndimage
import time

# 障碍物膨胀参数
expansion_v = 12  # 竖直膨胀 1.2m
expansion_h = 8   # 水平膨胀 0.8m

# 处理成本图数据，扩展障碍物（椭圆形膨胀）
def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)  # 重塑数据为矩阵
    wall_mask = data == 100

    # 使用 scipy.ndimage 做高效的椭圆形二值膨胀，替代原来的双重循环
    y, x = np.ogrid[-expansion_v:expansion_v+1, -expansion_h:expansion_h+1]
    ellipse_mask = (x / expansion_h) ** 2 + (y / expansion_v) ** 2 <= 1
    dilated = ndimage.binary_dilation(wall_mask, structure=ellipse_mask)
    data[dilated] = 100

    data = data * resolution  # 将成本图中的值乘以分辨率
    return data

def bezier_smoothing(array, num_points):
    try:
        array = np.array(array)
        if len(array) < 4:
            return array
        x = array[:, 0]
        y = array[:, 1]

        # 计算基于弦长的参数t
        dx = np.diff(x, prepend=x[0])
        dy = np.diff(y, prepend=y[0])
        chord_lengths = np.sqrt(dx**2 + dy**2)  # 弦长
        t = np.concatenate(([0], np.cumsum(chord_lengths)))  # 累积弦长作为参数t
        t /= t[-1]  # 规范化到[0, 1]

        k = 3  # 固定为三次B样条，避免高阶(k=num_points-1)导致严重振荡和偏离

        # 添加重复的节点，确保有足够的节点来定义样条
        t_knots = np.concatenate(([0]*k, t, [1]*k))
        # 根据新的节点数组调整x和y的长度
        x_padded = np.pad(x, (k, k), 'edge')
        y_padded = np.pad(y, (k, k), 'edge')

        # 创建B样条对象
        spline_x = BSpline(t_knots, x_padded, k, extrapolate=False)
        spline_y = BSpline(t_knots, y_padded, k, extrapolate=False)

        # 基于等间距的t_new重新采样
        t_new = np.linspace(0, 1, num_points)
        x_smoothed = spline_x(t_new)
        y_smoothed = spline_y(t_new)

        path = np.column_stack((x_smoothed, y_smoothed))
    except Exception:
        path = array
    return path

# A*算法
def astar(start, goal, grid):
    def heuristic(a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)  # 使用欧几里得距离作为启发式函数
    rows, cols = grid.shape

    # 检查起点和终点是否在地图范围内
    if not (0 <= start[0] < rows and 0 <= start[1] < cols):
        return []
    if not (0 <= goal[0] < rows and 0 <= goal[1] < cols):
        return []

    # 如果终点恰好在障碍物中，尝试在附近找一个自由点作为替代终点
    if grid[goal] == 100:
        found = False
        for r in range(-3, 4):
            for c in range(-3, 4):
                ng = (goal[0] + r, goal[1] + c)
                if 0 <= ng[0] < rows and 0 <= ng[1] < cols and grid[ng] != 100:
                    goal = ng
                    found = True
                    break
            if found:
                break
        if not found:
            return []

    open_set = []
    heapq.heappush(open_set, (heuristic(start, goal), 0, start))
    came_from = {}
    cost_so_far = {start: 0}
    closed_set = set()  # 使用集合来存储已访问的节点
    while open_set:
        _, current_cost, current = heapq.heappop(open_set)
        if current == goal:
            # 构建路径
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path
        if current in closed_set:  # 检查节点是否已被访问过
            continue
        closed_set.add(current)  # 标记为已访问
        for d in [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor = (current[0] + d[0], current[1] + d[1])
            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols and grid[neighbor] != 100:
                # 斜向移动的代价应乘以 sqrt(2)，否则算法会过度偏好斜走
                move_cost = math.sqrt(2) if abs(d[0]) + abs(d[1]) == 2 else 1.0
                new_cost = cost_so_far[current] + grid[neighbor] * move_cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    cost_so_far[neighbor] = new_cost
                    priority = new_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, new_cost, neighbor))
                    came_from[neighbor] = current
    return []  # No path found

# 导航控制节点类
class NavigationControl(Node):
    def __init__(self):
        super().__init__('Navigation')  # 初始化ROS 2节点
        # 创建订阅器订阅地图数据
        self.map_subscription = self.create_subscription(OccupancyGrid, 'combined_grid', self.map_callback, 10)
        self.path_publisher = self.create_publisher(Path, 'path', 10)
        self.path_publisher2 = self.create_publisher(Path, 'path2', 10)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.pose_subscriber = self.create_subscription(PoseStamped,'/goal_pose',self.goal_callback,10)
        self.x = 0.0
        self.y =0.0
        self.waypoints = []         # 途经点队列，依次导航
        self.current_goal = None    # 当前正在导航到的目标
        self.last_map_msg = None    # 缓存最近的地图消息
        self.create_timer(0.1, self.publish_path)
        self.path = None
        self.path2 = None

    def goal_callback(self, msg):
        # 每次收到目标点，追加到途经点队列末尾
        wp = (msg.pose.position.x, msg.pose.position.y)
        self.waypoints.append(wp)
        self.get_logger().info(f'添加途经点: ({wp[0]:.1f}, {wp[1]:.1f})，队列长度 {len(self.waypoints)}')
        # 如果当前没有目标，立即激活下一个途经点并用缓存地图规划
        if self.current_goal is None:
            self.advance_waypoint()
            if self.last_map_msg is not None:
                self.map_callback(self.last_map_msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    # 自动切换到下一个途经点
    def advance_waypoint(self):
        if self.waypoints:
            self.current_goal = self.waypoints.pop(0)
            self.get_logger().info(f'前往途经点: ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f})，剩余 {len(self.waypoints)} 个')
        else:
            self.current_goal = None
            self.get_logger().info('所有途经点已完成')

    # 地图数据回调函数
    def map_callback(self, msg):
        self.last_map_msg = msg  # 缓存最近的地图消息
        # 如果还没有当前目标且有途经点，取下一个
        if self.current_goal is None and self.waypoints:
            self.advance_waypoint()
        if self.current_goal is None:
            return
        distance = abs(math.hypot(self.x - self.current_goal[0], self.y - self.current_goal[1]))
        if distance > 0.2:
            t0 = time.time()
            path = []
            resolution = msg.info.resolution
            originX = msg.info.origin.position.x
            originY = msg.info.origin.position.y
            column = int((self.x - originX) / resolution)
            row = int((self.y - originY) / resolution)
            columnH = int((self.current_goal[0] - originX) / resolution)
            rowH = int((self.current_goal[1] - originY) / resolution)
            data = costmap(msg.data, msg.info.width, msg.info.height, resolution)
            data[row][column] = 1
            cost_map = np.ones_like(data)
            cost_map[data > 5] = 100
            cost_map[data < -10] = 5
            cost_map[(data <= -0.5) & (data >= -2)] = 20
            cost_map[(data >= 0.3) & (data <= 2)] = 50

            # 关键修改：使用距离变换让 A* 优先选择远离障碍物的路径（赛道中间）
            # 计算每个自由格子到最近障碍物的距离（单位：米）
            obstacle_mask = (cost_map == 100)
            dist_grid = ndimage.distance_transform_edt(~obstacle_mask)
            dist_m = dist_grid * resolution
            # 距离惩罚：越靠近障碍物代价越高，指数衰减
            # 0m 处惩罚约 25，0.5m 处约 15，1.0m 处约 6，2.0m 处约 1
            penalty = np.exp(-dist_m / 0.5) * 25.0
            cost_map = cost_map + penalty
            # 确保障碍物本身仍然是 100（不可通行）
            cost_map[obstacle_mask] = 100

            data = cost_map
            start = (row, column)
            goal = (rowH, columnH)
            path = astar(start, goal, data)
            paths = [(p[1] * resolution + originX, p[0] * resolution + originY) for p in path]
            t1 = time.time()
            if len(paths) == 0:
                self.get_logger().warn(
                    f'A* no path! start=({self.x:.1f},{self.y:.1f}) goal=({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}), '
                    f'grid_start=({row},{column}) grid_goal=({rowH},{columnH}), '
                    f'start_cost={data[row][column]}, goal_cost={data[rowH][columnH]}, time={t1-t0:.3f}s'
                )
            else:
                self.path = paths
                self.path2 = bezier_smoothing(paths, len(paths))
                self.get_logger().info(
                    f'A* 规划成功: start=({self.x:.1f},{self.y:.1f}) goal=({self.current_goal[0]:.1f},{self.current_goal[1]:.1f}), '
                    f'路径点数={len(paths)}, time={t1-t0:.3f}s'
                )
        else:
            # 到达当前途经点，切换到下一个并立即重新规划
            self.get_logger().info(f'到达途经点 ({self.current_goal[0]:.1f}, {self.current_goal[1]:.1f})')
            self.advance_waypoint()
            self.path = []
            if self.current_goal is not None and self.last_map_msg is not None:
                self.map_callback(self.last_map_msg)
            # 如果还有下一个目标，立即用缓存地图重新规划
            if self.current_goal is not None and self.last_map_msg is not None:
                self.map_callback(self.last_map_msg)
    
    # 发布路径
    def publish_path(self):
        if self.path is None or len(self.path)==0:
            # print('no path')
            return
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        for (px, py) in self.path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(px)
            pose.pose.position.y = float(py)
            path_msg.poses.append(pose)
        self.path_publisher.publish(path_msg)


        path2_msg = Path()
        path2_msg.header.frame_id = 'map'
        for (px, py) in self.path2:
            pose2 = PoseStamped()
            pose2.header.frame_id = 'map'
            pose2.pose.position.x = float(px)
            pose2.pose.position.y = float(py)
            path2_msg.poses.append(pose2)
        self.path_publisher2.publish(path2_msg)

# 主函数
def main(args=None):
    rclpy.init(args=args)  # 初始化ROS 2
    navigation_control = NavigationControl()  # 创建导航控制节点实例
    rclpy.spin(navigation_control)  # 运行节点
    navigation_control.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭ROS 2

# 如果直接运行此文件，则执行主函数
if __name__ == '__main__':
    main()
