#!/usr/bin/env python3
"""
功能: 启动路径规划节点、控制节点和RViz可视化。

启动的节点:
  1. perception_path_planner: 基于锥桶/激光雷达生成局部路径。
  2. start_nav: 纯追踪控制器，发布控制指令。
  3. rviz2: 可视化界面，加载 rviz.rviz 配置。

说明: 启动前先确保已启动仿真环境和感知节点。
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    package_name = 'nav_slam'
    config_dir = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')

    path_planner = Node(
        package='nav_slam',
        executable='perception_path_planner',
        name='perception_path_planner',
        output='screen',
    )

    start_nav = Node(
        package='nav_slam',
        executable='start_nav',
        name='start_nav',
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    ld.add_action(path_planner)
    ld.add_action(start_nav)
    ld.add_action(rviz2_node)

    return ld
