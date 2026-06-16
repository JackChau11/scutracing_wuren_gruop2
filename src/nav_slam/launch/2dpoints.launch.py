import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    package_name = 'nav_slam'
    config_dir = get_package_share_directory(package_name)
    # 构建 rviz 配置文件的路径
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')

    
    
    astar = Node( # 基于2d图规划路径
        package='nav_slam',
        executable='astar',
        name='astar',
        output='screen',
    )
    map_pub = Node( # 基于优化后的点构建2d地图/lio_sam/mapping/cloud_registered
        package='nav_slam',
        executable='map_pub',
        name='map_pub',
        output='screen',
    )
    # odom->map TF 已在仿真 launch 中由 C++ 节点发布，这里注释掉避免冲突
    # odom_map_tf = Node(
    #     package='nav_slam',
    #     executable='odom_map_tf',
    #     name='odom_map_tf',
    #     output='screen',
    # )
    points_pub_map = Node( # 发布优化后的点云
        package='nav_slam',
        executable='points_pub_map',
        name='points_pub_map',
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
  
   
    ld.add_action(astar)
    ld.add_action(map_pub)
    # ld.add_action(odom_map_tf)  # 已由仿真 launch 中的 C++ 节点替代
    ld.add_action(points_pub_map)
    ld.add_action(start_nav)

    ld.add_action(rviz2_node)


   
    

    return ld
