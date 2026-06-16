import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    package_name = 'nav_slam'
    config_dir = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(config_dir, 'config', 'rviz.rviz')

    # 使用 astar 节点（已被新代码覆盖）
    astar = Node(
        package='nav_slam',
        executable='astar',
        name='astar',
        output='screen',
    )

    start_nav = Node(
        package='nav_slam',
        executable='start_nav',
        name='start_nav',
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen',
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    # 移除旧的 map_pub, points_pub_map 等
    ld.add_action(astar)
    ld.add_action(start_nav)
    ld.add_action(static_tf)
    ld.add_action(rviz2_node)

    return ld
