import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('four_wheeled_vehicle')
    world_file = os.path.join(pkg_path, 'worlds', 'race_track_harmonic.sdf')
    model_path = os.path.join(pkg_path, 'models')
    percep_models = os.path.join(
        '/home/zhuziyuan/scutracing_ws/src/percep_node_track/tracks/models'
    )

    # ---- GZ_SIM_RESOURCE_PATH (模型搜索路径) ----
    gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    merged = f'{model_path}:{percep_models}'
    if gz_path:
        merged = f'{gz_path}:{merged}'

    set_gz_resource = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', merged)
    # 旧名兼容
    set_ign_resource = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', merged)

    # ---- NVIDIA GPU 环境 ----
    set_nvidia1 = SetEnvironmentVariable('__NV_PRIME_RENDER_OFFLOAD', '1')
    set_nvidia2 = SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia')
    set_nvidia3 = SetEnvironmentVariable('__GL_SYNC_TO_VBLANK', '0')

    # ---- 1. Launch Gazebo Harmonic ----
    start_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': (
                f'-r {world_file} '
                '--render-engine ogre2 '
                '--render-engine-api-backend opengl'
            ),
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ---- 2. ros_gz_bridge (每个 bridge 独立 node + remap) ----

    # cmd_vel: ROS→GZ. start_nav 发 /cmd_vel, 桥接到 /model/.../cmd_vel
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmd_vel',
        arguments=[
            '/model/four_wheeled_vehicle/cmd_vel'
            '@geometry_msgs/msg/Twist'
            ']gz.msgs.Twist'
        ],
        remappings=[
            ('/model/four_wheeled_vehicle/cmd_vel', '/cmd_vel'),
        ],
        output='screen'
    )

    # odometry: GZ→ROS
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom',
        arguments=[
            '/model/four_wheeled_vehicle/odometry'
            '@nav_msgs/msg/Odometry'
            '[gz.msgs.Odometry',
        ],
        remappings=[
            ('/model/four_wheeled_vehicle/odometry', '/odom'),
        ],
        output='screen'
    )

    # LiDAR: GZ→ROS
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_lidar',
        arguments=[
            '/world/race_track/model/four_wheeled_vehicle/link/'
            'lidar3d_link/sensor/lidar3d/scan/points'
            '@sensor_msgs/msg/PointCloud2'
            '[gz.msgs.PointCloudPacked',
        ],
        remappings=[
            ('/world/race_track/model/four_wheeled_vehicle/link/'
             'lidar3d_link/sensor/lidar3d/scan/points', '/points_raw'),
        ],
        output='screen'
    )

    # IMU: GZ→ROS
    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=[
            '/world/race_track/model/four_wheeled_vehicle/link/'
            'imu_link/sensor/imu/imu'
            '@sensor_msgs/msg/Imu'
            '[gz.msgs.IMU',
        ],
        remappings=[
            ('/world/race_track/model/four_wheeled_vehicle/link/'
             'imu_link/sensor/imu/imu', '/imu_raw'),
        ],
        output='screen'
    )

    # GPS: GZ→ROS
    bridge_gps = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_gps',
        arguments=[
            '/world/race_track/model/four_wheeled_vehicle/link/'
            'gps_link/sensor/gps/navsat'
            '@sensor_msgs/msg/NavSatFix'
            '[gz.msgs.NavSat',
        ],
        remappings=[
            ('/world/race_track/model/four_wheeled_vehicle/link/'
             'gps_link/sensor/gps/navsat', '/vehicle/gps/fix'),
        ],
        output='screen'
    )

    # Camera: GZ→ROS
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera',
        arguments=[
            '/world/race_track/model/four_wheeled_vehicle/link/'
            'camera_link/sensor/front_camera/image'
            '@sensor_msgs/msg/Image'
            '[gz.msgs.Image',
        ],
        remappings=[
            ('/world/race_track/model/four_wheeled_vehicle/link/'
             'camera_link/sensor/front_camera/image', '/image_raw'),
        ],
        output='screen'
    )

    # Camera Info: GZ→ROS
    bridge_camera_info = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera_info',
        arguments=[
            '/world/race_track/model/four_wheeled_vehicle/link/'
            'camera_link/sensor/front_camera/camera_info'
            '@sensor_msgs/msg/CameraInfo'
            '[gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/world/race_track/model/four_wheeled_vehicle/link/'
             'camera_link/sensor/front_camera/camera_info', '/camera_info'),
        ],
        output='screen'
    )

    # Clock: GZ→ROS
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_clock',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # ---- 3. Static TF publishers ----
    def make_stf(name, x, y, z, parent, child):
        return Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=['--x', x, '--y', y, '--z', z,
                       '--frame-id', parent, '--child-frame-id', child],
            output='screen'
        )

    stf_lidar  = make_stf('stf_lidar3d',  '0', '0', '2.25', 'base_link', 'lidar3d_link')
    stf_gps    = make_stf('stf_gps',      '0', '0', '2.25', 'base_link', 'gps_link')
    stf_imu    = make_stf('stf_imu',      '0', '0', '2.25', 'base_link', 'imu_link')
    stf_camera = make_stf('stf_camera',   '0', '0', '2.35', 'base_link', 'camera_link')
    stf_base   = make_stf('odom_base_tf', '0', '0', '2',    'odom',      'base_link')

    # ---- 4. world -> map 静态 TF (感知节点需要 "world" 坐标系) ----
    stf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='stf_world_map',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--frame-id', 'world', '--child-frame-id', 'map'],
        output='screen'
    )

    # ---- 5. odom -> map TF ----
    odom_map_tf_node = Node(
        package='four_wheeled_vehicle',
        executable='odom_mapTF',
        name='odom_map_tf_node',
        output='screen'
    )

    return LaunchDescription([
        # 环境变量 (按顺序设置，后面的覆盖前面的)
        set_gz_resource,
        set_ign_resource,
        set_nvidia1,
        set_nvidia2,
        set_nvidia3,
        # 主程序
        start_gz,
        bridge_cmd_vel,
        bridge_odom,
        bridge_lidar,
        bridge_imu,
        bridge_gps,
        bridge_camera,
        bridge_camera_info,
        bridge_clock,
        stf_lidar, stf_gps, stf_imu, stf_camera, stf_base, stf_world,
        odom_map_tf_node,
    ])
