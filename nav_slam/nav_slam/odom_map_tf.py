#!/usr/bin/env python3
"""
节点: odom_to_map_transformer
功能: 发布 map → odom 动态TF，将里程计位姿映射到地图坐标系（用于全局定位或与地图对齐）

输入:
订阅 /odom (nav_msgs/Odometry) : 里程计数据

输出:
发布 TransformStamped 到 /tf，帧: map → odom

逻辑:
  1. 接收到 /odom 后，直接将其位置和姿态作为 map 下 odom 的变换。
  2. 通过 TransformBroadcaster 广播变换，使 map 与 odom 同步。

说明: 此节点与建图节点配合使用，使地图坐标系与里程计坐标系相关联。
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros


class OdomToMapTransformer(Node):
    def __init__(self):
        super().__init__('odom_to_map_transformer')
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        # self.odom_subscriber = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        # self.publish_static_transform()
    def odom_callback(self, odom_msg):
        # 创建变换消息
        t = TransformStamped()

        # 设置 header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'

        # 设置变换的位置和朝向
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation = odom_msg.pose.pose.orientation

        # 发布变换
        self.tf_broadcaster.sendTransform(t)
        # 发布静态变换
        
    def publish_static_transform(self):
        # 创建静态变换消息
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'odom'
        static_transform.child_frame_id = 'base_link'
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0  # 单位四元数表示没有旋转
        # 发布静态变换
        # self.static_tf_broadcaster.sendTransform(static_transform)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapTransformer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
