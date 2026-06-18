#!/usr/bin/env python3
"""
节点: odom_tf_publisher
功能: 发布 odom → base_link 动态TF，将里程计位姿传递给TF树，供其他节点进行坐标变换

输入:
  订阅 /odom (nav_msgs/Odometry) : 里程计数据

输出:
  发布 TransformStamped 到 /tf，帧: odom转换到base_link

逻辑:
  1. 收到 /odom 消息后，提取位置 (x,y,z) 和姿态
  2. 构造 TransformStamped，设置 header.frame_id = 'odom', child_frame_id = 'base_link'
  3. 通过 TransformBroadcaster 广播该变换
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_tf_publisher')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.br = TransformBroadcaster(self)

    def odom_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
