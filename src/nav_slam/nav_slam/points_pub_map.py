import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

class PointCloudTransformNode(Node):
    def __init__(self):
        super().__init__('pointcloud_transform_node')
        self.declare_parameter('frame_id', 'map')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.pointcloud_sub = self.create_subscription(PointCloud2, '/points_raw', self.pointcloud_callback, 10)
        self.transformed_pointcloud_pub = self.create_publisher(PointCloud2, '/mapokk', 10)

        self.rotation_matrix = None
        self.old_quat = None
        self.old_trans = None

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        t = msg.pose.pose.position
        new_quat = (q.x, q.y, q.z, q.w)
        new_trans = (t.x, t.y, t.z)

        if (self.old_quat is None or self.old_trans is None or
            new_quat != self.old_quat or new_trans != self.old_trans):
            self.old_quat = new_quat
            self.old_trans = new_trans
            qx, qy, qz, qw = new_quat
            sqx, sqy, sqz = qx*qx, qy*qy, qz*qz
            m00 = 1 - 2*(sqy + sqz)
            m01 = 2*(qx*qy - qw*qz)
            m02 = 2*(qx*qz + qw*qy)
            m10 = 2*(qx*qy + qw*qz)
            m11 = 1 - 2*(sqx + sqz)
            m12 = 2*(qy*qz - qw*qx)
            m20 = 2*(qx*qz - qw*qy)
            m21 = 2*(qy*qz + qw*qx)
            m22 = 1 - 2*(sqx + sqy)

            # 修正：加上 lidar3d_link 相对于 base_link 的偏移 (1.30, 0, 0.95)
            # 原代码直接把 lidar3d_link 中的点当作 base_link 中的点做变换，
            # 导致所有障碍物在 odom/map 坐标系中偏移了约 1.30m（沿 base_link x 方向）。
            lidar_offset = np.array([1.30, 0.0, 0.95], dtype=np.float32)
            rot = np.array([
                [m00, m01, m02],
                [m10, m11, m12],
                [m20, m21, m22]
            ], dtype=np.float32)
            corrected_trans = np.array(new_trans, dtype=np.float32) + (rot @ lidar_offset)

            self.rotation_matrix = np.array([
                [m00, m01, m02, corrected_trans[0]],
                [m10, m11, m12, corrected_trans[1]],
                [m20, m21, m22, corrected_trans[2]],
                [0, 0, 0, 1]
            ], dtype=np.float32)

    def pointcloud_callback(self, msg):
        if self.rotation_matrix is None:
            return

        # 手动构建标准数组
        points_list = []
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points_list.append((p[0], p[1], p[2]))
        if not points_list:
            return

        N = len(points_list)
        points = np.empty((N, 3), dtype=np.float32)
        for i, (x, y, z) in enumerate(points_list):
            points[i, 0] = x
            points[i, 1] = y
            points[i, 2] = z

        ones = np.ones((N, 1), dtype=np.float32)
        points_hom = np.hstack([points, ones])
        transformed = (self.rotation_matrix @ points_hom.T).T[:, :3]

        header = msg.header
        header.frame_id = self.get_parameter('frame_id').value
        cloud_msg = pc2.create_cloud_xyz32(header, transformed)
        self.transformed_pointcloud_pub.publish(cloud_msg)
        
def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
