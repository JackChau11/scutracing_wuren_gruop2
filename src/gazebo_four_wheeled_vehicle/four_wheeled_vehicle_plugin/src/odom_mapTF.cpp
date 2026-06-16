#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomToMapTF : public rclcpp::Node {
public:
  OdomToMapTF() : Node("odom_to_map_tf")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomToMapTF::odomCallback, this, std::placeholders::_1));

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Publishing dynamic TF: map -> odom");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "map";     
    tf.child_frame_id  = "odom";     


    // 修正：在完美仿真中，map、world、odom 三者是重合的。
    // 原代码把机器人在 odom 中的位姿作为 map->odom 的偏移，这是错误的，
    // 会导致 TF 树中 base_link 的 map 坐标被重复计算（变成 2x, 2y）。
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;

    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;

    tf_pub_->sendTransform(tf);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToMapTF>());
  rclcpp::shutdown();
  return 0;
}
