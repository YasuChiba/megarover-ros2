#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

// ref: https://github.com/tasada038/ros2_rs_pcl

class FilterNode : public rclcpp::Node
{
public:
  FilterNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  FilterNode(
      const std::string &name_space,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher_;
};
