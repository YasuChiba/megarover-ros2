#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace pointcloud_filter
{

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
  };

}