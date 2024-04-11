#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "c_megarover_common/pointcloud_filter/pointcloud_filter.hpp"

namespace pointcloud_filter {

  FilterNode::FilterNode(
    const rclcpp::NodeOptions& options
  ): FilterNode("", options)
  {}

  FilterNode::FilterNode(
    const std::string& name_space,
    const rclcpp::NodeOptions& options
  ): Node("pointcloud_filter_node", name_space, options)
  {
    RCLCPP_INFO(this->get_logger(),"minimal comp 1 test");

    pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "in_cloud", 10, std::bind(&FilterNode::pcl_callback, this, _1));

  }

  void FilterNode::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    RCLCPP_INFO(this->get_logger(),"pcl callback");
  }

}

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_filter::FilterNode)
