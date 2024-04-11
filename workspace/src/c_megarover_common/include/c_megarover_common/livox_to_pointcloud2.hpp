#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "livox_ros_driver2/msg/custom_msg.hpp"


// ref: https://github.com/porizou/livox_to_pointcloud2
class LivoxToPointCloud2 : public rclcpp::Node
{
public:
    LivoxToPointCloud2(
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    LivoxToPointCloud2(
        const std::string &name_space,
        const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    void callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg);
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};
