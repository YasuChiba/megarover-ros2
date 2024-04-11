#include <rclcpp/rclcpp.hpp>

namespace minimal_comp {

class MinimalCompNode1 : public rclcpp::Node{
public:
  MinimalCompNode1(
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
  MinimalCompNode1(
    const std::string& name_space,
    const rclcpp::NodeOptions& options=rclcpp::NodeOptions()
  );
};

}