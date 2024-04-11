#include <rclcpp/rclcpp.hpp>
#include "rclcpp_components/register_node_macro.hpp"
#include "c_megarover_common/ minimal_comp_node1.hpp"

namespace minimal_comp {

MinimalCompNode1::MinimalCompNode1(
  const rclcpp::NodeOptions& options
): MinimalCompNode1("", options)
{}

MinimalCompNode1::MinimalCompNode1(
  const std::string& name_space,
  const rclcpp::NodeOptions& options
): Node("minimal_comp1", name_space, options)
{
  RCLCPP_INFO(this->get_logger(),"minimal comp 1 test");
}

}

RCLCPP_COMPONENTS_REGISTER_NODE(minimal_comp::MinimalCompNode1)
