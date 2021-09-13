#include <memory>
#include <string>

#include "actions/initialize_navigation.hpp"

namespace behavior_tree {
namespace action {

InitializeNavigation::InitializeNavigation(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<task_msgs::action::InitializeNavigation>(xml_tag_name, action_name, conf)
{
}

void InitializeNavigation::on_tick()
{
  // TODO: Make a template to check that we get the goal_ parameters correctly
  if (!getInput("pose", goal_.pose)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "InitializeNavigation: initial pose not provided");
    return;
  }
  if (!getInput("timeout_s", goal_.timeout_s)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "InitializeNavigation: timeout_s not provided");
    return;
  }
}

}  // namespace action
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<behavior_tree::InitializeNavigation>(
        name, "initialize_navigation", config);
    };

  factory.registerBuilder<behavior_tree::InitializeNavigation>(
    "InitializeNavigation", builder);
}
