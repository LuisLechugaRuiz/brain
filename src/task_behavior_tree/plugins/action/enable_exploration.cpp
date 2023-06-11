#include <memory>
#include <string>

#include "action/enable_exploration.hpp"

namespace behavior_tree {
namespace action {

EnableExploration::EnableExploration(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<task_msgs::action::EnableExploration>(xml_tag_name, action_name, conf)
{
  RCLCPP_ERROR(node_->get_logger(),"Construct enable exploration");
}

void EnableExploration::on_tick()
{
  RCLCPP_ERROR(node_->get_logger(),"On tick");
  if (!getInput("enable", goal_.enable)) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "EnableExploration: enable flag not provided");
    return;
  }
  RCLCPP_ERROR(node_->get_logger(),"Leaving On Tick");
}

}  // namespace action
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<behavior_tree::action::EnableExploration>(
        name, "enable_exploration", config);
    };

  factory.registerBuilder<behavior_tree::action::EnableExploration>(
    "EnableExploration", builder);
}
