#include <string>

#include "condition/element_found.hpp"

namespace behavior_tree {
namespace condition {

namespace {
  constexpr const char *kElementFoundTopic = "/perception_node/element_found";
}

ElementFoundCondition::ElementFoundCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), element_found_state_{false} {
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  element_found_sub = node->create_subscription<std_msgs::msg::Bool>(
    kElementFoundTopic, 1, std::bind(&ElementFoundCondition::CallbackElementFound, this, std::placeholders::_1));
}

BT::NodeStatus ElementFoundCondition::tick() {
  return element_found_state_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

void ElementFoundCondition::CallbackElementFound(std_msgs::msg::Bool::UniquePtr msg) {
  element_found_state_ = msg->data;
}

}  // namespace condition
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::condition::ElementFoundCondition>("ElementFound");
}