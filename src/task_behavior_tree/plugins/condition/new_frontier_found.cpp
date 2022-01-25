#include <string>

#include "condition/new_frontier_found.hpp"

namespace behavior_tree {
namespace condition {

namespace {
  constexpr const char *kNewFrontierTopic = "new_frontier";
}

NewFrontierFoundCondition::NewFrontierFoundCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf) {
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  new_frontier_sub_ = node->create_subscription<geometry_msgs::msg::Pose>(
    kNewFrontierTopic, 1, std::bind(&NewFrontierFoundCondition::CallbackNewFrontier, this, std::placeholders::_1));
}

BT::NodeStatus NewFrontierFoundCondition::tick() {
  if (!new_frontier_) {
    return BT::NodeStatus::FAILURE;
  }

  setOutput("new_position_goal", new_frontier_->position);
  setOutput("new_orientation_goal", new_frontier_->orientation);
  new_frontier_.reset();
  return BT::NodeStatus::SUCCESS;
}

void NewFrontierFoundCondition::CallbackNewFrontier(geometry_msgs::msg::Pose::SharedPtr msg) {
  new_frontier_ = *msg;
}

}  // namespace condition
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::condition::NewFrontierFoundCondition>("NewFrontierFound");
}