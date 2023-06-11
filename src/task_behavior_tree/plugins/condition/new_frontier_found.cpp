#include <string>

#include "condition/new_frontier_found.hpp"

namespace behavior_tree {
namespace condition {

namespace {
  constexpr const char *kNewFrontierTopic = "explore/goal_frontier";
}

NewFrontierFoundCondition::NewFrontierFoundCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf) {
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  new_frontier_sub_ = node->create_subscription<geometry_msgs::msg::Point>(
    kNewFrontierTopic, rclcpp::SystemDefaultsQoS(), std::bind(&NewFrontierFoundCondition::CallbackNewFrontier, this, std::placeholders::_1));
}

BT::NodeStatus NewFrontierFoundCondition::tick() {
  // No new frontier or new one is equal to old frontier
  if (!new_frontier_ || (last_frontier_ && last_frontier_.value() == new_frontier_.value())) {
    return BT::NodeStatus::FAILURE;
  }
  // New frontier found
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.frame_id = "map";
  goal_pose.header.stamp = config().blackboard->get<rclcpp::Node::SharedPtr>("node")->now();
  goal_pose.pose.position = new_frontier_.value();
  goal_pose.pose.orientation.w = 1.0;
  setOutput("new_goal", goal_pose);
  last_frontier_ = new_frontier_;
  new_frontier_.reset();
  return BT::NodeStatus::SUCCESS;
}

void NewFrontierFoundCondition::CallbackNewFrontier(geometry_msgs::msg::Point::SharedPtr msg) {
  new_frontier_ = *msg;
}

}  // namespace condition
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::condition::NewFrontierFoundCondition>("NewFrontierFound");
}