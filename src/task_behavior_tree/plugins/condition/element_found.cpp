#include <string>

#include "condition/element_found.hpp"

namespace behavior_tree {
namespace condition {

namespace {
  constexpr const char *kElementFoundTopic = "/perception_node/element_found";
}

// We can create a factory for each object
ElementFoundCondition::ElementFoundCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf) {
  auto node = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  RCLCPP_ERROR(node->get_logger(),"Element found constructor");
  getInput("name", name_);

  element_pose_sub_ = node->create_subscription<geometry_msgs::msg::Pose>(
    kElementFoundTopic, 1, std::bind(&ElementFoundCondition::CallbackElementFound, this, std::placeholders::_1));
}

BT::NodeStatus ElementFoundCondition::tick() {
  if (element_pose_) {
    setOutput("new_position_goal", element_pose_->position);
    setOutput("new_orientation_goal", element_pose_->orientation);
    element_pose_.reset();
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void ElementFoundCondition::CallbackElementFound(geometry_msgs::msg::Pose::SharedPtr msg) {
  element_pose_ = *msg;
}

}  // namespace condition
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::condition::ElementFoundCondition>("ElementFound");
}