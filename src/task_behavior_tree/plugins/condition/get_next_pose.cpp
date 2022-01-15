#include <string>

#include "condition/get_next_pose.hpp"

namespace behavior_tree {
namespace condition {

GetNextPoseCondition::GetNextPoseCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf) {
  bt_node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("goal_poses", goal_poses_);
  goal_poses_it_ = goal_poses_.begin();
}

BT::NodeStatus GetNextPoseCondition::tick() {
  RCLCPP_INFO(bt_node_->get_logger(), "Number of poses is: %d", goal_poses_.size());
  if (goal_poses_it_ != goal_poses_.end()) {
    setOutput("new_position_goal", goal_poses_it_->position);
    setOutput("new_orientation_goal", goal_poses_it_->orientation);
    goal_poses_it_++;
    RCLCPP_INFO(bt_node_->get_logger(), "Last pose not reached yet, scheduling new one.");
    return BT::NodeStatus::SUCCESS;
  }

  RCLCPP_INFO(bt_node_->get_logger(), "Last pose reached!!");
  return BT::NodeStatus::FAILURE;
}

}  // namespace condition
}  // namespace behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<behavior_tree::condition::GetNextPoseCondition>("GetNextPose");
}
