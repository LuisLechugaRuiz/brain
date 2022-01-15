#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__GET_NEXT_POSE_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__GET_NEXT_POSE_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace behavior_tree {
namespace condition {

class GetNextPoseCondition : public BT::ConditionNode {
public:
  GetNextPoseCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::vector<geometry_msgs::msg::Pose>>("goal_poses", "Goal poses"),
        BT::OutputPort<geometry_msgs::msg::Point>("new_position_goal", "Position goal"),
        BT::OutputPort<geometry_msgs::msg::Quaternion>("new_orientation_goal", "Orientation goal")
    };
  }

private:
  BT::NodeStatus tick() override;


  rclcpp::Node::SharedPtr bt_node_;
  std::vector<geometry_msgs::msg::Pose> goal_poses_;
  std::vector<geometry_msgs::msg::Pose>::iterator goal_poses_it_;
};

}  // namespace behavior_tree
}  // namespace condition

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__GET_NEXT_POSE_CONDITION_HPP_