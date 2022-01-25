#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

namespace behavior_tree {
namespace condition {

class NewFrontierFoundCondition : public BT::ConditionNode {
public:
  NewFrontierFoundCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {
        BT::OutputPort<geometry_msgs::msg::Point>("new_position_goal", "Position goal"),
        BT::OutputPort<geometry_msgs::msg::Quaternion>("new_orientation_goal", "Orientation goal")
    };
  }

private:
  BT::NodeStatus tick() override;

  void CallbackNewFrontier(geometry_msgs::msg::Pose::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr new_frontier_sub_;

  std::optional<geometry_msgs::msg::Pose> new_frontier_;
};

}  // namespace behavior_tree
}  // namespace condition

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_
