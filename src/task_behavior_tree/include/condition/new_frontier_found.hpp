#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("new_goal", "Goal Pose")
    };
  }

private:
  BT::NodeStatus tick() override;

  void CallbackNewFrontier(geometry_msgs::msg::Point::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr new_frontier_sub_;

  std::optional<geometry_msgs::msg::Point> new_frontier_;
};

}  // namespace behavior_tree
}  // namespace condition

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__NEW_FRONTIER_FOUND_CONDITION_HPP_
