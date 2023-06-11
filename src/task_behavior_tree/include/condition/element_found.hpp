#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_

#include <string>
#include <optional>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace behavior_tree {
namespace condition {

class ElementFoundCondition : public BT::ConditionNode {
public:
  ElementFoundCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<std::string>("name", "Name of the object"),
        BT::OutputPort<geometry_msgs::msg::Point>("position", "Element position"),
        BT::OutputPort<geometry_msgs::msg::Quaternion>("orientation", "Element orientation")};
  }

private:
  BT::NodeStatus tick() override;

  void CallbackElementFound(geometry_msgs::msg::Pose::SharedPtr msg);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::ConstSharedPtr element_pose_sub_;

  std::optional<geometry_msgs::msg::Pose> element_pose_;
  std::string name_;
};

}  // namespace behavior_tree
}  // namespace condition

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_
