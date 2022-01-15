#ifndef BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_
#define BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_

#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

namespace behavior_tree {
namespace condition {

class ElementFoundCondition : public BT::ConditionNode {
public:
  ElementFoundCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts()
  {
    return {};
  }

private:
  BT::NodeStatus tick() override;

  void CallbackElementFound(std_msgs::msg::Bool::UniquePtr msg);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr element_found_sub;

  bool element_found_state_;
};

}  // namespace behavior_tree
}  // namespace condition

#endif  // BEHAVIOR_TREE__PLUGINS__CONDITION__ELEMENT_FOUND_CONDITION_HPP_
