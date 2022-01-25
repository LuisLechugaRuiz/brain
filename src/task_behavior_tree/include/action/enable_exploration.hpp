#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__ENABLE_EXPLORATION_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__ENABLE_EXPLORATION_ACTION_HPP_

#include <string>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "task_msgs/action/enable_exploration.hpp"

namespace behavior_tree {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps behavior_tree::action::EnableExploration
 */
class EnableExploration : public nav2_behavior_tree::BtActionNode<task_msgs::action::EnableExploration>
{
public:
  /**
   * @brief Constructor of EnableExploration
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  EnableExploration(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<bool>("enable", true, "Enable/disable exploration"),
      });
  }
};

}  // namespace action
}  // namespace behavior_tree

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__ENABLE_EXPLORATION_ACTION_HPP_
