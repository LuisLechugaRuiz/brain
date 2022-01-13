#ifndef BEHAVIOR_TREE__PLUGINS__ACTION__INITIALIZE_NAVIGATION_ACTION_HPP_
#define BEHAVIOR_TREE__PLUGINS__ACTION__INITIALIZE_NAVIGATION_ACTION_HPP_

#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_action_node.hpp"
#include "task_msgs/action/initialize_navigation.hpp"

namespace behavior_tree {
namespace action {

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps behavior_tree::action::InitializeNavigation
 */
class InitializeNavigation : public nav2_behavior_tree::BtActionNode<task_msgs::action::InitializeNavigation>
{
public:
  /**
   * @brief Constructor of InitializeNavigation
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  InitializeNavigation(
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
        BT::InputPort<double>("timeout_s", 10.0, "Timeout in seconds"),
      });
  }
};

}  // namespace action
}  // namespace behavior_tree

#endif  // BEHAVIOR_TREE__PLUGINS__ACTION__INITIALIZE_NAVIGATION_ACTION_HPP_
