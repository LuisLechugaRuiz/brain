#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace bt_navigate_and_find {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @class bt_navigate_and_find::BtNavigateAndFind
 * @brief A bt class which navigates and tries to find and specific object
 */
class BtNavigateAndFind : public rclcpp_lifecycle::LifecycleNode {
public:
  /**
   * @brief Constructor
   */
  BtNavigateAndFind();
  /**
   * @brief Destructor
   */
  ~BtNavigateAndFind();

protected:
  /**
   * @brief Configures member variables
   *
   * Builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

  /**
   * @brief Load the desired behavior tree
   * @param bt_xml_filename The file containing the new BT
   * @return true if the resulting BT correspond to the one in bt_xml_filename, false otherwise.
   */
  bool loadBehaviorTree(const std::string & bt_id);

  // The blackboard shared by all of the nodes in the tree
  BT::Blackboard::Ptr blackboard_;
  // The factory that will be used to construct the behavior tree
  BT::BehaviorTreeFactory factory_;
  // The tree of the BT
  BT::Tree tree_;
  // The XML file that cointains the Behavior Tree to create
  std::string default_bt_xml_filename_;
  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

};

} // namespace bt_navigate_and_find