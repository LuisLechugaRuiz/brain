#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/xml_parsing.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"

namespace bt_navigate_and_find {
/**
 * @class bt_navigate_and_find::BtNavigateAndFind
 * @brief A bt class which navigates and tries to find and specific object
 */
class BtNavigateAndFind : public rclcpp::Node {
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
   * @brief Configures the behavior tree
   * @return bool with the status
   */
  bool ConfigureBT();
  /**
   * @brief Run the behavior tree
   */
  void RunBT();
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
  // A regular, non-spinning ROS node that we can use for calls to the action client
  rclcpp::Node::SharedPtr client_node_;
  // The XML file that cointains the Behavior Tree to create
  std::string default_bt_xml_filename_;
  // Libraries to pull plugins (BT Nodes) from
  std::vector<std::string> plugin_lib_names_;

};

} // namespace bt_navigate_and_find