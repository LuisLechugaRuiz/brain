#include "bt_navigate_and_find.hpp"

// TODO: clean this deps
#include <fstream>
#include <memory>
#include <streambuf>
#include <string>
#include <utility>
#include <vector>
#include <exception>

#include "behaviortree_cpp_v3/utils/shared_library.h"

#define LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)

namespace bt_navigate_and_find {

BtNavigateAndFind::BtNavigateAndFind() : rclcpp::Node("bt_navigate_and_find") {
  LOG(INFO, "Constructor");

  const std::vector<std::string> plugin_libs = {
    "nav2_pipeline_sequence_bt_node",
    "task_initialize_navigation_action_bt_node",
  };

  // Declare this node's parameters
  declare_parameter("default_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);

  if (ConfigureBT()) {
    LOG(INFO, "BT configured, starting the run.");
    RunBT();
  } else {
    LOG(ERROR, "Fail to configure BT.");
  }
}

BtNavigateAndFind::~BtNavigateAndFind() {
  LOG(INFO, "Destructor");
}

bool BtNavigateAndFind::ConfigureBT() {
  LOG(INFO, "Configuring");

  // Support for handling the topic-based goal pose from rviz
  client_node_ = std::make_shared<rclcpp::Node>("_");

  // Get the BT filename to use from the node parameter
  get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

  // Register the plugins at the factory
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();
  BT::SharedLibrary loader;
  for (const auto & p : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();
  // Put items on the blackboard
  blackboard_->set<double>("timeout_s", 10.0);
  blackboard_->set<rclcpp::Node::SharedPtr>("node", client_node_);
  blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(1000));

  // Load the behavior tree
  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    LOG(ERROR, "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return false;
  }

  return true;
}

void BtNavigateAndFind::RunBT() {
  LOG(INFO, "Activating");
  rclcpp::WallRate loopRate(std::chrono::milliseconds(10)); // TODO: Make it a cfg
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_.tickRoot();
    loopRate.sleep();
  }
}

bool BtNavigateAndFind::loadBehaviorTree(const std::string & bt_xml_filename) {
  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);
  if (!xml_file.good()) {
    LOG(ERROR, "Couldn't open input XML file: %s", bt_xml_filename.c_str());
    return false;
  }
  auto xml_string = std::string(
    std::istreambuf_iterator<char>(xml_file),
    std::istreambuf_iterator<char>());

  // Create the Behavior Tree from the XML input
  tree_ = factory_.createTreeFromText(xml_string, blackboard_);

  return true;
}

} // namespace bt_navigate_and_find
