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

namespace bt_navigate_and_find {

BtNavigateAndFind::BtNavigateAndFind()
: rclcpp_lifecycle::LifecycleNode("bt_navigate_and_find") {
  RCLCPP_INFO(get_logger(), "Constructor");

  const std::vector<std::string> plugin_libs = {
    "task_initialize_navigation_action_bt_node",
  };

  // Declare this node's parameters
  declare_parameter("default_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);

  // TODO: Make this an util later
  // How to construct a nav2::SimpleActionServer
  // initialize_navigation_action_server_ = std::make_unique<T>(
  // get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
  // get_node_waitables_interface(), name, std::bind(&Class::Member, this));
}

BtNavigateAndFind::~BtNavigateAndFind()
{
  RCLCPP_INFO(get_logger(), "Destructor");
}

CallbackReturn BtNavigateAndFind::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the BT filename to use from the node parameter
  get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

  if (!loadBehaviorTree(default_bt_xml_filename_)) {
    RCLCPP_ERROR(get_logger(), "Error loading XML file: %s", default_bt_xml_filename_.c_str());
    return CallbackReturn::FAILURE;
  }

  // Register the plugins at the factory
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();
  BT::SharedLibrary loader;
  for (const auto & p : plugin_lib_names) {
    factory_.registerFromPlugin(loader.getOSName(p));
  }

  // Create the blackboard that will be shared by all of the nodes in the tree
  blackboard_ = BT::Blackboard::create();

  // Put items on the blackboard
  // Example:
  // blackboard_->set<int>("number_recoveries", 0);

  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigateAndFind::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");
  rclcpp::WallRate loopRate(std::chrono::milliseconds(10)); // TODO: Make it a cfg
  BT::NodeStatus result = BT::NodeStatus::RUNNING;
  // Loop until something happens with ROS or the node completes
  while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
    result = tree_.tickRoot();
    loopRate.sleep();
  }

  return (result == BT::NodeStatus::SUCCESS) ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
}

CallbackReturn BtNavigateAndFind::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigateAndFind::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return CallbackReturn::SUCCESS;
}

CallbackReturn BtNavigateAndFind::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");
  return CallbackReturn::SUCCESS;
}


bool BtNavigateAndFind::loadBehaviorTree(const std::string & bt_xml_filename)
{
  // Read the input BT XML from the specified file into a string
  std::ifstream xml_file(bt_xml_filename);
  if (!xml_file.good()) {
    RCLCPP_ERROR(get_logger(), "Couldn't open input XML file: %s", bt_xml_filename.c_str());
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
