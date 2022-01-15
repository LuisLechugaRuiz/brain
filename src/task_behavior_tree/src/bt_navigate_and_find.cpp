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
#include "geometry_msgs/msg/pose.hpp"
#include "pose_3d.hpp"

#define LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)

namespace bt_navigate_and_find {

BtNavigateAndFind::BtNavigateAndFind() : rclcpp::Node("bt_navigate_and_find") {
  LOG(INFO, "Constructor");

  const std::vector<std::string> plugin_libs = {
    "task_initialize_navigation_action_bt_node",
    "nav2_recovery_node_bt_node",
    "task_element_found_condition_bt_node",
    "task_get_next_pose_condition_bt_node",
    "nav2_navigate_to_pose_action_bt_node",
    "nav2_spin_action_bt_node",
    "nav2_wait_action_bt_node",
  };

  // Declare this node's parameters
  declare_parameter("default_bt_xml_filename");
  declare_parameter("plugin_lib_names", plugin_libs);
  // Operation paremeters
  declare_parameter("goal_poses_names");
  get_parameter("goal_poses_names", goal_poses_names_);
  for (auto& goal_pose_name : goal_poses_names_) {
    declare_parameter(goal_pose_name);
  }
  declare_parameter("spin_angle", 0.7854);
  declare_parameter("wait_time", 3.0);

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
  // Operation items
  std::vector<geometry_msgs::msg::Pose> goal_poses;
  LOG(INFO, "Goal poses names size: %d", goal_poses_names_.size());
  for (auto& goal_pose_name : goal_poses_names_) {
    std::vector<double> goal_pose;
    get_parameter(goal_pose_name, goal_pose);
    utils::types::Pose3D new_pose_3D(goal_pose[0], goal_pose[1], goal_pose[2]);
    goal_poses.emplace_back(new_pose_3D.ToMsg());
  }
  LOG(INFO, "Goal poses size: %d", goal_poses.size());
  blackboard_->set<std::vector<geometry_msgs::msg::Pose>>("goal_poses", goal_poses);
  double spin_angle;
  get_parameter("spin_angle", spin_angle);
  blackboard_->set<double>("spin_angle", spin_angle);
  blackboard_->set<int>("spin_cycles", int(3.14 / spin_angle));
  blackboard_->set<double>("wait_time", (get_parameter("wait_time")).get_value<double>());

  // Get the BT filename to use from the node parameter
  get_parameter("default_bt_xml_filename", default_bt_xml_filename_);

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
