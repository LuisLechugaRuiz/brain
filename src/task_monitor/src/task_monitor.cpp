#include "task_monitor.hpp"

#include "nav2_util/simple_action_server.hpp"
#include "pose_3d.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// #define LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)

namespace brain {

TaskMonitor::TaskMonitor() {
  rclcpp_node_ = std::make_shared<rclcpp::Node>("task_monitor");
  RCLCPP_INFO(rclcpp_node_->get_logger(), "Constructor");

  rclcpp_node_->declare_parameter<double>("initial_x", 0.0);
  rclcpp_node_->declare_parameter<double>("initial_y", 0.0);
  rclcpp_node_->declare_parameter<double>("initial_z", 0.0);
  rclcpp_node_->declare_parameter<double>("pose_distance_tolerance", 0.0);
  rclcpp_node_->declare_parameter<double>("pose_angle_tolerance", 0.0);

  rclcpp_node_->get_parameter_or("pose_distance_tolerance", pose_distance_tolerance_, 0.25);
  rclcpp_node_->get_parameter_or("pose_angle_tolerance", pose_angle_tolerance_, 0.09); // 5 degrees by default

  // Publisher to send to amcl node the initial pose
  initial_pose_pub_ = rclcpp_node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Subscriber to get the robot pose from amcl node
  robot_pose_sub_ = rclcpp_node_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TaskMonitor::AmclPoseCallback, this, std::placeholders::_1));

  // Action server to initialize navigation properly
  initialize_navigation_action_server_ = std::make_unique<InitializeNavigationActionServer>(
      rclcpp_node_, "initialize_navigation", std::bind(&TaskMonitor::SetInitialPose, this));

  initial_pose_ = GetInitialPose();
}

TaskMonitor::~TaskMonitor() {
  RCLCPP_WARN(rclcpp_node_->get_logger(), "Destructor");
}

// TODO: Fill the initial pose in the BT blackboard at the start
geometry_msgs::msg::PoseWithCovarianceStamped TaskMonitor::GetInitialPose() {
  // Get parameters
  double initial_x, initial_y, initial_yaw;
  rclcpp_node_->get_parameter_or("initial_x", initial_x, 0.0);
  rclcpp_node_->get_parameter_or("initial_y", initial_y, 0.0);
  rclcpp_node_->get_parameter_or("initial_yaw", initial_yaw, 0.0);
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance_msg;
  utils::types::Pose3D initial_pose(initial_x, initial_y, initial_yaw);
  initial_pose_with_covariance_msg.pose = initial_pose.ToMsgWithCovariance();
  initial_pose_with_covariance_msg.header.frame_id = "map";
  initial_pose_with_covariance_msg.header.stamp = rclcpp::Time();
  RCLCPP_INFO(rclcpp_node_->get_logger(), "Initial pose: x: %f" ", y: %f" ", z: %f" ", yaw: %f",
             initial_pose.x(), initial_pose.y(), 0.0, initial_pose.yaw());
  return initial_pose_with_covariance_msg;
}

void TaskMonitor::AmclPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  if (msg) {
    robot_pose_ = *msg;
  }
}

bool TaskMonitor::IsRobotAtPose(geometry_msgs::msg::PoseWithCovarianceStamped desired_pose) {
  // Get the relative tf from the actual robot pose to the desired pose
  tf2::Transform robot_pose_tf, desired_pose_tf;
  tf2::fromMsg(robot_pose_.pose.pose, robot_pose_tf);
  tf2::fromMsg(desired_pose.pose.pose, desired_pose_tf);
  tf2::Transform relative_tf = robot_pose_tf.inverseTimes(desired_pose_tf);

  // Check that location coordinates are lower than the distance tolerance
  auto relative_translation = relative_tf.getOrigin();
  std::vector<double> relative_translation_vector{relative_translation.getX(), relative_translation.getY(), relative_translation.getZ()};
  for (auto &coordinate : relative_translation_vector) {
    if (coordinate > pose_distance_tolerance_)
      return false;
  }

  // Check that the RPY angles are lower than the angle tolerance
  tf2::Matrix3x3 relative_orientation_matrix(relative_tf.getRotation());
  std::vector<double> relative_orientation_vector;
  relative_orientation_vector.reserve(3);
  relative_orientation_matrix.getRPY(relative_orientation_vector[0], relative_orientation_vector[1], relative_orientation_vector[2]);
  for (auto& angle : relative_orientation_vector) {
    if (angle > pose_angle_tolerance_)
      return false;
  }

  return true;
}

void TaskMonitor::SetInitialPose() {
  RCLCPP_INFO(rclcpp_node_->get_logger(), "Initialize navigation service called");
  auto start_time = rclcpp_node_->now();
  initial_pose_ = initialize_navigation_action_server_->get_current_goal()->pose;
  timeout_s_ = initialize_navigation_action_server_->get_current_goal()->timeout_s;
  initial_pose_pub_->publish(initial_pose_);
  
  // Check that the robot is at the desired pose
  while ((rclcpp_node_->now() - start_time).seconds() < timeout_s_) {
    if (IsRobotAtPose(initial_pose_)) {
      RCLCPP_INFO(rclcpp_node_->get_logger(), "Robot is at initial pose");
      initialize_navigation_action_server_->succeeded_current();
      return;
    }
    RCLCPP_INFO(rclcpp_node_->get_logger(), "Robot still not at initial pose");
  }
  RCLCPP_ERROR(rclcpp_node_->get_logger(), "Failed to set initial pose");
  initialize_navigation_action_server_->terminate_current();
}

} // namespace brain