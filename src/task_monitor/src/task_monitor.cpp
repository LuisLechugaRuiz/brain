#include "task_monitor.hpp"

#include <future>

#include "pose_3d.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "rclcpp/rclcpp.hpp"

#define LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)

namespace brain {

namespace {
  constexpr double kDefaultDistanceTolerance = 0.25;
  constexpr double kDefaultAngleTolerance = 0.09; // 5 degrees by default
}

TaskMonitor::TaskMonitor() : rclcpp::Node("task_monitor") {
  LOG(INFO, "Constructor");

  declare_parameter("initial_x");
  declare_parameter("initial_y");
  declare_parameter("initial_z");
  declare_parameter("pose_distance_tolerance");
  declare_parameter("pose_angle_tolerance");

  get_parameter_or("pose_distance_tolerance", pose_distance_tolerance_, kDefaultDistanceTolerance);
  get_parameter_or("pose_angle_tolerance", pose_angle_tolerance_, kDefaultAngleTolerance);

  // Publisher to send to amcl node the initial pose
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Subscriber to get the robot pose from amcl node
  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TaskMonitor::AmclPoseCallback, this, std::placeholders::_1));

  // Action server to initialize navigation properly
  initialize_navigation_action_server_ = std::make_unique<InitializeNavigationActionServer>(
      get_node_base_interface(), get_node_clock_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "initialize_navigation", std::bind(&TaskMonitor::SetInitialPose, this));

  initial_pose_ = GetInitialPose();

  LOG(INFO, "Configure completed, activating");
}

TaskMonitor::~TaskMonitor() {
  LOG(WARN, "Destructor");
}

geometry_msgs::msg::PoseWithCovarianceStamped TaskMonitor::GetInitialPose() {
  // Get parameters
  double initial_x, initial_y, initial_yaw;
  get_parameter_or("initial_x", initial_x, 0.0);
  get_parameter_or("initial_y", initial_y, 0.0);
  get_parameter_or("initial_yaw", initial_yaw, 0.0);
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance_msg;
  utils::types::Pose3D initial_pose(initial_x, initial_y, initial_yaw);
  initial_pose_with_covariance_msg.pose = initial_pose.ToMsgWithCovariance();
  initial_pose_with_covariance_msg.header.frame_id = "map";
  initial_pose_with_covariance_msg.header.stamp = rclcpp::Time();
  LOG(INFO, "Initial pose: x: %f" ", y: %f" ", z: %f" ", yaw: %f",
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
    if (coordinate > pose_distance_tolerance_) {
      LOG(ERROR, "A distance coordinate is not lower than %f, value is: %f", pose_distance_tolerance_, coordinate);
      return false;
    }
  }

  // Check that the RPY angles are lower than the angle tolerance
  tf2::Matrix3x3 relative_orientation_matrix(relative_tf.getRotation());
  std::vector<double> relative_orientation_vector;
  relative_orientation_vector.reserve(3);
  relative_orientation_matrix.getRPY(relative_orientation_vector[0], relative_orientation_vector[1], relative_orientation_vector[2]);
  for (auto& angle : relative_orientation_vector) {
    if (angle > pose_angle_tolerance_) {
      LOG(ERROR, "A angle coordinate is not lower than %f, value is: %f", pose_angle_tolerance_, angle);
      return false;
    }
  }

  return true;
}

void TaskMonitor::SetInitialPose() {
  LOG(INFO, "Initialize navigation service called");
  auto start_time = now();
  timeout_s_ = initialize_navigation_action_server_->get_current_goal()->timeout_s;
  initial_pose_pub_->publish(initial_pose_);
  
  rclcpp::WallRate loop_rate(4);
  // Check that the robot is at the desired pose
  while ((now() - start_time).seconds() < timeout_s_) {
    if (IsRobotAtPose(initial_pose_)) {
      LOG(INFO, "Robot is at initial pose");
      // Wait until nav is properly initialized. TODO: Make it better subscribing to any topic
      rclcpp::sleep_for(std::chrono::milliseconds(5000));
      initialize_navigation_action_server_->succeeded_current();
      return;
    }
    loop_rate.sleep();
  }
  LOG(ERROR, "Failed to set initial pose");
  initialize_navigation_action_server_->terminate_current();
}

} // namespace brain