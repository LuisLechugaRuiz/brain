#include "task_monitor.hpp"

#include <future>

#include "nav2_util/simple_action_server.hpp"
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
  static constexpr char const * kBTNode = "bt_navigate_and_find";
  static constexpr char const * kBTGetStateTopic = "bt_navigate_and_find/get_state";
  static constexpr char const * kBTChangeStateTopic = "bt_navigate_and_find/change_state";

  template<typename FutureT, typename WaitTimeT>
  std::future_status wait_for_result(FutureT & future, WaitTimeT time_to_wait) {
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do {
      auto now = std::chrono::steady_clock::now();
      auto time_left = end - now;
      if (time_left <= std::chrono::seconds(0)) {break;}
      status = future.wait_for((time_left < wait_period) ? time_left : wait_period);
    } while (rclcpp::ok() && status != std::future_status::ready);
    return status;
  }
}

TaskMonitor::TaskMonitor() : rclcpp::Node("task_monitor") {
  LOG(INFO, "Constructor");

  declare_parameter<double>("initial_x", 0.0);
  declare_parameter<double>("initial_y", 0.0);
  declare_parameter<double>("initial_z", 0.0);
  declare_parameter<double>("pose_distance_tolerance", 0.25);
  declare_parameter<double>("pose_angle_tolerance", 0.09); // 5 degrees by default

  get_parameter("pose_distance_tolerance", pose_distance_tolerance_);
  get_parameter("pose_angle_tolerance", pose_angle_tolerance_); 

  // Publisher to send to amcl node the initial pose
  initial_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  // Subscriber to get the robot pose from amcl node
  robot_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TaskMonitor::AmclPoseCallback, this, std::placeholders::_1));

  // Clients to change/get the bt state
  client_get_bt_state_ = create_client<lifecycle_msgs::srv::GetState>(kBTGetStateTopic);
  client_change_bt_state_ = create_client<lifecycle_msgs::srv::ChangeState>(kBTChangeStateTopic);

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

// TODO: Fill the initial pose in the BT blackboard at the start
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
      initialize_navigation_action_server_->succeeded_current();
      return;
    }
    loop_rate.sleep();
  }
  LOG(ERROR, "Failed to set initial pose");
  initialize_navigation_action_server_->terminate_current();
}

} // namespace brain