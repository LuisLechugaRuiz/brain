#include "task_monitor.hpp"

#include "pose_3d.hpp"
#define LOG(level, ...) RCLCPP_##level(this->get_logger(), __VA_ARGS__)

namespace brain {

TaskMonitor::TaskMonitor() : Node("task_monitor") {
  LOG(INFO, "constructor");
  initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",
                                                                                                  rclcpp::SystemDefaultsQoS());
  this->declare_parameter<double>("initial_x", 0.0);
  this->declare_parameter<double>("initial_y", 0.0);
  this->declare_parameter<double>("initial_z", 0.0);
}

TaskMonitor::~TaskMonitor() {
  LOG(WARN, "Destructor");
}

geometry_msgs::msg::PoseWithCovarianceStamped TaskMonitor::GetInitialPose() {
  // Get parameters
  double initial_x, initial_y, initial_yaw;
  this->get_parameter_or("initial_x", initial_x, 0.0);
  this->get_parameter_or("initial_y", initial_y, 0.0);
  this->get_parameter_or("initial_yaw", initial_yaw, 0.0);
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_with_covariance_msg;
  utils::types::Pose3D initial_pose(initial_x, initial_y, initial_yaw);
  initial_pose_with_covariance_msg.pose = initial_pose.ToMsgWithCovariance();
  initial_pose_with_covariance_msg.header.frame_id = "map";
  initial_pose_with_covariance_msg.header.stamp = rclcpp::Time();
  LOG(INFO, "Initial pose: x: %f" ", y: %f" ", z: %f" ", yaw: %f",
             initial_pose.x(), initial_pose.y(), 0.0, initial_pose.yaw());
  return initial_pose_with_covariance_msg;
}

void TaskMonitor::Start() {
  // don't do much at this moment, just send the initial pose to nav stack
  initial_pose_ = GetInitialPose();
  initial_pose_publisher_->publish(initial_pose_);
  LOG(INFO, "Initial pose sent to AMCL");
}

} // namespace brain