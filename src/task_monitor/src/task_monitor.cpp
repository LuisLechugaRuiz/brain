#include "task_monitor.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace brain {

TaskMonitor::TaskMonitor() : Node("task_monitor") {
  RCLCPP_INFO(this->get_logger(), "Constructor");
  initial_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose",
                                                                                                  rclcpp::SystemDefaultsQoS());
  this->declare_parameter<double>("initial_x", 0.0);
  this->declare_parameter<double>("initial_y", 0.0);
  this->declare_parameter<double>("initial_z", 0.0);
}


TaskMonitor::~TaskMonitor() {
  RCLCPP_INFO(this->get_logger(), "Destructor");
}


geometry_msgs::msg::PoseWithCovarianceStamped TaskMonitor::GetInitialPose() {
  // Get parameters
  double initial_x, initial_y, initial_yaw;
  this->get_parameter_or("initial_x", initial_x, 0.0);
  this->get_parameter_or("initial_y", initial_y, 0.0);
  this->get_parameter_or("initial_yaw", initial_yaw, 0.0);
  // Get the location
  geometry_msgs::msg::Point initial_location;
  initial_location.set__x(initial_x).set__y(initial_y).set__z(0.0);
  // Get the orientation
  geometry_msgs::msg::Quaternion initial_orientation;
  tf2::Quaternion initial_orientation_tf;
  initial_orientation_tf.setRPY(0, 0, initial_yaw);
  initial_orientation = tf2::toMsg(initial_orientation_tf);
  // Set initial pose
  geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
  initial_pose.pose.pose.position = initial_location;
  initial_pose.pose.pose.orientation = initial_orientation;
  initial_pose.header.frame_id = "map";
  initial_pose.header.stamp = rclcpp::Time();
  RCLCPP_INFO(this->get_logger(), "Initial pose: x: %f" ", y: %f" ", z: %f" ", yaw: %f", 
              initial_x, initial_y, 0.0, initial_yaw);
  return initial_pose;
}

void TaskMonitor::Start() {
  // don't do much at this moment, just send the initial pose to nav stack
  initial_pose_ = GetInitialPose();
  initial_pose_publisher_->publish(initial_pose_);
  RCLCPP_INFO(this->get_logger(), "Initial pose sent to AMCL");
}

} // namespace brain