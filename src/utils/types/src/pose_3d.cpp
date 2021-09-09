#include "pose_3d.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace utils {
namespace types {

Pose3D::Pose3D(double x, double y, double yaw) : x_(x), y_(y), yaw_(yaw) {
}

Pose3D::~Pose3D() {
}

geometry_msgs::msg::Pose Pose3D::ToMsg() {
  // Get the location
  geometry_msgs::msg::Point location;
  location.set__x(x_).set__y(y_).set__z(0.0);
  // Get the orientation
  geometry_msgs::msg::Quaternion orientation;
  tf2::Quaternion orientation_tf;
  orientation_tf.setRPY(0, 0, yaw_);
  orientation = tf2::toMsg(orientation_tf);
  // Set the pose
  geometry_msgs::msg::Pose pose;
  pose.position = location;
  pose.orientation = orientation;
  return pose;
}

geometry_msgs::msg::PoseWithCovariance Pose3D::ToMsgWithCovariance() {
  // Get pose 
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  pose_with_covariance.pose = ToMsg();
  return pose_with_covariance;
}

geometry_msgs::msg::PoseWithCovariance Pose3D::ToMsgWithCovariance(double covariance[36]) {
  // Get pose 
  geometry_msgs::msg::PoseWithCovariance pose_with_covariance;
  pose_with_covariance.pose = ToMsg();
  // Get covariance
  for (int i=0; i<36; covariance++) {
    pose_with_covariance.covariance[i] = *covariance;
  }
  return pose_with_covariance;
}

} // namespace utils
} // namespace types