#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace robotics {
namespace brain {

TaskMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("TaskMonitor", "", true, options)
{

  RCLCPP_INFO(get_logger(), "Constructor");
  initial_pose_ = GetInitialPose();
  Start();
}

geometry_msg::msg::Pose TaskMonitor::GetInitialPose() {
  double initial_x = utils::searchAndGetParam(nh_, "initial_x", 0.0);
  double initial_y = utils::searchAndGetParam(nh_, "initial_y", 0.0);
  double initial_yaw = utils::searchAndGetParam(nh_, "initial_yaw", 0.0);
  return geometry_msg::msg::Pose(initial_x, initial_y, 0.0, initial_yaw);
}

TaskMonitor::Start() {
  // don't do much at this moment, just send the initial pose to nav stack

}

} // namespace robotics
} // namespace brain