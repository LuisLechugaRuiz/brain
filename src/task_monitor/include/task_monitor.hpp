#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace brain {

class TaskMonitor : public rclcpp::Node {
  public:
    /**
     * @brief Default constructor
     */
    TaskMonitor();
    /**
     * @brief Default destructor
     */
    ~TaskMonitor();
  
  private:
    const double pose_distance_tolerance_;
    const double pose_distance_angle_;
    const double timeout_s_;
    const geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;

    // Action server that implements the InitializeNavigation action
    using task_msgs::action::InitializeNavigation InitializeNavigationAction;
    using nav2_util::SimpleActionServer<InitializeNavigationAction> InitializeNavigationActionServer;
    std::unique_ptr<nav2_util::SimpleActionServer<task_msgs::action::InitializeNavigation>> initialize_navigation_action_server_;

    /**
     * @brief Get the initial pose from a param or initialize to {0.0, 0.0, 0.0, 0.0}
     * @return geometry_msgs::msg::PoseWithCovarianceStamped with the initial pose
     */
    geometry_msgs::msg::PoseWithCovarianceStamped GetInitialPose();
    /**
     * @brief Set the initial pose as initial_pose
     * @return bool with the result of the action
     */
    bool SetInitialPose();
    /**
     * @brief Get the robot pose from AMCL
     */
    void AmclPoseCallback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    /**
     * @brief Check if the robot is close to the desired pose
     * @param desired_pose the pose where we want the robot at
     * @return bool with the comparation result
     */
    bool IsRobotAtPose(geometry_msgs::msg::PoseWithCovarianceStamped desired_pose);

};

} // namespace brain