#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "task_msgs/action/initialize_navigation.hpp"

namespace brain {
using namespace std::chrono_literals;

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
    /**
     * @brief Get the initial pose from a param or initialize to {0.0, 0.0, 0.0, 0.0}
     * @return geometry_msgs::msg::PoseWithCovarianceStamped with the initial pose
     */
    geometry_msgs::msg::PoseWithCovarianceStamped GetInitialPose();
    /**
     * @brief Set the initial pose as initial_pose
     */
    void SetInitialPose();
    /**
     * @brief Get the robot pose from AMCL
     */
    void AmclPoseCallback (const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    /**
     * @brief Check if the robot is close to the desired pose
     * @param desired_pose the pose where we want the robot at
     * @return bool with the comparation result
     */
    bool IsRobotAtPose(geometry_msgs::msg::PoseWithCovarianceStamped desired_pose);

    double pose_distance_tolerance_;
    double pose_angle_tolerance_;
    double timeout_s_;
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    geometry_msgs::msg::PoseWithCovarianceStamped robot_pose_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;

    // Action server that implements the InitializeNavigation action
    using InitializeNavigationAction = task_msgs::action::InitializeNavigation;
    using InitializeNavigationActionServer = nav2_util::SimpleActionServer<InitializeNavigationAction>;
    std::unique_ptr<InitializeNavigationActionServer> initialize_navigation_action_server_;
};

} // namespace brain