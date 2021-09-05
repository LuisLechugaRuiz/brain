#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotics {
namespace brain {

class TaskMonitor : public rclcpp::Node {
  public:
    /**
     * @brief Default constructor
     */
    TaskMonitor();
    /**
     * @brief Get the initial pose from a param or initialize to {0.0, 0.0, 0.0, 0.0}
     */
    geometry_msgs::msg::PoseWithCovarianceStamped GetInitialPose();
  
  private:
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_publisher_;
    
    /**
     * @brief Start task monitor
     */
    void Start();
};

} // namespace robotics
} // namespace brain