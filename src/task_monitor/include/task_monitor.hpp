#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_2d_utils/parameters.hpp"

namespace robotics {
namespace brain {

class TaskMonitor : public rclcpp::Node {
  public:
    /**
     * @brief Default constructor
     */
    TaskMonitor::TaskMonitor();
    /**
     * @brief Get the initial pose from a param or initialize to {0.0, 0.0, 0.0, 0.0}
     */
    geometry_msgs::msg::Pose TaskMonitor::GetInitialPose();
  
  private:
    geometry_msgs::msg::Pose initial_pose_;
    enum class { find_exit, rescue_person };
    ros::Nodehandle nh_; // ?? 
    
    /**
     * @brief Start the desired task
     * @param Task desired task
     */
    void TaskMonitor::Start(Task task);

  // From nav2d (bt_action_server.hpp) -> TODO: Edit and get the important stuf
  protected:
    /**
     * @brief Action server callback
     */
    void executeCallback();

    // Action name
    std::string action_name_;

    // Our action server implements the template action
    std::shared_ptr<ActionServer> action_server_;

    // Behavior Tree to be executed when goal is received
    BT::Tree tree_;

    // The blackboard shared by all of the nodes in the tree
    BT::Blackboard::Ptr blackboard_;

    // The XML file that cointains the Behavior Tree to create
    std::string current_bt_xml_filename_;
    std::string default_bt_xml_filename_;

    // The wrapper class for the BT functionality
    std::unique_ptr<nav2_behavior_tree::BehaviorTreeEngine> bt_;

    // Libraries to pull plugins (BT Nodes) from
    std::vector<std::string> plugin_lib_names_;

    // A regular, non-spinning ROS node that we can use for calls to the action client
    rclcpp::Node::SharedPtr client_node_;

    // Parent node
    rclcpp_lifecycle::LifecycleNode::WeakPtr node_;

    // Clock
    rclcpp::Clock::SharedPtr clock_;

    // Logger
    rclcpp::Logger logger_{rclcpp::get_logger("BtActionServer")};

    // To publish BT logs
    std::unique_ptr<RosTopicLogger> topic_logger_;

    // Duration for each iteration of BT execution
    std::chrono::milliseconds bt_loop_duration_;

    // Default timeout value while waiting for response from a server
    std::chrono::milliseconds default_server_timeout_;

    // Parameters for Groot monitoring
    bool enable_groot_monitoring_;
    int groot_zmq_publisher_port_;
    int groot_zmq_server_port_;

    // User-provided callbacks
    OnGoalReceivedCallback on_goal_received_callback_;
    OnLoopCallback on_loop_callback_;
    OnPreemptCallback on_preempt_callback_;
    OnCompletionCallback on_completion_callback_;
};

} // namespace robotics
} // namespace brain