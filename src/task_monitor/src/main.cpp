#include "task_monitor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<brain::TaskMonitor>();
  node->Start();
  // Sleep_for expects ns
  // std::chrono::seconds sleep_time_sec(1);
  // rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time_sec));
  RCLCPP_INFO(node->get_logger(), "Spin");
  rclcpp::spin_some(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
