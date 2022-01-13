#include "task_monitor.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto task_monitor_node = std::make_shared<brain::TaskMonitor>();
  // Need to wait some time for the publisher to stablish connection, bug ?
  std::chrono::seconds sleep_time_sec(1);
  rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(sleep_time_sec));
  rclcpp::spin(task_monitor_node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
