#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace robotics {
namespace brain {
namespace utils {

/**
 * @brief Search for a parameter and load it, or use the default value
 *
 * This templated function shortens a commonly used ROS pattern in which you
 * search for a parameter and get its value if it exists, otherwise returning a default value.
 *
 * @param nh NodeHandle to start the parameter search from
 * @param param_name Name of the parameter to search for
 * @param default_value Value to return if not found
 * @return Value of parameter if found, otherwise the default_value
 */
template<class param_t>
param_t searchAndGetParam(
  const nav2_util::LifecycleNode::SharedPtr & nh, const std::string & param_name,
  const param_t & default_value)
{
  param_t value = 0;
  nav2_util::declare_parameter_if_not_declared(
    nh, param_name,
    rclcpp::ParameterValue(default_value));
  nh->get_parameter(param_name, value);
  return value;
}

} // namespace robotics
} // namespace brain
} // namespace utils