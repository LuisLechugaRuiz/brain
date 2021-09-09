#include <string>

#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"

namespace utils {
namespace types {

class Pose3D {
  public:
    /**
     * @brief Default constructor
     */
    explicit Pose3D(double x, double y, double yaw);
    /**
     * @brief Default destructor
     */
    ~Pose3D();
    /**
     * @brief Convert to geometry_msgs::msg::Pose type
     * @return geometry_msgs::msg::Pose
     */
    geometry_msgs::msg::Pose ToMsg();
    /**
     * @brief Convert to geometry_msgs::msg::PoseWithCovarianceStamped type
     * @return geometry_msgs::msg::PoseWithCovarianceStamped
     */
    geometry_msgs::msg::PoseWithCovariance ToMsgWithCovariance();
    /**
     * @brief Convert to geometry_msgs::msg::PoseWithCovarianceStamped type
     * @param covariance array with the covariance
     * @return geometry_msgs::msg::PoseWithCovarianceStamped
     */
    geometry_msgs::msg::PoseWithCovariance ToMsgWithCovariance(double covariance[36]);

    /**
     * @brief Get x_ value
     * @return x_
     */
    double x() { return x_; };
    /**
     * @brief Get y_ value
     * @return y_
     */
    double y() { return y_; };
    /**
     * @brief Get yaw_ value
     * @return yaw_
     */
    double yaw() { return yaw_; };

  private:
    double x_;          //!< x position
    double y_;          //!< y position
    double yaw_;        //!< yaw angle

};

} // namespace utils
} // namespace types