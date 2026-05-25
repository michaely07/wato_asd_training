#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <optional>

namespace robot
{

class ControlCore {
public:
  ControlCore(const rclcpp::Logger& logger);

  std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint(
      const nav_msgs::msg::Path& path,
      double robot_x, double robot_y) const;

  geometry_msgs::msg::Twist computeVelocity(
      const geometry_msgs::msg::PoseStamped& lookahead,
      double robot_x, double robot_y, double robot_yaw) const;

  double computeDistance(
      const geometry_msgs::msg::Point& a,
      const geometry_msgs::msg::Point& b) const;

  double extractYaw(const geometry_msgs::msg::Quaternion& quat) const;

  double lookahead_distance_;
  double goal_tolerance_;
  double linear_speed_;

private:
  rclcpp::Logger logger_;
};

}  // namespace robot

#endif
