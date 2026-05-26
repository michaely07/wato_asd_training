#include "control_core.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger)
  : logger_(logger),
    lookahead_distance_(0.8),
    goal_tolerance_(0.1),
    linear_speed_(1.0) {}

std::optional<geometry_msgs::msg::PoseStamped> ControlCore::findLookaheadPoint(
    const nav_msgs::msg::Path& path,
    double robot_x, double robot_y) const
{
  if (path.poses.empty()) {
    return std::nullopt;
  }

  geometry_msgs::msg::Point robot_pt;
  robot_pt.x = robot_x;
  robot_pt.y = robot_y;

  size_t closest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double d = computeDistance(path.poses[i].pose.position, robot_pt);
    if (d < min_dist) {
      min_dist = d;
      closest_idx = i;
    }
  }

  for (size_t i = closest_idx; i < path.poses.size(); ++i) {
    if (computeDistance(path.poses[i].pose.position, robot_pt) >= lookahead_distance_) {
      return path.poses[i];
    }
  }

  return path.poses.back();
}

geometry_msgs::msg::Twist ControlCore::computeVelocity(
    const geometry_msgs::msg::PoseStamped& lookahead,
    double robot_x, double robot_y, double robot_yaw) const
{
  geometry_msgs::msg::Twist cmd_vel;

  double dx = lookahead.pose.position.x - robot_x;
  double dy = lookahead.pose.position.y - robot_y;
  double L = std::sqrt(dx * dx + dy * dy);

  if (L < 1e-6) {
    return cmd_vel;
  }

  double local_x = std::cos(robot_yaw) * dx + std::sin(robot_yaw) * dy;
  double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;
  double alpha = std::atan2(local_y, local_x);

  double curvature = 2.0 * std::sin(alpha) / L;

  // Slow down proportionally for sharp turns; minimum 0.2 of normal speed to avoid stalling
  cmd_vel.linear.x = linear_speed_ * std::max(0.2, std::cos(alpha));
  cmd_vel.angular.z = std::clamp(linear_speed_ * curvature, -2.0, 2.0);

  return cmd_vel;
}

double ControlCore::computeDistance(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b) const
{
  double dx = a.x - b.x;
  double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& quat) const
{
  double siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y);
  double cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

}  // namespace robot
