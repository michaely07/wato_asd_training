#include "control_node.hpp"

ControlNode::ControlNode()
  : Node("control_node"), control_(robot::ControlCore(this->get_logger()))
{
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1));

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  control_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this));
}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
{
  current_path_ = msg;
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_odom_ = msg;
}

void ControlNode::controlLoop()
{
  if (!current_path_ || !robot_odom_) {
    return;
  }

  if (current_path_->poses.empty()) {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    return;
  }

  double robot_x = robot_odom_->pose.pose.position.x;
  double robot_y = robot_odom_->pose.pose.position.y;
  double robot_yaw = control_.extractYaw(robot_odom_->pose.pose.orientation);

  geometry_msgs::msg::Point robot_pt;
  robot_pt.x = robot_x;
  robot_pt.y = robot_y;
  if (control_.computeDistance(robot_pt, current_path_->poses.back().pose.position)
      < control_.goal_tolerance_)
  {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    current_path_.reset();
    return;
  }

  auto lookahead = control_.findLookaheadPoint(*current_path_, robot_x, robot_y);
  if (!lookahead) {
    return;
  }

  cmd_vel_pub_->publish(control_.computeVelocity(*lookahead, robot_x, robot_y, robot_yaw));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
