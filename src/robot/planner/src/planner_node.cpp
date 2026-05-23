#include "planner_node.hpp"

#include <cmath>

PlannerNode::PlannerNode()
  : Node("planner_node"), planner_(robot::PlannerCore(this->get_logger()))
{
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));

  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));
}

void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  current_map_ = msg;
  has_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  goal_ = *msg;
  has_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;
  has_odom_ = true;
}

void PlannerNode::timerCallback()
{
  if (state_ != State::WAITING_FOR_ROBOT_TO_REACH_GOAL) return;

  if (goalReached()) {
    RCLCPP_INFO(this->get_logger(), "Goal reached!");
    state_ = State::WAITING_FOR_GOAL;
  } else {
    planPath();
  }
}

bool PlannerNode::goalReached() const
{
  double dx = goal_.point.x - robot_x_;
  double dy = goal_.point.y - robot_y_;
  return std::sqrt(dx * dx + dy * dy) < 0.5;
}

void PlannerNode::planPath()
{
  if (!has_map_ || !has_goal_ || !has_odom_) {
    RCLCPP_WARN_ONCE(this->get_logger(), "Cannot plan: waiting for map, goal, and odom");
    return;
  }

  auto path = planner_.planPath(*current_map_, robot_x_, robot_y_, goal_.point.x, goal_.point.y);
  path.header.stamp = this->get_clock()->now();
  path_pub_->publish(path);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
