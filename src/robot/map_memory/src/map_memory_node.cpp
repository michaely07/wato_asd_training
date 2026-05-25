#include "map_memory_node.hpp"

#include <cmath>

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"),
    map_memory_(robot::MapMemoryCore(this->get_logger()))
{
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10,
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10,
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&MapMemoryNode::timerCallback, this));

  constexpr float  resolution = 0.1f;
  constexpr int    width      = 1000;
  constexpr int    height     = 1000;
  constexpr double origin_x   = -50.0;
  constexpr double origin_y   = -50.0;

  global_map_.header.frame_id = "sim_world";
  global_map_.info.resolution = resolution;
  global_map_.info.width      = width;
  global_map_.info.height     = height;
  global_map_.info.origin.position.x  = origin_x;
  global_map_.info.origin.position.y  = origin_y;
  global_map_.info.origin.orientation.w = 1.0;
  global_map_.data.assign(width * height, -1);  // -1 = unknown
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_costmap_ = msg;
  has_costmap_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_x_ = msg->pose.pose.position.x;
  current_y_ = msg->pose.pose.position.y;

  const auto& q = msg->pose.pose.orientation;
  double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

  has_odom_ = true;
}

void MapMemoryNode::timerCallback()
{
  if (!has_odom_ || !has_costmap_) return;

  double dx   = current_x_ - last_update_x_;
  double dy   = current_y_ - last_update_y_;
  bool moved  = std::sqrt(dx * dx + dy * dy) >= 0.5;

  if (first_update_ || moved) {
    map_memory_.updateMap(
      global_map_, *latest_costmap_,
      current_x_, current_y_, current_yaw_);
    last_update_x_ = current_x_;
    last_update_y_ = current_y_;
    first_update_  = false;
  }

  global_map_.header.stamp = this->get_clock()->now();
  map_pub_->publish(global_map_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
