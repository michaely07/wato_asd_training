#include "costmap_core.hpp"

#include <cmath>
#include <algorithm>

namespace robot
{

static constexpr int freespace = 0;

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
: logger_(logger), grid_(height_, std::vector<int8_t>(width_, freespace)) {}

void CostmapCore::initCostmap()
{
  grid_.assign(height_, std::vector<int8_t>(width_, freespace));
}

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                                 double robot_x, double robot_y, double robot_yaw)
{
  robot_x_   = robot_x;
  robot_y_   = robot_y;
  robot_yaw_ = robot_yaw;
  initCostmap();

  std::vector<std::pair<int,int>> obstacle_cells;

  float angle = scan->angle_min;
  for (size_t i = 0; i < scan->ranges.size(); ++i, angle += scan->angle_increment) {
    float range = scan->ranges[i];

    if (range < scan->range_min || range > scan->range_max) {
      continue;
    }

    int x_index, y_index;
    if (convertGrid(range, angle, x_index, y_index)) {
      markObstacles(x_index, y_index);
      obstacle_cells.emplace_back(x_index, y_index);
    }
  }

  inflateObstacles(obstacle_cells);
}

bool CostmapCore::convertGrid(double range, double angle, int& x_index, int& y_index)
{
  // Hit in lidar frame
  double local_x = range * std::cos(angle);
  double local_y = range * std::sin(angle);

  // Rotate into world-aligned axes (robot_yaw_ = lidar heading in sim_world).
  // This keeps grid columns/rows parallel to the world axes regardless of
  // robot orientation, so the published grid never appears to spin.
  double dx = local_x * std::cos(robot_yaw_) - local_y * std::sin(robot_yaw_);
  double dy = local_x * std::sin(robot_yaw_) + local_y * std::cos(robot_yaw_);

  // Grid origin is at (robot + origin_offset) in world space.
  // robot terms cancel: x_index = (robot_x + dx - (robot_x + origin_x_)) / res
  x_index = static_cast<int>((dx - origin_x_) / resolution_);
  y_index = static_cast<int>((dy - origin_y_) / resolution_);

  return (x_index >= 0 && x_index < width_) && (y_index >= 0 && y_index < height_);
}

void CostmapCore::markObstacles(int x_index, int y_index)
{
  grid_[y_index][x_index] = max_cost_;
}

void CostmapCore::inflateObstacles(const std::vector<std::pair<int,int>>& obstacle_cells)
{
  int inflation_cells = static_cast<int>(std::ceil(inflation_radius_ / resolution_));

  for (const auto& [ocx, ocy] : obstacle_cells) {
    for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
      for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
        int nx = ocx + dx;
        int ny = ocy + dy;

        if (nx < 0 || nx >= width_ || ny < 0 || ny >= height_) {
          continue;
        }

        double dist = std::hypot(dx * resolution_, dy * resolution_);
        if (dist > inflation_radius_) {
          continue;
        }

        int8_t cost = static_cast<int8_t>(max_cost_ * (1.0 - dist / inflation_radius_));

        if (cost > grid_[ny][nx]) {
          grid_[ny][nx] = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::getOccupancyGrid(const std_msgs::msg::Header& header) const
{
  nav_msgs::msg::OccupancyGrid msg;
  msg.header = header;
  msg.info.resolution = static_cast<float>(resolution_);
  msg.info.width = static_cast<uint32_t>(width_);
  msg.info.height = static_cast<uint32_t>(height_);
  msg.info.origin.position.x = robot_x_ + origin_x_;
  msg.info.origin.position.y = robot_y_ + origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.data.reserve(width_ * height_);
  for (const auto& row : grid_) {
    for (int8_t cell : row) {
      msg.data.push_back(cell);
    }
  }

  return msg;
}

}  // namespace robot
