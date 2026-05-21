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

void CostmapCore::updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{
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
  double x_cartesian = range * std::cos(angle);
  double y_cartesian = range * std::sin(angle);

  x_index = static_cast<int>((x_cartesian - origin_x_) / resolution_);
  y_index = static_cast<int>((y_cartesian - origin_y_) / resolution_);

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
  msg.info.origin.position.x = origin_x_;
  msg.info.origin.position.y = origin_y_;
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
