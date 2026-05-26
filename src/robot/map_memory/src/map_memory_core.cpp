#include "map_memory_core.hpp"

#include <cmath>

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger)
  : logger_(logger) {}

void MapMemoryCore::updateMap(
  nav_msgs::msg::OccupancyGrid& global_map,
  const nav_msgs::msg::OccupancyGrid& costmap)
{
  const double res  = costmap.info.resolution;
  // costmap is published in sim_world with a world-aligned origin,
  // so cell centres are already in world coordinates — no rotation needed.
  const double ox   = costmap.info.origin.position.x;
  const double oy   = costmap.info.origin.position.y;
  const int32_t cw  = static_cast<int32_t>(costmap.info.width);
  const int32_t ch  = static_cast<int32_t>(costmap.info.height);

  const double gres = global_map.info.resolution;
  const double gox  = global_map.info.origin.position.x;
  const double goy  = global_map.info.origin.position.y;
  const int32_t gw  = static_cast<int32_t>(global_map.info.width);
  const int32_t gh  = static_cast<int32_t>(global_map.info.height);

  for (int32_t i = 0; i < ch; ++i) {
    for (int32_t j = 0; j < cw; ++j) {
      int8_t val = costmap.data[i * cw + j];
      if (val < 0) continue;

      double world_x = ox + (j + 0.5) * res;
      double world_y = oy + (i + 0.5) * res;

      int32_t gj = static_cast<int32_t>((world_x - gox) / gres);
      int32_t gi = static_cast<int32_t>((world_y - goy) / gres);

      if (gi < 0 || gi >= gh || gj < 0 || gj >= gw) continue;

      global_map.data[gi * gw + gj] = val;
    }
  }
}

}  // namespace robot
