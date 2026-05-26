#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);

    void updateMap(
      nav_msgs::msg::OccupancyGrid& global_map,
      const nav_msgs::msg::OccupancyGrid& costmap);

  private:
    rclcpp::Logger logger_;
};

}

#endif
