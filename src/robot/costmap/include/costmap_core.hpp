#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>

namespace robot
{

class CostmapCore {
  public:
    explicit CostmapCore(const rclcpp::Logger& logger);

    void initCostmap();
    void updateCostmap();
    void markObstacles();
    void inflateObstacles();
    bool convertGrid();
    nav_msgs::msg::OccupancyGrid getOccupancyGrid() const;

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.1;
    int width_ = 100;
    int height_ = 100;
    double origin_x_ = width_ / 2.0;
    double origin_y_ = height_ / 2.0;
    double inflation_radius_ 1.0;
    int max_cost_ = 100;

    std::vector<std::vector<int8_t>> grid_;
};

}

#endif
