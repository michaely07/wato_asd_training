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
    void updateCostmap(const sensor_msgs::msg::LaserScan::SharedPtr& scan,
                       double robot_x, double robot_y, double robot_yaw);
    void markObstacles(int x_index, int y_index);
    void inflateObstacles(const std::vector<std::pair<int,int>>& obstacle_cells);
    bool convertGrid(double range, double angle, int& x_index, int& y_index);
    nav_msgs::msg::OccupancyGrid getOccupancyGrid(const std_msgs::msg::Header& header) const;

  private:
    rclcpp::Logger logger_;

    double resolution_ = 0.05;  // finer than global map (0.1) to avoid holes when projecting
    int width_ = 200;
    int height_ = 200;
    double origin_x_ = -5.0;
    double origin_y_ = -5.0;
    double inflation_radius_ = 1.0;
    int max_cost_ = 100;

    // robot pose in sim_world, updated each scan to set world-space grid origin
    double robot_x_ = 0.0;
    double robot_y_ = 0.0;
    double robot_yaw_ = 0.0;

    std::vector<std::vector<int8_t>> grid_;
};

}  // namespace robot

#endif
