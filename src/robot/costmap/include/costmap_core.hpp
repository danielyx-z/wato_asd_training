#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Cost written into a cell a laser reading landed in
    static constexpr int8_t kObstacleCost = 100;

    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Allocate the grid and set every cell to the default cost.
    void initCostmap(double resolution, int width, int height, int8_t default_cost);

    // Clear every cell back to the default cost, reusing the existing allocation.
    void resetCostmap();

    // Convert a point in costmap-frame metres to grid indices.
    // Returns false when the point falls outside the grid.
    bool worldToGrid(double x, double y, int& gx, int& gy) const;

    // Mark a single cell as an obstacle.
    void markObstacle(int gx, int gy);

    // Spread cost outward from every marked obstacle, falling off linearly
    // with distance: cost = max_cost * (1 - distance / inflation_radius).
    void inflateObstacles(double inflation_radius, int8_t max_cost);

    // Flatten the 2D array into a message the 3D panel can render.
    nav_msgs::msg::OccupancyGrid toOccupancyGrid(const std_msgs::msg::Header& header) const;

    double resolution() const { return resolution_; }
    int width() const { return width_; }
    int height() const { return height_; }

  private:
    rclcpp::Logger logger_;

    double resolution_;
    int width_;
    int height_;
    int8_t default_cost_;

    // Metric position of the grid's bottom-left corner, relative to the sensor
    double origin_x_;
    double origin_y_;

    // grid_[y][x], indexed from the bottom-left corner of the costmap
    std::vector<std::vector<int8_t>> grid_;

    // Cells marked this scan, so inflation does not have to rescan the whole grid
    std::vector<std::pair<int, int>> obstacle_cells_;
};

}  

#endif  
