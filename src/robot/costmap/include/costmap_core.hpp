#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    // Allocate the grid and set every cell to the default cost.
    void initCostmap(double resolution, int width, int height, int8_t default_cost);

    // Clear every cell back to the default cost, reusing the existing allocation.
    void resetCostmap();

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

    // grid_[y][x], indexed from the bottom-left corner of the costmap
    std::vector<std::vector<int8_t>> grid_;
};

}  

#endif  
