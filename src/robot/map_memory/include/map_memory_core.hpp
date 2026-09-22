#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot
{

class MapMemoryCore {
  public:
    // OccupancyGrid's value for a cell nothing is known about
    static constexpr int8_t kUnknown = -1;

    explicit MapMemoryCore(const rclcpp::Logger& logger);

    // Allocate the global map and mark every cell unknown.
    void initMap(const std::string& frame_id, double resolution, int width, int height);

    // Fuse one costmap into the global map. The pose is that of the costmap's
    // own frame, expressed in the global map's frame.
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                          double robot_x, double robot_y, double robot_yaw);

    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;

    nav_msgs::msg::OccupancyGrid map_;
};

}  

#endif  
