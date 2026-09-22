#include <cmath>

#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}

void MapMemoryCore::initMap(const std::string& frame_id, double resolution, int width, int height) {
  map_.header.frame_id = frame_id;

  map_.info.resolution = resolution;
  map_.info.width = width;
  map_.info.height = height;

  // The world is centred on the origin, so centre the map there too.
  map_.info.origin.position.x = -(width * resolution) / 2.0;
  map_.info.origin.position.y = -(height * resolution) / 2.0;
  map_.info.origin.orientation.w = 1.0;

  // Everything starts unknown; cells stay that way until the robot sees them.
  map_.data.assign(static_cast<size_t>(width) * height, kUnknown);

  RCLCPP_INFO(logger_, "Global map initialized: %d x %d cells at %.2f m/cell (%.1f x %.1f m) in frame '%s'",
              width, height, resolution, width * resolution, height * resolution, frame_id.c_str());
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap,
                                     double robot_x, double robot_y, double robot_yaw) {
  if (costmap.info.width == 0 || costmap.info.height == 0 || map_.data.empty()) {
    return;
  }

  const double cos_yaw = std::cos(robot_yaw);
  const double sin_yaw = std::sin(robot_yaw);

  const int map_w = static_cast<int>(map_.info.width);
  const int map_h = static_cast<int>(map_.info.height);
  const double map_res = map_.info.resolution;
  const double map_ox = map_.info.origin.position.x;
  const double map_oy = map_.info.origin.position.y;

  const int cm_w = static_cast<int>(costmap.info.width);
  const int cm_h = static_cast<int>(costmap.info.height);
  const double cm_res = costmap.info.resolution;
  const double cm_ox = costmap.info.origin.position.x;
  const double cm_oy = costmap.info.origin.position.y;

  // Walk the global map and pull from the costmap rather than pushing costmap
  // cells outward. At equal resolutions a forward mapping drops cells wherever
  // the rotation lands two source cells on one destination, leaving speckled
  // holes; pulling gives every destination cell exactly one source.
  for (int my = 0; my < map_h; ++my) {
    const double wy = map_oy + (my + 0.5) * map_res;

    for (int mx = 0; mx < map_w; ++mx) {
      const double wx = map_ox + (mx + 0.5) * map_res;

      // Global frame -> robot frame, the inverse of the robot's pose.
      const double dx = wx - robot_x;
      const double dy = wy - robot_y;
      const double lx =  cos_yaw * dx + sin_yaw * dy;
      const double ly = -sin_yaw * dx + cos_yaw * dy;

      const int cx = static_cast<int>(std::floor((lx - cm_ox) / cm_res));
      const int cy = static_cast<int>(std::floor((ly - cm_oy) / cm_res));

      // Outside the costmap's footprint the map keeps what it already had.
      if (cx < 0 || cx >= cm_w || cy < 0 || cy >= cm_h) {
        continue;
      }

      const int8_t value = costmap.data[static_cast<size_t>(cy) * cm_w + cx];

      // Unknown readings never erase what the map remembers.
      if (value == kUnknown) {
        continue;
      }

      map_.data[static_cast<size_t>(my) * map_w + mx] = value;
    }
  }
}

} 
