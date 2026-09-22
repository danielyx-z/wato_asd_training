#include <algorithm>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), resolution_(0.1), width_(0), height_(0), default_cost_(0) {}

void CostmapCore::initCostmap(double resolution, int width, int height, int8_t default_cost) {
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  default_cost_ = default_cost;

  grid_.assign(height_, std::vector<int8_t>(width_, default_cost_));

  RCLCPP_INFO(logger_, "Costmap initialized: %d x %d cells at %.2f m/cell (%.1f x %.1f m)",
              width_, height_, resolution_, width_ * resolution_, height_ * resolution_);
}

void CostmapCore::resetCostmap() {
  for (auto & row : grid_) {
    std::fill(row.begin(), row.end(), default_cost_);
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::toOccupancyGrid(const std_msgs::msg::Header& header) const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;

  // Centre the grid on the sensor so obstacles in every direction fall inside it.
  grid.info.origin.position.x = -(width_ * resolution_) / 2.0;
  grid.info.origin.position.y = -(height_ * resolution_) / 2.0;
  grid.info.origin.orientation.w = 1.0;

  // OccupancyGrid data is a single row-major array starting at the origin corner.
  grid.data.reserve(static_cast<size_t>(width_) * height_);
  for (const auto & row : grid_) {
    grid.data.insert(grid.data.end(), row.begin(), row.end());
  }

  return grid;
}

}
