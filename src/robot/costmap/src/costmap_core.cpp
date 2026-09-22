#include <algorithm>
#include <cmath>

#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger)
  : logger_(logger), resolution_(0.1), width_(0), height_(0), default_cost_(0),
    origin_x_(0.0), origin_y_(0.0) {}

void CostmapCore::initCostmap(double resolution, int width, int height, int8_t default_cost) {
  resolution_ = resolution;
  width_ = width;
  height_ = height;
  default_cost_ = default_cost;

  // Centre the grid on the sensor so obstacles in every direction fall inside it.
  origin_x_ = -(width_ * resolution_) / 2.0;
  origin_y_ = -(height_ * resolution_) / 2.0;

  grid_.assign(height_, std::vector<int8_t>(width_, default_cost_));

  RCLCPP_INFO(logger_, "Costmap initialized: %d x %d cells at %.2f m/cell (%.1f x %.1f m)",
              width_, height_, resolution_, width_ * resolution_, height_ * resolution_);
}

void CostmapCore::resetCostmap() {
  for (auto & row : grid_) {
    std::fill(row.begin(), row.end(), default_cost_);
  }
  obstacle_cells_.clear();
}

bool CostmapCore::worldToGrid(double x, double y, int& gx, int& gy) const {
  gx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
  gy = static_cast<int>(std::floor((y - origin_y_) / resolution_));

  return gx >= 0 && gx < width_ && gy >= 0 && gy < height_;
}

void CostmapCore::markObstacle(int gx, int gy) {
  if (gx < 0 || gx >= width_ || gy < 0 || gy >= height_) {
    return;
  }

  // Two readings can land in the same cell, only inflate around it once.
  if (grid_[gy][gx] != kObstacleCost) {
    grid_[gy][gx] = kObstacleCost;
    obstacle_cells_.emplace_back(gx, gy);
  }
}

void CostmapCore::inflateObstacles(double inflation_radius, int8_t max_cost) {
  if (inflation_radius <= 0.0) {
    return;
  }

  const int radius_cells = static_cast<int>(std::ceil(inflation_radius / resolution_));

  // Walk the obstacle cells recorded this scan. Inflated cells are never added
  // to that list, so cost cannot cascade outward past the inflation radius.
  for (const auto & obstacle : obstacle_cells_) {
    const int ox = obstacle.first;
    const int oy = obstacle.second;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int x = ox + dx;
        const int y = oy + dy;
        if (x < 0 || x >= width_ || y < 0 || y >= height_) {
          continue;
        }

        const double distance = std::hypot(dx, dy) * resolution_;
        if (distance > inflation_radius) {
          continue;
        }

        const auto cost = static_cast<int8_t>(max_cost * (1.0 - distance / inflation_radius));

        // Cells near two obstacles keep the higher cost.
        if (cost > grid_[y][x]) {
          grid_[y][x] = cost;
        }
      }
    }
  }
}

nav_msgs::msg::OccupancyGrid CostmapCore::toOccupancyGrid(const std_msgs::msg::Header& header) const {
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = header;

  grid.info.resolution = resolution_;
  grid.info.width = width_;
  grid.info.height = height_;
  grid.info.origin.position.x = origin_x_;
  grid.info.origin.position.y = origin_y_;
  grid.info.origin.orientation.w = 1.0;

  // OccupancyGrid data is a single row-major array starting at the origin corner.
  grid.data.reserve(static_cast<size_t>(width_) * height_);
  for (const auto & row : grid_) {
    grid.data.insert(grid.data.end(), row.begin(), row.end());
  }

  return grid;
}

}
