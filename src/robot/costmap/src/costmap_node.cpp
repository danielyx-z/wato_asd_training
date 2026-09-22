#include <cmath>
#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Step 2: 30 m x 30 m of coverage at 0.1 m/cell, every cell free to start.
  costmap_.initCostmap(0.1, 300, 300, 0);

  // Safety buffer grown around each obstacle, in metres.
  inflation_radius_ = this->declare_parameter<double>("inflation_radius", 1.0);

  // Step 1: laser scans in, step 6: finished grid out.
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Each scan is turned into a fresh costmap, so drop the previous scan's cells.
  costmap_.resetCostmap();

  // Step 3: every reading from polar (range, angle) to Cartesian, then to grid indices.
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    const double range = scan->ranges[i];

    // Infinities mean the beam hit nothing, and readings outside the sensor's
    // stated limits are not trustworthy.
    if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
      continue;
    }

    const double angle = scan->angle_min + i * scan->angle_increment;

    int gx = 0;
    int gy = 0;
    if (costmap_.worldToGrid(range * std::cos(angle), range * std::sin(angle), gx, gy)) {
      // Step 4: the beam terminated here, so this cell is occupied.
      costmap_.markObstacle(gx, gy);
    }
  }

  // Step 5: grow a buffer around each obstacle so the planner keeps its distance.
  costmap_.inflateObstacles(inflation_radius_, robot::CostmapCore::kObstacleCost);

  // Step 6: publish in the scan's own frame so /tf places it in sim_world for us.
  costmap_pub_->publish(costmap_.toOccupancyGrid(scan->header));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
