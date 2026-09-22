#include <memory>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
  // Step 2: 30 m x 30 m of coverage at 0.1 m/cell, every cell free to start.
  costmap_.initCostmap(0.1, 300, 300, 0);

  // Step 1: laser scans in, step 6: finished grid out.
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", 10, std::bind(&CostmapNode::lidarCallback, this, std::placeholders::_1));
  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
  // Each scan is turned into a fresh costmap, so drop the previous scan's cells.
  costmap_.resetCostmap();

  // TODO step 3: convert each (range, angle) reading to Cartesian, then to grid indices.
  // TODO step 4: mark the cells those readings land in at cost 100.
  // TODO step 5: inflate each obstacle out to the inflation radius using
  //              cost = max_cost * (1 - distance / inflation_radius).

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
