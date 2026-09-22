#include <chrono>
#include <cmath>
#include <memory>

#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode()
  : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())),
    costmap_received_(false), robot_x_(0.0), robot_y_(0.0), robot_yaw_(0.0),
    last_update_x_(0.0), last_update_y_(0.0), has_integrated_(false) {
  // The arena walls are 30 m long, so 40 m of map leaves room around them.
  map_memory_.initMap("sim_world", 0.1, 400, 400);

  update_distance_ = this->declare_parameter<double>("update_distance", 1.5);

  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  timer_ = this->create_wall_timer(
    std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // Keep only the newest one, the timer decides when it gets used.
  latest_costmap_ = *msg;
  costmap_received_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_x_ = msg->pose.pose.position.x;
  robot_y_ = msg->pose.pose.position.y;

  // Yaw out of the quaternion. Roll and pitch are irrelevant for a ground robot.
  const auto & q = msg->pose.pose.orientation;
  robot_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

void MapMemoryNode::updateMap() {
  if (!costmap_received_) {
    return;
  }

  const double distance = std::hypot(robot_x_ - last_update_x_, robot_y_ - last_update_y_);

  // Fuse the first costmap immediately, then only once the robot has actually
  // travelled far enough to have seen something new.
  if (!has_integrated_ || distance >= update_distance_) {
    map_memory_.integrateCostmap(latest_costmap_, robot_x_, robot_y_, robot_yaw_);

    last_update_x_ = robot_x_;
    last_update_y_ = robot_y_;
    has_integrated_ = true;

    RCLCPP_INFO(this->get_logger(), "Fused costmap at (%.2f, %.2f), moved %.2f m", 
                robot_x_, robot_y_, distance);
  }

  // Republish every tick so the panel keeps a map even while the robot is parked.
  auto map = map_memory_.map();
  map.header.stamp = this->now();
  map_pub_->publish(map);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
