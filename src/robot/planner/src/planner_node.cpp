#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

// ------------------- Supporting Structures -------------------

// 2D grid index
struct CellIndex {
    int x;
    int y;

    CellIndex(int xx, int yy) : x(xx), y(yy) {}
    CellIndex() : x(0), y(0) {}

    bool operator==(const CellIndex &other) const {
        return (x == other.x && y == other.y);
    }

    bool operator!=(const CellIndex &other) const {
        return (x != other.x || y != other.y);
    }
};

// Hash function for CellIndex so it can be used in std::unordered_map
struct CellIndexHash {
    std::size_t operator()(const CellIndex &idx) const {
        return std::hash<int>()(idx.x) ^ (std::hash<int>()(idx.y) << 1);
    }
};

// Structure representing a node in the A* open set
struct AStarNode {
    CellIndex index;
    double f_score;  // f = g + h

    AStarNode(CellIndex idx, double f) : index(idx), f_score(f) {}
};

// Comparator for the priority queue (min-heap by f_score)
struct CompareF {
    bool operator()(const AStarNode &a, const AStarNode &b) {
        return a.f_score > b.f_score;
    }
};

// ------------------- Planner Node -------------------

class PlannerNode : public rclcpp::Node {
public:
    PlannerNode() : Node("planner_node"), state_(State::WAITING_FOR_GOAL) {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

        // Publisher
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

        // Timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this));

        RCLCPP_INFO(this->get_logger(), "PlannerNode initialized. Waiting for goal...");
    }

private:
    enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
    State state_;

    // Subscribers and Publisher
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Data Storage
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PointStamped goal_;
    geometry_msgs::msg::Pose robot_pose_;

    bool goal_received_ = false;
    bool odom_received_ = false;

    // ------------------- Callbacks -------------------

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = *msg;
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            planPath();
        }
    }

    void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        goal_ = *msg;
        goal_received_ = true;
        state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
        RCLCPP_INFO(this->get_logger(), "New goal received: (%.2f, %.2f)", goal_.point.x, goal_.point.y);
        planPath();
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pose_ = msg->pose.pose;
        odom_received_ = true;
    }

    void timerCallback() {
        if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
            if (goalReached()) {
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
                state_ = State::WAITING_FOR_GOAL;
            } else {
                RCLCPP_INFO(this->get_logger(), "Periodic replan execution...");
                planPath();
            }
        }
    }

    // ------------------- Helper Functions -------------------

    bool goalReached() {
        if (!odom_received_ || !goal_received_) return false;
        double dx = goal_.point.x - robot_pose_.position.x;
        double dy = goal_.point.y - robot_pose_.position.y;
        return std::sqrt(dx * dx + dy * dy) < 0.5; // Threshold: 0.5 meters
    }

    CellIndex worldToGrid(double wx, double wy) const {
        double origin_x = current_map_.info.origin.position.x;
        double origin_y = current_map_.info.origin.position.y;
        double resolution = current_map_.info.resolution;

        int gx = static_cast<int>(std::floor((wx - origin_x) / resolution));
        int gy = static_cast<int>(std::floor((wy - origin_y) / resolution));
        return CellIndex(gx, gy);
    }

    void gridToWorld(const CellIndex &idx, double &wx, double &wy) const {
        double origin_x = current_map_.info.origin.position.x;
        double origin_y = current_map_.info.origin.position.y;
        double resolution = current_map_.info.resolution;

        // Cell center coordinates
        wx = origin_x + (idx.x + 0.5) * resolution;
        wy = origin_y + (idx.y + 0.5) * resolution;
    }

    bool isValidCell(const CellIndex &idx) const {
        if (idx.x < 0 || idx.x >= static_cast<int>(current_map_.info.width) ||
            idx.y < 0 || idx.y >= static_cast<int>(current_map_.info.height)) {
            return false;
        }

        int flat_index = idx.y * current_map_.info.width + idx.x;
        int8_t cost = current_map_.data[flat_index];

        // Treat unknown (-1) or occupied (> 50) cells as impassable
        return cost >= 0 && cost < 50;
    }

    double heuristic(const CellIndex &a, const CellIndex &b) const {
        // Euclidean distance heuristic
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    // ------------------- A* Algorithm Implementation -------------------

    void planPath() {
        if (!goal_received_ || !odom_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map, goal, or odometry!");
            return;
        }

        CellIndex start_idx = worldToGrid(robot_pose_.position.x, robot_pose_.position.y);
        CellIndex goal_idx = worldToGrid(goal_.point.x, goal_.point.y);

        if (!isValidCell(start_idx)) {
            RCLCPP_ERROR(this->get_logger(), "Start position is invalid or inside an obstacle!");
            return;
        }
        if (!isValidCell(goal_idx)) {
            RCLCPP_ERROR(this->get_logger(), "Goal position is invalid or inside an obstacle!");
            return;
        }

        // Priority Queue (Open Set)
        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

        // Tracking scores and parents
        std::unordered_map<CellIndex, double, CellIndexHash> g_score;
        std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;

        g_score[start_idx] = 0.0;
        open_set.push(AStarNode(start_idx, heuristic(start_idx, goal_idx)));

        bool path_found = false;

        // 8-connected grid direction offsets (dx, dy, step_cost)
        const std::vector<std::tuple<int, int, double>> neighbors = {
            { 1,  0, 1.0}, {-1,  0, 1.0}, { 0,  1, 1.0}, { 0, -1, 1.0},
            { 1,  1, 1.414}, { 1, -1, 1.414}, {-1,  1, 1.414}, {-1, -1, 1.414}
        };

        while (!open_set.empty()) {
            CellIndex current = open_set.top().index;
            open_set.pop();

            if (current == goal_idx) {
                path_found = true;
                break;
            }

            for (const auto &[dx, dy, step_cost] : neighbors) {
                CellIndex neighbor(current.x + dx, current.y + dy);

                if (!isValidCell(neighbor)) continue;

                double tentative_g = g_score[current] + step_cost;

                if (g_score.find(neighbor) == g_score.end() || tentative_g < g_score[neighbor]) {
                    came_from[neighbor] = current;
                    g_score[neighbor] = tentative_g;
                    double f = tentative_g + heuristic(neighbor, goal_idx);
                    open_set.push(AStarNode(neighbor, f));
                }
            }
        }

        if (!path_found) {
            RCLCPP_WARN(this->get_logger(), "A* search completed: No valid path found!");
            return;
        }

        // ------------------- Path Reconstruction -------------------

        nav_msgs::msg::Path path;
        path.header.stamp = this->get_clock()->now();
        path.header.frame_id = current_map_.header.frame_id.empty() ? "map" : current_map_.header.frame_id;

        std::vector<CellIndex> cell_path;
        CellIndex curr = goal_idx;
        while (curr != start_idx) {
            cell_path.push_back(curr);
            curr = came_from[curr];
        }
        cell_path.push_back(start_idx);
        std::reverse(cell_path.begin(), cell_path.end());

        for (const auto &idx : cell_path) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header = path.header;

            double wx, wy;
            gridToWorld(idx, wx, wy);

            pose_stamped.pose.position.x = wx;
            pose_stamped.pose.position.y = wy;
            pose_stamped.pose.position.z = 0.0;

            pose_stamped.pose.orientation.w = 1.0;
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = 0.0;

            path.poses.push_back(pose_stamped);
        }

        path_pub_->publish(path);
        RCLCPP_INFO(this->get_logger(), "Successfully published path with %i waypoints.", static_cast<int>(path.poses.size()));
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}