#include <algorithm>
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

        // Cells at or above this cost are only crossed as a last resort. With the
        // costmap's 2 m inflation radius, 60 keeps the path 0.8 m clear of anything
        // solid, which a robot with a 0.5 m half-width needs to turn without clipping.
        lethal_cost_ = this->declare_parameter<int>("lethal_cost", 60);
        // How hard to steer away from the inflation gradient out in open space.
        cost_weight_ = this->declare_parameter<double>("cost_weight", 10.0);
        // Extra multiplier applied on top of that inside the lethal band.
        lethal_penalty_ = this->declare_parameter<double>("lethal_penalty", 50.0);
        // Plan optimistically through unexplored ground, else distant goals fail.
        allow_unknown_ = this->declare_parameter<bool>("allow_unknown", true);
        unknown_penalty_ = this->declare_parameter<double>("unknown_penalty", 2.0);

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

    // Planner tuning
    int lethal_cost_ = 60;
    double cost_weight_ = 10.0;
    double lethal_penalty_ = 50.0;
    bool allow_unknown_ = true;
    double unknown_penalty_ = 2.0;

    // A cell holding an actual laser return, never traversable
    static constexpr int8_t kOccupied = 100;

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
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                     "Periodic replan execution...");
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

    bool inBounds(const CellIndex &idx) const {
        return idx.x >= 0 && idx.x < static_cast<int>(current_map_.info.width) &&
               idx.y >= 0 && idx.y < static_cast<int>(current_map_.info.height);
    }

    // Raw occupancy cost, or -1 for unknown / out of bounds.
    int8_t cellCost(const CellIndex &idx) const {
        if (!inBounds(idx)) return -1;
        return current_map_.data[idx.y * current_map_.info.width + idx.x];
    }

    bool isValidCell(const CellIndex &idx) const {
        if (!inBounds(idx)) return false;

        const int8_t cost = cellCost(idx);
        // Unexplored ground is assumed drivable, otherwise no goal outside the
        // area already mapped could ever be reached. The robot replans twice a
        // second, so anything really there is found and routed around en route.
        if (cost < 0) return allow_unknown_;

        // Anything the lidar actually returned from is solid.
        return cost < kOccupied;
    }

    // Multiplier on the distance cost of stepping into a cell. Hugging a wall is
    // legal but expensive, so A* only does it when there is genuinely no room --
    // which is also what lets the robot escape if it starts inside the inflation.
    double cellPenalty(const CellIndex &idx) const {
        const int8_t cost = cellCost(idx);
        // Mildly prefer ground already seen over guessing about unexplored ground.
        if (cost < 0) return unknown_penalty_;
        if (cost == 0) return 1.0;

        double penalty = 1.0 + cost_weight_ * (static_cast<double>(cost) / 100.0);
        if (cost >= lethal_cost_) penalty *= lethal_penalty_;
        return penalty;
    }

    double heuristic(const CellIndex &a, const CellIndex &b) const {
        // Octile distance: the exact cost of an unobstructed 8-connected walk, so
        // it dominates the Euclidean estimate and expands far fewer cells.
        const double dx = std::abs(a.x - b.x);
        const double dy = std::abs(a.y - b.y);
        return (dx + dy) - (2.0 - 1.41421356237) * std::min(dx, dy);
    }

    // ------------------- A* Algorithm Implementation -------------------

    void planPath() {
        if (!goal_received_ || !odom_received_ || current_map_.data.empty()) {
            RCLCPP_WARN(this->get_logger(), "Cannot plan path: Missing map, goal, or odometry!");
            return;
        }

        const auto plan_start = std::chrono::steady_clock::now();

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

        // The map is a fixed grid, so index scores by cell rather than hashing.
        // The old unordered_map hash collided heavily (x ^ (y << 1) maps (2,1) and
        // (0,0) to the same bucket), which dominated the search time.
        const int width = static_cast<int>(current_map_.info.width);
        const int height = static_cast<int>(current_map_.info.height);
        const size_t cell_count = static_cast<size_t>(width) * height;
        const auto flat = [width](const CellIndex &c) {
            return static_cast<size_t>(c.y) * width + c.x;
        };

        constexpr double kInf = std::numeric_limits<double>::infinity();
        std::vector<double> g_score(cell_count, kInf);
        std::vector<int> came_from(cell_count, -1);
        std::vector<char> closed(cell_count, 0);

        std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open_set;

        g_score[flat(start_idx)] = 0.0;
        open_set.push(AStarNode(start_idx, heuristic(start_idx, goal_idx)));

        bool path_found = false;
        size_t expanded = 0;

        // 8-connected grid direction offsets (dx, dy, step_cost)
        const std::vector<std::tuple<int, int, double>> neighbors = {
            { 1,  0, 1.0}, {-1,  0, 1.0}, { 0,  1, 1.0}, { 0, -1, 1.0},
            { 1,  1, 1.41421356237}, { 1, -1, 1.41421356237},
            {-1,  1, 1.41421356237}, {-1, -1, 1.41421356237}
        };

        while (!open_set.empty()) {
            CellIndex current = open_set.top().index;
            open_set.pop();

            const size_t current_flat = flat(current);

            // Stale duplicate left in the queue by an earlier, worse relaxation.
            if (closed[current_flat]) continue;
            closed[current_flat] = 1;
            ++expanded;

            if (current == goal_idx) {
                path_found = true;
                break;
            }

            for (const auto &[dx, dy, step_cost] : neighbors) {
                CellIndex neighbor(current.x + dx, current.y + dy);

                if (!isValidCell(neighbor)) continue;

                const size_t neighbor_flat = flat(neighbor);
                if (closed[neighbor_flat]) continue;

                // Distance travelled, scaled by how close to an obstacle it runs.
                const double tentative_g =
                    g_score[current_flat] + step_cost * cellPenalty(neighbor);

                if (tentative_g < g_score[neighbor_flat]) {
                    came_from[neighbor_flat] = static_cast<int>(current_flat);
                    g_score[neighbor_flat] = tentative_g;
                    open_set.push(AStarNode(neighbor, tentative_g + heuristic(neighbor, goal_idx)));
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
        for (int curr = static_cast<int>(flat(goal_idx)); curr != -1; curr = came_from[curr]) {
            cell_path.push_back(CellIndex(curr % width, curr / width));
            if (curr == static_cast<int>(flat(start_idx))) break;
        }
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

        const double plan_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - plan_start).count();
        RCLCPP_INFO(this->get_logger(),
                    "Path %i pts in %.1f ms (%zu expanded): robot(%.2f,%.2f) -> goal(%.2f,%.2f); path ends (%.2f,%.2f)",
                    static_cast<int>(path.poses.size()), plan_ms, expanded,
                    robot_pose_.position.x, robot_pose_.position.y,
                    goal_.point.x, goal_.point.y,
                    path.poses.back().pose.position.x, path.poses.back().pose.position.y);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}