#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

class PurePursuitController : public rclcpp::Node {
public:
    PurePursuitController() : Node("pure_pursuit_controller") {
        // Control Parameters
        lookahead_distance_ = 0.8;  // Lookahead distance L (meters)
        goal_tolerance_ = 0.25;     // Stop distance threshold from final waypoint (meters)
        linear_speed_ = 1;       // Forward linear velocity (m/s)
        max_angular_vel_ = 1.5;     // Maximum allowed rotation speed (rad/s)
        // Beyond this bearing to the lookahead point the pure pursuit curvature is
        // useless, so turn on the spot until the target is back in front. 1.0 rad
        // (~57 deg) is wide enough that normal tracking never trips it.
        heading_tolerance_ = 1.0;   // Bearing that forces a pivot (rad)
        pivot_gain_ = 2.0;          // Proportional gain used while pivoting

        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Control Timer running at 10 Hz
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });

        RCLCPP_INFO(this->get_logger(), "PurePursuitController initialized.");
    }

private:
    void controlLoop() {
        // 1. Guard check for missing data
        if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
            return;
        }

        const auto &current_pose = robot_odom_->pose.pose;
        const auto &final_goal = current_path_->poses.back().pose.position;

        // 2. Goal reach check
        double dist_to_final_goal = computeDistance(current_pose.position, final_goal);
        if (dist_to_final_goal < goal_tolerance_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached! Stopping robot.");
            stopRobot();
            return;
        }

        // 3. Find lookahead target point
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No valid lookahead point found on current path!");
            stopRobot();
            return;
        }

        // 4. Compute and publish command velocities
        auto cmd_vel = computeVelocity(*lookahead_point);
        cmd_vel_pub_->publish(cmd_vel);
    }

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {
        const auto &robot_pos = robot_odom_->pose.pose.position;
        std::optional<geometry_msgs::msg::PoseStamped> target;

        // Search backward from the end of the path to find the first waypoint 
        // at or beyond the lookahead distance L.
        for (const auto &pose_stamped : current_path_->poses) {
            double dist = computeDistance(robot_pos, pose_stamped.pose.position);
            if (dist >= lookahead_distance_) {
                target = pose_stamped;
                break;
            }
        }

        // Fallback: If all remaining waypoints are within lookahead distance,
        // target the final waypoint on the path.
        if (!target && !current_path_->poses.empty()) {
            target = current_path_->poses.back();
        }

        return target;
    }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        geometry_msgs::msg::Twist cmd_vel;

        const auto &robot_pose = robot_odom_->pose.pose;
        double robot_x = robot_pose.position.x;
        double robot_y = robot_pose.position.y;
        double yaw = extractYaw(robot_pose.orientation);

        double target_x = target.pose.position.x;
        double target_y = target.pose.position.y;

        // Transform target coordinates to the robot's local frame:
        // dx, dy in world frame
        double dx = target_x - robot_x;
        double dy = target_y - robot_y;

        // Rotate into local frame (x_local: forward, y_local: left)
        double local_x =  std::cos(yaw) * dx + std::sin(yaw) * dy;
        double local_y = -std::sin(yaw) * dx + std::cos(yaw) * dy;

        // Actual distance L to target
        double Ld = computeDistance(robot_pose.position, target.pose.position);

        if (Ld < 1e-4) {
            return cmd_vel;
        }

        // Bearing to the target, measured from straight ahead.
        double alpha = std::atan2(local_y, local_x);

        // Pure pursuit steers by y_local alone, which collapses for a target that
        // is directly behind: y_local goes to zero there just as it does straight
        // ahead, so the controller commands full speed and no turn and drives away
        // from the goal. A differential drive can spin on the spot, so do that
        // until the target is in front and the curvature law is meaningful again.
        if (std::abs(alpha) > heading_tolerance_) {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = std::clamp(pivot_gain_ * alpha, -max_angular_vel_, max_angular_vel_);
            return cmd_vel;
        }

        // Pure Pursuit Curvature Equation: gamma = 2 * y_local / (Ld^2)
        double curvature = (2.0 * local_y) / (Ld * Ld);

        // Compute linear and angular velocities
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = cmd_vel.linear.x * curvature;

        // Clamp angular velocity within safe limits
        cmd_vel.angular.z = std::clamp(cmd_vel.angular.z, -max_angular_vel_, max_angular_vel_);

        return cmd_vel;
    }

    void stopRobot() {
        geometry_msgs::msg::Twist zero_vel;
        cmd_vel_pub_->publish(zero_vel);
    }

    double computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    double extractYaw(const geometry_msgs::msg::Quaternion &q) {
        // Standard Quaternion to Euler Yaw conversion (Z-axis rotation)
        double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    // ROS Handles
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Data Storage
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // Parameters
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double max_angular_vel_;
    double heading_tolerance_;
    double pivot_gain_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PurePursuitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}