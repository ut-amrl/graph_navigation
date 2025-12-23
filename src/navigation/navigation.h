//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <deque>
#include <memory>
#include <vector>
#include <mutex>
#include <unordered_set>
#include <set>
#include <ctime>
#include <cstdio>
#include <fstream>

#include "eigen3/Eigen/Dense"

#include "config_reader/config_reader.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "navigation_parameters.h"
#include "motion_primitives.h"
#include "gflags/gflags.h"

#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "visualization/visualization.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"

// Declare gflags in global namespace to avoid namespace-mismatch at link time.
DECLARE_string(debug_file);

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace navigation {

inline std::string GetMapPath(const std::string& dir, const std::string& name) {
    return dir + "/" + name + "/" + name + ".navigation.json";
}

static inline double overlap(double a0, double a1, double b0, double b1) {
    const double lo = std::max(a0, b0);
    const double hi = std::min(a1, b1);
    return std::max(0.0, hi - lo);
}

static inline float YawFromQuat(float x, float y, float z, float w) {
    const float siny_cosp = 2.f * (w * z + x * y);
    const float cosy_cosp = 1.f - 2.f * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Apply command mapping using linear models (matches driver's velocityToCounts behavior)
void ApplyCommandMapping(const NavigationParameters& params, Eigen::Vector2f& vel_cmd, float& ang_vel_cmd);

struct PathOption {
    float curvature;
    float clearance;
    float free_path_length;
    float clearance_to_goal;
    float dist_to_goal;
    explicit PathOption(float c) : curvature(c) {}
    PathOption() {}
    Eigen::Vector2f obstruction;
    Eigen::Vector2f closest_point;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Twist {
    double cmd_exec_start_time;
    double cmd_plan_start_time;
    Eigen::Vector3f linear;
    Eigen::Vector3f angular;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Odom {
    double time;
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

enum class NavigationState { kStopped = 0, kGoto = 1, kTurnInPlace = 2 };

class Navigation {
   public:
    explicit Navigation();
    // Update navigation map from file.
    void UpdateMap(const std::string& map_file);
    // Update robot location in map frame.
    void UpdateLocation(const Eigen::Vector2f& loc, float angle);
    // Update odometry based location (odometry frame).
    void UpdateOdometry(const Odom& msg);
    // Add command to history for latency compensation.
    void UpdateCommandHistory(Twist twist);
    // Update laser scan data for obstacle detection (robot frame).
    void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud, double time);
    // Main navigation control loop
    bool Run(const double& time, Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Set navigation goal (map frame).
    void SetNavGoal(const Eigen::Vector2f& loc, float angle);
    // Reset navigation goals by setting goal to current robot position (map frame).
    void ResetNavGoals();
    // Check if current global plan is still valid.
    bool PlanStillValid();
    // Plan global path between two points using A* on navigation graph (map frame).
    std::vector<GraphDomain::State> Plan(const Eigen::Vector2f& initial, const Eigen::Vector2f& end);
    // Get carrot point for local navigation from global path (map frame).
    bool GetCarrot(Eigen::Vector2f& carrot, float carrot_dist = -1.0f);
    // Initialize navigation system with parameters and map.
    void Initialize(const NavigationParameters& params, const std::string& map_file);
    // Point cloud in robot frame from last laser scan observed, forward predicted for latency compensation.
    std::vector<Eigen::Vector2f> fp_point_cloud_;
    // Global path plan computed by A* planner on the navigation graph (map frame).
    std::vector<GraphDomain::State> plan_path_;
    // Navigation parameters.
    NavigationParameters params_;
    // Current robot location in map frame (from localization).
    Eigen::Vector2f robot_loc_;
    // Current robot orientation in map frame (from localization system).
    float robot_angle_;
    // Forward-predicted robot location in map frame at actuation time.
    Eigen::Vector2f robot_loc_fp_;
    // Forward-predicted robot yaw in map frame at actuation time.
    float robot_angle_fp_;
    // Global carrot transformed to robot's reference frame for local navigation (robot frame).
    Eigen::Vector2f local_target_;
    // Last set of sampled path options from local planner (robot frame).
    std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> sampled_paths_;
    // Best path option selected by evaluator from last sampling iteration (robot frame).
    std::shared_ptr<motion_primitives::PathRolloutBase> best_option_;
    // Current navigation state.
    NavigationState nav_state_;
    // Sub-state within kGoto: distinguishes between turning toward carrot (false) vs. actively
    // driving with obstacle avoidance (true). Used for hysteresis in FOV check to prevent oscillation.
    bool in_obstacle_avoidance_mode_;
    // Yaw alignment setpoint in map frame (for visualization).
    float yaw_align_sp_map_ = 0.0f;
    // Whether yaw alignment setpoint has been initialized (for visualization).
    bool yaw_align_sp_init_ = false;
    // Final navigation goal location in map frame.
    Eigen::Vector2f nav_goal_loc_;
    // Final navigation goal orientation in map frame.
    float nav_goal_angle_;
    // Penultimate-to-forward-predicted-time robot linear velocity command
    Eigen::Vector2f robot_vel_;
    // Penultimate-to-forward-predicted-time robot angular velocity command
    float robot_omega_;

   private:
    // Run local obstacle avoidance planner (robot frame).
    void RunObstacleAvoidance(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Remove commands older than latest real robot updates (odometry and LIDAR), for latency compensation.
    void PruneLatencyQueue();
    // Perform latency compensation by forward-predicting the state and observations.
    void ForwardPredict(double t);
    // Come to a halt.
    void Halt(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Turn around in-place to face local target.
    void TurnInPlace(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Draw the robot's outline for visualization (robot frame).
    void DrawRobot();
    // Publish a status message.
    void PublishNavStatus(const Eigen::Vector2f& carrot);
    // Forward-predicted odometry location (odometry frame)
    Eigen::Vector2f odom_loc_;
    // Forward-predicted odometry orientation (odometry frame)
    float odom_angle_;
    // Latest odometry message received (odometry frame).
    Odom latest_odom_msg_;
    // Indicates whether an odometry message has been received.
    bool odom_initialized_;
    // Indicates whether localization system has been initialized.
    bool loc_initialized_;
    // Odometry-reported starting location.
    Eigen::Vector2f starting_loc_;
    // Raw point cloud from latest laser scan observed (robot frame).
    std::vector<Eigen::Vector2f> point_cloud_;
    // Time stamp of observation of latest point cloud.
    double t_point_cloud_;
    // Time stamp of latest odometry message.
    double t_odometry_;
    // Directory containing navigation maps (global).
    const std::string maps_dir_;
    // Planning domain for A* global path planner (map frame).
    GraphDomain planning_domain_;
    // History of commands sent, to perform latency compensation.
    std::deque<Twist> command_history_;
    // Whether or not things have been initialized.
    bool initialized_;
    // Motion primitive sampler for local planning (generates path options).
    std::unique_ptr<motion_primitives::PathRolloutSamplerBase> sampler_;
    // Motion primitive evaluator for local planning (selects best path).
    std::unique_ptr<motion_primitives::PathEvaluatorBase> evaluator_;
};

namespace navigation_debug {
inline void DebugLog(const std::string& line) {
    if (::FLAGS_debug_file.empty()) return;
    // Print to console
    printf("%s\n", line.c_str());
    // Log to file
    static std::mutex mtx;
    std::lock_guard<std::mutex> lock(mtx);
    std::ofstream ofs(::FLAGS_debug_file, std::ios::out | std::ios::app);
    if (!ofs.good()) return;
    ofs << line << '\n';
}
}  // namespace navigation_debug

}  // namespace navigation

#endif  // NAVIGATION_H
