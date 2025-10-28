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

#include "eigen3/Eigen/Dense"

#include "config_reader/config_reader.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "navigation_parameters.h"
#include "motion_primitives.h"

#include "amrl_msgs/msg/localization2_d_msg.hpp"
#include "amrl_msgs/msg/visualization_msg.hpp"
#include "visualization/visualization.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "amrl_msgs/msg/ackermann_curvature_drive_msg.hpp"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace navigation {

inline std::string GetMapPath(const std::string& dir, const std::string& name) {
    return dir + "/" + name + "/" + name + ".navigation.json";
}

inline std::string GetDeprecatedMapPath(const std::string& dir, const std::string& name) {
    return dir + "/" + name + "/" + name + ".navigation.txt";
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
    // Get free path length in straight line direction.
    void GetStraightFreePathLength(float* free_path_length, float* clearance);
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
    // Set path evaluator clearance weight.
    void SetEvaluatorClearanceWeight(const float weight);

    // Point cloud in robot frame from last laser scan observed, forward predicted for latency compensation.
    std::vector<Eigen::Vector2f> fp_point_cloud_;
    // Global path plan computed by A* planner on the navigation graph (map frame).
    std::vector<GraphDomain::State> plan_path_;
    // Navigation parameters.
    NavigationParameters params_;
    // Global carrot transformed to robot's reference frame for local navigation.
    Eigen::Vector2f local_target_;
    // Last set of sampled path options from local planner (robot frame).
    std::vector<std::shared_ptr<motion_primitives::PathRolloutBase>> sampled_paths_;
    // Best path option selected by evaluator from last sampling iteration (robot frame).
    std::shared_ptr<motion_primitives::PathRolloutBase> best_option_;
    // Current navigation state.
    NavigationState nav_state_;

   private:
    // Test 1D time-optimal control motion in a straight line.
    void TrapezoidTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Test driving straight up to the next obstacle.
    void ObstacleTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Test local obstacle avoidance planner.
    void ObstAvTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Test global path planner.
    void PlannerTest();
    // Latency testing routine.
    void LatencyTest(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);
    // Run local obstacle avoidance planner (robot frame).
    void RunObstacleAvoidance(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);

    // Remove commands older than latest real robot updates (odometry and LIDAR),
    // accounting for latency (global).
    // SUGGESTED RENAME: RemoveOldCommandsFromLatencyHistory()
    void PruneLatencyQueue();

    // Perform latency compensation by forward-predicting the commands within the latency
    // interval (global).
    // SUGGESTED RENAME: ForwardPredictRobotState()
    void ForwardPredict(double t);

    // Run 1D time-optimal control (global).
    // SUGGESTED RENAME: Run1DTimeOptimalControl()
    float Run1DTOC(float x_now, float x_target, float v_now, float max_speed, float a_max, float d_max, float dt) const;

    // Come to a halt (global).
    // SUGGESTED RENAME: HaltRobot()
    void Halt(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);

    // Turn around in-place to face the next waypoint (robot frame).
    // SUGGESTED RENAME: TurnInPlaceToFaceTarget()
    void TurnInPlace(Eigen::Vector2f& cmd_vel, float& cmd_angle_vel);

    // Draw the robot's outline for visualization (robot frame).
    // SUGGESTED RENAME: DrawRobotOutline()
    void DrawRobot();

    // Publish a status message (global).
    // SUGGESTED RENAME: PublishNavigationStatus()
    void PublishNavStatus(const Eigen::Vector2f& carrot);

    // Current robot location in map frame (from localization system).
    // USAGE: Used for global planning, carrot computation, and state updates
    // SUGGESTED RENAME: current_robot_location_map_frame_
    Eigen::Vector2f robot_loc_;

    // Current robot orientation in map frame (from localization system).
    // USAGE: Used for coordinate transformations and navigation state
    // SUGGESTED RENAME: current_robot_orientation_map_frame_
    float robot_angle_;

    // Current robot linear velocity in map frame (forward predicted).
    // USAGE: Used by motion primitive sampler and control generation
    // SUGGESTED RENAME: current_robot_velocity_map_frame_
    Eigen::Vector2f robot_vel_;

    // Current robot angular velocity (forward predicted).
    // USAGE: Used by motion primitive sampler and control generation
    // SUGGESTED RENAME: current_robot_angular_velocity_
    float robot_omega_;

    // Current robot location in odometry frame (from wheel encoders).
    // USAGE: Used for latency compensation and forward prediction
    // SUGGESTED RENAME: current_robot_location_odom_frame_
    Eigen::Vector2f odom_loc_;

    // Current robot orientation in odometry frame (from wheel encoders).
    // USAGE: Used for latency compensation and forward prediction
    // SUGGESTED RENAME: current_robot_orientation_odom_frame_
    float odom_angle_;

    // Newest odometry message received (odometry frame).
    // USAGE: Used for forward prediction and latency compensation
    // SUGGESTED RENAME: latest_odometry_message_
    Odom latest_odom_msg_;

    // Final navigation goal location in map frame.
    // USAGE: Target location for global planning
    // SUGGESTED RENAME: final_goal_location_map_frame_
    Eigen::Vector2f nav_goal_loc_;

    // Final navigation goal orientation in map frame.
    // USAGE: Target orientation for final approach
    // SUGGESTED RENAME: final_goal_orientation_map_frame_
    float nav_goal_angle_;

    // Indicates whether an odometry message has been received (global).
    // USAGE: Tracks if odometry system is ready
    // SUGGESTED RENAME: odometry_initialized_
    bool odom_initialized_;

    // Indicates whether localization system has been initialized (global).
    // USAGE: Tracks if localization system is ready
    // SUGGESTED RENAME: localization_initialized_
    bool loc_initialized_;

    // Odometry-reported starting location (for testing).
    // USAGE: Used in trapezoid test for distance calculation
    // SUGGESTED RENAME: odom_starting_location_odom_frame_
    Eigen::Vector2f starting_loc_;

    // Raw point cloud from last laser scan observed (robot frame).
    // USAGE: Source data for forward prediction, not used directly by planner
    // SUGGESTED RENAME: raw_laser_cloud_robot_frame_
    std::vector<Eigen::Vector2f> point_cloud_;

    // Time stamp of observation of point cloud (global).
    // USAGE: Used for latency compensation and data synchronization
    // SUGGESTED RENAME: laser_scan_timestamp_
    double t_point_cloud_;

    // Time stamp of latest odometry message (global).
    // USAGE: Used for latency compensation and data synchronization
    // SUGGESTED RENAME: odometry_timestamp_
    double t_odometry_;

    // Directory containing navigation maps (global).
    // USAGE: Path to map files for loading navigation graphs
    // SUGGESTED RENAME: navigation_maps_directory_
    const std::string maps_dir_;

    // Planning domain for A* global path planner (map frame).
    // USAGE: Navigation graph used for global path planning
    // SUGGESTED RENAME: global_path_planning_domain_
    GraphDomain planning_domain_;

    // History of commands sent, to perform latency compensation.
    // USAGE: Used for forward prediction and latency compensation
    // SUGGESTED RENAME: sent_command_history_for_latency_compensation_
    std::deque<Twist> command_history_;

    // Whether to enable autonomous navigation or not (global).
    // USAGE: Controls whether navigation system is active
    // SUGGESTED RENAME: navigation_enabled_
    bool enabled_;

    // Whether or not things have been initialized (global).
    // USAGE: Indicates if navigation system is ready to operate
    // SUGGESTED RENAME: navigation_initialized_
    bool initialized_;

    // Motion primitive sampler for local planning (generates path options).
    // USAGE: Samples different path options in robot frame for obstacle avoidance
    // SUGGESTED RENAME: local_path_sampler_
    std::unique_ptr<motion_primitives::PathRolloutSamplerBase> sampler_;

    // Motion primitive evaluator for local planning (selects best path).
    // USAGE: Evaluates and selects the best path option from sampled options
    // SUGGESTED RENAME: local_path_evaluator_
    std::unique_ptr<motion_primitives::PathEvaluatorBase> evaluator_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
