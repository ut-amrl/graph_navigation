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
\file    navigation.cc
\brief   Implementation for reference Navigation class.
\author  Joydeep Biswas, Jarrett Holtz, Kavan Sikand (C) 2021
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <string>
#include <unordered_map>
#include <chrono>
#include <iostream>
#include <fstream>
#include <queue>
#include <limits>

#include "navigation.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "glog/logging.h"
#include "shared/math/math_util.h"
#include "shared/util/helpers.h"
#include "shared/util/timer.h"
#include "shared/util/timer.h"
#include "eight_connected_domain.h"
#include "graph_domain.h"
#include "astar.h"
#include "simple_queue.h"

#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "omnidirectional_motion_primitives.h"
#include "linear_evaluator.h"
#include "amrl_msgs/msg/nav_status_msg.hpp"
#include "amrl_msgs/msg/pose2_df.hpp"

#include "nlohmann/json.hpp"
using json = nlohmann::json;

using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using navigation::MotionLimits;
using navigation::Odom;
using navigation::Twist;
using std::atan2;
using std::deque;
using std::max;
using std::min;
using std::set;
using std::shared_ptr;
using std::string;
using std::swap;
using std::unordered_map;
using std::unordered_set;
using std::vector;

using namespace math_util;
using namespace motion_primitives;

#include <fcntl.h>
#include <cfloat>
#include <glog/logging.h>

// Utility macro for vector component access in printf statements
#define V2COMP(v) v.x(), v.y()

DEFINE_double(max_plan_deviation, 0.5, "Maximum premissible deviation from the plan");

namespace {
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;

// TODO(jaholtz) figure out how to handle this visualization without
// having astar contain ros dependencies
struct EightGridVisualizer {
    EightGridVisualizer(bool visualize) : kVisualize(visualize) {}

    void DrawEdge(const navigation::EightConnectedDomain::State& s1,
                  const navigation::EightConnectedDomain::State& s2) {
        if (!kVisualize) return;
        static const bool kDebug = false;
        if (kDebug) {
            printf("%7.2f,%7.2f -> %7.2f,%7.2f\n", s1.x(), s1.y(), s2.x(), s2.y());
        }
        // visualization::DrawLine(s1, s2, 0x606060, global_viz_msg_);
        // viz_pub_.publish(global_viz_msg_);
        if (kDebug) Sleep(0.05);
    }

    const bool kVisualize;
};

struct GraphVisualizer {
    GraphVisualizer(bool visualize) : kVisualize(visualize) {}

    void DrawEdge(const navigation::GraphDomain::State& s1, const navigation::GraphDomain::State& s2) {
        if (!kVisualize) return;
        static const bool kDebug = false;
        if (kDebug) {
            printf("%7.2f,%7.2f -> %7.2f,%7.2f\n", s1.loc.x(), s1.loc.y(), s2.loc.x(), s2.loc.y());
        }
        // visualization::DrawLine(s1.loc, s2.loc, 0xC0C0C0, global_viz_msg_);
        // viz_pub_.publish(global_viz_msg_);
        if (kDebug) Sleep(0.05);
    }

    const bool kVisualize;
};

struct PointCost {
    int index;
    double cost;

    // Constructor
    PointCost(int k, double c) : index(k), cost(c) {}
};

struct CompareCost {
    bool operator()(const PointCost& lhs, const PointCost& rhs) const {
        // Using > for max heap (change to < for min heap)
        return lhs.cost > rhs.cost;
    }
};

}  // namespace

namespace navigation {

Navigation::Navigation()
    : robot_loc_(0, 0),
      robot_angle_(0),
      robot_loc_fp_(0, 0),
      robot_angle_fp_(0),
      robot_vel_(0, 0),
      nav_state_(NavigationState::kStopped),
      in_obstacle_avoidance_mode_(false),
      robot_omega_(0),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0),
      odom_initialized_(false),
      loc_initialized_(false),
      t_point_cloud_(std::numeric_limits<double>::quiet_NaN()),
      t_odometry_(std::numeric_limits<double>::quiet_NaN()),
      initialized_(false),
      sampler_(nullptr),
      evaluator_(nullptr) {}

void Navigation::Initialize(const NavigationParameters& params, const string& map_file) {
    // Initialize status message
    params_ = params;
    planning_domain_ = GraphDomain(map_file, &params_);
    initialized_ = true;

    // Select motion primitive sampler based on mode
    PathRolloutSamplerBase* sampler = nullptr;
    if (params_.motion_primitives_mode == "ackermann") {
        sampler = new AckermannSampler();
    } else if (params_.motion_primitives_mode == "omni") {
        sampler = new OmniSampler();
    } else {
        printf("Unknown motion primitives mode %s, defaulting to ackermann\n", params_.motion_primitives_mode.c_str());
        sampler = new AckermannSampler();
    }
    sampler_ = std::unique_ptr<PathRolloutSamplerBase>(sampler);
    sampler_->SetNavParams(params);

    PathEvaluatorBase* evaluator = nullptr;
    if (params_.evaluator_type == "linear") {
        evaluator = (PathEvaluatorBase*)new LinearEvaluator();
    } else {
        printf("Unknown evaluator type %s\n", params_.evaluator_type.c_str());
        exit(1);
    }
    evaluator_ = std::unique_ptr<PathEvaluatorBase>(evaluator);
}

void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    plan_path_.clear();
    nav_state_ = NavigationState::kGoto;
    in_obstacle_avoidance_mode_ = false;
    // Disable angular TOC when setting new goal
    if (params_.motion_primitives_mode == "omni") {
        auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
        omni_sampler->enable_angular_toc_runtime_ = false;
    }
}

void Navigation::ResetNavGoals() {
    nav_state_ = NavigationState::kStopped;
    nav_goal_loc_ = robot_loc_;
    nav_goal_angle_ = robot_angle_;
    local_target_.setZero();
    plan_path_.clear();
    in_obstacle_avoidance_mode_ = false;
    // Disable angular TOC when resetting goals
    if (params_.motion_primitives_mode == "omni") {
        auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
        omni_sampler->enable_angular_toc_runtime_ = false;
    }
}

void Navigation::UpdateMap(const string& map_path) {
    planning_domain_.Load(map_path);
    plan_path_.clear();
    in_obstacle_avoidance_mode_ = false;  // Reset sub-state when plan is cleared
}

void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
    robot_loc_ = loc;
    robot_angle_ = angle;
    loc_initialized_ = true;
}

void Navigation::PruneLatencyQueue() {
    if (command_history_.empty()) return;
    // If one sensor time is uninitialized, use the other.
    const bool has_odom = std::isfinite(t_odometry_);
    const bool has_lidar = std::isfinite(t_point_cloud_);
    if (!has_odom && !has_lidar) return;
    const double update_time =
        has_odom && has_lidar ? std::min(t_point_cloud_, t_odometry_) : (has_odom ? t_odometry_ : t_point_cloud_);
    // Drop any segment whose active window [t_cmd, t_cmd+dt) ends at or before update_time.
    const double dt = params_.dt;
    auto keep = [&](const Twist& c) {
        return (c.cmd_exec_start_time + dt) > update_time;  // strictly overlaps [update_time, ∞)
    };
    command_history_.erase(
        std::remove_if(command_history_.begin(), command_history_.end(), [&](const Twist& c) { return !keep(c); }),
        command_history_.end());
}

void Navigation::UpdateOdometry(const Odom& msg) {
    latest_odom_msg_ = msg;
    t_odometry_ = msg.time;
    if (!odom_initialized_) {
        starting_loc_ = Vector2f(msg.position.x(), msg.position.y());
        odom_initialized_ = true;
    }
}

void Navigation::UpdateCommandHistory(Twist twist) {
    // Keep history sorted by execution start time (robust to rare out-of-order inserts).
    if (!command_history_.empty() && twist.cmd_exec_start_time < command_history_.back().cmd_exec_start_time) {
        auto it = std::upper_bound(command_history_.begin(), command_history_.end(), twist.cmd_exec_start_time,
                                   [](const double t, const Twist& a) { return t < a.cmd_exec_start_time; });
        command_history_.insert(it, twist);
    } else {
        command_history_.push_back(twist);
    }
}

void Navigation::ForwardPredict(double t) {
    const double dt_seg = params_.dt;
    // Predicted velocity just BEFORE time t, which we are computing now
    if (command_history_.empty()) {
        robot_vel_ = Vector2f(0.f, 0.f);
        robot_omega_ = 0.f;
    } else {
        // Find the last command with start time <= t (active on [t-dt, t) if present)
        // Loop iterates from front to back of the queue (earliest to latest commands)
        // Since command_history_ is sorted by cmd_exec_start_time in ascending order,
        // we iterate through commands chronologically to find the last command
        // whose execution start time is <= t (i.e., the command active at time t)
        const Twist* active = nullptr;
        for (const Twist& c : command_history_) {
            if (c.cmd_exec_start_time <= t)
                active = &c;  // Keep updating to find the latest valid command
            else
                break;  // Since sorted, no later commands will have start_time <= t
        }
        if (!active) active = &command_history_.front();
        robot_vel_ = Vector2f(active->linear.x(), active->linear.y());
        robot_omega_ = static_cast<float>(active->angular.z());
    }
    // Set the latest odometry location and angle
    odom_loc_ = Vector2f(latest_odom_msg_.position.x(), latest_odom_msg_.position.y());
    {
        const auto& q = latest_odom_msg_.orientation;
        odom_angle_ = YawFromQuat(q.x(), q.y(), q.z(), q.w());
    }
    // Forward predict the robot's pose and accumulate inverse LiDAR transform
    Affine2f lidar_tf = Affine2f::Identity();
    for (const Twist& c : command_history_) {
        const double seg0 = c.cmd_exec_start_time;
        const double seg1 = c.cmd_exec_start_time + dt_seg;
        if (seg0 >= t) break;  // sorted history => nothing else overlaps [*, t)
        // ---- Odom: integrate forward over [t_odometry_, t)
        {
            const double dto = overlap(seg0, seg1, t_odometry_, t);
            if (dto > 0.0) {
                const float fdto = static_cast<float>(dto);
                const Vector2f v_b(c.linear.x(), c.linear.y());
                odom_loc_ += fdto * (Rotation2Df(odom_angle_) * v_b);
                odom_angle_ = AngleMod(odom_angle_ + fdto * static_cast<float>(c.angular.z()));
            }
        }
        // ---- LiDAR: accumulate inverse motion over [t_point_cloud_, t)
        {
            const double dtl = overlap(seg0, seg1, t_point_cloud_, t);
            if (dtl > 0.0) {
                const float fdtl = static_cast<float>(dtl);
                const float dth = -static_cast<float>(c.angular.z()) * fdtl;  // inverse rotation
                Rotation2Df Rstep(dth);
                const Vector2f v_b(c.linear.x(), c.linear.y());
                const Vector2f tstep = -(Rstep * v_b) * fdtl;  // rotate translation for inverse step
                lidar_tf = Translation2f(tstep) * Rstep * lidar_tf;
            }
        }
    }
    // Transform the cloud from t_point_cloud_ to t
    fp_point_cloud_.resize(point_cloud_.size());
    for (size_t i = 0; i < point_cloud_.size(); ++i) {
        fp_point_cloud_[i] = lidar_tf * point_cloud_[i];
    }

    // Compute predicted base pose in map frame at actuation time
    {
        Affine2f T_map_odom = Affine2f::Identity();
        const float odom_yaw_last = YawFromQuat(latest_odom_msg_.orientation.x(), latest_odom_msg_.orientation.y(),
                                                latest_odom_msg_.orientation.z(), latest_odom_msg_.orientation.w());
        const Affine2f T_odom_base_last =
            Translation2f(latest_odom_msg_.position.x(), latest_odom_msg_.position.y()) * Rotation2Df(odom_yaw_last);
        if (loc_initialized_) {
            const Affine2f T_map_base_last = Translation2f(robot_loc_) * Rotation2Df(robot_angle_);
            T_map_odom = T_map_base_last * T_odom_base_last.inverse();
        }
        const Affine2f T_map_base_pred = T_map_odom * (Translation2f(odom_loc_) * Rotation2Df(odom_angle_));
        robot_loc_fp_ = T_map_base_pred.translation();
        robot_angle_fp_ = std::atan2(T_map_base_pred.linear()(1, 0), T_map_base_pred.linear()(0, 0));
    }
}

void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
    point_cloud_ = cloud;
    t_point_cloud_ = time;
}

vector<GraphDomain::State> Navigation::Plan(const Vector2f& initial, const Vector2f& end) {
    vector<GraphDomain::State> path;
    static CumulativeFunctionTimer function_timer_(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&function_timer_);
    static const bool kVisualize = true;
    typedef navigation::GraphDomain Domain;
    planning_domain_.ResetDynamicStates();
    const uint64_t start_id = planning_domain_.AddDynamicState(initial);
    const uint64_t goal_id = planning_domain_.AddDynamicState(end);
    Domain::State start = planning_domain_.states[start_id];
    Domain::State goal = planning_domain_.states[goal_id];
    GraphVisualizer graph_viz(kVisualize);
    // ?? figure out whats the planning domain and graph for empty map, and is there a default grid that it fallbacks to
    // when no nodes?
    const bool found_path = AStar(start, goal, planning_domain_, &graph_viz, &path);
    if (!found_path) {
        printf("No path found!\n");
    }
    return path;
}

bool Navigation::PlanStillValid() {
    // ??, why max_plan_deviation is needed? it should just go to the closest point on path right howsoever far?
    if (plan_path_.size() < 2)
        return false;  // ?? is it due to (start, end) atleast. In that case, why would the distance check be false
    // ever?
    const Vector2f pose = robot_loc_fp_;  // predicted pose at actuation time
    for (size_t i = 0; i + 1 < plan_path_.size(); ++i) {
        const float dist_from_segment =
            geometry::DistanceFromLineSegment(pose, plan_path_[i].loc, plan_path_[i + 1].loc);
        if (dist_from_segment < FLAGS_max_plan_deviation) {
            return true;
        }
    }
    return false;
}

bool Navigation::GetCarrot(Vector2f& carrot, float carrot_dist) {
    if (carrot_dist < 0) {
        carrot_dist = params_.carrot_dist;
    }
    const auto& plan_path = plan_path_;
    if (plan_path.size() < 2u) {  // guard, ?? is this needed?
        return false;
    }
    // Predicted map pose at actuation time.
    const Vector2f pose = robot_loc_fp_;
    const float kSqCarrotDist = Sq(carrot_dist);

    // If goal (map frame) is within the carrot dist, set the carrot (map frame) to the goal
    if ((plan_path[0].loc - pose).squaredNorm() < kSqCarrotDist) {
        carrot = plan_path[0].loc;
        return true;
    }

    // Find closest line segment in plan to current robot location (all in map frame)
    float closest_dist = FLT_MAX;
    int i0 = 0, i1 = 1;
    for (size_t i = 0; i + 1 < plan_path.size(); ++i) {
        const Vector2f v0 = plan_path[i].loc;
        const Vector2f v1 = plan_path[i + 1].loc;
        const float dist_to_segment = geometry::DistanceFromLineSegment(pose, v0, v1);
        if (dist_to_segment < closest_dist) {
            closest_dist = dist_to_segment;
            i0 = static_cast<int>(i);
            i1 = static_cast<int>(i + 1);
        }
    }

    // Fallback: if robot is too far from path, project robot position onto closest path segment (all in map frame).
    if (closest_dist > carrot_dist) {
        const Vector2f v0 = plan_path[i0].loc;
        const Vector2f v1 = plan_path[i1].loc;
        carrot = geometry::ProjectPointOntoLineSegment(pose, v0, v1);
        return true;
    }

    // Find path segment that crosses carrot circle boundary (one vertex inside, one outside)
    // Iterate backward along the path toward the goal (index 0) to find a line segment
    // that spans the carrot circle boundary. All calculations are in map frame.
    // The goal is not within carrot dist of the robot, and the robot is within
    // carrot dist of some line segment. Hence, there must exist at least one
    // vertex along the plan towards the goal that is outside the carrot dist.
    // This ensures we find a segment where one endpoint is inside carrot distance
    // and one is outside, allowing proper circle-line intersection calculation.
    for (int i = i1; i - 1 >= 0; --i) {
        i0 = i;
        // const Vector2f v0 = plan_path_[i].loc;
        const Vector2f v1 = plan_path[i - 1].loc;  // vertex closer to goal (map frame)
        if ((v1 - pose).squaredNorm() > kSqCarrotDist) {
            break;  // Found first vertex outside carrot distance - this defines our target segment
        }
    }
    i1 = i0 - 1;
    if (i1 < 0) {
        carrot = plan_path[0].loc;
        return true;
    }

    const Vector2f v0 = plan_path[i0].loc;
    const Vector2f v1 = plan_path[i1].loc;
    Vector2f r0, r1;
    // Calculate where carrot circle intersects the target path segment
    const int num_intersections = geometry::CircleLineIntersection<float>(pose, carrot_dist, v0, v1, &r0, &r1);
    if (num_intersections == 0) {
        fprintf(stderr,
                "GetCarrot: Error obtaining intersections; v0:(%f %f) v1:(%f %f) pose:(%f %f) carrot^2:%f closest:%f\n",
                v0.x(), v0.y(), v1.x(), v1.y(), pose.x(), pose.y(), kSqCarrotDist, closest_dist);
        return false;
    }

    // Choose intersection point closer to goal (v1 is goal-ward from v0)
    if (num_intersections == 1 || (r0 - v1).squaredNorm() < (r1 - v1).squaredNorm()) {
        carrot = r0;
    } else {
        carrot = r1;
    }
    return true;
}

void Navigation::RunObstacleAvoidance(Vector2f& vel_cmd, float& ang_vel_cmd) {
    static CumulativeFunctionTimer function_timer_(__FUNCTION__);
    CumulativeFunctionTimer::Invocation invoke(&function_timer_);
    Vector2f local_target = local_target_;

    // Update planner components with current state and obstacles
    // Reconstruct predicted map pose at actuation time
    const Vector2f map_loc_pred = robot_loc_fp_;
    const float yaw_map_pred = robot_angle_fp_;

    // Enable angular TOC in omnidirectional sampler only when in obstacle avoidance mode
    // (RunObstacleAvoidance is only called when nav_state_ == kGoto && in_obstacle_avoidance_mode_ == true)
    if (params_.motion_primitives_mode == "omni") {
        auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
        omni_sampler->enable_angular_toc_runtime_ = true;
    }

    sampler_->Update(robot_vel_, robot_omega_, local_target, fp_point_cloud_);
    evaluator_->Update(map_loc_pred, yaw_map_pred, robot_vel_, robot_omega_, local_target, fp_point_cloud_);

    // Generate path options
    auto paths = sampler_->GetSamples(params_.num_options);
    if (paths.size() == 0) {
        // Fallback: no path options available
        Halt(vel_cmd, ang_vel_cmd);
        return;
    }
    // Select best path from options
    auto best_path = evaluator_->FindBest(paths);
    if (best_path == nullptr) {
        // Fallback: no valid path found
        TurnInPlace(vel_cmd, ang_vel_cmd);
        return;
    }

    float max_map_speed = params_.linear_limits.max_speed;
    planning_domain_.GetClearanceAndSpeedFromLoc(map_loc_pred, nullptr, &max_map_speed);
    auto linear_limits = params_.linear_limits;
    linear_limits.max_speed = min(max_map_speed, params_.linear_limits.max_speed);

    best_path->GetControls(linear_limits, params_.angular_limits, params_.dt, robot_vel_, robot_omega_, vel_cmd,
                           ang_vel_cmd);
    sampled_paths_ = paths;
    best_option_ = best_path;
}

void Navigation::Halt(Vector2f& cmd_vel, float& angular_vel_cmd) {
    const float kEpsSpeed = 0.01f;
    const float kEpsOmega = 0.01f;

    // Decelerate linear velocity vector toward zero without assuming 1D motion
    const Vector2f current_v = robot_vel_;
    const float current_speed = current_v.norm();
    Vector2f next_v(0.f, 0.f);
    if (current_speed > kEpsSpeed) {
        const float dv = params_.linear_limits.max_deceleration * params_.dt;
        if (current_speed > dv) {
            next_v = current_v * ((current_speed - dv) / current_speed);
        } else {
            next_v.setZero();
        }
    }
    cmd_vel = next_v;

    // Decelerate angular velocity toward zero
    const float omega = robot_omega_;
    float next_omega = 0.f;
    if (fabs(omega) > kEpsOmega) {
        const float d_omega = params_.angular_limits.max_deceleration * params_.dt;
        if (fabs(omega) > d_omega) {
            next_omega = omega - Sign(omega) * d_omega;
        } else {
            next_omega = 0.f;
        }
    }
    angular_vel_cmd = next_omega;
}

void Navigation::TurnInPlace(Vector2f& cmd_vel, float& cmd_angle_vel) {
    // If we're moving too fast linearly, slow down first (use full 2D speed)
    const float kMaxLinearSpeedDuringTurn = 0.1f;
    const float lin_speed = robot_vel_.norm();
    if (lin_speed > kMaxLinearSpeedDuringTurn) {
        // Decelerate linear velocity; don't spin and drive at once when too fast
        Halt(cmd_vel, cmd_angle_vel);
        return;
    }

    // Predicted yaw in map frame at actuation time
    const float yaw_map_pred = robot_angle_fp_;

    // Desired heading error
    float dTheta = 0.0f;
    if (nav_state_ == NavigationState::kGoto) {
        // Turn towards local_target (robot frame)
        dTheta = atan2(local_target_.y(), local_target_.x());
    } else if (nav_state_ == NavigationState::kTurnInPlace) {
        // Turn towards nav_goal_angle_ in map frame
        dTheta = AngleDiff(nav_goal_angle_, yaw_map_pred);
    }

    // If already close enough, stop
    if (fabs(dTheta) < 1e-3f) {
        cmd_vel = Vector2f(0.f, 0.f);
        cmd_angle_vel = 0.f;
        return;
    }

    // Angular motion profiling: if rotating the wrong way, bleed off omega first
    const float s = Sign(dTheta);
    if (robot_omega_ * dTheta < 0.0f) {
        const float domega = params_.angular_limits.max_deceleration * params_.dt;
        cmd_angle_vel = (fabs(robot_omega_) < domega) ? 0.f : (robot_omega_ - Sign(robot_omega_) * domega);
    } else {
        // Early-brake guard: if remaining angle is less than stopping distance, brake now
        const float omega = robot_omega_;
        const float stop_angle = (omega * omega) / (2.0f * params_.angular_limits.max_deceleration);
        if (stop_angle >= std::fabs(dTheta)) {
            const float domega = params_.angular_limits.max_deceleration * params_.dt;
            cmd_angle_vel = (std::fabs(omega) <= domega) ? 0.0f : (omega - Sign(omega) * domega);
        } else {
            cmd_angle_vel = s * motion_primitives::Run1DTimeOptimalControl(params_.angular_limits, 0.f, s * omega,
                                                                           s * dTheta, 0.f, params_.dt);
        }
    }

    // No linear motion while turning in place
    cmd_vel = Vector2f(0.f, 0.f);
}

bool Navigation::Run(const double& time, Vector2f& cmd_vel, float& cmd_angle_vel) {
    // Early exit checks
    if (!initialized_) {
        return false;
    }
    if (!odom_initialized_) {
        return false;
    }
    // Ensure sensor data is available before proceeding
    if (!std::isfinite(t_point_cloud_) || !std::isfinite(t_odometry_)) {
        return false;
    }

    navigation_debug::DebugLog(std::string("[") + std::to_string(static_cast<int>(nav_state_)) +
                               "] command_history_ length: " + std::to_string(command_history_.size()));

    PruneLatencyQueue();
    // Forward predict robot state to account for actuation latency
    ForwardPredict(time + params_.actuation_latency);

    // Local target in predicted base frame at actuation time
    const Affine2f T_map_base_pred = Translation2f(robot_loc_fp_) * Rotation2Df(robot_angle_fp_);

    if (nav_state_ == NavigationState::kGoto) {
        // Recompute global plan if current plan is invalid
        if (!PlanStillValid()) {
            plan_path_ = Plan(robot_loc_, nav_goal_loc_);
        }
        // Get carrot point from global plan
        Vector2f carrot(0, 0);
        bool foundCarrot = GetCarrot(carrot);
        if (!foundCarrot) {
            // No valid carrot found, halt and fail
            in_obstacle_avoidance_mode_ = false;  // Reset sub-state when carrot unavailable
            Halt(cmd_vel, cmd_angle_vel);
            return false;
        }
        // Transform carrot from map frame to robot frame for local planning
        // local_target_ = Rotation2Df(-robot_angle_) * (carrot - robot_loc_);
        // Compute local target in the predicted base frame at actuation time.
        const Affine2f T_base_map_pred = T_map_base_pred.inverse();
        local_target_ = T_base_map_pred * carrot;

        // Clamp local target length for OA
        if (local_target_.squaredNorm() > Sq(params_.carrot_dist)) {
            local_target_ = params_.carrot_dist * local_target_.normalized();
        }
    }

    // Switch between navigation states.
    NavigationState prev_state = nav_state_;
    do {
        prev_state = nav_state_;
        // Transition from kGoto to kTurnInPlace when close to target and slow enough
        if (nav_state_ == NavigationState::kGoto && local_target_.squaredNorm() < Sq(params_.target_dist_tolerance) &&
            robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance) &&
            std::fabs(robot_omega_) < params_.target_omega_tolerance) {
            nav_state_ = NavigationState::kTurnInPlace;
            in_obstacle_avoidance_mode_ = false;  // Reset sub-state when leaving kGoto
            // Disable angular TOC when leaving kGoto state
            if (params_.motion_primitives_mode == "omni") {
                auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
                omni_sampler->enable_angular_toc_runtime_ = false;
            }
            // Transition from kTurnInPlace to kStopped when final orientation is reached
        } else if (nav_state_ == NavigationState::kTurnInPlace &&
                   AngleDist(robot_angle_fp_, nav_goal_angle_) < params_.target_angle_tolerance &&
                   std::fabs(robot_omega_) < params_.target_omega_tolerance) {
            nav_state_ = NavigationState::kStopped;
        }
        // continue until no more state changes can happen
        // loop allows for multiple state changes in the same control loop iteration
    } while (prev_state != nav_state_);

    switch (nav_state_) {
        case NavigationState::kStopped: {
        } break;
        case NavigationState::kGoto: {
        } break;
        case NavigationState::kTurnInPlace: {
        } break;
        default: {
            fprintf(stderr, "ERROR: Unknown nav state %d\n", static_cast<int>(nav_state_));
        }
    }

    if (nav_state_ == NavigationState::kStopped) {
        // Disable angular TOC in kStopped state
        if (params_.motion_primitives_mode == "omni") {
            auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
            omni_sampler->enable_angular_toc_runtime_ = false;
        }
        Halt(cmd_vel, cmd_angle_vel);
        return true;
    } else if (nav_state_ == NavigationState::kGoto) {
        const float theta = atan2(local_target_.y(), local_target_.x());

        // Hysteresis-based FOV check to prevent oscillation:
        // - To START obstacle avoidance: target must be well-centered (±center_threshold)
        // - To CONTINUE obstacle avoidance: target can be anywhere in FOV (±local_half_fov)

        if (in_obstacle_avoidance_mode_) {
            // Already doing obstacle avoidance: keep going unless target leaves FOV
            if (fabs(theta) > params_.local_half_fov) {
                // Target left FOV: switch back to turning
                in_obstacle_avoidance_mode_ = false;
                // Disable angular TOC since we're no longer in obstacle avoidance mode
                if (params_.motion_primitives_mode == "omni") {
                    auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
                    omni_sampler->enable_angular_toc_runtime_ = false;
                }
                TurnInPlace(cmd_vel, cmd_angle_vel);
            } else {
                // Target still in FOV: continue obstacle avoidance
                RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
            }
        } else {
            // Currently turning: only start obstacle avoidance when target is well-centered
            // Disable angular TOC when turning (not in obstacle avoidance mode)
            if (params_.motion_primitives_mode == "omni") {
                auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
                omni_sampler->enable_angular_toc_runtime_ = false;
            }
            if (fabs(theta) <= params_.center_threshold) {
                // Target is centered: start obstacle avoidance
                in_obstacle_avoidance_mode_ = true;
                RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
            } else {
                // Target not centered: keep turning
                TurnInPlace(cmd_vel, cmd_angle_vel);
            }
        }
    } else if (nav_state_ == NavigationState::kTurnInPlace) {
        // Disable angular TOC in kTurnInPlace state
        if (params_.motion_primitives_mode == "omni") {
            auto* omni_sampler = static_cast<motion_primitives::OmniSampler*>(sampler_.get());
            omni_sampler->enable_angular_toc_runtime_ = false;
        }
        TurnInPlace(cmd_vel, cmd_angle_vel);
    }

    return true;
}

}  // namespace navigation
