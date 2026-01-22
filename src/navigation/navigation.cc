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

// Declare gflags in global namespace to avoid namespace-mismatch at link time.
DECLARE_double(min_ang_toc_sample_length);

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

namespace {
inline int VelocityToMotorCounts(float vel, float slope_pos, float intercept_pos, float slope_neg,
                                 float intercept_neg) {
    if (vel > 0.0f) {
        double counts = static_cast<double>(slope_pos) * static_cast<double>(vel) + static_cast<double>(intercept_pos);
        return static_cast<int>(std::floor(counts));
    } else if (vel < 0.0f) {
        double counts = static_cast<double>(slope_neg) * static_cast<double>(vel) + static_cast<double>(intercept_neg);
        return static_cast<int>(std::ceil(counts));
    }
    return 0;
}

inline float MotorCountsToVelocity(int counts, float slope_pos, float intercept_pos, float slope_neg,
                                   float intercept_neg) {
    if (counts == 0) {
        return 0.0f;
    }
    if (counts > 0) {
        return static_cast<float>((static_cast<double>(counts) - static_cast<double>(intercept_pos)) /
                                  static_cast<double>(slope_pos));
    } else {
        return static_cast<float>((static_cast<double>(counts) - static_cast<double>(intercept_neg)) /
                                  static_cast<double>(slope_neg));
    }
}
}  // namespace

void ApplyCommandMapping(const NavigationParameters& params, Eigen::Vector2f& vel_cmd, float& ang_vel_cmd) {
    if (!params.apply_custom_cmd_map) {
        return;
    }

    // Convert velocity commands to motor counts (as driver does), then back to effective velocity
    // to get the actual velocity that will be executed, accounting for quantization
    int counts_x = VelocityToMotorCounts(vel_cmd.x(), params.cmd_map_x_slope_pos, params.cmd_map_x_intercept_pos,
                                         params.cmd_map_x_slope_neg, params.cmd_map_x_intercept_neg);
    int counts_y = VelocityToMotorCounts(vel_cmd.y(), params.cmd_map_y_slope_pos, params.cmd_map_y_intercept_pos,
                                         params.cmd_map_y_slope_neg, params.cmd_map_y_intercept_neg);
    float vx = MotorCountsToVelocity(counts_x, params.cmd_map_x_slope_pos, params.cmd_map_x_intercept_pos,
                                     params.cmd_map_x_slope_neg, params.cmd_map_x_intercept_neg);
    float vy = MotorCountsToVelocity(counts_y, params.cmd_map_y_slope_pos, params.cmd_map_y_intercept_pos,
                                     params.cmd_map_y_slope_neg, params.cmd_map_y_intercept_neg);
    vel_cmd = Eigen::Vector2f(vx, vy);

    int counts_r = VelocityToMotorCounts(ang_vel_cmd, params.cmd_map_r_slope_pos, params.cmd_map_r_intercept_pos,
                                         params.cmd_map_r_slope_neg, params.cmd_map_r_intercept_neg);
    ang_vel_cmd = MotorCountsToVelocity(counts_r, params.cmd_map_r_slope_pos, params.cmd_map_r_intercept_pos,
                                        params.cmd_map_r_slope_neg, params.cmd_map_r_intercept_neg);
}

Navigation::Navigation()
    : robot_loc_(0, 0),
      robot_angle_(0),
      robot_loc_fp_(0, 0),
      robot_angle_fp_(0),
      nav_state_(NavigationState::kStopped),
      in_obstacle_avoidance_mode_(false),
      yaw_align_sp_map_(0.0f),
      yaw_align_sp_init_(false),
      nav_goal_loc_(0, 0),
      nav_goal_angle_(0),
      robot_vel_(0, 0),
      robot_omega_(0),
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
        evaluator->SetNavParams(params);
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
    yaw_align_sp_init_ = false;
}

void Navigation::ResetNavGoals() {
    nav_state_ = NavigationState::kStopped;
    nav_goal_loc_ = robot_loc_;
    nav_goal_angle_ = robot_angle_;
    local_target_.setZero();
    plan_path_.clear();
    in_obstacle_avoidance_mode_ = false;
    yaw_align_sp_init_ = false;
}

void Navigation::UpdateGeometryParams(float width, float length, float offset_x, float offset_y, float obstacle_margin,
                                      bool do_ang_toc) {
    // Basic validation (avoid corrupting planner state with NaNs / invalid geometry)
    if (!std::isfinite(width) || !std::isfinite(length) || !std::isfinite(offset_x) || !std::isfinite(offset_y) ||
        !std::isfinite(obstacle_margin)) {
        LOG(WARNING) << "UpdateGeometryParams ignored: non-finite input(s).";
        return;
    }
    if (width <= 0.0f || length <= 0.0f || obstacle_margin < 0.0f) {
        LOG(WARNING) << "UpdateGeometryParams ignored: invalid geometry. width=" << width << " length=" << length
                     << " margin=" << obstacle_margin;
        return;
    }

    const bool changed = (params_.robot_width != width) || (params_.robot_length != length) ||
                         (params_.geometric_center_offset.x != offset_x) ||
                         (params_.geometric_center_offset.y != offset_y) ||
                         (params_.obstacle_margin != obstacle_margin) || (params_.do_ang_toc != do_ang_toc);

    if (!changed) return;

    params_.robot_width = width;
    params_.robot_length = length;
    params_.geometric_center_offset.x = offset_x;
    params_.geometric_center_offset.y = offset_y;
    params_.obstacle_margin = obstacle_margin;
    params_.do_ang_toc = do_ang_toc;

    // Propagate updated footprint to local planner components.
    if (sampler_) sampler_->SetNavParams(params_);
    if (evaluator_) evaluator_->SetNavParams(params_);

    // IMPORTANT: clear any state that would be inconsistent with old geometry.
    plan_path_.clear();  // force replan under new geometry
    in_obstacle_avoidance_mode_ = false;
    yaw_align_sp_init_ = false;

    // Clear stale per-tick artifacts so visualization/planner doesn't reuse old-geometry samples.
    sampled_paths_.clear();
    best_option_.reset();

    LOG(INFO) << "Robot geometry updated: width=" << params_.robot_width << " length=" << params_.robot_length
              << " offset=(" << params_.geometric_center_offset.x << "," << params_.geometric_center_offset.y << ")"
              << " margin=" << params_.obstacle_margin << " do_ang_toc=" << (params_.do_ang_toc ? "true" : "false");
}

void Navigation::UpdateMap(const string& map_path) {
    planning_domain_.Load(map_path);
    plan_path_.clear();
    in_obstacle_avoidance_mode_ = false;  // Reset sub-state when plan is cleared
    yaw_align_sp_init_ = false;           // Reset yaw alignment setpoint when map is updated
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
    const bool use_omni = (params_.motion_primitives_mode == "omni");
    // Find the last command with start time <= t (active on [t-dt, t) if present)
    // Loop iterates from front to back of the queue (earliest to latest commands)
    // Since command_history_ is sorted by cmd_exec_start_time in ascending order,
    // we iterate through commands chronologically to find the last command
    // whose execution start time is <= t (i.e., the command active at time t)
    const Twist* active = nullptr;
    if (!command_history_.empty()) {
        for (const Twist& c : command_history_) {
            if (c.cmd_exec_start_time <= t)
                active = &c;  // Keep updating to find the latest valid command
            else
                break;  // Since sorted, no later commands will have start_time <= t
        }
    }

    // Set the latest odometry location and angle
    odom_loc_ = Vector2f(latest_odom_msg_.position.x(), latest_odom_msg_.position.y());
    {
        const auto& q = latest_odom_msg_.orientation;
        odom_angle_ = YawFromQuat(q.x(), q.y(), q.z(), q.w());
    }
    // Anchor yaw at t_odometry_ (do not mutate this variable)
    const float yaw_anchor_at_t_odom = odom_angle_;

    // Returns yaw at arbitrary time s by integrating ω relative to t_odometry_.
    auto YawAt = [&](double s) -> float {
        float yaw = yaw_anchor_at_t_odom;
        if (s < t_odometry_) {
            // integrate backward
            for (const Twist& c : command_history_) {
                const double seg0 = c.cmd_exec_start_time;
                const double seg1 = seg0 + dt_seg;
                const double dt_back = overlap(seg0, seg1, s, t_odometry_);
                if (dt_back > 0.0) {
                    yaw -= static_cast<float>(c.angular.z()) * static_cast<float>(dt_back);
                }
            }
        } else if (s > t_odometry_) {
            // integrate forward
            for (const Twist& c : command_history_) {
                const double seg0 = c.cmd_exec_start_time;
                const double seg1 = seg0 + dt_seg;
                const double dt_fwd = overlap(seg0, seg1, t_odometry_, s);
                if (dt_fwd > 0.0) {
                    yaw += static_cast<float>(c.angular.z()) * static_cast<float>(dt_fwd);
                }
            }
        }
        return AngleMod(yaw);
    };

    // Predicted velocity just BEFORE time t (using selected motion primitive semantics)
    if (!active) {
        // Before the first command: zero twist
        robot_vel_ = Vector2f::Zero();
        robot_omega_ = 0.f;
    } else if (use_omni) {
        // STRAIGHT+SPIN: v^b(t) = R(-theta(t)) * ( R(theta(t_k)) * v^b_cmd )
        const Vector2f v_b_cmd(active->linear.x(), active->linear.y());
        const float theta_cmd = YawAt(active->cmd_exec_start_time);  // θ(t_k)
        const Vector2f u_w = Rotation2Df(theta_cmd) * v_b_cmd;       // latched world vector
        const float theta_now = YawAt(t);                            // θ(t^-)
        robot_vel_ = Rotation2Df(-theta_now) * u_w;                  // executed base-frame linear vel at t^-
        robot_omega_ = static_cast<float>(active->angular.z());
    } else {
        // ARC (old) semantics: commanded body twist is the executed twist
        robot_vel_ = Vector2f(active->linear.x(), active->linear.y());
        robot_omega_ = static_cast<float>(active->angular.z());
    }

    // Forward predict the robot's pose and accumulate inverse LiDAR transform
    Affine2f lidar_tf = Affine2f::Identity();
    float lidar_angle = YawAt(t_point_cloud_);  // yaw at LiDAR timestamp
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

                if (use_omni) {
                    // STRAIGHT+SPIN: u^W is latched at command start
                    const float theta_cmd = YawAt(seg0);
                    const Vector2f u_w = Rotation2Df(theta_cmd) * v_b;
                    odom_loc_ += fdto * u_w;  // world-frame linear vel is constant
                } else {
                    // ARC (old): body-frame linear vel, coupled with heading
                    odom_loc_ += fdto * (Rotation2Df(odom_angle_) * v_b);
                }

                // yaw always integrates independently
                odom_angle_ = AngleMod(odom_angle_ + fdto * static_cast<float>(c.angular.z()));
            }
        }
        // ---- LiDAR: accumulate inverse motion over [t_point_cloud_, t)
        {
            const double dtl = overlap(seg0, seg1, t_point_cloud_, t);
            if (dtl > 0.0) {
                const float fdtl = static_cast<float>(dtl);
                const float dth = static_cast<float>(c.angular.z()) * fdtl;
                Rotation2Df Rstep(-dth);  // inverse rotation for the small step
                const Vector2f v_b(c.linear.x(), c.linear.y());

                Vector2f tstep;
                if (use_omni) {
                    // STRAIGHT+SPIN: t = -R(theta_end)^T * u^W * dt
                    const float theta_cmd = YawAt(seg0);
                    const Vector2f u_w = Rotation2Df(theta_cmd) * v_b;
                    const float theta_end = AngleMod(lidar_angle + dth);
                    tstep = -(Rotation2Df(-theta_end) * u_w) * fdtl;  // -R(theta_end)^T * u^W * dt
                    lidar_angle = theta_end;                          // advance LiDAR-side yaw
                } else {
                    // ARC (old): t = -R(-dth) * v_b * dt  (i.e., -(Rstep * v_b) * dt)
                    tstep = -(Rstep * v_b) * fdtl;
                    // optional: lidar_angle += dth; // not needed by ARC math, safe either way
                }

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
    static const bool kVisualize = true;
    typedef navigation::GraphDomain Domain;
    // Fallback: if map has no graph (EmptyMap or load failure), return a direct start->goal segment.
    if (planning_domain_.states.empty() || planning_domain_.edges.empty()) {
        path.emplace_back(0, initial);
        path.emplace_back(1, end);
        return path;
    }
    planning_domain_.ResetDynamicStates();
    const uint64_t start_id = planning_domain_.AddDynamicState(initial);
    const uint64_t goal_id = planning_domain_.AddDynamicState(end);
    Domain::State start = planning_domain_.states[start_id];
    Domain::State goal = planning_domain_.states[goal_id];
    GraphVisualizer graph_viz(kVisualize);
    const bool found_path = AStar(start, goal, planning_domain_, &graph_viz, &path);
    if (!found_path) {
        printf("No path found!\n");
    }
    return path;
}

bool Navigation::PlanStillValid() {
    if (plan_path_.size() < 2) return false;
    const Vector2f pose = robot_loc_fp_;  // predicted pose at actuation time
    for (size_t i = 0; i + 1 < plan_path_.size(); ++i) {
        const float dist_from_segment =
            geometry::DistanceFromLineSegment(pose, plan_path_[i].loc, plan_path_[i + 1].loc);
        if (dist_from_segment < params_.max_plan_deviation) {
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
    if (plan_path.size() < 2u) {
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
    Vector2f local_target = local_target_;

    // Update planner components with current state and obstacles
    // Reconstruct predicted map pose at actuation time
    const Vector2f map_loc_pred = robot_loc_fp_;
    const float yaw_map_pred = robot_angle_fp_;

    // NUDGE window: within nudge distance tolerance of final goal (MAP frame)
    const bool near_goal_nudge = (nav_goal_loc_ - map_loc_pred).squaredNorm() <= Sq(params_.nudge_dist_tolerance);

    sampler_->Update(robot_vel_, robot_omega_, local_target, fp_point_cloud_);
    evaluator_->Update(map_loc_pred, yaw_map_pred, robot_vel_, robot_omega_, local_target, fp_point_cloud_);

    // Generate path options
    auto paths = sampler_->GetSamples(params_.num_options);
    if (paths.size() == 0) {
        // Clear stale visualization data
        sampled_paths_.clear();
        best_option_.reset();
        // Fallback: no path options available
        Halt(vel_cmd, ang_vel_cmd);
        return;
    }
    // Select best path from options
    auto best_path = evaluator_->FindBest(paths);
    if (best_path == nullptr) {
        // Clear stale visualization data
        sampled_paths_.clear();
        best_option_.reset();
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

    // === Smooth "look-where-you-go" yaw alignment (Navigation-level) ===
    if (params_.motion_primitives_mode == "omni" && params_.do_ang_toc && !near_goal_nudge &&
        best_path->Length() >= FLAGS_min_ang_toc_sample_length) {
        const float speed = vel_cmd.norm();
        const float vmin = 0.05f;  // don't try to align while essentially stopped
        if (speed > vmin) {
            // Heading of the commanded linear velocity in MAP frame at actuation time
            const Eigen::Rotation2Df R_map_base(yaw_map_pred);
            const Eigen::Vector2f v_map_cmd = R_map_base * vel_cmd;
            const float heading_map_target = std::atan2(v_map_cmd.y(), v_map_cmd.x());

            // Initialize persistent setpoint once
            if (!yaw_align_sp_init_) {
                yaw_align_sp_map_ = heading_map_target;
                yaw_align_sp_init_ = true;
            }

            // Rate-limit how fast the setpoint can move (prevents wiggle)
            float err = AngleMod(heading_map_target - yaw_align_sp_map_);
            const float max_step = params_.angular_limits.max_speed * params_.dt;  // rad per control tick
            if (err > max_step) err = max_step;
            if (err < -max_step) err = -max_step;
            yaw_align_sp_map_ = AngleMod(yaw_align_sp_map_ + err);

            // 1D TOC to the filtered setpoint using predicted yaw
            const float dTheta = AngleDiff(yaw_align_sp_map_, yaw_map_pred);
            const float s = Sign(dTheta);

            if (robot_omega_ * dTheta < 0.0f) {
                // Wrong-way: brake using decel
                const float domega = params_.angular_limits.max_deceleration * params_.dt;
                ang_vel_cmd = (std::fabs(robot_omega_) <= domega) ? 0.0f : (robot_omega_ - Sign(robot_omega_) * domega);
            } else {
                // Small deadband to avoid dithering near alignment
                const float kDeadband = 0.02f;  // ~1.1 deg
                if (std::fabs(dTheta) < kDeadband) {
                    const float domega = params_.angular_limits.max_deceleration * params_.dt;
                    ang_vel_cmd =
                        (std::fabs(robot_omega_) <= domega) ? 0.0f : (robot_omega_ - Sign(robot_omega_) * domega);
                } else {
                    ang_vel_cmd = s * motion_primitives::Run1DTimeOptimalControl(
                                          params_.angular_limits, 0.0f, s * robot_omega_, s * dTheta, 0.0f, params_.dt);
                }
            }
        } else {
            // Essentially stopped → only brake omega (not actively aligning)
            const float domega = params_.angular_limits.max_deceleration * params_.dt;
            ang_vel_cmd = (std::fabs(robot_omega_) <= domega) ? 0.0f : (robot_omega_ - Sign(robot_omega_) * domega);
        }
    }

    // Apply command mapping before returning
    ApplyCommandMapping(params_, vel_cmd, ang_vel_cmd);
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

    // Apply command mapping before returning
    ApplyCommandMapping(params_, cmd_vel, angular_vel_cmd);
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
        // Apply command mapping before returning
        ApplyCommandMapping(params_, cmd_vel, cmd_angle_vel);
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

    // Apply command mapping before returning
    ApplyCommandMapping(params_, cmd_vel, cmd_angle_vel);
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
            yaw_align_sp_init_ = false;           // Reset yaw alignment setpoint when carrot unavailable
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

    // Switch between navigation states
    const float dist_goal_sq = (nav_goal_loc_ - robot_loc_fp_).squaredNorm();  // MAP-frame distance^2
    const bool transl_ok = (dist_goal_sq < Sq(params_.target_dist_tolerance));
    const bool vel_ok = (robot_vel_.squaredNorm() < Sq(params_.target_vel_tolerance));
    const bool omega_ok_tip = (std::fabs(robot_omega_) < params_.target_omega_tolerance / 2.0f);
    const bool omega_ok_stop = (std::fabs(robot_omega_) < params_.target_omega_tolerance);
    const bool angle_ok = AngleDist(robot_angle_fp_, nav_goal_angle_) < params_.target_angle_tolerance;

    NavigationState prev_state = nav_state_;
    do {
        prev_state = nav_state_;

        if (nav_state_ == NavigationState::kGoto) {
            // Transition from kGoto to kTurnInPlace when close to target and slow enough
            if (transl_ok && vel_ok && omega_ok_tip) {
                nav_state_ = NavigationState::kTurnInPlace;
                in_obstacle_avoidance_mode_ = false;  // Reset sub-state when leaving kGoto
                yaw_align_sp_init_ = false;
            }
        } else if (nav_state_ == NavigationState::kTurnInPlace) {
            if (angle_ok && omega_ok_stop) {
                if (transl_ok && vel_ok) {
                    // Final orientation AND translation within tolerance -> Stopped
                    nav_state_ = NavigationState::kStopped;
                } else {
                    // Rotated correctly but drifted outside translational tolerance -> go back to GoTo
                    nav_state_ = NavigationState::kGoto;
                    in_obstacle_avoidance_mode_ = false;
                    yaw_align_sp_init_ = false;
                }
            }
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

    auto ClearSampledPaths = [&]() {
        sampled_paths_.clear();
        best_option_.reset();
    };

    // TODO; turninplace is not obstacle aware currently, is there a way to do something about it?
    if (nav_state_ == NavigationState::kStopped) {
        plan_path_.clear();  // Drop stale plan so path viz clears once goal is done/stopped
        yaw_align_sp_init_ = false;
        ClearSampledPaths();
        Halt(cmd_vel, cmd_angle_vel);
        return true;
    } else if (nav_state_ == NavigationState::kGoto) {
        const float theta = atan2(local_target_.y(), local_target_.x());

        // "Nudge" window: when close to goal, prefer continuing OA over FOV-based turning
        const float goal_dist2 = (nav_goal_loc_ - robot_loc_fp_).squaredNorm();  // MAP-frame distance^2
        const bool near_goal_nudge = (goal_dist2 <= Sq(params_.nudge_dist_tolerance));
        // Hysteresis-based lidar FOV check to prevent oscillation:
        // - To START obstacle avoidance: target must be within conservative FOV (±0.8 * lidar_fov_half_angle)
        // - To CONTINUE obstacle avoidance: target can be anywhere in full FOV (±lidar_fov_half_angle)

        const bool fov_ok_for_continue = (fabs(theta) <= params_.lidar_fov_half_angle);
        const bool fov_ok_for_start = (fabs(theta) <= 0.8f * params_.lidar_fov_half_angle);

        // Check nudge condition first
        if (near_goal_nudge) {
            // fprintf(stderr, "DEBUG: Not in lidar FOV but goal nudge is active\n");
            RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
        } else {
            if (in_obstacle_avoidance_mode_) {
                // Already doing obstacle avoidance: keep going unless target leaves full FOV
                if (!fov_ok_for_continue) {
                    // Target left FOV: switch back to turning
                    in_obstacle_avoidance_mode_ = false;
                    ClearSampledPaths();
                    TurnInPlace(cmd_vel, cmd_angle_vel);
                } else {
                    // Target still in FOV: continue obstacle avoidance
                    RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
                }
            } else {
                // Currently turning: only start obstacle avoidance when target is well-centered
                yaw_align_sp_init_ = false;
                if (fov_ok_for_start) {
                    // Target is well-centered in FOV: start obstacle avoidance
                    in_obstacle_avoidance_mode_ = true;
                    RunObstacleAvoidance(cmd_vel, cmd_angle_vel);
                } else {
                    // Target not well-centered: keep turning
                    ClearSampledPaths();
                    TurnInPlace(cmd_vel, cmd_angle_vel);
                }
            }
        }
    } else if (nav_state_ == NavigationState::kTurnInPlace) {
        ClearSampledPaths();
        TurnInPlace(cmd_vel, cmd_angle_vel);
    }
    return true;
}

}  // namespace navigation
