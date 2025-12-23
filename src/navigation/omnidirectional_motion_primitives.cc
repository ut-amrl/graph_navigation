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
\file    omnidirectional_motion_primitives.cc
\brief   Omnidirectional motion primitives implementation.
\author  Sadanand Modak (C) 2025
*/
//========================================================================

#include <math.h>
#include <algorithm>
#include <memory>
#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>

#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "config_reader/config_reader.h"
#include "omnidirectional_motion_primitives.h"
#include "motion_primitives.h"
#include "navigation.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;
using namespace math_util;

namespace motion_primitives {

// OmnidirectionalMovePath implementation
float OmnidirectionalMovePath::Length() const { return length; }

float OmnidirectionalMovePath::FPL() const { return fpl; }

float OmnidirectionalMovePath::AngularLength() const {
    return 0.0f;  // No angular movement for straight line motion
}

float OmnidirectionalMovePath::Clearance() const { return clearance; }

void OmnidirectionalMovePath::GetControls(const navigation::MotionLimits& linear_limits,
                                          const navigation::MotionLimits& angular_limits, const float dt,
                                          const Vector2f& vel, const float ang_vel, Vector2f& vel_cmd,
                                          float& ang_vel_cmd) const {
    // Calculate velocity component along the path direction
    const float v_along = vel.dot(direction);

    if (v_along < 0.0f) {
        // Wrong-way: robot moving opposite to desired path direction.
        // Must brake first before accelerating toward goal.
        const float dv = linear_limits.max_deceleration * dt;
        const float speed_away = std::fabs(v_along);
        if (speed_away > dv) {
            // Still braking - continue in current (wrong) direction but slower
            vel_cmd = (speed_away - dv) * (-direction);
        } else {
            // Braked to near-zero - can now start toward goal
            vel_cmd = Vector2f::Zero();
        }
    } else {
        // Correct direction or stopped - use 1D TOC
        const float speed = Run1DTimeOptimalControl(linear_limits, 0, v_along, length, 0, dt);
        vel_cmd = speed * direction;
    }

    // No rotation during straight-line motion; navigation owns yaw alignment
    ang_vel_cmd = 0;
}

Pose2Df OmnidirectionalMovePath::GetIntermediateState(float f) const {
    // Straight line movement only
    return Pose2Df(0, f * length * direction);
}

Pose2Df OmnidirectionalMovePath::EndPoint() const { return GetIntermediateState(1.0); }

// OmniSampler implementation
OmniSampler::OmniSampler() {}

namespace {
inline void PrecomputeUnitDirs(int n, std::vector<Eigen::Vector2f>& cache) {
    cache.resize(n);
    const float step = 2.0f * static_cast<float>(M_PI) / static_cast<float>(n);
    for (int i = 0; i < n; ++i) {
        const float a = step * i;
        cache[i] = Eigen::Vector2f(cos(a), sin(a));
    }
}
}  // namespace

void OmniSampler::SetMaxPathLength(OmnidirectionalMovePath* move) {
    // Projection of goal onto this direction (negative = pointing away from goal)
    const float distance_to_goal_along_direction = local_target.dot(move->direction);

    // Desired travel distance: toward goal, or 0 if pointing away
    const float desired_dist = std::clamp(distance_to_goal_along_direction, 0.0f, nav_params.max_free_path_length);
    move->length = desired_dist;
    move->fpl = desired_dist;
}

vector<shared_ptr<PathRolloutBase>> OmniSampler::GetSamples(int n) {
    vector<shared_ptr<PathRolloutBase>> samples(n);

    // Cache unit directions per n to avoid trig every cycle.
    static int cached_n = -1;
    static std::vector<Eigen::Vector2f> unit_dirs;
    if (cached_n != n) {
        PrecomputeUnitDirs(n, unit_dirs);
        cached_n = n;
    }

#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; ++i) {
        auto move = std::make_shared<OmnidirectionalMovePath>(unit_dirs[i], 0.0f);
        SetMaxPathLength(move.get());
        CheckObstacles(move.get());
        samples[i] = std::static_pointer_cast<PathRolloutBase>(move);
    }
    return samples;
}

void OmniSampler::CheckObstacles(OmnidirectionalMovePath* move) {
    // Path-aligned basis vectors (expressed in base_link frame)
    const Eigen::Vector2f dir_forward = move->direction;                   // motion direction (unit)
    const Eigen::Vector2f dir_lateral(-dir_forward.y(), dir_forward.x());  // perpendicular (CCW 90°)

    // Robot body
    const OffsetRect robot_body = {
        Eigen::Vector2f(nav_params.geometric_center_offset.x, nav_params.geometric_center_offset.y),
        0.5f * nav_params.robot_length, 0.5f * nav_params.robot_width};

    // Inflated robot body
    const OffsetRect robot_with_margin = {robot_body.center, robot_body.half_x + nav_params.obstacle_margin,
                                          robot_body.half_y + nav_params.obstacle_margin};

    // Swept area bounds (with margin) for collision detection
    const float front_dist = robot_with_margin.support(dir_forward);
    const float lateral_max = robot_with_margin.support(dir_lateral);
    const float lateral_min = -robot_with_margin.support(-dir_lateral);

    // Robot body bounds (no margin) for clearance calculation
    const float body_lateral_max = robot_body.support(dir_lateral);
    const float body_lateral_min = -robot_body.support(-dir_lateral);

    // ---- Compute FPL: find first obstacle that blocks the path ----
    for (const Eigen::Vector2f& p : *point_cloud) {
        if (robot_body.contains(p)) continue;  // skip points on robot itself

        // Transform p to path-aligned frame
        const float p_forward = p.dot(dir_forward);
        const float p_lateral = p.dot(dir_lateral);

        // Skip points behind us or outside the swept lateral band
        if (p_forward < 0.0f) continue;
        if (p_lateral < lateral_min || p_lateral > lateral_max) continue;

        // Obstacle limits the free path length
        move->fpl = std::min(move->fpl, p_forward - front_dist);
    }

    // ---- Compute clearance: min distance from obstacles to robot body within traversable region ----
    move->clearance = nav_params.max_clearance;
    for (const Eigen::Vector2f& p : *point_cloud) {
        if (robot_body.contains(p)) continue;  // skip points on robot itself

        // Transform p to path-aligned frame
        const float p_forward = p.dot(dir_forward);
        const float p_lateral = p.dot(dir_lateral);

        // Skip points behind us, outside the traversable region in forward direction, or outside the swept lateral band
        if (p_forward < 0.0f || p_forward > move->fpl + front_dist) continue;
        if (p_lateral < lateral_min || p_lateral > lateral_max) continue;

        // Distance from point to robot body edge (no margin)
        const float dist_to_body =
            (p_lateral > body_lateral_max) ? (p_lateral - body_lateral_max) : (body_lateral_min - p_lateral);
        move->clearance = std::min(move->clearance, dist_to_body);
    }

    // ---- Finalize outputs ----
    move->clearance = std::max(0.0f, move->clearance);
    move->fpl = std::max(0.0f, move->fpl);
    move->length = std::min(move->length, move->fpl);

    // Safety check: if can't stop before obstacle, mark path as unusable
    const float vel_forward = std::max(0.0f, vel.dot(move->direction));
    const float stopping_dist = (vel_forward * vel_forward) / (2.0f * nav_params.linear_limits.max_deceleration);
    if (move->fpl < stopping_dist) {
        move->length = 0.0f;
    }
}

}  // namespace motion_primitives