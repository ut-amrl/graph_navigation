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

#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "config_reader/config_reader.h"
#include "omnidirectional_motion_primitives.h"
#include "motion_primitives.h"

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
    const float velocity_along_path = vel.dot(direction);

    // Use 1D Time Optimal Control: accelerate/decelerate to reach target distance
    const float speed = Run1DTimeOptimalControl(linear_limits, 0, velocity_along_path, length, 0, dt);

    // Command velocity in the direction of motion (2D velocity vector)
    vel_cmd = speed * direction;

    if (do_ang_toc) {
        // Simultaneously apply 1D TOC for angular rotation to face the direction of motion
        // Target angle: direction of motion
        const float target_angle = atan2(direction.y(), direction.x());
        // Current angle is 0 in robot frame, so angle difference = target_angle
        const float dTheta = AngleMod(target_angle);

        // Use 1D TOC with sign handling
        const float s = Sign(dTheta);
        if (ang_vel * dTheta < 0.0f) {
            // Turning the wrong way - decelerate first
            const float dv = dt * angular_limits.max_acceleration;
            if (fabs(ang_vel) < dv) {
                ang_vel_cmd = 0;
            } else {
                ang_vel_cmd = ang_vel - Sign(ang_vel) * dv;
            }
        } else {
            // Apply 1D TOC to reach target orientation
            ang_vel_cmd = s * Run1DTimeOptimalControl(angular_limits, 0, s * ang_vel, s * dTheta, 0, dt);
        }
    } else {
        // No rotation during straight-line motion
        ang_vel_cmd = 0;
    }
}

Pose2Df OmnidirectionalMovePath::GetIntermediateState(float f) const {
    if (do_ang_toc) {
        // Position: straight line movement
        // Orientation: gradually rotate to face the direction of motion
        const float target_angle = atan2(direction.y(), direction.x());
        return Pose2Df(f * target_angle, f * length * direction);
    } else {
        // Straight line movement only
        return Pose2Df(0, f * length * direction);
    }
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
    // Distance to goal along this direction
    const float distance_to_goal_along_direction = local_target.dot(move->direction);

    // Limit by max free path length and distance to goal
    // Only go forward (positive direction)
    if (distance_to_goal_along_direction > 0.0f) {
        move->length = min(nav_params.max_free_path_length, distance_to_goal_along_direction);
    } else {
        move->length = 0.0f;  // Don't move backward
    }
    move->fpl = move->length;

    // Ensure we can stop safely
    const float stopping_dist = vel.squaredNorm() / (2.0 * nav_params.linear_limits.max_deceleration);
    move->length = max(move->length, stopping_dist);
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

    const bool enable_ang_toc = nav_params.do_ang_toc && enable_angular_toc_runtime_;

#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < n; ++i) {
        auto move = std::make_shared<OmnidirectionalMovePath>(unit_dirs[i], 0.0f, enable_ang_toc);
        SetMaxPathLength(move.get());
        CheckObstacles(move.get());
        samples[i] = std::static_pointer_cast<PathRolloutBase>(move);
    }
    return samples;
}

void OmniSampler::CheckObstacles(OmnidirectionalMovePath* move) {
    // Same logic, cheaper math.
    const float l = 0.5f * nav_params.robot_length - nav_params.base_link_offset + nav_params.obstacle_margin;
    const float w = 0.5f * nav_params.robot_width + nav_params.obstacle_margin;
    const float w2 = w * w;

    // Body box (for filtering points on the robot itself, no margin).
    const float l_body = 0.5f * nav_params.robot_length - nav_params.base_link_offset;
    const float w_body = 0.5f * nav_params.robot_width;
    const float w_body2 = w_body * w_body;
    const float x_min_body = -0.5f * nav_params.robot_length + nav_params.base_link_offset;

    // Pass 1: determine FPL (no sqrt)
    for (const Vector2f& p : *point_cloud) {
        const float along = p.dot(move->direction);        // projection onto unit direction
        const float r2 = p.squaredNorm() - along * along;  // lateral distance^2

        // Skip points inside robot body (without obstacle margin)
        if (along > x_min_body && along < l_body && r2 < w_body2) continue;

        if (along < 0.0f || r2 > w2) continue;  // outside swept rect
        move->fpl = std::min(move->fpl, along - l);

        // NOTE: do not break—must still compute clearance below against the final FPL.
    }

    // Pass 2: clearance within [0, fpl] (sqrt only when needed)
    move->clearance = nav_params.max_clearance;
    for (const Vector2f& p : *point_cloud) {
        const float along = p.dot(move->direction);
        if (along < 0.0f || (along - l) > move->fpl) continue;

        const float r2 = p.squaredNorm() - along * along;

        // Skip points inside robot body (without obstacle margin)
        if (along > x_min_body && along < l_body && r2 < w_body2) continue;

        if (r2 > w2) continue;

        const float lateral = std::sqrt(std::max(0.0f, r2));
        move->clearance = std::min<float>(move->clearance, std::fabs(lateral - w));
    }

    move->clearance = std::max(0.0f, move->clearance);
    move->fpl = std::max(0.0f, move->fpl);
    move->length = std::min(move->fpl, move->length);

    const float stopping_dist = vel.squaredNorm() / (2.0f * nav_params.linear_limits.max_deceleration);
    if (move->fpl < stopping_dist) {
        move->length = 0.0f;
    }
}

}  // namespace motion_primitives