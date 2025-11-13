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

        // If already close enough in angle, stop rotating
        if (fabs(dTheta) < target_angle_tolerance) {
            ang_vel_cmd = 0;
        } else if (ang_vel * dTheta < 0.0f) {
            // Turning the wrong way - decelerate first using max_deceleration
            const float dv = dt * angular_limits.max_deceleration;
            if (fabs(ang_vel) < dv) {
                ang_vel_cmd = 0;
            } else {
                ang_vel_cmd = ang_vel - Sign(ang_vel) * dv;
            }
        } else {
            // Early-brake guard: if remaining angle is less than stopping angle, brake now
            const float omega = ang_vel;
            const float stop_angle = (omega * omega) / (2.0f * angular_limits.max_deceleration);
            if (stop_angle >= fabs(dTheta)) {
                const float dv = dt * angular_limits.max_deceleration;
                ang_vel_cmd = (fabs(omega) <= dv) ? 0.0f : (omega - Sign(omega) * dv);
            } else {
                // Apply 1D TOC to reach target orientation
                ang_vel_cmd = s * Run1DTimeOptimalControl(angular_limits, 0, s * omega, s * dTheta, 0, dt);
            }
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

    if (allow_full_360_runtime_) {
        // NUDGE: permit moving in any direction; cap by max free-path length.
        move->length = nav_params.max_free_path_length;
    } else {
        // Default: only move if the step reduces distance to the local target.
        if (distance_to_goal_along_direction > 0.0f) {
            move->length = min(nav_params.max_free_path_length, distance_to_goal_along_direction);
        } else {
            move->length = 0.0f;  // Don't move backward
        }
    }
    move->fpl = move->length;

    // Ensure we can stop safely (use forward component of current vel along this sample)
    const float v_along = std::max(0.0f, vel.dot(move->direction));
    const float stopping_dist = (v_along * v_along) / (2.0f * nav_params.linear_limits.max_deceleration);
    move->length = std::max(move->length, stopping_dist);
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
        auto move = std::make_shared<OmnidirectionalMovePath>(unit_dirs[i], 0.0f, enable_ang_toc,
                                                              nav_params.target_angle_tolerance);
        SetMaxPathLength(move.get());
        CheckObstacles(move.get());
        samples[i] = std::static_pointer_cast<PathRolloutBase>(move);
    }
    return samples;
}

void OmniSampler::CheckObstacles(OmnidirectionalMovePath* move) {
    // Path-aligned basis: u = direction of motion, v = its left-normal.
    const Eigen::Vector2f u = move->direction;  // unit
    const Eigen::Vector2f v(-u.y(), u.x());     // unit (CCW 90°)

    // Robot half-dimensions.
    const float hl = 0.5f * nav_params.robot_length;
    const float hw = 0.5f * nav_params.robot_width;

    // Center of the rectangle (geometric center) in base_link frame.
    const Eigen::Vector2f c(nav_params.base_link_offset_x, nav_params.base_link_offset_y);
    const float cu = c.dot(u);  // center offset along the path direction
    const float cv = c.dot(v);  // center offset lateral to the path direction

    // Front "overhang" from base_link origin to the foremost point (incl. margin) along u.
    const float l_front = hl + cu + nav_params.obstacle_margin;

    // Lateral half-extent (incl. margin) around the center line in the path frame.
    const float w_lat = hw + nav_params.obstacle_margin;

    // Body extents for filtering points on the robot itself (no margin).
    const float x_min_body = cu - hl;
    const float x_max_body = cu + hl;
    const float w_body_lat = hw;

    // ---- Pass 1: compute FPL (no sqrt needed) ----
    for (const Eigen::Vector2f& p : *point_cloud) {
        const float along = p.dot(u);  // position along motion
        const float lat = p.dot(v);    // lateral position

        // Skip points inside current robot body (no margin).
        if (along > x_min_body && along < x_max_body && std::fabs(lat - cv) < w_body_lat) {
            continue;
        }
        // Outside swept lateral band or behind us.
        if (along < 0.0f || std::fabs(lat - cv) > w_lat) {
            continue;
        }

        // Candidate obstacle limits free path length.
        move->fpl = std::min(move->fpl, along - l_front);
    }

    // ---- Pass 2: clearance within [0, fpl] (sqrt still not needed) ----
    move->clearance = nav_params.max_clearance;
    for (const Eigen::Vector2f& p : *point_cloud) {
        const float along = p.dot(u);
        if (along < 0.0f || (along - l_front) > move->fpl) continue;

        const float lat = p.dot(v);

        // Skip points inside current robot body (no margin).
        if (along > x_min_body && along < x_max_body && std::fabs(lat - cv) < w_body_lat) {
            continue;
        }
        if (std::fabs(lat - cv) > w_lat) continue;

        // Distance to the lateral boundary of the swept rectangle.
        const float lateral = std::fabs(lat - cv);
        move->clearance = std::min(move->clearance, std::fabs(lateral - w_lat));
    }

    // Post-conditions
    move->clearance = std::max(0.0f, move->clearance);
    move->fpl = std::max(0.0f, move->fpl);
    move->length = std::min(move->length, move->fpl);

    const float v_along0 = std::max(0.0f, vel.dot(move->direction));
    const float stopping_dist = (v_along0 * v_along0) / (2.0f * nav_params.linear_limits.max_deceleration);
    if (move->fpl < stopping_dist) {
        move->length = 0.0f;
    }
}

}  // namespace motion_primitives