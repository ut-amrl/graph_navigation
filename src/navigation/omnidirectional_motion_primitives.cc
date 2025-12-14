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
    // Projection of goal onto this direction
    const float distance_to_goal_along_direction = local_target.dot(move->direction);

    // Desired distance: how far we WANT to travel (limited by goal and max path length)
    // Directions pointing away from goal get length=0
    const float desired_dist =
        (distance_to_goal_along_direction > 0.0f) ? std::min(nav_params.max_free_path_length, distance_to_goal_along_direction) : 0.0f;

    // Initialize both to desired; CheckObstacles will limit fpl and finalize length
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
    // Path-aligned coordinate system
    const Eigen::Vector2f dir_forward = move->direction;                      // unit vector along motion
    const Eigen::Vector2f dir_lateral(-dir_forward.y(), dir_forward.x());     // unit vector perpendicular (CCW 90°)

    // Robot half-dimensions in base_link frame
    const float half_length = 0.5f * nav_params.robot_length;  // along robot x-axis
    const float half_width = 0.5f * nav_params.robot_width;    // along robot y-axis

    // Robot center offset in path-aligned frame
    const Eigen::Vector2f robot_center(nav_params.base_link_offset_x, nav_params.base_link_offset_y);
    const float center_forward = robot_center.dot(dir_forward);   // center offset along motion
    const float center_lateral = robot_center.dot(dir_lateral);   // center offset perpendicular

    // Direction-dependent robot extents (support function of rectangle)
    // For motion direction u, extent = half_length*|u.x| + half_width*|u.y|
    const float abs_dir_x = std::fabs(dir_forward.x());
    const float abs_dir_y = std::fabs(dir_forward.y());
    const float robot_forward_extent = half_length * abs_dir_x + half_width * abs_dir_y;
    const float robot_lateral_extent = half_length * abs_dir_y + half_width * abs_dir_x;

    // Swept area bounds WITH margin (for collision detection)
    const float front_clearance_dist = center_forward + robot_forward_extent + nav_params.obstacle_margin;
    const float swept_lateral_half_width = robot_lateral_extent + nav_params.obstacle_margin;

    // Body bounds WITHOUT margin (for filtering points on robot itself)
    const float body_rear = center_forward - robot_forward_extent;
    const float body_front = center_forward + robot_forward_extent;
    const float body_lateral_half_width = robot_lateral_extent;

    // ---- Compute FPL: find first obstacle that blocks the path ----
    for (const Eigen::Vector2f& p : *point_cloud) {
        const float dist_forward = p.dot(dir_forward);    // point's position along motion
        const float dist_lateral = p.dot(dir_lateral);    // point's lateral position

        // Skip points inside current robot body (no margin)
        if (dist_forward > body_rear && dist_forward < body_front &&
            std::fabs(dist_lateral - center_lateral) < body_lateral_half_width) {
            continue;
        }
        // Skip points outside swept band or behind us
        if (dist_forward < 0.0f || std::fabs(dist_lateral - center_lateral) > swept_lateral_half_width) {
            continue;
        }

        // Obstacle limits free path length
        move->fpl = std::min(move->fpl, dist_forward - front_clearance_dist);
    }

    // ---- Compute clearance: min distance to swept boundary within traversable segment ----
    move->clearance = nav_params.max_clearance;
    for (const Eigen::Vector2f& p : *point_cloud) {
        const float dist_forward = p.dot(dir_forward);
        if (dist_forward < 0.0f || (dist_forward - front_clearance_dist) > move->fpl) continue;

        const float dist_lateral = p.dot(dir_lateral);

        // Skip points inside current robot body (no margin)
        if (dist_forward > body_rear && dist_forward < body_front &&
            std::fabs(dist_lateral - center_lateral) < body_lateral_half_width) {
            continue;
        }
        if (std::fabs(dist_lateral - center_lateral) > swept_lateral_half_width) continue;

        // Distance from point to lateral boundary of swept rectangle
        const float lateral_offset = std::fabs(dist_lateral - center_lateral);
        move->clearance = std::min(move->clearance, std::fabs(lateral_offset - swept_lateral_half_width));
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