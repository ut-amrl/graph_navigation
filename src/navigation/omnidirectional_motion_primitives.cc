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

// OmnidirectionalMove implementation
float OmnidirectionalMove::Length() const {
    return length;
}

float OmnidirectionalMove::FPL() const {
    return fpl;
}

float OmnidirectionalMove::AngularLength() const {
    return 0.0f;  // No angular movement for straight line motion
}

float OmnidirectionalMove::Clearance() const {
    return clearance;
}

void OmnidirectionalMove::GetControls(const navigation::MotionLimits& linear_limits,
                                      const navigation::MotionLimits& angular_limits,
                                      const float dt,
                                      const Vector2f& vel,
                                      const float ang_vel,
                                      Vector2f& vel_cmd,
                                      float& ang_vel_cmd) const {
    // Calculate velocity component along the path direction
    const float velocity_along_path = vel.dot(direction);

    // Use 1D Time Optimal Control: accelerate/decelerate to reach target distance
    const float speed = Run1DTimeOptimalControl(
        linear_limits, 0, velocity_along_path, length, 0, dt);

    // Command velocity in the direction of motion (2D velocity vector)
    vel_cmd = speed * direction;

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
        ang_vel_cmd = s * Run1DTimeOptimalControl(
                              angular_limits, 0, s * ang_vel, s * dTheta, 0, dt);
    }
}

Pose2Df OmnidirectionalMove::GetIntermediateState(float f) const {
    // Position: straight line movement
    // Orientation: gradually rotate to face the direction of motion
    const float target_angle = atan2(direction.y(), direction.x());
    return Pose2Df(f * target_angle, f * length * direction);
}

Pose2Df OmnidirectionalMove::EndPoint() const {
    return GetIntermediateState(1.0);
}

// OmniSampler implementation
OmniSampler::OmniSampler() {
}

void OmniSampler::SetMaxPathLength(OmnidirectionalMove* move) {
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
    vector<shared_ptr<PathRolloutBase>> samples;

    // Sample uniformly in full 360° circle, just like Ackermann samples all curvatures
    // The FOV check in Run() will handle turning in place if target is outside FOV
    for (int i = 0; i < n; ++i) {
        const float angle = (2.0f * M_PI * i) / n;
        const Vector2f direction(cos(angle), sin(angle));

        auto move = new OmnidirectionalMove(direction, 0);
        SetMaxPathLength(move);
        CheckObstacles(move);
        samples.push_back(shared_ptr<PathRolloutBase>(move));
    }

    return samples;
}

void OmniSampler::CheckObstacles(OmnidirectionalMove* move) {
    // Follow exact same logic as Ackermann CheckObstacles for straight lines
    const float l = 0.5 * nav_params.robot_length - nav_params.base_link_offset + nav_params.obstacle_margin;
    const float w = 0.5 * nav_params.robot_width + nav_params.obstacle_margin;
    // The x-coordinate of the rear margin (behind base_link)
    const float x_min = -0.5 * nav_params.robot_length + nav_params.base_link_offset - nav_params.obstacle_margin;

    // Replicate Ackermann straight-line obstacle checking logic
    for (const Vector2f& p : point_cloud) {
        // Transform point to path-aligned coordinate system
        const float along_path = p.dot(move->direction);  // equivalent to p.x() in Ackermann
        const Vector2f perpendicular_vec = p - along_path * move->direction;
        const float lateral_distance = perpendicular_vec.norm();  // equivalent to fabs(p.y()) in Ackermann

        // Skip points inside robot body (between x_min and l, within width w)
        if (along_path > x_min && along_path < l && lateral_distance < w) {
            continue;  // Point is within robot body boundary
        }

        if (lateral_distance > w || along_path < 0.0f) continue;
        move->fpl = min(move->fpl, along_path - l);
    }

    move->clearance = nav_params.max_clearance;
    for (const Vector2f& p : point_cloud) {
        const float along_path = p.dot(move->direction);
        const Vector2f perpendicular_vec = p - along_path * move->direction;
        const float lateral_distance = perpendicular_vec.norm();

        // Skip points inside robot body
        if (along_path > x_min && along_path < l && lateral_distance < w) {
            continue;
        }

        if (along_path - l > move->fpl || along_path < 0.0) continue;
        move->clearance = min<float>(move->clearance, fabs(lateral_distance - w));
    }
    move->clearance = max(0.0f, move->clearance);
    move->fpl = max(0.0f, move->fpl);
    move->length = min(move->fpl, move->length);

    const float stopping_dist =
        vel.squaredNorm() / (2.0 * nav_params.linear_limits.max_deceleration);
    if (move->fpl < stopping_dist) {
        move->length = 0;
    }
}

}  // namespace motion_primitives