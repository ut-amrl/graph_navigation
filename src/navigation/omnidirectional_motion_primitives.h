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
\file    omnidirectional_motion_primitives.h
\brief   Omnidirectional motion primitives header.
\author  Sadanand Modak (C) 2025
*/
//========================================================================

#ifndef OMNIDIRECTIONAL_MOTION_PRIMITIVES_H
#define OMNIDIRECTIONAL_MOTION_PRIMITIVES_H

#include "motion_primitives.h"

namespace motion_primitives {

// Rectangle with center offset from base_link, sides aligned with base_link axes.
// ASSUMPTION: base_link is inside rectangle still, ie, offset not larger than half_x and half_y.
struct OffsetRect {
    Eigen::Vector2f center;  // center position in base_link frame
    float half_x, half_y;    // half-extents measured from geometric center along base_link X and Y

    // Check if point (in base_link frame) is inside this rectangle
    bool contains(const Eigen::Vector2f& p) const {
        const Eigen::Vector2f rel = p - center;
        return std::fabs(rel.x()) < half_x && std::fabs(rel.y()) < half_y;
    }

    // Extent from geometric center in direction u: half_x*|u.x| + half_y*|u.y|, which measures
    // the furthest projection of the rectangle in the direction u.
    // Note: this is symmetric, i.e. extent(u) = extent(-u).
    float extent(const Eigen::Vector2f& u) const {
        return half_x * std::fabs(u.x()) + half_y * std::fabs(u.y());
    }

    // Support function from BASE_LINK origin in direction u (unit vector in base_link frame)
    // h(u) = (center · u) + extent(u) = distance from base_link to furthest edge in direction u
    // Note: this is NOT symmetric, i.e. support(u) != support(-u).
    float support(const Eigen::Vector2f& u) const {
        return center.dot(u) + extent(u);
    }
};

// Omnidirectional motion primitive - straight line movement in x-y plane
struct OmnidirectionalMovePath : PathRolloutBase {
    ~OmnidirectionalMovePath() = default;

    float Length() const override;
    float FPL() const override;
    float AngularLength() const override;
    float Clearance() const override;
    pose_2d::Pose2Df EndPoint() const override;
    pose_2d::Pose2Df GetIntermediateState(float f) const override;
    void GetControls(const navigation::MotionLimits& linear_limits, const navigation::MotionLimits& angular_limits,
                     const float dt, const Eigen::Vector2f& linear_vel, const float angular_vel,
                     Eigen::Vector2f& vel_cmd, float& ang_vel_cmd) const override;

    // Default constructor
    OmnidirectionalMovePath() : direction(0, 0), length(0), fpl(0), clearance(0) {}

    // Constructor for straight line movement
    OmnidirectionalMovePath(const Eigen::Vector2f& dir, float len)
        : direction(dir.normalized()), length(len), fpl(len), clearance(0) {}

    Eigen::Vector2f direction;    // Unit vector for movement direction
    float length;                 // Actual traversable distance = min(desired_to_goal, fpl)
    float fpl;                    // Free path length = obstacle-free distance ahead (up to sensor range)
    float clearance;              // Min lateral distance from obstacles to robot body along path
};

// Omnidirectional path rollout sampler
struct OmniSampler : PathRolloutSamplerBase {
    std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
    OmniSampler();

    void CheckObstacles(OmnidirectionalMovePath* move);
    void SetMaxPathLength(OmnidirectionalMovePath* move);
};

}  // namespace motion_primitives

#endif  // OMNIDIRECTIONAL_MOTION_PRIMITIVES_H