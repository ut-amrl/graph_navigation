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

// Omnidirectional motion primitive - straight line movement in x-y plane
struct OmnidirectionalMove : PathRolloutBase {
    ~OmnidirectionalMove() = default;

    float Length() const override;
    float FPL() const override;
    float AngularLength() const override;
    float Clearance() const override;
    pose_2d::Pose2Df EndPoint() const override;
    pose_2d::Pose2Df GetIntermediateState(float f) const override;
    void GetControls(const navigation::MotionLimits& linear_limits,
                     const navigation::MotionLimits& angular_limits,
                     const float dt,
                     const Eigen::Vector2f& linear_vel,
                     const float angular_vel,
                     Eigen::Vector2f& vel_cmd,
                     float& ang_vel_cmd) const override;

    // Default constructor
    OmnidirectionalMove() : direction(0, 0), length(0), fpl(0), clearance(0) {}

    // Constructor for straight line movement
    OmnidirectionalMove(const Eigen::Vector2f& dir, float len) : direction(dir.normalized()), length(len), fpl(len), clearance(0) {}

    Eigen::Vector2f direction;    // Unit vector for movement direction
    float length;                 // Distance to travel
    float fpl;                    // Free path length
    float clearance;              // Minimum clearance to obstacles
    Eigen::Vector2f obstruction;  // Location of closest obstacle
};

// Omnidirectional path rollout sampler
struct OmniSampler : PathRolloutSamplerBase {
    std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
    OmniSampler();

    void CheckObstacles(OmnidirectionalMove* move);
    void SetMaxPathLength(OmnidirectionalMove* move);
};

}  // namespace motion_primitives

#endif  // OMNIDIRECTIONAL_MOTION_PRIMITIVES_H