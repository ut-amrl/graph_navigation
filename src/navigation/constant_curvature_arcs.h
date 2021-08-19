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
\file    constant_curvature_arcs.h
\brief   Constant curvature arc path rollouts.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================


#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"

#include "motion_primitives.h"

#ifndef CONSTANT_CURVATURE_ARCS_H
#define CONSTANT_CURVATURE_ARCS_H

namespace motion_primitives {

struct ConstantCurvatureArc : PathRolloutBase {
  // Length of the path rollout.
  float Length() const override;

  // FPL
  float FPL() const override;

  // Angular distance traversed.
  float AngularLength() const override;

  // Clearance along path.
  float Clearance() const override;

  // Default constructor.
  ConstantCurvatureArc() : curvature(0), length(0), angular_length(0) {}

  // Explicit constructor from curvature.
  explicit ConstantCurvatureArc(float curvature) : 
      curvature(curvature), length(0), angular_length(0) {}

  // The pose of the robot at the end of the path rollout.
  pose_2d::Pose2Df EndPoint() const override;

  pose_2d::Pose2Df GetIntermediateState(float f) const override;

  // The pose of the robot at the end of the path rollout.
  void GetControls(const navigation::MotionLimits& linear_limits,
                   const navigation::MotionLimits& angular_limits,
                   const float dt,
                   const Eigen::Vector2f& linear_vel,
                   const float angular_vel,
                   Eigen::Vector2f& vel_cmd,
                   float& ang_vel_cmd) const override;

  float curvature;
  float length;
  float fpl;
  float angular_length;
  float clearance;
  Eigen::Vector2f obstruction;
};

}  // namespace motion_primitives

#endif  // CONSTANT_CURVATURE_ARCS_H