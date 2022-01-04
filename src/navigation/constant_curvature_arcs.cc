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
\file    constant_curvature_arcs.cc
\brief   Constant curvature arc path rollouts.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================

#include <float.h>

#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "math/math_util.h"
#include "eigen3/Eigen/Dense"

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "constant_curvature_arcs.h"

using Eigen::Vector2f;
using pose_2d::Pose2Df;
using namespace math_util;
using navigation::MotionLimits;

namespace motion_primitives {

float ConstantCurvatureArc::Length() const {
  return length;
}

float ConstantCurvatureArc::FPL() const {
  return fpl;
}

float ConstantCurvatureArc::AngularLength() const {
  return angular_length;
}

float ConstantCurvatureArc::Clearance() const {
  return clearance;
}

void ConstantCurvatureArc::GetControls(const MotionLimits& linear_limits,
                                       const MotionLimits& angular_limits,
                                       const float dt,
                                       const Vector2f& vel,
                                       const float ang_vel,
                                       Vector2f& vel_cmd,
                                       float& ang_vel_cmd) const {
  vel_cmd.y() = 0;
  vel_cmd.x() = Run1DTimeOptimalControl(
      linear_limits, 0, vel.x(), length, 0, dt);
  ang_vel_cmd = vel_cmd.x() * curvature;
}

Pose2Df ConstantCurvatureArc::GetIntermediateState(float f) const {
  const float a = Sign(curvature) * angular_length * f;
  if (length == 0) {
    // Pure rotational motion
    return Pose2Df(a, Vector2f(0, 0));
  }
  if (fabs(curvature) < FLT_MIN) {
    // Straight-line motion
    return Pose2Df(0, Vector2f(length, 0));
  }
  const float r = 1.0 / curvature;
  return Pose2Df(a, r * Vector2f(sin(a), 1.0 - cos(a)));
}

Pose2Df ConstantCurvatureArc::EndPoint() const {
  const float a = Sign(curvature) * angular_length;
  if (length == 0) {
    // Pure rotational motion.
    return Pose2Df(a, Vector2f(0, 0));
  }
  if (fabs(curvature) < FLT_MIN) {
    // Straight-line motion
    return Pose2Df(0, Vector2f(length, 0));
  }
  const float r = 1.0 / curvature;
  return Pose2Df(a, r * Vector2f(sin(a), 1.0 - cos(a)));
}

}  // namespace motion_primitives
