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
\file    ackermann_motion_primitives.h
\brief   Motion primitives for Ackermann steering platforms.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================

#include <math.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "motion_primitives.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"

using std::min;
using std::max;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using namespace math_util;

namespace {
// Epsilon value for handling limited numerical precision.
const float kEpsilon = 1e-5;
}  // namespace

namespace motion_primitives {

AckermannSampler::AckermannSampler() {
  angular_limits = MotionLimits(5.0 * M_PI, 5.0 * M_PI, 3.0 * M_PI);
  linear_limits = MotionLimits(5.0, 5.0, 1.0);
  dt = 0.05;
  max_curvature = 1.5;
  max_free_path_length = 6;
  robot_length = 0.5;
  robot_width = 0.5;
  base_link_offset = 0;
  obstacle_margin = 0.2;
  max_clearance = 0.5;
}

void AckermannSampler::SetMaxPathLength(ConstantCurvatureArc* path_ptr) {
  ConstantCurvatureArc& path = *path_ptr;
  if (fabs(path.curvature) < kEpsilon) {
    path.length = min(max_free_path_length, local_target.x());
    return;
  } 
  const float turn_radius = 1.0f / path.curvature;
  const Vector2f turn_center(0, turn_radius);
  const Vector2f target_radial = local_target - turn_center;
  const Vector2f middle_radial =
      fabs(turn_radius) * target_radial.normalized();
  const float middle_angle =
      atan2(fabs(middle_radial.x()), fabs(middle_radial.y()));
  path.length = min<float>({
      max_free_path_length, 
      middle_angle * float(abs(turn_radius)),
      float(fabs(turn_radius) * M_PI_2)});
}

vector<shared_ptr<PathRolloutBase>> AckermannSampler::GetSamples(int n) {
  vector<shared_ptr<PathRolloutBase>> samples;
  if (false) {
    samples = {
      shared_ptr<PathRolloutBase>(new ConstantCurvatureArc(-0.1)),
      shared_ptr<PathRolloutBase>(new ConstantCurvatureArc(0)),
      shared_ptr<PathRolloutBase>(new ConstantCurvatureArc(0.1)),
    };
    return samples;
  }
  const float max_domega = dt * angular_limits.max_acceleration;
  const float max_dv = dt * linear_limits.max_acceleration;
  const float robot_speed = fabs(vel.x());
  float c_min = -max_curvature;
  float c_max = max_curvature;
  if (robot_speed > max_dv + kEpsilon) {
    c_min = max<float>(
        c_min, (ang_vel - max_domega) / (robot_speed - max_dv));
    c_max = min<float>(
        c_max, (ang_vel + max_domega) / (robot_speed - max_dv));
  }
  const float dc = (c_max - c_min) / static_cast<float>(n - 1);
  // printf("Options: %6.2f : %6.2f : %6.2f\n", c_min, dc, c_max);
  if (false) {
    for (float c = c_min; c <= c_max; c+= dc) {
      auto sample = new ConstantCurvatureArc(c);
      SetMaxPathLength(sample);
      CheckObstacles(sample);
      sample->angular_length = fabs(sample->length * c);
      samples.push_back(shared_ptr<PathRolloutBase>(sample));
    }
  } else {
    const float dc = (2.0f * max_curvature) / static_cast<float>(n - 1);
    for (float c = -max_curvature; c <= max_curvature; c+= dc) {
      auto sample = new ConstantCurvatureArc(c);
      SetMaxPathLength(sample);
      CheckObstacles(sample);
      sample->angular_length = fabs(sample->length * c);
      samples.push_back(shared_ptr<PathRolloutBase>(sample));
    }
  }
  
  return samples;
}

void AckermannSampler::CheckObstacles(ConstantCurvatureArc* path_ptr) {
  ConstantCurvatureArc& path = *path_ptr;
  // How much the robot's body extends in front of its base link frame.
  const float l = 0.5 * robot_length - base_link_offset + obstacle_margin;
  // The robot's half-width.
  const float w = 0.5 * robot_width + obstacle_margin;
  if (fabs(path.curvature) < kEpsilon) {
    for (const Vector2f& p : point_cloud) {
      if (fabs(p.y()) > w || p.x() < 0.0f) continue;
      path.length = min(path.length, p.x() - l);
    }
    path.clearance = max_clearance;
    for (const Vector2f& p : point_cloud) {
      if (p.x() - l > path.length || p.x() < 0.0) continue;
      path.clearance = min<float>(path.clearance, fabs(fabs(p.y() - w)));
    }
    path.clearance = max(0.0f, path.clearance);
    path.length = max(0.0f, path.length);
    return;
  }
  const float path_radius = 1.0 / path.curvature;
  const Vector2f c(0, path_radius );
  const float s = ((path_radius > 0.0) ? 1.0 : -1.0);
  const Vector2f inner_front_corner(l, s * w);
  const Vector2f outer_front_corner(l, -s * w);
  const float r1 = max<float>(0.0f, fabs(path_radius) - w);
  const float r1_sq = Sq(r1);
  const float r2_sq = (inner_front_corner - c).squaredNorm();
  const float r3_sq = (outer_front_corner - c).squaredNorm();
  float angle_min = M_PI;
  path.obstruction = Vector2f(-max_free_path_length, 0);
  // printf("%7.3f %7.3f %7.3f %7.3f\n", 
  //     path.curvature, sqrt(r1_sq), sqrt(r2_sq), sqrt(r3_sq));
  using std::isfinite;
  for (const Vector2f& p : point_cloud) {
    if (!isfinite(p.x()) || !isfinite(p.y()) || p.x() < 0.0f) continue;
    const float r_sq = (p - c).squaredNorm();
    // printf("c:%.2f r:%.3f r1:%.3f r2:%.3f r3:%.3f\n",
    //        path.curvature, sqrt(r_sq), r1, sqrt(r2_sq), sqrt(r3_sq));
    if (r_sq < r1_sq || r_sq > r3_sq) continue;
    const float r = sqrt(r_sq);
    const float theta = ((path.curvature > 0.0f) ?
      atan2<float>(p.x(), path_radius - p.y()) :
      atan2<float>(p.x(), p.y() - path_radius));
    float alpha;
    if (r_sq < r2_sq) {
      // Point will hit the side of the robot first.
      alpha = acosf((fabs(path_radius) - w) / r);
    } else {
      // Point will hit the front of the robot first.
      alpha = asinf(l / r);
    }
    // if (theta < 0.0f) continue;
    CHECK(std::isfinite(r));
    CHECK(std::isfinite(path_radius));
    CHECK(std::isfinite(alpha));
    CHECK(std::isfinite(theta));
    const float path_length =
        max<float>(0.0f, fabs(path_radius) * (theta - alpha));
    if (path.length > path_length) {
      path.length = path_length;
      path.obstruction = p;
      angle_min = theta;
    }
  }
  path.length = max(0.0f, path.length);
  angle_min = min<float>(angle_min, path.length * fabs(path.curvature));
  path.clearance = max_clearance;

  for (const Vector2f& p : point_cloud) {
    const float theta = ((path.curvature > 0.0f) ?
        atan2<float>(p.x(), path_radius - p.y()) :
        atan2<float>(p.x(), p.y() - path_radius));
    if (theta < angle_min && theta > 0.0) {
      const float r = (p - c).norm();
      const float current_clearance = fabs(r - fabs(path_radius));
      if (path.clearance > current_clearance) {
        path.clearance = current_clearance;
      }
    }
  }
  path.clearance = max(0.0f, path.clearance);
}

}  // namespace motion_primitives