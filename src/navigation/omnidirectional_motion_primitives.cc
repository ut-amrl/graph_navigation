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

using std::min;
using std::max;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using namespace math_util;

CONFIG_FLOAT(max_speed, "OmniSampler.max_speed");
CONFIG_FLOAT(max_angular_speed, "OmniSampler.max_angular_speed");
CONFIG_INT(num_directions, "OmniSampler.num_directions");

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
  // Calculate velocity component along the path direction (like vel.x() for Ackermann)
  const float velocity_along_path = vel.dot(direction);
  
  // Use 1D Time Optimal Control exactly like Ackermann: from 0 to length, ending at 0 velocity
  const float speed = Run1DTimeOptimalControl(
      linear_limits, 0, velocity_along_path, length, 0, dt);
  vel_cmd = speed * direction;
  ang_vel_cmd = 0;
}

Pose2Df OmnidirectionalMove::GetIntermediateState(float f) const {
  // Straight line movement only
  return Pose2Df(0, f * length * direction);
}

Pose2Df OmnidirectionalMove::EndPoint() const {
  return GetIntermediateState(1.0);
}

// OmniSampler implementation
OmniSampler::OmniSampler() {
}

void OmniSampler::SetMaxPathLength(OmnidirectionalMove* move) {
  // Follow exact same logic as Ackermann SetMaxPathLength for straight lines
  // Distance to goal along this direction (equivalent to local_target.x() for Ackermann)
  const float distance_to_goal_along_direction = local_target.dot(move->direction);
  
  // Ackermann for straight lines does this and returns immediately (no stopping distance applied):
  move->length = min(nav_params.max_free_path_length, distance_to_goal_along_direction);
  move->fpl = move->length;
  // Note: Ackermann doesn't apply stopping distance constraint for straight lines, only for curves
}

vector<shared_ptr<PathRolloutBase>> OmniSampler::GetSamples(int n) {
  vector<shared_ptr<PathRolloutBase>> samples;
  
  // Generate straight line movements in different directions only
  // Turn-in-place is handled by the navigation state machine
  for (int i = 0; i < n; ++i) {
    const float angle = 2.0 * M_PI * i / n;
    const Vector2f direction(cos(angle), sin(angle));
    
    auto move = new OmnidirectionalMove(direction, 0);  // Length will be set by SetMaxPathLength
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
  
  // Replicate Ackermann straight-line obstacle checking logic
  for (const Vector2f& p : point_cloud) {
    // Transform point to path-aligned coordinate system
    const float along_path = p.dot(move->direction);  // equivalent to p.x() in Ackermann
    const Vector2f perpendicular_vec = p - along_path * move->direction;
    const float lateral_distance = perpendicular_vec.norm();  // equivalent to fabs(p.y()) in Ackermann
    
    if (lateral_distance > w || along_path < 0.0f) continue;
    move->fpl = min(move->fpl, along_path - l);
  }
  
  move->clearance = nav_params.max_clearance;
  for (const Vector2f& p : point_cloud) {
    const float along_path = p.dot(move->direction);
    const Vector2f perpendicular_vec = p - along_path * move->direction;
    const float lateral_distance = perpendicular_vec.norm();
    
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
