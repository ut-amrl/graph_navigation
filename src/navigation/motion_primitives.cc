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
\file    motion_primitives.cc
\brief   Generic interfaces for motion primitives of different platforms.
\author  Joydeep Biswas, (C) 2021
*/
//========================================================================

#include <float.h>
#include <math.h>
#include <stdio.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "math/line2d.h"
#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"

#include "shared/math/math_util.h"
#include "motion_primitives.h"
#include "navigation_parameters.h"

using Eigen::Vector2f;
using std::max;
using std::min;
using std::shared_ptr;
using std::string;
using std::vector;
using navigation::MotionLimits;
using namespace math_util;
using namespace geometry;

namespace motion_primitives {

float Run1DTimeOptimalControl(const MotionLimits& limits, 
                              const float x_now, 
                              const float v_now, 
                              const float x_final, 
                              const float v_final,
                              const float dt) {
  // Non-zero final vel not yet implemented.
  CHECK_EQ(v_final, 0.0f) << "Non-zero final vel not yet implemented!";
  static FILE* fid = nullptr;
  const bool kTest = false;
  const string kTestLogFile = "1DTOC.txt";
  if (kTest && fid == nullptr) {
    fid = fopen(kTestLogFile.c_str(), "w");
  }
  const float dist_left = x_final - x_now;
  float velocity_cmd = 0;
  const float speed = fabs(v_now);
  const float dv_a = dt * limits.max_acceleration;
  const float dv_d = dt * limits.max_deceleration;
  float accel_stopping_dist =
      (speed + 0.5 * dv_a) * dt +
      Sq(speed + dv_a) / (2.0 * limits.max_deceleration);
  float cruise_stopping_dist =
      speed * dt +
      Sq(speed) / (2.0 * limits.max_deceleration);
  char phase = '?';
  if (dist_left >  0) {
    if (speed > limits.max_speed) {
      // Over max speed, slow down.
      phase = 'O';
      velocity_cmd = max<float>(0.0f, speed - dv_d);
    } else if (speed < limits.max_speed && accel_stopping_dist < dist_left) {
      // Acceleration possible.
      phase = 'A';
      velocity_cmd = min<float>(limits.max_speed, speed + dv_a);
    } else if (cruise_stopping_dist < dist_left) {
      // Must maintain speed, cruise phase.
      phase = 'C';
      velocity_cmd = speed;
    } else {
      // Must decelerate.
      phase = 'D';
      velocity_cmd = max<float>(0, speed - dv_d);
    }
  } else if (speed > 0.0f) {
    phase = 'X';
    velocity_cmd = max<float>(0, speed - dv_d);
  }
  if (kTest) {
    printf("%c x:%f dist_left:%f a_dist:%f c_dist:%f v:%f cmd:%f\n",
           phase,
           x_now,
           dist_left,
           accel_stopping_dist,
           cruise_stopping_dist,
           v_now,
           velocity_cmd);
    if (fid != nullptr) {
      fprintf(fid, "%f %f %f %f %f %f\n",
              x_now,
              dist_left,
              accel_stopping_dist,
              cruise_stopping_dist,
              v_now,
              velocity_cmd);
      fflush(fid);
    }
  }
  return velocity_cmd;
}


float StraightLineClearance(const Line2f& l, 
                            const vector<Vector2f>& points) {
  const Vector2f d = l.Dir();
  const float len = l.Length();
  float clearance = FLT_MAX;
  for (const Vector2f& p  : points) {
    const float x = d.dot(p - l.p0);
    if (x < 0.0f || x > len) continue;
    clearance = min<float>(clearance, l.Distance(p));
  }
  return clearance;
}

}  // namespace motion_primitives