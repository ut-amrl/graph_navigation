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


#include <memory>
#include <vector>

#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"

#include "motion_primitives.h"

#ifndef ACKERMANN_MOTION_PRIMITIVES_H
#define ACKERMANN_MOTION_PRIMITIVES_H

namespace motion_primitives {

// Path rollout sampler. 
struct AckermannSampler : PathRolloutSamplerBase {
  // Given the robot's current dynamic state and an obstacle point cloud, return
  // a set of valid path rollout options that are collision-free.
  std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
  // Default constructor, init parameters.
  AckermannSampler();

  // Compute free path lengths and clearances.
  void CheckObstacles(
      std::vector<std::shared_ptr<PathRolloutBase>>& samples);

  // Limit the maximum path length to the closest point of approach to the local
  // target.
  void SetMaxPathLength(ConstantCurvatureArc* path);

  void CheckObstacles(ConstantCurvatureArc* path);

};

}  // namespace motion_primitives

#endif  // ACKERMANN_MOTION_PRIMITIVES_H