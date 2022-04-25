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
\file    linear_evaluator.h
\brief   Path rollout evaluator using a linear weighted cost.
\author  Kavan Sikand, (C) 2020
*/
//========================================================================

#include <math.h>
#include <float.h>

#include <algorithm>
#include <memory>
#include <vector>

#include "gflags/gflags.h"
#include "math/line2d.h"
#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "linear_evaluator.h"

using std::min;
using std::max;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using navigation::MotionLimits;
using namespace geometry;
using namespace math_util;

DEFINE_double(dw, 0, "Distance weight");
DEFINE_double(cw, -1, "Clearance weight");
DEFINE_double(ow, 0.0, "Option clearance weight");
DEFINE_double(fw, 0.0, "Free path weight");
DEFINE_double(subopt, 1.5, "Max path increase for clearance");
DEFINE_double(fpl_avg_window,
             0.1,
             "The fraction of the total samples that are used to compute the "
             "moving average local free path length");

namespace motion_primitives {

shared_ptr<PathRolloutBase> LinearEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;

  // Check if there is any path with an obstacle-free path from the end to the
  // local target.
  vector<float> clearance_to_goal(paths.size(), 0.0);
  vector<float> dist_to_goal(paths.size(), FLT_MAX);
  bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] = StraightLineClearance(
        Line2f(endpoint, local_target), point_cloud);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - local_target).norm();
      path_to_goal_exists = true;
    }
  }
  // If there is no path with an obstacle-free path from the end to the local target,
  // then reset all distance to goals.
  if (!path_to_goal_exists) {
    for (size_t i = 0; i < paths.size(); ++i) {
      const auto endpoint = paths[i]->EndPoint().translation;
      dist_to_goal[i] = (endpoint - local_target).norm();
    }
  }

  // First find the shortest path.
  shared_ptr<PathRolloutBase> best = nullptr;
  float best_path_length = FLT_MAX;
  int best_i = 0;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length = paths[i]->Length() + dist_to_goal[i];
    if (path_length < best_path_length) {
      best_path_length = path_length;
      best = paths[i];
      best_i = i;
    }
  }

  // Recompute clearance as the local weighted mean of the fpl around and
  // including the current option.
  vector<float> clearance(paths.size());
  vector<float> option_clearance(paths.size());
  const int n = ceil(static_cast<float>(paths.size()) * FLAGS_fpl_avg_window);
  const float w = 1.0f / static_cast<float>(n);
  for (size_t i = 0; i < paths.size(); ++i) {
    clearance[i] = w * paths[i]->Length();
    option_clearance[i] = 0.0;
    for (int j = 1;
         j < n && i + j < paths.size() && static_cast<int>(i) >= j;
         ++j) {
      if (paths[i - j]->Length() <= 0.0f ||
          paths[i + j]->Length() <= 0.0f) break;
      clearance[i] += w * paths[i - j]->Length();
      clearance[i] += w * paths[i + j]->Length();
      option_clearance[i] =+ 2.0;
    }
    if (FLAGS_v > 2) {
      printf("%3d: curvature=%7.3f, fpl=%7.3f, avg_fpl=%7.3f, "
             "option_clearance=%4.1f clearance=%7.3f, dist_to_goal=%7.3f\n",
             static_cast<int>(i),
             reinterpret_cast<const ConstantCurvatureArc*>(paths[i].get())->curvature,
             paths[i]->Length(),
             clearance[i],
             option_clearance[i],
             paths[i]->Clearance(),
             dist_to_goal[i] < FLT_MAX ? dist_to_goal[i] : 1.0 /0.0f);
    }
  }

  if (best == nullptr) {
    // No valid paths!
    return nullptr;
  }

  // Next try to find better paths.
  float best_cost = FLAGS_dw * (FLAGS_subopt * best_path_length) +
      FLAGS_fw * best->Length() +
      FLAGS_ow * option_clearance[best_i] +
      FLAGS_cw * clearance[best_i];
  for (size_t i = 0; i < paths.size(); ++i) {

    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length = paths[i]->Length() + dist_to_goal[i];
    // const float cost = FLAGS_dw * path_length +
    //   FLAGS_fw * paths[i]->Length() +
    //   FLAGS_cw * paths[i]->Clearance();
    const float cost = FLAGS_dw * path_length +
      FLAGS_fw * paths[i]->Length() +
      FLAGS_ow * option_clearance[i] +
      FLAGS_cw * clearance[i];
    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
    }
  }
  return best;
}

void LinearEvaluator::SetClearanceWeight(const float &weight) {
  FLAGS_cw = weight;
}

void LinearEvaluator::SetDistanceWeight(const float &weight) {
  FLAGS_dw = weight;
}

void LinearEvaluator::SetFreePathWeight(const float &weight) {
  FLAGS_fw = weight;
}

void LinearEvaluator::SetSubOpt(const float &threshold) {
  FLAGS_subopt = threshold;
}

}  // namespace motion_primitives
