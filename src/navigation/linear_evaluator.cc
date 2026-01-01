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
#include "shared/math/line2d.h"
#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "linear_evaluator.h"

using Eigen::Vector2f;
using navigation::MotionLimits;
using pose_2d::Pose2Df;
using std::max;
using std::min;
using std::shared_ptr;
using std::vector;
using namespace geometry;
using namespace math_util;

// Cost function weights for safety optimization in pass 2 (negative = prefer higher values)
// Pass 1 handles goal-reaching (shortest total distance), pass 2 handles safety among near-optimal paths.
DEFINE_double(clearance_weight, -0.5, "Weight for lateral clearance");
DEFINE_double(freepath_weight, -1.0, "Weight for progress among near-optimal paths");
DEFINE_double(subopt_tolerance, 1.5, "Max total distance multiplier for constrained optimization");

namespace motion_primitives {

shared_ptr<PathRolloutBase> LinearEvaluator::FindBest(const vector<shared_ptr<PathRolloutBase>> &paths) {
    if (paths.size() == 0) return nullptr;

    // Check line-of-sight (LOS) from each path's endpoint to the goal
    const size_t N = paths.size();
    vector<float> remaining_dist(N, FLT_MAX);  // distance from endpoint to goal
    vector<bool> has_los(N, false);            // whether each path has LOS
    bool any_path_has_los = false;

#pragma omp parallel
    {
        bool local_has_los = false;
#pragma omp for schedule(runtime)
        for (int i = 0; i < static_cast<int>(N); ++i) {
            const auto endpoint = paths[i]->EndPoint().translation;
            // Always compute remaining distance
            remaining_dist[i] = (endpoint - local_target).norm();

            // Check LOS
            const float los_clearance = LOSClearanceToLine(Line2f(endpoint, local_target), *point_cloud);
            // Require clearance >= half min dimension + margin for meaningful LOS
            const float min_los_clearance =
                0.5f * std::min(nav_params.robot_width, nav_params.robot_length) + nav_params.obstacle_margin;
            if (los_clearance > min_los_clearance) {
                has_los[i] = true;
                local_has_los = true;
            }
        }
#pragma omp critical
        {
            any_path_has_los = any_path_has_los || local_has_los;
        }
    }

    // Pass 1: Find shortest total path to goal
    // Prefer LOS paths when available, but fall back to all paths if none have LOS
    shared_ptr<PathRolloutBase> best = nullptr;
    float best_total_dist = FLT_MAX;
    for (size_t i = 0; i < paths.size(); ++i) {
        if (paths[i]->Length() <= 0.0f) continue;
        // If any path has LOS, restrict to LOS paths; otherwise, allow all paths
        if (any_path_has_los && !has_los[i]) continue;
        const float total_dist = paths[i]->Length() + remaining_dist[i];
        if (total_dist < best_total_dist) {
            best_total_dist = total_dist;
            best = paths[i];
        }
    }
    if (best == nullptr) {
        printf("No valid path found\n");
        return nullptr;
    }

    // Pass 2: Among candidate paths within subopt tolerance of shortest, find best progress/safety tradeoff
    // Use same LOS restriction as pass 1
    const float max_allowed_dist = FLAGS_subopt_tolerance * best_total_dist;
    float best_cost = FLAGS_freepath_weight * best->Length() + FLAGS_clearance_weight * best->Clearance();
    for (size_t i = 0; i < paths.size(); ++i) {
        if (paths[i]->Length() <= 0.0f) continue;
        // Use same LOS restriction as pass 1
        if (any_path_has_los && !has_los[i]) continue;
        const float total_dist = paths[i]->Length() + remaining_dist[i];
        if (total_dist > max_allowed_dist) continue;
        const float cost = FLAGS_freepath_weight * paths[i]->Length() + FLAGS_clearance_weight * paths[i]->Clearance();
        if (cost < best_cost) {
            best = paths[i];
            best_cost = cost;
        }
    }
    return best;
}

}  // namespace motion_primitives
