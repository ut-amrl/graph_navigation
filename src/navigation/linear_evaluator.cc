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

#include <cmath>
#include <cfloat>
#include <cstdio>

#include <algorithm>
#include <memory>
#include <vector>

#include "gflags/gflags.h"
#include "shared/math/poses_2d.h"
#include "eigen3/Eigen/Dense"

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "linear_evaluator.h"

using Eigen::Vector2f;
using std::shared_ptr;
using std::vector;

DEFINE_double(progress_reward, 1.0, "Reward weight for path progress (length)");
DEFINE_double(clearance_reward, 0.5, "Reward weight for lateral clearance");
DEFINE_double(fpl_reward, 0.1, "Reward weight for free path length (lookahead safety)");
DEFINE_double(smoothness_reward, 0.2, "Reward weight for path alignment with current velocity (reduces jitter)");
DEFINE_double(subopt_tolerance, 1.5, "Max total distance multiplier for constrained optimization");

namespace motion_primitives {

// For k positive:
// - Exponentially saturating to band for positive values
// - Exponentially increasing for negative values
inline float SaturatingUtility(float value, float band, float k) {
    const float b = std::max(1e-3f, band);
    const float c = std::min(value, b);
    const float x = std::clamp((-k * c) / b, -10.0f, 10.0f);
    return b * std::expm1(x) / std::expm1(-k);  // equals b*(1-exp(x)) / (1-exp(-k))
}

inline float ClearanceUtility(float clearance, float clearance_band) {
    return SaturatingUtility(clearance, clearance_band, 2.0f);
}

inline float FPLUtility(float fpl, float max_lookahead_fpl) {
    if (fpl >= 0.0f) return 0.0f;  // no reward for forward free distance; Length() reward already includes this
    return SaturatingUtility(fpl, max_lookahead_fpl, 10.0f);
}

inline float ComputeReward(const shared_ptr<PathRolloutBase>& path, float alignment, float clearance_band,
                           float max_lookahead_fpl) {
    return FLAGS_progress_reward * path->Length() +
           FLAGS_clearance_reward * ClearanceUtility(path->Clearance(), clearance_band) +
           FLAGS_fpl_reward * FPLUtility(path->FPL(), max_lookahead_fpl) + FLAGS_smoothness_reward * alignment;
}

// Returns the best path rollout; nullptr if no valid path found
shared_ptr<PathRolloutBase> LinearEvaluator::FindBest(const vector<shared_ptr<PathRolloutBase>>& paths) {
    if (paths.empty()) return nullptr;

    const size_t N = paths.size();
    vector<float> total_dist(N, FLT_MAX);
    vector<float> alignment(N, 0.0f);

    // LOS clearance threshold
    const float inflated_width = nav_params.robot_width + 2.0f * nav_params.obstacle_margin;
    const float inflated_length = nav_params.robot_length + 2.0f * nav_params.obstacle_margin;
    const float los_radius = 0.5f * std::min(inflated_width, inflated_length);

    // Current velocity direction (for smoothness reward)
    const float current_speed = vel.norm();
    const Vector2f vel_dir = (current_speed > 1e-3f) ? vel.normalized() : Vector2f::Zero();

#pragma omp parallel for schedule(runtime)
    for (int i = 0; i < static_cast<int>(N); ++i) {
        const auto& path = paths[i];
        const float len = path->Length();
        if (len <= 0.0f) continue;  // skip unusable paths

        const Vector2f endpoint = path->EndPoint().translation;
        const float remaining_dist = (endpoint - local_target).norm();
        total_dist[i] = len + remaining_dist;

        if (current_speed > 1e-3f) {
            const Vector2f path_dir = endpoint.normalized();
            alignment[i] = vel_dir.dot(path_dir);
        } else {
            alignment[i] = 0.0f;  // no alignment reward if currently stopped
        }
    }

    bool any_path_has_los = false;
    bool in_penetration = false;
    for (size_t i = 0; i < N && !(any_path_has_los && in_penetration); ++i) {
        if (paths[i]->Length() <= 0.0f) continue;  // skip unusable paths
        any_path_has_los |= (paths[i]->LOSClearance() > los_radius);
        in_penetration |= (paths[i]->FPL() < 0.0f);
    }

    const bool require_los = (!in_penetration && any_path_has_los);

    // Pass 1: compute best (minimum) total_dist among eligible paths
    float best_total_dist = FLT_MAX;
    for (size_t i = 0; i < N; ++i) {
        if (paths[i]->Length() <= 0.0f) continue;                             // skip unusable paths
        if (require_los && paths[i]->LOSClearance() <= los_radius) continue;  // skip non-LOS paths
        best_total_dist = std::min(best_total_dist, total_dist[i]);
    }
    if (best_total_dist == FLT_MAX) {
        printf("No valid path found\n");
        return nullptr;
    }

    // Pass 2: choose max reward among near-optimal total distance candidates
    const float max_allowed_dist = in_penetration ? FLT_MAX : (FLAGS_subopt_tolerance * best_total_dist);

    shared_ptr<PathRolloutBase> best = nullptr;
    size_t best_idx = N;
    float best_reward = -FLT_MAX;

    for (size_t i = 0; i < N; ++i) {
        if (paths[i]->Length() <= 0.0f) continue;                             // skip unusable paths
        if (require_los && paths[i]->LOSClearance() <= los_radius) continue;  // skip non-LOS paths
        if (total_dist[i] > max_allowed_dist) continue;                       // skip paths too far from optimal

        const float reward =
            ComputeReward(paths[i], alignment[i], nav_params.clearance_band, nav_params.max_lookahead_fpl);
        if (best == nullptr || reward > best_reward) {
            best = paths[i];
            best_idx = i;
            best_reward = reward;
        }
    }

    // Debug output for path evaluation
    if (best) {
        printf("=== Path Evaluation Debug ===\n");

        const float best_reward_dbg =
            ComputeReward(best, alignment[best_idx], nav_params.clearance_band, nav_params.max_lookahead_fpl);
        printf(
            "BEST: Length=%.3f, FPL=%.3f, Clearance=%.3f, LOS=%c, Align=%.3f, "
            "ClearanceReward=%.3f, FPLReward=%.3f, Reward=%.3f, BestTotalDist=%.3f, MaxAllowedDist=%.3f\n",
            best->Length(), best->FPL(), best->Clearance(), best->LOSClearance() > los_radius ? 'Y' : 'N',
            alignment[best_idx],
            FLAGS_clearance_reward * ClearanceUtility(best->Clearance(), nav_params.clearance_band),
            FLAGS_fpl_reward * FPLUtility(best->FPL(), nav_params.max_lookahead_fpl), best_reward_dbg, best_total_dist,
            max_allowed_dist);

        printf("ALL SAMPLES:\n");
        for (size_t i = 0; i < N; ++i) {
            if (paths[i]->Length() <= 0.0f) continue;

            const float sample_reward =
                ComputeReward(paths[i], alignment[i], nav_params.clearance_band, nav_params.max_lookahead_fpl);
            printf(
                "  [%zu]: Length=%.3f, FPL=%.3f, Clearance=%.3f, LOS=%c, Align=%.3f, "
                "ClearanceReward=%.3f, FPLReward=%.3f, Reward=%.3f, TotalDist=%.3f, Allowed=%c\n",
                i, paths[i]->Length(), paths[i]->FPL(), paths[i]->Clearance(),
                paths[i]->LOSClearance() > los_radius ? 'Y' : 'N', alignment[i],
                FLAGS_clearance_reward * ClearanceUtility(paths[i]->Clearance(), nav_params.clearance_band),
                FLAGS_fpl_reward * FPLUtility(paths[i]->FPL(), nav_params.max_lookahead_fpl), sample_reward,
                total_dist[i], (total_dist[i] <= max_allowed_dist) ? 'Y' : 'N');
        }

        printf("=== End Debug ===\n");
    }

    return best;
}

}  // namespace motion_primitives
