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
#include "torch/script.h"
#include <fstream>

using std::min;
using std::max;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using navigation::MotionLimits;
using namespace geometry;
using namespace math_util;

DEFINE_double(dw, 1, "Distance weight");
DEFINE_double(cw, -0.5, "Clearance weight");
DEFINE_double(fw, -1, "Free path weight");
DEFINE_double(subopt, 1.5, "Max path increase for clearance");

namespace motion_primitives {
auto module = torch::jit::load("/home/bwilab/jackal_ws/src/graph_navigation/src/navigation/IRL_script.pt");
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

  // First find the shortest path.
  shared_ptr<PathRolloutBase> best = nullptr;
  float best_path_length = FLT_MAX;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length = (path_to_goal_exists ?
        (paths[i]->Length() + dist_to_goal[i]) : dist_to_goal[i]);
    if (path_length < best_path_length) {
      best_path_length = path_length;
      best = paths[i];
    }
  }

  if (best == nullptr) {
    // No valid paths!
    return nullptr;
  }

  // Next try to find better paths.
  std::ofstream my_write_file("/home/bwilab/jackal_ws/src/graph_navigation/src/navigation/data.txt", std::ios::app);
  float best_cost = FLAGS_dw * (FLAGS_subopt * best_path_length) +
      FLAGS_fw * best->Length() +
      FLAGS_cw * best->Clearance();
  // int model_best_index = 0;
  // float model_best =9999;
  // int bestIndex =0;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_length = (path_to_goal_exists ?
        (paths[i]->Length() + dist_to_goal[i]) : dist_to_goal[i]);
    
    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(torch::tensor({path_length, paths[i]->Length(), paths[i]->Clearance()}));
    auto output = module.forward(inputs).toTensor().to(torch::kCPU);
    float model_cost = output[0].item<float>();
    model_cost = std::abs(model_cost);


    //default cost
    float cost = FLAGS_dw * path_length + FLAGS_fw * paths[i]->Length() + FLAGS_cw * paths[i]->Clearance();

    //Use the model cost
    // cost = model_cost;


    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
      // bestIndex = i;
    }
    // if(model_cost<model_best){
    //   model_best = model_cost;
    //   model_best_index = i;
    // }
    my_write_file << path_length;
      my_write_file << " ";
      my_write_file << paths[i]->Length();
      my_write_file << " ";
      my_write_file << paths[i]->Clearance();
      my_write_file << " ";
      my_write_file << "\n";

    // std::printf("default best index = %d, model best index = %d, default cost = %f, model Cost = %f\n", bestIndex, model_best_index, best_cost, model_best);
  }
  my_write_file.close();
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