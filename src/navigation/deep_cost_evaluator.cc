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
\file    deep_irl_evaluator.cc
\brief   Path rollout evaluator using a NN learned via deep IRL.
\author  Joydeep Biswas, Kavan Sikand, (C) 2021
*/
//========================================================================

#include <math.h>
#include <float.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "gflags/gflags.h"
#include "math/line2d.h"
#include "math/poses_2d.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "torch/torch.h"
#include "torch/script.h"
#include <chrono>

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "deep_cost_evaluator.h"
#include "deep_cost_model.h"

using std::min;
using std::max;
using std::string;
using std::vector;
using std::shared_ptr;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using navigation::MotionLimits;
using namespace geometry;
using namespace math_util;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;


#define PERF_BENCHMARK 1
#define VIS_IMAGES 1

namespace motion_primitives {

bool DeepCostEvaluator::LoadModel(const string& cost_model_path) {
  try {
    cost_module = torch::jit::load(cost_model_path);
    return true;
  } catch(const c10::Error& e) {
    std::cout << "Error loading cost model:\n" << e.msg();
    return false;
  }
}

shared_ptr<PathRolloutBase> DeepCostEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;
  shared_ptr<PathRolloutBase> best = nullptr;

  cv::Mat warped = GetWarpedImage();

  # if VIS_IMAGES
  cv::Mat warped_vis = warped.clone();
  #endif

  // std::vector<std::vector<cv::Mat>> patches;
  std::vector<std::pair<size_t, size_t>> patch_location_indices;
  std::vector<at::Tensor> patch_tensors;

  #if PERF_BENCHMARK
  auto t1 = high_resolution_clock::now();
  #endif

  for (size_t i = 0; i < paths.size(); i++) {
    for(size_t j = 0; j <= 10; j++) {
      float f = 0.1 * j;
      auto state = paths[i]->GetIntermediateState(f);

      // printf("path state %f: %f, (%f %f)\n", f, state.angle, state.translation.x(), state.translation.y());
      cv::Mat patch = GetPatchAtLocation(warped, state.translation).clone();
      if (patch.rows > 0) {
        patch_location_indices.emplace_back(i, j);
        cv::cvtColor(patch, patch, cv::COLOR_BGR2RGB); // BGR -> RGB
        auto tensor_patch = torch::from_blob(patch.data, { patch.rows, patch.cols, patch.channels() }, at::kByte).to(torch::kFloat);
        tensor_patch = tensor_patch.permute({ 2,0,1 }); 
        patch_tensors.push_back(tensor_patch);
      }
    }
  }

  auto input_tensor = torch::stack(patch_tensors);

  std::vector<torch::jit::IValue> input;
  input.push_back(input_tensor);

  #if PERF_BENCHMARK
  auto t2 = high_resolution_clock::now();
  #endif

  at::Tensor output = cost_module.forward(input).toTensor();

  #if PERF_BENCHMARK
  auto t3 = high_resolution_clock::now();
  #endif

  at::Tensor path_costs = torch::zeros({(int)paths.size(), 1});
  for(int i = 0; i < output.size(0); i++) {
    auto patch_loc_index = patch_location_indices[i];
    path_costs[patch_loc_index.first] += output[i];
  }

  #if PERF_BENCHMARK
  auto t4 = high_resolution_clock::now();
  #endif

  # if VIS_IMAGES
  for(int i = 0; i < output.size(0); i++) {
    auto patch_loc_index = patch_location_indices[i];
    float f = 0.1 * patch_loc_index.second;
    auto state = paths[patch_loc_index.first]->GetIntermediateState(f);
    auto image_loc = GetImageLocation(state.translation);
    cv::rectangle(warped_vis,
      cv::Point(image_loc[0] - ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] - ImageBasedEvaluator::PATCH_SIZE / 2),
      cv::Point(image_loc[0] + ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] + ImageBasedEvaluator::PATCH_SIZE / 2),
      cv::Scalar(0, 0, (int) (output[i].item<float>() * 2.0)),
      cv::FILLED
    );
  }

  cv::imwrite("warped_vis.png", warped_vis);
  #endif


  #if PERF_BENCHMARK
  std::cout << "Patch Collection Time" << (duration_cast<milliseconds>(t2 - t1)).count() << "ms" << std::endl;
  std::cout << "Network Execution Time" << (duration_cast<milliseconds>(t3 - t2)).count() << "ms" << std::endl;
  std::cout << "Score Summation Time" << (duration_cast<milliseconds>(t4 - t3)).count() << "ms" << std::endl;
  #endif
  return best;
}

}  // namespace motion_primitives
