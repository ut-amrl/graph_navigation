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

#include "motion_primitives.h"
#include "navigation_parameters.h"
#include "constant_curvature_arcs.h"
#include "ackermann_motion_primitives.h"
#include "deep_irl_evaluator.h"

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

#define VIS_IMAGES 0
#define PERF_BENCHMARK 0

namespace motion_primitives {

bool DeepIRLEvaluator::LoadModel(const string& irl_model_path) {
  try {
    irl_module = torch::jit::load(irl_model_path);
    return true;
  } catch(const c10::Error& e) {
    std::cout << "Error loading IRL model:\n" << e.msg();
    return false;
  }
}

shared_ptr<PathRolloutBase> DeepIRLEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;
  shared_ptr<PathRolloutBase> best = nullptr;

  cv::Mat warped = GetWarpedImage();

  # if VIS_IMAGES
  cv::Mat warped_vis = warped.clone();
  #endif

  std::vector<std::pair<size_t, size_t>> patch_location_indices;
  std::vector<at::Tensor> irl_input_tensors;

  #if PERF_BENCHMARK
  auto t1 = high_resolution_clock::now();
  #endif

  at::Tensor patch_rewards = torch::zeros({(int)paths.size(), 1});
  for (size_t i = 0; i < paths.size(); i++) {
    for(size_t j = 0; j <= ImageBasedEvaluator::ROLLOUT_DENSITY; j++) {
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * j;
      auto state = paths[i]->GetIntermediateState(f);

      cv::Mat patch = GetPatchAtLocation(warped, state.translation, true).clone();
      if (patch.rows > 0) {
        patch_location_indices.emplace_back(i, j);
        cv::cvtColor(patch, patch, cv::COLOR_BGR2RGB); // BGR -> RGB

        auto tensor_patch = torch::from_blob(patch.data, { patch.rows, patch.cols, patch.channels() }, at::kByte).to(torch::kFloat);
        tensor_patch = tensor_patch.permute({ 2,0,1 }); 
        
        const float angle_progress = 0.0f; // TODO: Compute angle progress (how much closer is the angle at this state to facing the goal vs. the original angle)
        auto tensor_angle = torch::full(1, angle_progress);
        const float distance_travelled = 0.0f; // TODO: compute distance travelled (should just be the norm of `state.translation`)
        auto tensor_distance = torch::full(1, distance_travelled);
        const float goal_progress = 0.0f;  // TODO: Compute goal progress (how much closer is the location at this state to facing the goal vs. the original location) (should be able to extract from linear)
        auto tensor_progress = torch::full(1, goal_progress);
        const float validity = 0.0f; // TODO: Compute validity (maybe just leave 0 for now?)
        auto tensor_validity = torch::full(1, validity);

        irl_input_tensors.push_back(torch::cat({
          tensor_validity,
          tensor_progress,
          tensor_distance,
          tensor_angle,
          tensor_patch
        }));
      } else {
        patch_rewards[i] += DeepIRLEvaluator::UNCERTAINTY_REWARD;
      }
    }
  }

  auto input_tensor = torch::stack(irl_input_tensors);

  std::vector<torch::jit::IValue> input;
  input.push_back(input_tensor);

  #if PERF_BENCHMARK
  auto t2 = high_resolution_clock::now();
  printf("Evaluating %ld values...\n", input_tensor.size(0));
  #endif

  at::Tensor output = irl_module.forward(input).toTensor();

  #if PERF_BENCHMARK
  auto t3 = high_resolution_clock::now();
  #endif

  for(int i = 0; i < output.size(0); i++) {
    auto patch_loc_index = patch_location_indices[i];
    patch_rewards[patch_loc_index.first] += output[i];
  }

  auto best_idx = torch::argmax(patch_rewards).item<int>();
  best = paths[best_idx];
  #if PERF_BENCHMARK
  auto t4 = high_resolution_clock::now();
  #endif

  # if VIS_IMAGES
  cv::Mat rewards = cv::Mat(output.size(0), output.size(1), CV_32F, output.data_ptr());
  cv::Mat vis_rewards;
  cv::normalize(rewards, vis_rewards, 0, 255.0f, cv::NORM_MINMAX, CV_32F);
  for(int i = 0; i < vis_rewards.rows; i++) {
    auto patch_loc_index = patch_location_indices[i];
    float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * patch_loc_index.second;
    auto state = paths[patch_loc_index.first]->GetIntermediateState(f);
    auto image_loc = GetImageLocation(state.translation);
    cv::rectangle(warped_vis,
      cv::Point(image_loc[0] - ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] - ImageBasedEvaluator::PATCH_SIZE / 2),
      cv::Point(image_loc[0] + ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] + ImageBasedEvaluator::PATCH_SIZE / 2),
      cv::Scalar(int(vis_costs.at<float>(i, 0), 0, 0)),
      cv::FILLED
    );
  }

  for(float f = 0; f < 1.0; f += 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY) {
    auto state = best->GetIntermediateState(f);
    auto image_loc = GetImageLocation(state.translation);
    cv::circle(warped_vis, cv::Point(image_loc.x(), image_loc.y()), 3, cv::Scalar(255, 0, 0), 2);
  }

  cv::imwrite("vis/warped_vis.png", warped_vis);
  #endif

  #if PERF_BENCHMARK
  auto t5 = high_resolution_clock::now();
  #endif

  #if PERF_BENCHMARK
  std::cout << "Patch Collection Time" << (duration_cast<milliseconds>(t2 - t1)).count() << "ms" << std::endl;
  std::cout << "Network Execution Time" << (duration_cast<milliseconds>(t3 - t2)).count() << "ms" << std::endl;
  std::cout << "Linear Evaluation Time" << (duration_cast<milliseconds>(t4 - t3)).count() << "ms" << std::endl;
  std::cout << "Visualization Time" << (duration_cast<milliseconds>(t5 - t4)).count() << "ms" << std::endl;
  #endif
  return best;
}

}  // namespace motion_primitives
