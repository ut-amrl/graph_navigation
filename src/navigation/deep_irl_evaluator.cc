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
#include <chrono>

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

#define VIS_IMAGES 1
#define PERF_BENCHMARK False

namespace motion_primitives {

bool DeepIRLEvaluator::LoadModels(const string& embedding_model_path, const string& irl_model_path) {
  try {
    embedding_module = torch::jit::load(embedding_model_path);
    irl_module = torch::jit::load(irl_model_path);
    return true;
  } catch(const c10::Error& e) {
    std::cout << "Error loading models:\n" << e.msg();
    return false;
  }
}

shared_ptr<PathRolloutBase> DeepIRLEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;
  shared_ptr<PathRolloutBase> best = nullptr;

  cv::Mat warped = GetWarpedImage();
  cv::cvtColor(warped, warped, cv::COLOR_BGR2RGB); // BGR -> RGB

  # if VIS_IMAGES
  cv::Mat warped_vis = warped.clone();
  #endif

  std::vector<std::pair<size_t, size_t>> patch_location_indices;
  std::vector<at::Tensor> patch_tensors;
  std::vector<at::Tensor> irl_feature_tensors;

  #if PERF_BENCHMARK
  auto t1 = high_resolution_clock::now();
  #endif

  at::Tensor patch_rewards = torch::zeros({(int)paths.size(), 1});
  for (size_t i = 0; i < paths.size(); i++) {
    for(size_t j = 0; j <= ImageBasedEvaluator::ROLLOUT_DENSITY; j++) {
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * j;
      auto state = paths[i]->GetIntermediateState(f);
      float validity;
      Eigen::Vector2f image_loc = GetImageLocation(state.translation);
      cv::Point coord = cv::Point(image_loc.x(), image_loc.y());
      cv::Mat patch = GetPatchAtLocation(warped, coord, &validity, true);
      if (patch.rows > 0) {
        patch_location_indices.emplace_back(i, j);

        auto tensor_patch = torch::from_blob(patch.data, { patch.rows, patch.cols, patch.channels() }, at::kByte).to(torch::kFloat);
        tensor_patch = tensor_patch.permute({ 2,0,1 }); 

        auto future_target = local_target - state.translation;
        
        const float future_target_angle = atan2(-future_target[1], -future_target[0]);
        const float angle_progress = abs(min(fmod(future_target_angle - state.angle, M_PI), fmod(state.angle - future_target_angle, M_PI)));
        auto tensor_angle = torch::full(1, angle_progress);
        const float distance_travelled = state.translation.norm();
        auto tensor_distance = torch::full(1, distance_travelled);
        const float goal_progress = local_target.norm() - future_target.norm();
        auto tensor_progress = torch::full(1, goal_progress);
        auto tensor_validity = torch::full(1, validity);

        // printf("features %f %f %f %f\n", angle_progress, distance_travelled, goal_progress, validity);

        irl_feature_tensors.push_back(torch::cat({
          tensor_validity,
          tensor_progress,
          tensor_distance,
          tensor_angle
        }));
        patch_tensors.push_back(tensor_patch);
      } else {
        patch_rewards[i] += DeepIRLEvaluator::UNCERTAINTY_REWARD;
      }
    }
  }

  #if PERF_BENCHMARK
  auto t2 = high_resolution_clock::now();
  printf("Evaluating %ld values...\n", patch_tensors.size());
  #endif


  at::Tensor output;
  if (patch_tensors.size() > 0) {
    auto patch_tensor = torch::stack(patch_tensors);

    std::vector<torch::jit::IValue> emb_input;
    emb_input.push_back(patch_tensor);

    at::Tensor embeddings = embedding_module.forward(emb_input).toTensor();

    auto feature_tensor = torch::stack(irl_feature_tensors);

    auto input_tensor = torch::cat({feature_tensor, embeddings}, 1);

    std::vector<torch::jit::IValue> input;
    input.push_back(input_tensor);

    output = irl_module.forward(input).toTensor();

    for(int i = 0; i < output.size(0); i++) {
      auto patch_loc_index = patch_location_indices[i];
      patch_rewards[patch_loc_index.first] += output[i];
    }
  }

  #if PERF_BENCHMARK
  auto t3 = high_resolution_clock::now();
  #endif

  auto best_idx = torch::argmax(patch_rewards).item<int>();
  std::cout << "Best" << best_idx << std::endl;

  best = paths[best_idx];
  #if PERF_BENCHMARK
  auto t4 = high_resolution_clock::now();
  #endif

  # if VIS_IMAGES
  if (patch_tensors.size() > 0) {
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
        cv::Scalar(int(vis_rewards.at<float>(i, 0)), 0, 0),
        cv::FILLED
      );
    }
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
