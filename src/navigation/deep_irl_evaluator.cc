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
#include <fstream>

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
#include "image_tiler.h"
#include <chrono>
#include "json.hpp"

using std::min;
using std::max;
using std::string;
using std::vector;
using std::shared_ptr;
using std::ofstream;
using pose_2d::Pose2Df;
using Eigen::Vector2f;
using navigation::MotionLimits;
using namespace geometry;
using namespace math_util;
using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;
using std::chrono::duration;
using std::chrono::milliseconds;
using nlohmann::json;

#define VIS_IMAGES 1
#define VIS_PATCHES 1
#define PERF_BENCHMARK 0
#define VIS_FEATURES 1
#define WRITE_FEATURES 0

namespace motion_primitives {

bool DeepIRLEvaluator::LoadModels(const string& embedding_model_path, const string& irl_model_path) {
  # if VIS_IMAGES
  int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
  outputVideo.open("vis/video_vis.avi", codec, 4.0, cv::Size(1280, 1024), true);
  if (!outputVideo.isOpened())
  {
      cout  << "Could not open the output video for write" << endl;
      return -1;
  }
  # endif

  try {
    embedding_module = torch::jit::load(embedding_model_path);
    irl_module = torch::jit::load(irl_model_path);
    return true;
  } catch(const c10::Error& e) {
    std::cout << "Error loading models:\n" << e.msg();
    return false;
  }
}

float DeepIRLEvaluator::ComputeAngleToGoal(Eigen::Vector2f target, pose_2d::Pose2Df state) {
  auto future_target = target - state.translation;
  float future_target_angle = atan2(future_target[1], future_target[0]);
  float angle_to_goal = abs(AngleDiff(future_target_angle, state.angle));
  return angle_to_goal;
}

shared_ptr<PathRolloutBase> DeepIRLEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;
  plan_idx++;
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

  at::Tensor path_rewards = torch::zeros({(int)paths.size(), 1});
  for (size_t i = 0; i < paths.size(); i++) {
    for(size_t j = 0; j <= ImageBasedEvaluator::ROLLOUT_DENSITY; j++) {
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * j;
      auto state = paths[i]->GetIntermediateState(f);
      
      float validity;
      cv::Mat patch = GetPatchAtLocation(warped, state.translation, &validity, true);
      if (patch.rows > 0) {
        patch_location_indices.emplace_back(i, j);
        auto tensor_patch = torch::from_blob(patch.data, { patch.rows, patch.cols, patch.channels() }, at::kByte).to(torch::kFloat);
        tensor_patch = tensor_patch.permute({ 2,0,1 }); 

        auto future_target = local_target - state.translation;
        
        float angle_progress = ComputeAngleToGoal(local_target, state);
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
        path_rewards[i] += DeepIRLEvaluator::UNCERTAINTY_REWARD;
      }
    }
  }

  #if PERF_BENCHMARK
  auto t2 = high_resolution_clock::now();
  printf("Evaluating %ld values...\n", patch_tensors.size());
  #endif


  at::Tensor output;
  at::Tensor feature_tensor;
  at::Tensor embeddings_tensor;
  if (patch_tensors.size() > 0) {
    auto patch_tensor = torch::stack(patch_tensors);

    std::vector<torch::jit::IValue> emb_input;
    emb_input.push_back(patch_tensor);

    embeddings_tensor = embedding_module.forward(emb_input).toTensor();

    feature_tensor = torch::stack(irl_feature_tensors);

    auto input_tensor = torch::cat({feature_tensor, embeddings_tensor}, 1);

    std::vector<torch::jit::IValue> input;
    input.push_back(input_tensor);

    output = irl_module.forward(input).toTensor();

    for(int i = 0; i < output.size(0); i++) {
      auto patch_loc_index = patch_location_indices[i];
      path_rewards[patch_loc_index.first] += output[i];
    }
  }

  #if PERF_BENCHMARK
  auto t3 = high_resolution_clock::now();
  #endif

  auto best_idx = torch::argmax(path_rewards).item<int>();

  best = paths[best_idx];
  #if PERF_BENCHMARK
  auto t4 = high_resolution_clock::now();
  #endif

  # if VIS_IMAGES
  # if VIS_PATCHES
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
  # endif

  for(float f = 0; f < 1.0; f += 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY) {
    auto state = best->GetIntermediateState(f);
    auto image_loc = GetImageLocation(state.translation);
    cv::circle(warped_vis, cv::Point(image_loc.x(), image_loc.y()), 3, cv::Scalar(255, 0, 0), 2);
  }

  # if VIS_FEATURES
  if (feature_tensor.size(0) > 0) {
    int num_cells = feature_tensor.size(1) + 1;
    auto cell_height = (int) (warped_vis.rows * 1.0 / (num_cells));
    auto tiler = ImageCells(num_cells, 1, warped_vis.cols, cell_height);
    cv::Mat orig_warped_vis;
    cv::resize(warped_vis, orig_warped_vis, cv::Size(warped_vis.cols, cell_height));
    tiler.setCell(0, 0, orig_warped_vis);
    // std::cout << "NEW SIZE (" << feature_tensor.size(1) << " features): " << orig_warped_vis.rows << " " << orig_warped_vis.cols << std::endl;
    for(int ftr_idx = 0; ftr_idx < feature_tensor.size(1); ftr_idx++) {
      cv::Mat feat_vis = warped.clone();
      auto ftr_tens = feature_tensor.index({"...", ftr_idx});
      cv::Mat feature_mat = cv::Mat(ftr_tens.size(0), 1, CV_32F);
      for(int j = 0; j < ftr_tens.size(0); j++) {
        feature_mat.at<float>(j, 0) = ftr_tens[j].item<float>();
      }
      cv::Mat vis_feature_mat;
      cv::normalize(feature_mat, vis_feature_mat, 0, 255.0f, cv::NORM_MINMAX, CV_32F);
      for(int i = 0; i < ftr_tens.size(0); i++) {
        json patch_info;
        auto patch_loc_index = patch_location_indices[i];
        float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * patch_loc_index.second;
        auto state = paths[patch_loc_index.first]->GetIntermediateState(f);
        auto image_loc = GetImageLocation(state.translation);
        cv::rectangle(feat_vis,
          cv::Point(image_loc[0] - ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] - ImageBasedEvaluator::PATCH_SIZE / 2),
          cv::Point(image_loc[0] + ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] + ImageBasedEvaluator::PATCH_SIZE / 2),
          cv::Scalar(0, int(vis_feature_mat.at<float>(i, 0)), 0),
          cv::FILLED
        );
      }
      cv::resize(feat_vis, feat_vis, cv::Size(feat_vis.cols, cell_height));
      tiler.setCell(0, ftr_idx + 1, feat_vis);
      // warped_vis(cv::Rect(0, warped_vis.rows * (ftr_idx + 1.0) / (irl_feature_tensors.size() + 1), feat_vis.cols, feat_vis.rows)) = feat_vis;
    }
    warped_vis = tiler.image.clone();
  }

  // if (embeddings_tensor.size() > 0) {
  //   // cv::Mat embeddings_mat = cv::Mat(embeddings_tensor.size(0), embeddings_tensor.size(1), CV_32F, embeddings_tensor.data_ptr());
  //   // cv::Mat vis_embeddings;
  //   // cv::normalize(embeddings_mat, vis_embeddings, 0, 255.0f, cv::NORM_MINMAX, CV_32F);
  //   for(int i = 0; i < vis_rewards.rows; i++) {
  //     auto patch_loc_index = patch_location_indices[i];
  //     float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * patch_loc_index.second;
  //     auto state = paths[patch_loc_index.first]->GetIntermediateState(f);
  //     auto image_loc = GetImageLocation(state.translation);
  //     cv::rectangle(warped_vis,
  //       cv::Point(image_loc[0] - ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] - ImageBasedEvaluator::PATCH_SIZE / 2),
  //       cv::Point(image_loc[0] + ImageBasedEvaluator::PATCH_SIZE / 2, image_loc[1] + ImageBasedEvaluator::PATCH_SIZE / 2),
  //       cv::Scalar(int(vis_rewards.at<float>(i, 0)), 0, 0),
  //       cv::FILLED
  //     );
  //   }
  // }
  #endif

  #if WRITE_FEATURES
    ofstream json_output;
    std::vector<json> json_info;
    for(int i = 0; i < output.size(0); i++) {
      auto patch_loc_index = patch_location_indices[i];
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * patch_loc_index.second;
      auto state = paths[patch_loc_index.first]->GetIntermediateState(f);
      auto image_loc = GetImageLocation(state.translation);
      json patch_info;
      patch_info["image_loc"] = { image_loc[0], image_loc[1] };
      patch_info["loc"] = { state.translation.x(), state.translation.y() };
      patch_info["reward"] = output[i].item<double>();
      patch_info["features"] = std::vector<double>();
      patch_info["embeddings"] = std::vector<double>();
      for(int ftr_idx = 0; ftr_idx < feature_tensor.size(1); ftr_idx++) {
        patch_info["features"].push_back(feature_tensor[i][ftr_idx].item<double>());
      }
      for(int emb_idx = 0; emb_idx < embeddings_tensor.size(1); emb_idx++) {
        patch_info["embeddings"].push_back(embeddings_tensor[i][emb_idx].item<double>());
      }
      json_info.push_back(patch_info);
    }
    std::ostringstream out_json_stream;
    out_json_stream << "vis/patch_info/patch_info_" << plan_idx << ".json";
    std::string out_json_name = out_json_stream.str();
    json_output.open (out_json_name, ios::out); 
    json_output << json(json_info).dump();
    json_output.close();
  #endif

  outputVideo.write(warped_vis);
  std::ostringstream out_img_stream;
  out_img_stream << "vis/images/warped_vis_" << plan_idx << ".png";
  std::string out_img_name = out_img_stream.str();
  cv::imwrite(out_img_name, warped_vis);
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
