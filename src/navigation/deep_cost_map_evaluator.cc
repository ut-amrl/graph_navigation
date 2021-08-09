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
#include "deep_cost_map_evaluator.h"
#include "deep_cost_model.h"
#include "image_tiler.h"
#include <opencv2/core/eigen.hpp>

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

bool DeepCostMapEvaluator::LoadModel() {
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
    auto device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    cost_module = torch::jit::load(params_.model_path, device);
    return true;
  } catch(const c10::Error& e) {
    std::cout << "Error loading cost model:\n" << e.msg();
    return false;
  }
}

cv::Rect GetPatchRect(const cv::Mat& img, const Eigen::Vector2f& patch_loc) {
  return cv::Rect(patch_loc.x() - ImageBasedEvaluator::PATCH_SIZE / 2, patch_loc.y() - ImageBasedEvaluator::PATCH_SIZE / 2,
    min(ImageBasedEvaluator::PATCH_SIZE, img.cols - (int)patch_loc.x()),
    min(ImageBasedEvaluator::PATCH_SIZE, img.rows - (int)patch_loc.y()));
}

void DeepCostMapEvaluator::UpdateLocalMap() {
  Eigen::Affine2f curr_trans = Eigen::Translation2f(curr_loc) * Eigen::Rotation2Df(curr_ang);
  Eigen::Affine2f prev_trans = Eigen::Translation2f(prev_loc) * Eigen::Rotation2Df(prev_ang);

  Eigen::Affine2f delta_trans = prev_trans.inverse() * curr_trans;
  // auto reverseTrans = delta_trans.inverse();
  cv::Mat transformMatrix;
  cv::Mat rotationMat;
  cv::Mat translationMat;
  Eigen::Matrix2f eigenRotation = delta_trans.rotation().matrix().inverse();
  Eigen::Vector2f eigenTranslation = Eigen::Rotation2Df(M_PI_2) * delta_trans.translation().matrix();
  eigenTranslation = eigenTranslation.cwiseProduct(ImageBasedEvaluator::SCALING);
  cv::eigen2cv(eigenRotation, rotationMat);
  cv::eigen2cv(eigenTranslation, translationMat);

  cv::hconcat(rotationMat, translationMat, transformMatrix);

  auto prev_cost_map = local_cost_map.clone();
  local_cost_map.setTo(0);
  cv::warpAffine(prev_cost_map, local_cost_map, transformMatrix, local_cost_map.size(),
               cv::INTER_LINEAR,
               cv::BORDER_CONSTANT,
               UNCERTAINTY_COST);
  
  // cv::Mat resized_cost_img;
  // cv::normalize(local_cost_map, resized_cost_img, 0, 255.0f, cv::NORM_MINMAX, CV_8U);
  // cv::Mat color_cost_img;
  // cvtColor(resized_cost_img, color_cost_img, cv::COLOR_GRAY2RGB);
  // cv::line(color_cost_img, cv::Point(0, CENTER.y()), cv::Point(CENTER.x() * 2, CENTER.y()), cv::Scalar(200, 0, 0), 3);

  // std::ostringstream out_img_stream;
  // out_img_stream << "vis/images/local_transformed_" << plan_idx << ".png";
  // std::string out_img_name = out_img_stream.str();
  // cv::imwrite(out_img_name, color_cost_img);

}

shared_ptr<PathRolloutBase> DeepCostMapEvaluator::FindBest(
    const vector<shared_ptr<PathRolloutBase>>& paths) {
  if (paths.size() == 0) return nullptr;
  shared_ptr<PathRolloutBase> best = nullptr;
  plan_idx++;

  cv::Mat warped = GetWarpedImage();
  cv::cvtColor(warped, warped, cv::COLOR_BGR2RGB); // BGR -> RGB

  # if VIS_IMAGES
  cv::Mat warped_vis = warped.clone();
  #endif
  auto device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;

  std::vector<size_t> patch_location_indices;
  std::vector<at::Tensor> patch_tensors;

  int blur_factor = 1;
  float blur_step = ImageBasedEvaluator::PATCH_SIZE / blur_factor;

  #if PERF_BENCHMARK
  auto t1 = high_resolution_clock::now();
  #endif

  if (plan_idx > 1) {
    UpdateLocalMap();
  }

  std::vector<Eigen::Vector2f> tile_locations = GetTilingLocations(warped, blur_step);
  at::Tensor path_costs = torch::zeros({(int)paths.size(), 1});
  for(size_t i = 0; i < tile_locations.size(); i++) {
    auto image_loc = tile_locations[i];
    float validity;
    cv::Mat patch = GetPatchAtImageLocation(warped, image_loc, &validity, true).clone();

    if (patch.rows > 0) {
      auto tensor_patch = torch::from_blob(patch.data, { patch.rows, patch.cols, patch.channels() }, at::kByte).to(torch::kFloat);
      tensor_patch = tensor_patch.permute({ 2,0,1 });

      patch_tensors.push_back(tensor_patch);
      patch_location_indices.emplace_back(i);

      // auto patch_rect = GetPatchRect(warped, image_loc);
      // local_cost_map(patch_rect).setTo(0);
    } else {
      // auto patch_loc = tile_locations[i];
      // auto patch_rect = GetPatchRect(warped, patch_loc);
      // local_cost_map(patch_rect) += DeepCostMapEvaluator::UNCERTAINTY_COST / blur_factor;
    }
  }

  #if PERF_BENCHMARK
  auto t2 = high_resolution_clock::now();
  #endif

  at::Tensor output;
  if (patch_tensors.size() > 0) {
    auto input_tensor = torch::stack(patch_tensors).to(device);

    std::vector<torch::jit::IValue> input;
    input.push_back(input_tensor);

    output = cost_module.forward(input).toTensor().to(torch::kCPU);

    for(int i = 0; i < output.size(0); i++) {
      auto patch_idx = patch_location_indices[i];
      auto patch_loc = tile_locations[patch_idx];
      auto patch_rect = GetPatchRect(warped, patch_loc);
      local_cost_map(patch_rect) = output[i].item<double>();
    }
  }

  #if PERF_BENCHMARK
  auto t3 = high_resolution_clock::now();
  #endif

  for (size_t i = 0; i < paths.size(); i++) {
    for(size_t j = 0; j <= ImageBasedEvaluator::ROLLOUT_DENSITY; j+= blur_ ? 5 : 1) {
      float f = 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * j;
      auto state = paths[i]->GetIntermediateState(f);
      auto discount = pow(DISCOUNT_FACTOR, j);
      if (blur_) {
        std::vector<Eigen::Vector2f> locations = GetWheelLocations(state, params_.robot_width, params_.robot_length);
        for (auto loc : locations) {
          auto cost = local_cost_map.at<float>(cv::Point((int)loc.x(), (int)loc.y()));
          path_costs[i] += cost * discount / ImageBasedEvaluator::ROLLOUT_DENSITY;
        }
      } else {
        auto loc = GetImageLocation(state.translation);
        auto cost = local_cost_map.at<float>(cv::Point((int)loc.x(), (int)loc.y()));
        path_costs[i] += cost * discount / ImageBasedEvaluator::ROLLOUT_DENSITY;
      }
    }
  }

  at::Tensor blurred_path_costs;
  if (BLUR_FACTOR > 0) {
    blurred_path_costs = torch::zeros({(int)paths.size(), 1});
    for (size_t i = 0; i < paths.size(); i++) {
      float remaining = 1.0;
      if (i > 0) {
        blurred_path_costs[i] += DeepCostMapEvaluator::BLUR_FACTOR * path_costs[i - 1];
        remaining -= -DeepCostMapEvaluator::BLUR_FACTOR;
      }
      if (i < paths.size() - 1) {
        blurred_path_costs[i] += DeepCostMapEvaluator::BLUR_FACTOR * path_costs[i + 1];
        remaining -= -DeepCostMapEvaluator::BLUR_FACTOR;
      }

      blurred_path_costs[i] += remaining * path_costs[i];
    }
  } else {
    blurred_path_costs = path_costs;
  }

  cv::Mat path_cost_mat = cv::Mat(blurred_path_costs.size(0), blurred_path_costs.size(1), CV_32F, blurred_path_costs.data_ptr());
  cv::Mat normalized_path_costs = path_cost_mat;
  // cv::normalize(path_cost_mat, normalized_path_costs, 1.0f, 10.0f, cv::NORM_MINMAX, CV_32F);

  // Lifted from linear evaluator
  // Check if there is any path with an obstacle-free path from the end to the
  // local target.
  float curr_goal_dist = local_target.norm();

  vector<float> clearance_to_goal(paths.size(), 0.0);
  vector<float> dist_to_goal(paths.size(), FLT_MAX);
  // bool path_to_goal_exists = false;
  for (size_t i = 0; i < paths.size(); ++i) {
    const auto endpoint = paths[i]->EndPoint().translation;
    clearance_to_goal[i] = StraightLineClearance(
        Line2f(endpoint, local_target), point_cloud);
    if (clearance_to_goal[i] > 0.0) {
      dist_to_goal[i] = (endpoint - local_target).norm();
      // path_to_goal_exists = true;
    }
  }

  // First find the shortest path.
  int best_idx = -1;
  float best_path_progress = -FLT_MAX;
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_progress = curr_goal_dist - dist_to_goal[i];
    if (path_progress > best_path_progress) {
      best_path_progress = path_progress;
      best_idx = i;
      best = paths[i];
    }
  }

  if (best_idx == -1) {
    // No valid paths!
    return nullptr;
  }

  // Now try to find better paths.
  float best_cost = DISTANCE_WEIGHT * best_path_progress +
      FPL_WEIGHT * paths[best_idx]->Length() +
      CLEARANCE_WEIGHT * paths[best_idx]->Clearance() + 
      COST_WEIGHT * normalized_path_costs.at<float>(best_idx, 0);
  for (size_t i = 0; i < paths.size(); ++i) {
    if (paths[i]->Length() <= 0.0f) continue;
    const float path_progress = curr_goal_dist - dist_to_goal[i];
    const float cost = DISTANCE_WEIGHT * path_progress +
      FPL_WEIGHT * paths[i]->Length() +
      CLEARANCE_WEIGHT * paths[i]->Clearance() + 
      COST_WEIGHT * normalized_path_costs.at<float>(i, 0);
    printf("COMPONENTS %d c: %ld total %f (dist: %f fpl: %f clearance: %f cost: %f)\n", plan_idx, i, cost, DISTANCE_WEIGHT * path_progress,FPL_WEIGHT * paths[i]->Length(), CLEARANCE_WEIGHT * paths[i]->Clearance(), COST_WEIGHT * normalized_path_costs.at<float>(i, 0) );
    if (cost < best_cost) {
      best = paths[i];
      best_cost = cost;
      best_idx = i;
    }
  }
  printf("CHOSEN %d\n", best_idx);
  #if PERF_BENCHMARK
  auto t4 = high_resolution_clock::now();
  #endif

  # if VIS_IMAGES

  auto cell_height = (int) (warped_vis.rows / 2);
  auto tiler = ImageCells(2, 1, warped_vis.cols, cell_height);
  cv::Mat orig_warped_vis;
  cv::resize(warped_vis, orig_warped_vis, cv::Size(warped_vis.cols, cell_height));
  tiler.setCell(0, 0, orig_warped_vis);
  cv::Mat color_cost_img;
  cvtColor(local_cost_map, color_cost_img, cv::COLOR_GRAY2RGB);
  cv::normalize(color_cost_img, color_cost_img, 0, 255.0f, cv::NORM_MINMAX, CV_8U);

  for(size_t i = 0; i < paths.size(); i++) {
    for(float f = 0; f < 1.0; f += 1.0f / ImageBasedEvaluator::ROLLOUT_DENSITY * (blur_ ? 5 : 1)) {
      auto state = paths[i]->GetIntermediateState(f);
      auto image_loc = GetImageLocation(state.translation);
      auto color = (paths[i] == best) ? cv::Scalar(255, 0, 0) : cv::Scalar(0, normalized_path_costs.at<float>(i, 0) * 25, 0);
      cv::circle(warped_vis, cv::Point(image_loc.x(), image_loc.y()), 3, color, 2);
      cv::circle(color_cost_img, cv::Point(image_loc.x(), image_loc.y()), 3, color, 2);
    }
  }

  cv::Mat resized_cost_img;
  cv::resize(color_cost_img, resized_cost_img, cv::Size(warped_vis.cols, cell_height));

  tiler.setCell(0, 1, resized_cost_img);
  warped_vis = tiler.image;

  // std::cout << local_cost_map << std::endl;
  
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

  prev_loc = curr_loc;
  prev_ang = curr_ang;

  return best;
}

}  // namespace motion_primitives
