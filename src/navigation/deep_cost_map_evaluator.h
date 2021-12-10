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
\file    deep_irl_evaluator.h
\brief   Path rollout evaluator using a NN learned via deep IRL.
\author  Joydeep Biswas, Kavan Sikand, (C) 2021
*/
//========================================================================

#include <string>
#include <vector>
#include <mutex>

#include "torch/torch.h"

#include "motion_primitives.h"
#include "image_based_evaluator.h"
#include "shared/util/timer.h"

#ifndef DEEP_COST_MAP_EVALUATOR_H
#define DEEP_COST_MAP_EVALUATOR_H

namespace motion_primitives {

struct DeepCostMapEvaluator :  ImageBasedEvaluator {
  DeepCostMapEvaluator(const navigation::NavigationParameters& params) :
    ImageBasedEvaluator(params) {
      local_cost_map_ = cv::Mat((int)ImageBasedEvaluator::CENTER.y() * 2, (int)ImageBasedEvaluator::CENTER.x() * 2, CV_32F, UNCERTAINTY_COST);
    };
    //cost_module(navigation::EmbeddingNet(6), navigation::CostNet(6)) 

  bool LoadModel();
  void UpdateMapToLocalFrame(cv::Mat& map, const Eigen::Vector2f& loc, float ang);
  void UpdateLocalCostMap();

  // Return the best path rollout from the provided set of paths.
  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) override;

  // Torch definition of the network.
  torch::jit::script::Module cost_module;

  static constexpr float UNCERTAINTY_COST = 10.0f;
  static constexpr double DISTANCE_WEIGHT = -3.5;
  static constexpr double CLEARANCE_WEIGHT = -0.25;
  static constexpr double FPL_WEIGHT = -0.75;
  static constexpr double COST_WEIGHT = 4.0;
  static constexpr double BLUR_FACTOR = 0.05;
  static constexpr double DISCOUNT_FACTOR = 0.25; // discount per meter from the robot
  

  int plan_idx = 0;

  std::vector<float> latest_cost_components_;

  cv::Mat local_cost_map_;
  Eigen::Vector2f cost_map_loc_;
  float cost_map_ang_;
  std::mutex cost_map_mutex;
};

}  // namespace motion_primitives


#endif  // DEEP_COST_MAP_EVALUATOR_H