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

#include "torch/torch.h"

#include "motion_primitives.h"
#include "image_based_evaluator.h"
#include <opencv2/videoio.hpp>

#ifndef DEEP_IRL_EVALUATOR_H
#define DEEP_IRL_EVALUATOR_H

namespace motion_primitives {

struct DeepIRLEvaluator :  ImageBasedEvaluator {
  DeepIRLEvaluator(const navigation::NavigationParameters& params) :
    ImageBasedEvaluator(params), blur_(params.blur) {
    };

  bool LoadModels();

  // Return the best path rollout from the provided set of paths.
  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) override;

  float ComputeAngleToGoal(Eigen::Vector2f target, pose_2d::Pose2Df state);

  // Torchscript definition of the deep irl network.
  torch::jit::script::Module irl_module;
    // Torchscript definition of the embedding network.
  torch::jit::script::Module embedding_module;

  static constexpr float UNCERTAINTY_REWARD = 0.0f;
  cv::VideoWriter outputVideo;
  bool blur_;
  int plan_idx = 0;
};

}  // namespace motion_primitives


#endif  // DEEP_IRL_EVALUATOR_H