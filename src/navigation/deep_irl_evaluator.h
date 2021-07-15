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

#ifndef DEEP_IRL_EVALUATOR_H
#define DEEP_IRL_EVALUATOR_H

namespace motion_primitives {

struct DeepIRLEvaluator :  PathEvaluatorBase {
  bool LoadModel(const std::string& irl_model_path);

  // Return the best path rollout from the provided set of paths.
  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) override;

  // Torchscript definition of the network.
  torch::jit::script::Module irl_module;
};

}  // namespace motion_primitives


#endif  // DEEP_IRL_EVALUATOR_H