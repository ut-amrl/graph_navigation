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

#include <vector>

#include "motion_primitives.h"

#ifndef LINEAR_EVALUATOR_H
#define LINEAR_EVALUATOR_H

namespace motion_primitives {

struct LinearEvaluator :  PathEvaluatorBase {
  // Return the best path rollout from the provided set of paths.
  std::shared_ptr<PathRolloutBase> FindBest(
      const std::vector<std::shared_ptr<PathRolloutBase>>& paths) override;
  void SetClearanceWeight(const float& weight);
  void SetDistanceWeight(const float& weight);
  void SetFreePathWeight(const float& weight);
  void SetSubOpt(const float& threshold);
};

}  // namespace motion_primitives


#endif  // LINEAR_EVALUATOR_H
