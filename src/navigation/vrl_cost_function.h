#ifndef VRL_COST_FUNCTION_H
#define VRL_COST_FUNCTION_H

#include "rgb_cost_map_evaluator.h"
#include "torch/script.h"

namespace motion_primitives {

struct VRLCostFunction : CostFunction {
  VRLCostFunction(navigation::NavigationParameters &params) : params_(params) {}

  void operator()(cv::Mat &rgb_map, cv::Mat &cost_map) override;
  bool LoadModel();

  navigation::NavigationParameters params_;
  torch::jit::script::Module cost_module;
};

}  // namespace motion_primitives

#endif