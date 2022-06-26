#include "vrl_cost_function.h"

#include "torch/torch.h"

namespace motion_primitives {

void VRLCostFunction::operator()(cv::Mat &rgb_map, cv::Mat &cost_map) {}

bool VRLCostFunction::LoadModel() {
  try {
    auto device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    cost_module = torch::jit::load(params_.model_path, device);
    return true;
  } catch (const c10::Error &e) {
    std::cerr << "Error loading cost model:\n" << e.msg();
    return false;
  }
}

}  // namespace motion_primitives