#include <vector>

#include "track_evaluator.h"
#include "torch/torch.h"

using std::vector;


namespace motion_primitives {

bool TrackEvaluator::LoadModel() {
  try {
    printf("Loading model\n");
    model = torch::jit::load("convmodel.pt");
    printf("Loaded model\n");
    return true;
  } catch(const c10::Error& e) {
    std::cerr << "Error loading cost model:\n" << e.msg();
    return false;
  }
}

float TrackEvaluator::PredictWeights(vector<float> pc) {
  printf("here 1\n");
  at::Tensor scan_tensor = torch::from_blob(pc.data(), {1, 1033}).to(torch::kFloat);
  printf("here 2\n");
  
  std::vector<torch::jit::IValue> input;
  printf("here 4\n");
  
  // auto device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
  input.push_back(scan_tensor);
  printf("here 5\n");

  at::Tensor output = model.forward(input).toTensor().to(torch::kCPU);
  printf("here 6\n");

  auto result = output[0].item<double>();
  printf("%.5f\n", result);

  return 1.0;
}	
}
