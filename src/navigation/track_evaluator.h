#include <vector>

#include "torch/script.h"
#include "torch/torch.h"

using std::vector;

namespace motion_primitives {

struct TrackEvaluator {
  
  bool LoadModel();

  float PredictWeights(vector<float> pc);
  
  // Torch definition of the network.
  torch::jit::script::Module model;

};
}
