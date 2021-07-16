#include <vector>

#include "torch/torch.h"

#ifndef DEEP_IRL_MODEL_H
#define DEEP_IRL_MODEL_H

namespace navigation {

struct IRLModel : torch::nn::Module {
  explicit IRLModel(int input_dim) {
    fc = torch::nn::Sequential(
      torch::nn::Linear(input_dim, 24),
      torch::nn::PReLU(),
      torch::nn::Linear(24, 12),
      torch::nn::PReLU(),
      torch::nn::Linear(12, 12),
      torch::nn::PReLU(),
      torch::nn::Linear(12, 6),
      torch::nn::PReLU(),
      torch::nn::Linear(6, 1),
      torch::nn::Sigmoid());
  }

  explicit IRLModel() : IRLModel(64) {}

  torch::Tensor forward(torch::Tensor x) {
    std::vector<torch::Tensor> results;
    for (int i = 0; i < x.size(1); ++i) {
      // For reference on slicing, see: https://pytorch.org/cppdocs/notes/tensor_indexing.html#translating-between-python-c-index-types
      torch::Tensor x_slice = x.index({
          torch::indexing::Slice(),
          i,
          torch::indexing::Slice()
      });
      torch::Tensor single_result = fc->forward(x_slice);
      results.push_back(single_result);
    }
    return torch::stack(results, 1);
  }
  torch::nn::Sequential fc;
};

struct IRLBatchModel : torch::nn::Module {
  explicit IRLBatchModel(IRLModel irl_model) : irl_model(irl_model) {}

  torch::Tensor forward(torch::Tensor x) {
    std::vector<torch::Tensor> results;
    for (int i = 0; i < x.size(1); ++i) {
      torch::Tensor x_slice = x.index({
          torch::indexing::Slice(),
          i,
          torch::indexing::Slice(),
          torch::indexing::Slice()
      });
      results.push_back(irl_model.forward(x_slice));
    }
    return torch::stack(results, 1)
  }

  IRLModel irl_model;
};

}  // namespace navigation

#endif  // DEEP_IRL_MODEL_H