#include <vector>

#include "torch/torch.h"

#ifndef DEEP_COST_MODEL_H
#define DEEP_COST_MODEL_H

namespace navigation {


struct EmbeddingNet: torch::nn::Module {
  explicit EmbeddingNet(int embedding_dim) {
    convnet = torch::nn::Sequential(torch::nn::Conv2d(3, 32, 5),
                                    torch::nn::PReLU(),
                                    torch::nn::MaxPool2d(torch::nn::MaxPool2dOptions(2).stride(2)),
                                    torch::nn::Conv2d(32, 64, 5), torch::nn::PReLU(),
                                    torch::nn::MaxPool2d(torch::nn::MaxPool2dOptions(2).stride(2)));

    fc = torch::nn::Sequential(
          torch::nn::Linear(3136, 256),
          torch::nn::PReLU(),
          torch::nn::Linear(256, 256),
          torch::nn::PReLU(),
          torch::nn::Linear(256, embedding_dim));
  }


  torch::Tensor forward(torch::Tensor x) {
    torch::Tensor output = convnet->forward(x);
    output = output.view({output.size(0), -1});
    return fc->forward(output);
  }

  torch::nn::Sequential convnet;
  torch::nn::Sequential fc;
};

struct CostNet : torch::nn::Module {
  explicit CostNet(int input_dim) {
    fc = torch::nn::Sequential(
      torch::nn::Linear(input_dim, 32),
      torch::nn::PReLU(),
      torch::nn::Linear(32, 32),
      torch::nn::PReLU(),
      torch::nn::Linear(32, 16),
      torch::nn::PReLU(),
      torch::nn::Linear(16, 1)
    );
  }

  torch::Tensor forward(torch::Tensor x) {
    return fc->forward(x);
  }
  torch::nn::Sequential fc;
};


struct FullCostNet : torch::nn::Module {
  explicit FullCostNet(EmbeddingNet embedding_net, CostNet cost_net) : embedding_net(embedding_net), cost_net(cost_net) {}

  torch::Tensor forward(torch::Tensor x) {
    return cost_net.forward(embedding_net.forward(x));
  }
  
  EmbeddingNet embedding_net;
  CostNet cost_net;
};

}  // namespace navigation

#endif  // DEEP_COST_MODEL_H