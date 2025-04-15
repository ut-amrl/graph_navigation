#ifndef OMNI_SAMPLER_H
#define OMNI_SAMPLER_H

#include <memory>
#include <vector>
#include "motion_primitives.h"

namespace motion_primitives {

class OmniSampler : public PathRolloutSamplerBase {
 public:
  // Constructor
  OmniSampler();

  // Generate a set of omnidirectional path rollouts.
  std::vector<std::shared_ptr<PathRolloutBase>> GetSamples(int n) override;
};

}  // namespace motion_primitives

#endif  // OMNI_SAMPLER_H
