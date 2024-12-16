#ifndef CARROT_BASE_H
#define CARROT_BASE_H

#include <vector>

#include "eigen3/Eigen/Dense"
#include "navigation_types.h"

using navigation::CarrotPlan;
using navigation::Odom;

class CarrotBase {
 public:
  // Virtual destructor for proper cleanup of derived classes
  virtual ~CarrotBase() {}

  /**
   * Pure virtual function to be implemented by derived classes.
   * @param local_carrot: The target waypoint in local coordinates.
   * @return A struct containing the computed carrot plan.
   */
  virtual struct CarrotPlan GetCarrot(const Eigen::Vector2f& local_carrot,
                                      const Odom& odom) = 0;

 protected:
  Eigen::Vector2f current_carrot_;  // Current carrot location
};

#endif