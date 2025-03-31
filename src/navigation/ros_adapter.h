#pragma once

#include <memory>
#include <string>
#include <vector>

#include "navigation_parameters.h"

#ifdef ROS1
  #include <ros/ros.h>
#else
  #include <rclcpp/rclcpp.hpp>
#endif

// Forward declarations to avoid bringing in all ROS headers here:
namespace ros {
  class NodeHandle;
}

namespace navigation {

// Forward declaration for the Navigation class.
class Navigation; 

class RosAdapter {
 public:
  virtual ~RosAdapter() = default;

  // Initialize the ROS node and set up subscribers, publishers, etc.
  virtual void Initialize(std::shared_ptr<Navigation> navigation) = 0;

  // Spin or run the event loop. For ROS1, this may call ros::spin().
  // For ROS2, rclcpp::spin().
  virtual void spinLoop() = 0;
  
  #ifdef ROS2
  virtual rclcpp::Node::SharedPtr GetNodeHandle() = 0;
  #endif

  // Provide a factory method to create either a ROS1 or ROS2 adapter 
  // based on a compile-time flag (e.g. -DROS2).
  static std::shared_ptr<RosAdapter> create();

  #ifdef ROS2
    static std::shared_ptr<RosAdapter> create(rclcpp::Node::SharedPtr node, const navigation::NavigationParameters& params);
  #endif

  std::weak_ptr<Navigation> navigation_;
};

}  // namespace navigation