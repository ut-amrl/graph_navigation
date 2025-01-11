#include "carrot_service.h"

#include <stdexcept>
#include <thread>

using amrl_msgs::CarrotPlannerSrv;
using Eigen::Affine2f;
using Eigen::Vector2f;
using navigation::CarrotPlan;
using navigation::Odom;
using std::vector;

DEFINE_double(carrot_radius, 8.0, "Radius of the carrot planner circle");

// Constructor
CarrotService::CarrotService(const std::string& service_name) {
  // Initialize the ROS service client
  service_client_ = nh_.serviceClient<CarrotPlannerSrv>(service_name);
}

CarrotPlan CarrotService::GetCarrot(const Vector2f& local_waypoint,
                                    const Odom& odom) {
  // Initiate a service request if no request is ongoing
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!service_request_ongoing_) {
      service_request_ongoing_ = true;
      
      // Project local_waypoint to a point on the circle around the robot
      // float distance = local_waypoint.norm();
      // Vector2f projected_waypoint;
      // if (distance > 0.0f) {
      //   projected_waypoint = FLAGS_carrot_radius * local_waypoint.normalized();
      // } else {
      //   // If the waypoint is at the origin, arbitrarily choose a direction.
      //   projected_waypoint = Vector2f(FLAGS_carrot_radius, 0.0f);
      // }
      // printf("RequestCarrotUpdate start\n");
      Vector2f projected_waypoint = local_waypoint;
      std::thread(&CarrotService::RequestCarrotUpdate, this, projected_waypoint,
                  odom)
          .detach();
    }
  }
  // Transform the current path to the current odom frame
  CarrotPlan carrot_plan;
  TransformCarrot(odom, carrot_plan);
  return carrot_plan;
}

void CarrotService::TransformCarrot(const Odom& odom, CarrotPlan& carrot_plan) {
  // Block until a carrot exists
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this] { return has_carrot_; });
  // Transform the path to the local frame at t''
  CarrotPlan latest_carrot_plan;
  latest_carrot_plan = latest_carrot_plan_;
  lock.unlock();
  
  // Transformations at time t0 and t1
  Eigen::Affine2f T_baset0_odom =
      latest_odom_.toAffine2f();  // Transformation from local @ t' to odom
  Eigen::Affine2f T_baset1_odom =
      odom.toAffine2f();  // Transformation from local @ t'' to odom

  // Compute the relative transformation: local @ t' -> odom -> local @ t''
  Eigen::Affine2f T_baset0_baset1 = T_baset1_odom.inverse() * T_baset0_odom;

  for (const auto& point : latest_carrot_plan.path) {
    // Transform local frame @ t' to odom frame, then to local frame @ t''
    carrot_plan.path.emplace_back(T_baset0_baset1 * point);
  }
  carrot_plan.path_idx = carrot_plan.path.size() - 1;
}

void CarrotService::RequestCarrotUpdate(const Vector2f& local_waypoint,
                                        const Odom& odom) {
  try {
    CarrotPlannerSrv srv;

    // Prepare service request
    srv.request.carrot.header.stamp = ros::Time::now();
    srv.request.carrot.header.frame_id = "map";
    srv.request.carrot.pose.position.x = local_waypoint.x();
    srv.request.carrot.pose.position.y = local_waypoint.y();
    srv.request.carrot.pose.position.z = 0.0;
    srv.request.carrot.pose.orientation.x = 0.0;
    srv.request.carrot.pose.orientation.y = 0.0;
    srv.request.carrot.pose.orientation.z = 0.0;
    srv.request.carrot.pose.orientation.w = 1.0;

    // Call the service
    if (service_client_.call(srv)) {
      if (!srv.response.path.poses.empty()) {
        CarrotPlan new_plan;

        // Extract path
        for (const auto& pose : srv.response.path.poses) {
          new_plan.path.emplace_back(
              Eigen::Vector2f(pose.pose.position.x, pose.pose.position.y));
        }

        // Update shared carrot plan
        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_carrot_plan_ = new_plan;
          latest_odom_ = odom;
          has_carrot_ = true;
        }
        // printf("Service call successful\n");
        cv_.notify_all();
      } else {
        ROS_ERROR("Carrot planner service failed to compute a valid plan");
      }
    } else {
      ROS_ERROR("Failed to call carrot planner service");
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Exception in carrot planner service: %s", e.what());
  }

  // Mark service request as completed
  {
    std::lock_guard<std::mutex> lock(mutex_);
    service_request_ongoing_ = false;
  }
}
