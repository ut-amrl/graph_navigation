#include "carrot_service.h"
#include <stdexcept>
#include <thread>

// --------------------------------------------------------------------------
// Step 1: Define macros for logging/time so the rest of the code remains the same
// --------------------------------------------------------------------------
#ifdef ROS1
  #define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
  #define LOG_INFO(...)  ROS_INFO(__VA_ARGS__)
  #define GET_TIME()     ros::Time::now()
  using CarrotSrv = amrl_msgs::CarrotPlannerSrv;
#else
  #define LOG_ERROR(...) RCLCPP_ERROR(node_->get_logger(), __VA_ARGS__)
  #define LOG_INFO(...)  RCLCPP_INFO(node_->get_logger(), __VA_ARGS__)
  #define GET_TIME()     node_->get_clock()->now()
  using CarrotSrv = amrl_msgs::srv::CarrotPlannerSrv;
#endif

// If you use GFlags for your carrot radius:
DEFINE_double(carrot_radius, 8.0, "Radius of the carrot planner circle");

CarrotService::CarrotService(const std::string& service_name) {
  #ifdef ROS1
  service_client_ = nh_.serviceClient<CarrotSrv>(service_name);
  #else
  node_ = rclcpp::Node::make_shared("CarrotService");
  service_client_ = node_->create_client<CarrotSrv>(service_name);
  #endif
}

// --------------------------------------------------------------------------
// Step 2: Single method that calls the CarrotPlanner service differently in ROS1 vs. ROS2
// --------------------------------------------------------------------------
bool CarrotService::callCarrotPlannerService(
#ifdef ROS1
  CarrotSrv &srv
#else
  CarrotSrv::Request &req,
  CarrotSrv::Response &res
#endif
)
{
#ifdef ROS1
  return service_client_.call(srv);
#else
  if (!service_client_->wait_for_service(std::chrono::seconds(2))) {
    LOG_ERROR("Carrot planner service not available after waiting!");
    return false;
  }
  auto future_result = service_client_->async_send_request(
      std::make_shared<CarrotSrv::Request>(req));
  auto status = rclcpp::spin_until_future_complete(node_, future_result);
  if (status == rclcpp::FutureReturnCode::SUCCESS) {
    res = *(future_result.get());
    return true;
  } else {
    LOG_ERROR("Carrot planner service call failed (ROS2)!");
    return false;
  }
#endif
}

void CarrotService::processCarrotPath(const carrot_ros::PathMsg& path,
  const navigation::Odom& odom)
{
  // path is nav_msgs::Path in ROS1 or nav_msgs::msg::Path in ROS2
  if (path.poses.empty()) {
  LOG_ERROR("Carrot planner service returned an empty path");
  return;
  }
  navigation::CarrotPlan new_plan;
  for (const auto& pose : path.poses) {
  new_plan.path.emplace_back(Eigen::Vector2f(
  pose.pose.position.x, pose.pose.position.y));
  }

  {
  std::lock_guard<std::mutex> lock(mutex_);
  latest_carrot_plan_ = new_plan;
  latest_odom_ = odom;
  has_carrot_ = true;
  }
  cv_.notify_all();
}

navigation::CarrotPlan CarrotService::GetCarrot(const Eigen::Vector2f& local_waypoint,
                                                const navigation::Odom& odom) {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!service_request_ongoing_) {
      service_request_ongoing_ = true;

      // Start a thread to request carrot update
      std::thread(&CarrotService::RequestCarrotUpdate, this, local_waypoint, odom).detach();
    }
  }

  // Transform the current path to the current odom frame
  navigation::CarrotPlan carrot_plan;
  TransformCarrot(odom, carrot_plan);
  return carrot_plan;
}

void CarrotService::TransformCarrot(const navigation::Odom& odom,
                                    navigation::CarrotPlan& carrot_plan) {
  // Block until a carrot exists
  std::unique_lock<std::mutex> lock(mutex_);
  cv_.wait(lock, [this] { return has_carrot_; });

  // Make a local copy of the plan
  navigation::CarrotPlan latest_carrot_plan = latest_carrot_plan_;
  lock.unlock();

  // Compute transform from old to new odom frame
  Eigen::Affine2f T_baset0_odom = latest_odom_.toAffine2f();
  Eigen::Affine2f T_baset1_odom = odom.toAffine2f();
  Eigen::Affine2f T_baset0_baset1 = T_baset1_odom.inverse() * T_baset0_odom;

  for (const auto& point : latest_carrot_plan.path) {
    carrot_plan.path.emplace_back(T_baset0_baset1 * point);
  }
  carrot_plan.path_idx = carrot_plan.path.size() - 1;
}

void CarrotService::RequestCarrotUpdate(const Eigen::Vector2f& local_waypoint,
  const navigation::Odom& odom)
{
  try {
    #ifdef ROS1
    CarrotSrv srv;
    srv.request.carrot.header.stamp = GET_TIME();
    srv.request.carrot.header.frame_id = "base_link";
    srv.request.carrot.pose.position.x = local_waypoint.x();
    srv.request.carrot.pose.position.y = local_waypoint.y();
    srv.request.carrot.pose.orientation.w = 1.0;

    if (callCarrotPlannerService(srv)) {
      processCarrotPath(srv.response.path, odom);
    } else {
      LOG_ERROR("Failed to call carrot planner service (ROS1)!");
    }
    #else
    CarrotSrv::Request req;
    CarrotSrv::Response res;

    req.carrot.header.stamp = GET_TIME();
    req.carrot.header.frame_id = "base_link";
    req.carrot.pose.position.x = local_waypoint.x();
    req.carrot.pose.position.y = local_waypoint.y();
    req.carrot.pose.orientation.w = 1.0;

    if (callCarrotPlannerService(req, res)) {
      processCarrotPath(res.path, odom);
    } else {
      LOG_ERROR("Failed to call carrot planner service (ROS2)!");
    }
    #endif
  } catch (const std::exception& e) {
  LOG_ERROR("Exception in carrot planner service: %s", e.what());
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    service_request_ongoing_ = false;
  }
}