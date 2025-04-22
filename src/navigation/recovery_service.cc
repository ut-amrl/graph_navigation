#include "recovery_service.h"
#include <config_reader/config_reader.h>


#ifdef ROS1
  #define LOG_ERROR(...) ROS_ERROR(__VA_ARGS__)
  #define LOG_INFO(...)  ROS_INFO(__VA_ARGS__)
  #define LOG_WARN(...)  ROS_WARN(__VA_ARGS__)
#else
  #define LOG_ERROR(...) RCLCPP_ERROR(node_->get_logger(), __VA_ARGS__)
  #define LOG_INFO(...)  RCLCPP_INFO(node_->get_logger(), __VA_ARGS__)
  #define LOG_WARN(...)  RCLCPP_WARN(node_->get_logger(), __VA_ARGS__)
#endif

CONFIG_STRING(detect_service_name, "RecoveryParameters.detect_service_name");
CONFIG_STRING(recovery_action_name, "RecoveryParameters.recovery_action_name");
CONFIG_INT(wait_for_service_timeout,"RecoveryParameters.wait_for_service_timeout");
CONFIG_UINT(detection_buffer_size, "RecoveryParameters.detection_buffer_size");
CONFIG_FLOAT(hysteresis_thres, "RecoveryParameters.hysteresis_thres");

RecoveryService::RecoveryService(navigation::NavigationParameters& params) {
  params_ = params; // navigation params
  node_ = rclcpp::Node::make_shared("RecoveryService");
  detect_client_ = node_->create_client<amrl_msgs::srv::RecoveryDetectSrv>(CONFIG_detect_service_name);
  recovery_client_ = rclcpp_action::create_client<amrl_msgs::action::RecoveryExecute>(node_, CONFIG_recovery_action_name);
  detection_buffer_.set_capacity(CONFIG_detection_buffer_size);
  
  // Start spinning once
  exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec_->add_node(node_);
  spin_thread_ = std::thread([&]() {
    exec_->spin();
  });
}

RecoveryService::~RecoveryService() {
  if (exec_) {
    exec_->cancel();  // Stop spinning
  }

  if (spin_thread_.joinable()) {
    spin_thread_.join();  // Wait for spin thread to finish
  }

  if (exec_ && node_) {
    exec_->remove_node(node_);
  }
}

FailureStatus RecoveryService::DetectFailure(bool do_recovery) {
  std::unique_lock<std::mutex> lock(mutex_);

  if (!failure_detection_in_progress_) {
    failure_detection_in_progress_ = true;
    std::thread t(&RecoveryService::FailureDetectionRequestThread, this);
    t.detach();
  }

  if ( failure_status_ == FailureStatus::False && !do_recovery && recovery_in_progress_) {
    AbortRecoveryService();
  }

  return failure_status_;
}

void RecoveryService::FailureDetectionRequestThread() {
  bool service_result = false; // Example default
  std::shared_ptr<motion_primitives::PathRolloutBase> best_path;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    best_path = best_path_;
  }

  if (!detect_client_->wait_for_service(std::chrono::seconds(CONFIG_wait_for_service_timeout))) {
    LOG_ERROR("Stuck-check service not available! Default to false.");
  } else if (!best_path) {
    LOG_WARN("Best Path is null pointer.");
  } else {
    auto req = std::make_shared<amrl_msgs::srv::RecoveryDetectSrv::Request>();
    req->path_length = best_path->Length();
    auto future_result = detect_client_->async_send_request(req);

    if (future_result.wait_for(std::chrono::seconds(CONFIG_wait_for_service_timeout)) == std::future_status::ready) {
        auto resp = future_result.get();
        service_result = resp->failure_detected;
    } else {
        LOG_ERROR("Service call timed out or failed.");
    }
  }

  // 1) Acquire the lock to update shared state
  {
    std::lock_guard<std::mutex> lock(mutex_);
    detection_buffer_.push_back(service_result);

    // compute failure_status_ with hysteresis
    if (detection_buffer_.full()) {
      // Count how many are true
      int true_count = std::count(detection_buffer_.begin(), detection_buffer_.end(), true);
      int total = detection_buffer_.size();

      double true_percentage = static_cast<double>(true_count) / total;
      double false_percentage = 1.0 - true_percentage;

      if (true_percentage > CONFIG_hysteresis_thres) {
        failure_status_ = FailureStatus::True;
      } else if (false_percentage > CONFIG_hysteresis_thres) {
        failure_status_ = FailureStatus::False;
      } else {
        failure_status_ = FailureStatus::Uncertain;
      }

    } else {
      failure_status_ = FailureStatus::Uncertain;
    }
    failure_detection_in_progress_ = false;
  }
}

void RecoveryService::Recover() {
  bool recovery_in_progress;
  bool recovery_terminated;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recovery_in_progress = recovery_in_progress_;
    recovery_terminated = recovery_terminated_;
  }
  LOG_WARN("Recovery in progress3: %d %d", recovery_in_progress, recovery_terminated);
  if (!recovery_in_progress && !recovery_terminated) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      recovery_in_progress_ = true;
      std::thread t(&RecoveryService::RecoveryRequestThread, this);
      t.detach();
    }

    // blocking here once the goal handle is received
    int timeout_count = 0;
    rclcpp_action::ClientGoalHandle<amrl_msgs::action::RecoveryExecute>::SharedPtr goal_handle;
    while (true){
      {
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle = goal_handle_;
        recovery_in_progress = recovery_in_progress_;
      }
      if (goal_handle) {
        LOG_INFO("Goal handle received.");
        break;
      }
      if (!recovery_in_progress) {
        LOG_INFO("Recovery rejected. Continue executing.");
        break;
      }
      timeout_count++;
      if(timeout_count > 100) {
        LOG_ERROR("Recover Goal Handle Timeout exceeded.");
        break;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void RecoveryService::RecoveryRequestThread() {
  bool action_result = false;

  if (!recovery_client_->wait_for_action_server(std::chrono::seconds(1))) {
    LOG_ERROR("Recovery action server not available!");
  } else {
    // Create and fill the goal
    amrl_msgs::action::RecoveryExecute::Goal goal_msg;
    goal_msg.goal.header.frame_id = "map";  // or "odom", depending on your frame
    goal_msg.goal.header.stamp = node_->now();

    // TODO
    goal_msg.goal.pose.position.x = 1.0;  // replace with actual target
    goal_msg.goal.pose.position.y = 2.0;
    goal_msg.goal.pose.orientation.w = 1.0;  // assuming facing forward
    // TODO: Not finished here for this action: check line 1414 for navigation transform from gps to odom

    // Send the goal
    auto send_goal_options = rclcpp_action::Client<amrl_msgs::action::RecoveryExecute>::SendGoalOptions();

    // No feedback callback needed here, but you can add one if you like
    send_goal_options.result_callback = [&](const rclcpp_action::ClientGoalHandle<amrl_msgs::action::RecoveryExecute>::WrappedResult & result) {
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        action_result = result.result->completed;
      } else {
        LOG_WARN("Recovery action did not succeed: %d", static_cast<int>(result.code));
      }

      // Update shared state
      std::lock_guard<std::mutex> lock(mutex_);
      last_recovery_result_ = action_result;
      recovery_in_progress_ = false;
    };

      // Send the goal asynchronously
      auto future_goal_handle = recovery_client_->async_send_goal(goal_msg, send_goal_options);

      // Wait for goal to be accepted
      if (future_goal_handle.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        LOG_ERROR("Failed to send recovery goal (timeout).");
        std::lock_guard<std::mutex> lock(mutex_);
        last_recovery_result_ = false;
        recovery_in_progress_ = false;
        recovery_terminated_ = true;
        return;
      }

      auto goal_handle = future_goal_handle.get();
      if (!goal_handle) {
        LOG_ERROR("Recovery goal was rejected.");
        std::lock_guard<std::mutex> lock(mutex_);
        last_recovery_result_ = false;
        recovery_in_progress_ = false;
        recovery_terminated_ = true;
        return;
      }
      { // Goal was accepted — safe to store it
        std::lock_guard<std::mutex> lock(mutex_);
        goal_handle_ = goal_handle;
      }

    // Optionally wait for result (if not relying on result_callback)
    auto future_result = recovery_client_->async_get_result(goal_handle);
    future_result.wait();
    auto result = future_result.get();
    {
      // Update shared state (if not already done in callback)
      std::lock_guard<std::mutex> lock(mutex_);
      last_recovery_result_ = action_result;
      recovery_in_progress_ = false;
      recovery_terminated_ = true;

      LOG_WARN("Recovery Successfully Terminates action result: %d", static_cast<int>(action_result));
    }
  }
}

void RecoveryService::AbortRecoveryService() {
  bool abort_in_progress;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    abort_in_progress = abort_in_progress_;
  }
  if (!abort_in_progress) {
    {
      std::lock_guard<std::mutex> lock(mutex_);    
      abort_in_progress_ = true;
      std::thread t(&RecoveryService::AbortRecoveryRequestThread, this);
      t.detach();
    }
    
    // blocking here once the recovery is aborted
    int timeout_count = 0;
    while(true){
      {
        std::lock_guard<std::mutex> lock(mutex_);
        abort_in_progress = abort_in_progress_;
      }
      if(!abort_in_progress){
        LOG_INFO("Abort Recovery Service finished.");
        break;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(10));
      if(timeout_count > 100) {
        LOG_ERROR("Abort Recovery Service Timeout exceeded.");
        break;
      }
      timeout_count++;
    }
  }
}

void RecoveryService::AbortRecoveryRequestThread() {
  if (goal_handle_) {
    auto future = recovery_client_->async_cancel_goal(goal_handle_);
    if (future.wait_for(std::chrono::seconds(5)) == std::future_status::ready) {
      LOG_INFO("Recovery goal canceled.");
      goal_handle_ = nullptr;  // Reset the goal handle
    } else {
      LOG_ERROR("THIS SHOULD NOT OCCUR!! Failed to cancel recovery goal.");
    }
  } else{
    LOG_ERROR("THIS SHOULD NOT OCCUR!! No active goal handle to cancel.");
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    abort_in_progress_ = false;
  }
}
